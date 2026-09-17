// ============================================================================
// copc_partial_load_vtk
//
// "I have a huge COPC file and I want ONLY a small part of it in memory."
//
// This is the example that does it end to end:
//
//      1. peek at the file (header only) to learn its extent and point count
//      2. ask PDAL for ONE box at ONE level of detail
//      3. turn the returned points into a vtkPolyData
//      4. render it
//
// Nothing else is ever read off disk. If the file is 500 GB and you ask for a
// 50 m box at 1 m spacing, you get a few tens of thousands of points and a few
// hundred KB of I/O.
//
// Usage:
//
//   # what is in this file? (reads the header only, prints, exits)
//   ./copc_partial_load_vtk big.copc.laz --info
//
//   # load one box at full detail
//   ./copc_partial_load_vtk big.copc.laz --bounds 515370,515390,4918350,4918370
//
//   # same box, coarse -- far fewer points, far less I/O
//   ./copc_partial_load_vtk big.copc.laz --bounds 515370,515390,4918350,4918370 \
//       --resolution 0.5
//
//   # the middle 10% of whatever the file happens to cover, coarse
//   ./copc_partial_load_vtk big.copc.laz --fraction 0.1 --resolution 1.0
//
//   # compare five LODs over the same box and exit without rendering
//   ./copc_partial_load_vtk big.copc.laz --fraction 0.2 --ladder
//
// Requires -DUSE_PDAL=ON.
// ============================================================================

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCubeSource.h>
#include <vtkDoubleArray.h>
#include <vtkLookupTable.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkSmartPointer.h>
#include <vtkInteractorStyleTrackballCamera.h>

#include <algorithm>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

// ----------------------------------------------------------------------------
// A plain 2D box, and PDAL's string spelling of it.
//
// PDAL's `bounds` option is a string:
//
//      ([xmin,xmax],[ymin,ymax])           Z unconstrained
//      ([xmin,xmax],[ymin,ymax],[zmin,zmax])
//
// Everything outside it is pruned during the octree walk, before any LAZ
// chunk is decompressed.
// ----------------------------------------------------------------------------
struct Box2D
{
    double xmin = 0, xmax = 0, ymin = 0, ymax = 0;

    std::string toPdalBounds() const
    {
        std::ostringstream os;

        os << std::fixed << std::setprecision(6) << "([" << xmin << "," << xmax
           << "],[" << ymin << "," << ymax << "])";

        return os.str();
    }

    double width() const { return xmax - xmin; }
    double height() const { return ymax - ymin; }
};

// ----------------------------------------------------------------------------
// STEP 1 -- peek at the file without reading any points.
//
// Stage::preview() returns a QuickInfo. For readers.copc this costs a header
// read and nothing more: PDAL does NOT scan the points to find the extent,
// because the LAS header already states it.
//
// This is how a real viewer decides where to put the camera before it has
// loaded a single point.
// ----------------------------------------------------------------------------
struct FileInfo
{
    double minx, maxx, miny, maxy, minz, maxz;
    uint64_t pointCount;
};

static FileInfo peek(const std::string &filename)
{
    pdal::StageFactory factory;

    pdal::Stage *reader = factory.createStage("readers.copc");

    if (!reader)
        throw std::runtime_error(
            "readers.copc unavailable -- this PDAL build has no COPC support");

    pdal::Options options;
    options.add("filename", filename);
    reader->setOptions(options);

    const pdal::QuickInfo qi = reader->preview();

    if (!qi.valid())
        throw std::runtime_error("could not read header of " + filename);

    FileInfo info;

    info.minx = qi.m_bounds.minx;
    info.maxx = qi.m_bounds.maxx;
    info.miny = qi.m_bounds.miny;
    info.maxy = qi.m_bounds.maxy;
    info.minz = qi.m_bounds.minz;
    info.maxz = qi.m_bounds.maxz;
    info.pointCount = qi.m_pointCount;

    return info;
}

// ----------------------------------------------------------------------------
// STEP 2 -- the partial load itself.
//
// This is the ONLY function in the program that touches point data, and it is
// four PDAL calls long. `bounds` prunes in space, `resolution` prunes in
// depth, and both happen during the hierarchy walk -- so the cost of this call
// scales with what you ASKED FOR, not with the size of the file.
//
// resolution <= 0 means "no limit, give me everything you have in that box".
// ----------------------------------------------------------------------------
struct LoadResult
{
    vtkSmartPointer<vtkPolyData> polyData;
    std::size_t pointCount = 0;
    double seconds = 0.0;
};

static LoadResult loadRegion(const std::string &filename,
                             const Box2D &box,
                             double resolution)
{
    const auto t0 = std::chrono::steady_clock::now();

    pdal::StageFactory factory;

    pdal::Stage *reader = factory.createStage("readers.copc");

    if (!reader)
        throw std::runtime_error("readers.copc unavailable");

    pdal::Options options;

    options.add("filename", filename);

    // WHERE  -- prunes the octree walk in space.
    options.add("bounds", box.toPdalBounds());

    // HOW DETAILED -- prunes the octree walk in depth. PDAL descends only
    // while the node spacing is still coarser than this value. Omit the option
    // entirely to get every level the file contains.
    if (resolution > 0.0)
        options.add("resolution", resolution);

    reader->setOptions(options);

    pdal::PointTable table;

    reader->prepare(table);

    pdal::PointViewSet views = reader->execute(table);

    // ------------------------------------------------------------------------
    // STEP 3 -- PDAL PointView  ->  VTK vtkPolyData
    //
    // PDAL is columnar: you pull one dimension at a time, by point id. VTK
    // wants interleaved XYZ in a vtkPoints, plus one vertex cell per point so
    // that the mapper will actually draw them.
    // ------------------------------------------------------------------------

    std::size_t total = 0;

    for (const auto &view : views)
        total += view->size();

    vtkNew<vtkPoints> points;
    points->SetDataTypeToDouble();
    points->Allocate(static_cast<vtkIdType>(total));

    vtkNew<vtkCellArray> vertices;
    vertices->AllocateEstimate(static_cast<vtkIdType>(total), 1);

    // Colour by elevation. Carrying Z as a scalar array costs one double per
    // point and makes the LOD visible at a glance.
    vtkNew<vtkDoubleArray> elevation;
    elevation->SetName("Elevation");
    elevation->SetNumberOfComponents(1);
    elevation->Allocate(static_cast<vtkIdType>(total));

    for (const auto &view : views)
    {
        for (pdal::PointId i = 0; i < view->size(); ++i)
        {
            const double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
            const double y = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
            const double z = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);

            const vtkIdType id = points->InsertNextPoint(x, y, z);

            vertices->InsertNextCell(1, &id);

            elevation->InsertNextValue(z);
        }
    }

    vtkNew<vtkPolyData> polyData;

    polyData->SetPoints(points);
    polyData->SetVerts(vertices);
    polyData->GetPointData()->SetScalars(elevation);

    const auto t1 = std::chrono::steady_clock::now();

    LoadResult result;

    result.polyData = polyData;
    result.pointCount = total;
    result.seconds = std::chrono::duration<double>(t1 - t0).count();

    return result;
}

// ----------------------------------------------------------------------------
// Report a load in human terms. The ratio against the file's total point count
// is the number that makes the point.
// ----------------------------------------------------------------------------
static void report(const std::string &label,
                   const LoadResult &r,
                   const FileInfo &info)
{
    // Roughly what this costs in RAM once it is a vtkPolyData: 3 doubles for
    // the coordinates, 1 for the elevation scalar, and a vertex cell.
    const double megabytes =
        double(r.pointCount) * (3 * sizeof(double) + sizeof(double) + 2 * sizeof(vtkIdType)) /
        (1024.0 * 1024.0);

    std::cout << std::left << std::setw(22) << label << std::right
              << std::setw(10) << r.pointCount << " pts  " << std::fixed
              << std::setprecision(2) << std::setw(7)
              << (100.0 * double(r.pointCount) / double(std::max<uint64_t>(info.pointCount, 1)))
              << " % of file  " << std::setw(8) << std::setprecision(1)
              << megabytes << " MB  " << std::setw(7) << std::setprecision(3)
              << r.seconds << " s\n";
}

// ----------------------------------------------------------------------------
// A wireframe box showing exactly what we asked for, so the loaded points can
// be seen sitting inside the query region.
// ----------------------------------------------------------------------------
static vtkSmartPointer<vtkActor> makeBoxActor(const Box2D &box,
                                              double zmin,
                                              double zmax)
{
    vtkNew<vtkCubeSource> cube;

    cube->SetBounds(box.xmin, box.xmax, box.ymin, box.ymax, zmin, zmax);

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(cube->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

    actor->SetMapper(mapper);
    actor->GetProperty()->SetRepresentationToWireframe();
    actor->GetProperty()->SetColor(1.0, 0.6, 0.2);
    actor->GetProperty()->SetLineWidth(2.0);
    actor->PickableOff();

    return actor;
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr
            << "usage: " << argv[0] << " <file.copc.laz> [options]\n\n"
            << "  --info                        print header info and exit\n"
            << "  --bounds xmin,xmax,ymin,ymax  region to load\n"
            << "  --fraction F                  instead of --bounds, load the\n"
            << "                                centre F (0..1) of the extent\n"
            << "  --resolution R                target point spacing (omit for full)\n"
            << "  --ladder                      load the same box at 5 LODs, no render\n";

        return 1;
    }

    const std::string filename = argv[1];

    bool wantInfoOnly = false;
    bool wantLadder = false;
    bool haveBounds = false;
    Box2D box;
    double fraction = 0.25;
    double resolution = 0.0;

    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--info")
        {
            wantInfoOnly = true;
        }
        else if (arg == "--ladder")
        {
            wantLadder = true;
        }
        else if (arg == "--bounds" && i + 1 < argc)
        {
            if (std::sscanf(argv[++i], "%lf,%lf,%lf,%lf", &box.xmin, &box.xmax,
                            &box.ymin, &box.ymax) != 4)
            {
                std::cerr << "--bounds wants xmin,xmax,ymin,ymax\n";
                return 1;
            }
            haveBounds = true;
        }
        else if (arg == "--fraction" && i + 1 < argc)
        {
            fraction = std::atof(argv[++i]);
        }
        else if (arg == "--resolution" && i + 1 < argc)
        {
            resolution = std::atof(argv[++i]);
        }
        else
        {
            std::cerr << "unknown argument: " << arg << "\n";
            return 1;
        }
    }

    try
    {
        // --------------------------------------------------------------------
        // Peek. Header only.
        // --------------------------------------------------------------------

        const FileInfo info = peek(filename);

        std::cout << std::fixed << std::setprecision(3);

        std::cout << "\n=== FILE ====================================================\n";
        std::cout << "File          : " << filename << "\n";
        std::cout << "Total points  : " << info.pointCount << "\n";
        std::cout << "Extent X      : " << info.minx << " ... " << info.maxx
                  << "  (" << (info.maxx - info.minx) << ")\n";
        std::cout << "Extent Y      : " << info.miny << " ... " << info.maxy
                  << "  (" << (info.maxy - info.miny) << ")\n";
        std::cout << "Extent Z      : " << info.minz << " ... " << info.maxz
                  << "  (" << (info.maxz - info.minz) << ")\n";
        std::cout << "\n-> Known from the header alone. No points read yet.\n";

        if (wantInfoOnly)
            return 0;

        // --------------------------------------------------------------------
        // Decide what to ask for.
        // --------------------------------------------------------------------

        if (!haveBounds)
        {
            // Centre `fraction` of the extent, so the example runs on any file
            // without the user having to know its coordinate system.
            const double cx = 0.5 * (info.minx + info.maxx);
            const double cy = 0.5 * (info.miny + info.maxy);
            const double hx = 0.5 * fraction * (info.maxx - info.minx);
            const double hy = 0.5 * fraction * (info.maxy - info.miny);

            box = {cx - hx, cx + hx, cy - hy, cy + hy};

            std::cout << "\nNo --bounds given, using the centre " << (fraction * 100.0)
                      << " % of the extent.\n";
        }

        std::cout << "\n=== QUERY ===================================================\n";
        std::cout << "bounds     : " << box.toPdalBounds() << "\n";
        std::cout << "             " << box.width() << " x " << box.height()
                  << " units\n";

        // --------------------------------------------------------------------
        // --ladder: the same box at five detail levels. This is the clearest
        // demonstration of what `resolution` does, because only ONE option
        // changes between runs.
        // --------------------------------------------------------------------

        if (wantLadder)
        {
            // Scale the ladder to the size of the box so it is meaningful on
            // any dataset: from "one point per 1/8 of the box" down to full.
            const double base = std::max(box.width(), box.height());

            const double steps[4] = {base / 8.0, base / 32.0, base / 128.0,
                                     base / 512.0};

            std::cout << "\nSame bounds, five levels of detail:\n\n";

            for (double r : steps)
            {
                const LoadResult lr = loadRegion(filename, box, r);

                std::ostringstream label;
                label << "resolution " << std::fixed << std::setprecision(4) << r;

                report(label.str(), lr, info);
            }

            const LoadResult full = loadRegion(filename, box, 0.0);

            report("full detail", full, info);

            std::cout << "\nOnly the `resolution` option changed between those "
                         "runs.\nThe bounds, the file and the code were identical.\n\n";

            return 0;
        }

        // --------------------------------------------------------------------
        // The single partial load.
        // --------------------------------------------------------------------

        if (resolution > 0.0)
            std::cout << "resolution : " << resolution << "\n";
        else
            std::cout << "resolution : (none -- full available detail)\n";

        const LoadResult loaded = loadRegion(filename, box, resolution);

        std::cout << "\n=== RESULT ==================================================\n";

        report("loaded", loaded, info);

        std::cout << "\n-> " << loaded.pointCount << " points are in memory.\n"
                  << "   The other "
                  << (info.pointCount > loaded.pointCount
                          ? info.pointCount - loaded.pointCount
                          : 0)
                  << " points were never decompressed, never allocated,\n"
                  << "   and in the HTTP case never even downloaded.\n";

        if (loaded.pointCount == 0)
        {
            std::cerr << "\nNothing came back -- are those bounds inside the "
                         "file's extent?\n";
            return 1;
        }

        // --------------------------------------------------------------------
        // STEP 4 -- render it.
        // --------------------------------------------------------------------

        vtkNew<vtkLookupTable> lut;
        lut->SetHueRange(0.667, 0.0); // blue (low) -> red (high)
        lut->SetTableRange(info.minz, info.maxz);
        lut->Build();

        vtkNew<vtkPolyDataMapper> mapper;

        mapper->SetInputData(loaded.polyData);
        mapper->SetLookupTable(lut);
        mapper->SetScalarRange(info.minz, info.maxz);
        mapper->ScalarVisibilityOn();

        vtkNew<vtkActor> actor;

        actor->SetMapper(mapper);
        actor->GetProperty()->SetPointSize(2.0);

        vtkNew<vtkRenderer> renderer;

        renderer->SetBackground(0.08, 0.10, 0.15);
        renderer->AddActor(actor);
        renderer->AddActor(makeBoxActor(box, info.minz, info.maxz));

        vtkNew<vtkScalarBarActor> scalarBar;

        scalarBar->SetLookupTable(lut);
        scalarBar->SetTitle("Z");
        scalarBar->SetNumberOfLabels(5);

        renderer->AddActor(scalarBar);

        renderer->ResetCamera();

        vtkNew<vtkRenderWindow> window;

        window->AddRenderer(renderer);
        window->SetSize(1200, 800);
        window->SetWindowName("COPC partial load");

        vtkNew<vtkRenderWindowInteractor> interactor;

        interactor->SetRenderWindow(window);

        vtkNew<vtkInteractorStyleTrackballCamera> style;

        interactor->SetInteractorStyle(style);

        window->Render();
        interactor->Start();
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
