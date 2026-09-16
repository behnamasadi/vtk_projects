// ============================================================================
// copc_camera_streaming
//
// The closed loop. This is the example that answers "how is the level of
// detail actually set?", because here nothing is hard-coded -- the LOD is
// DERIVED, every time the camera stops moving:
//
//      camera position + focal point + view angle + window size
//              |
//              v
//      frustum planes  ->  visible XY box      (WHERE)
//      screen-space error ->  resolution       (HOW DETAILED)
//              |
//              v
//      readers.copc  with those two options
//              |
//              v
//      a new vtkPolyData replaces the old one
//
// Drag, zoom, pan. Every time you release the mouse the console prints the
// full decision: the numbers that went in, the query that came out, and how
// many points it cost. Zoom in and the resolution number gets smaller and the
// point count stays roughly constant -- that constancy IS the point of LOD.
//
// Usage:
//      ./copc_camera_streaming big.copc.laz [--budget 500000] [--error 2.0]
//
// Keys:
//      r   force a reload now
//      b   toggle the wireframe box that shows the current query region
//      q   quit
//
// Requires -DUSE_PDAL=ON.
//
// NOTE ON THE BUDGET: the level of detail is chosen from the COPC OCTREE
// INDEX, which this program parses itself with copc_index.hpp, before any PDAL
// query is issued. The naive alternative -- query, notice the result is too
// big, double the resolution, query again -- costs a full read per guess and
// wastes one entirely whenever doubling the resolution does not happen to
// change the depth. Because we hold the index, costing every candidate level
// against the visible box is pure arithmetic, so every camera move results in
// EXACTLY ONE query, at a level already known to fit. See SelectLevel below.
//
// NOTE ON THREADING: the load below is SYNCHRONOUS, and it runs on
// EndInteractionEvent -- that is, when you let go of the mouse, not while you
// are dragging. That keeps the example readable and keeps the drag itself at
// full frame rate. A production viewer instead loads per-octree-node on a
// worker thread and appends nodes as they arrive; see
// docs/copc_laz_lod_tutorial.md section 7.
// ============================================================================

#include "copc_index.hpp"

#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCellArray.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkDoubleArray.h>
#include <vtkFrustumSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLookupTable.h>
#include <vtkMath.h>
#include <vtkNew.h>
#include <vtkPlanes.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkTextProperty.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>

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

    // Clamp this box to another one; returns false if they do not overlap.
    bool clampTo(const Box2D &limit)
    {
        xmin = std::max(xmin, limit.xmin);
        xmax = std::min(xmax, limit.xmax);
        ymin = std::max(ymin, limit.ymin);
        ymax = std::min(ymax, limit.ymax);

        return xmax > xmin && ymax > ymin;
    }
};

// ============================================================================
// The streaming viewer state.
// ============================================================================
class CopcStreamer
{
public:
    std::string Filename;
    vtkRenderer *Renderer = nullptr;
    vtkActor *CloudActor = nullptr;
    vtkActor *BoxActor = nullptr;
    vtkTextActor *Hud = nullptr;

    // Dataset extent, from the header.
    Box2D FullExtent;
    double MinZ = 0.0, MaxZ = 0.0;
    uint64_t TotalPoints = 0;

    // The COPC octree index, parsed once at startup. Holding it is what lets
    // us answer "what will this cost?" offline, for any box at any level.
    copc::Index Octree;

    // Policy knobs.
    //
    // TargetPixelError: how far apart, in PIXELS, we are willing to let
    // neighbouring points be on screen. ~1-3 px looks solid; larger is
    // cheaper and sparser. THIS is the only "quality" dial -- everything else
    // is derived from the camera.
    double TargetPixelError = 2.0;

    // PointBudget: the hard ceiling. The level is coarsened until the index
    // says it fits -- before querying, not after. Frame rate is what we are
    // really defending.
    std::size_t PointBudget = 500000;

    // What we last asked for, so we can skip redundant reloads.
    Box2D LastBox;
    double LastResolution = -1.0;
    bool HasLoaded = false;

    // ------------------------------------------------------------------------
    // WHERE: turn the camera frustum into an XY box.
    //
    // vtkCamera::GetFrustumPlanes gives six inward-facing planes. Handing them
    // to vtkPlanes + vtkFrustumSource produces the actual frustum geometry,
    // whose axis-aligned bounds are a conservative superset of what is
    // visible. Clamped to the dataset extent, that is our query box.
    //
    // Conservative is correct here: we would rather load a little too much
    // than have data pop in at the edge of the screen.
    // ------------------------------------------------------------------------
    bool computeVisibleBox(Box2D &out) const
    {
        vtkCamera *camera = Renderer->GetActiveCamera();

        const int *size = Renderer->GetRenderWindow()->GetSize();

        const double aspect =
            double(std::max(size[0], 1)) / double(std::max(size[1], 1));

        double planes[24];
        camera->GetFrustumPlanes(aspect, planes);

        vtkNew<vtkPlanes> frustumPlanes;
        frustumPlanes->SetFrustumPlanes(planes);

        vtkNew<vtkFrustumSource> frustum;
        frustum->SetPlanes(frustumPlanes);
        frustum->ShowLinesOff();
        frustum->Update();

        double b[6];
        frustum->GetOutput()->GetBounds(b);

        out = {b[0], b[1], b[2], b[3]};

        // Outside the data entirely -> nothing to do.
        return out.clampTo(FullExtent);
    }

    // ------------------------------------------------------------------------
    // HOW DETAILED: turn the camera geometry into a point spacing, in world
    // units, via screen-space error.
    //
    //      focalLengthPixels = height / (2 * tan(fov/2))
    //
    // A world-space length L at distance D covers
    //
    //      pixels = L / D * focalLengthPixels
    //
    // Invert it. We want neighbouring points to land TargetPixelError apart,
    // so the spacing we should ask PDAL for is:
    //
    //      resolution = TargetPixelError * D / focalLengthPixels
    //
    // That is the whole derivation. Move the camera closer (D shrinks) and the
    // requested resolution shrinks with it -- which makes PDAL descend deeper
    // into the octree. Nobody chose "level 4"; level 4 is what falls out.
    // ------------------------------------------------------------------------
    double computeResolution(double &distanceOut,
                             double &focalLengthPixelsOut) const
    {
        vtkCamera *camera = Renderer->GetActiveCamera();

        double position[3], focal[3];
        camera->GetPosition(position);
        camera->GetFocalPoint(focal);

        const double dx = focal[0] - position[0];
        const double dy = focal[1] - position[1];
        const double dz = focal[2] - position[2];

        const double distance = std::sqrt(dx * dx + dy * dy + dz * dz);

        const int *size = Renderer->GetRenderWindow()->GetSize();
        const double height = double(std::max(size[1], 1));

        double focalLengthPixels;

        if (camera->GetParallelProjection())
        {
            // Orthographic: size on screen does not depend on distance at all.
            // ParallelScale is half the visible height in world units.
            const double worldPerPixel =
                (2.0 * camera->GetParallelScale()) / height;

            distanceOut = distance;
            focalLengthPixelsOut = 0.0;

            return TargetPixelError * worldPerPixel;
        }

        const double fovRad =
            vtkMath::RadiansFromDegrees(camera->GetViewAngle());

        focalLengthPixels = height / (2.0 * std::tan(fovRad * 0.5));

        distanceOut = distance;
        focalLengthPixelsOut = focalLengthPixels;

        if (focalLengthPixels < 1e-9)
            return 0.0;

        return TargetPixelError * distance / focalLengthPixels;
    }

    // ------------------------------------------------------------------------
    // The PDAL call. Identical to copc_partial_load_vtk::loadRegion -- the
    // only difference in this whole program is WHO computes the arguments.
    // ------------------------------------------------------------------------
    vtkSmartPointer<vtkPolyData> query(const Box2D &box,
                                       double resolution,
                                       std::size_t &pointsOut,
                                       double &secondsOut) const
    {
        const auto t0 = std::chrono::steady_clock::now();

        pdal::StageFactory factory;

        pdal::Stage *reader = factory.createStage("readers.copc");

        if (!reader)
            throw std::runtime_error("readers.copc unavailable");

        pdal::Options options;

        options.add("filename", Filename);
        options.add("bounds", box.toPdalBounds());

        if (resolution > 0.0)
            options.add("resolution", resolution);

        reader->setOptions(options);

        pdal::PointTable table;

        reader->prepare(table);

        pdal::PointViewSet views = reader->execute(table);

        std::size_t total = 0;

        for (const auto &view : views)
            total += view->size();

        vtkNew<vtkPoints> points;
        points->SetDataTypeToDouble();
        points->Allocate(static_cast<vtkIdType>(total));

        vtkNew<vtkCellArray> verts;
        verts->AllocateEstimate(static_cast<vtkIdType>(total), 1);

        vtkNew<vtkDoubleArray> elevation;
        elevation->SetName("Elevation");
        elevation->Allocate(static_cast<vtkIdType>(total));

        for (const auto &view : views)
        {
            for (pdal::PointId i = 0; i < view->size(); ++i)
            {
                const double x =
                    view->getFieldAs<double>(pdal::Dimension::Id::X, i);
                const double y =
                    view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
                const double z =
                    view->getFieldAs<double>(pdal::Dimension::Id::Z, i);

                const vtkIdType id = points->InsertNextPoint(x, y, z);

                verts->InsertNextCell(1, &id);
                elevation->InsertNextValue(z);
            }
        }

        vtkSmartPointer<vtkPolyData> polyData =
            vtkSmartPointer<vtkPolyData>::New();

        polyData->SetPoints(points);
        polyData->SetVerts(verts);
        polyData->GetPointData()->SetScalars(elevation);

        pointsOut = total;

        secondsOut = std::chrono::duration<double>(
                         std::chrono::steady_clock::now() - t0)
                         .count();

        return polyData;
    }

    // ------------------------------------------------------------------------
    // One full decision cycle.
    // ------------------------------------------------------------------------
    void update(bool force)
    {
        Box2D box;

        if (!computeVisibleBox(box))
        {
            std::cout << "\n[camera is looking away from the data -- nothing "
                         "to load]\n";
            return;
        }

        double distance = 0.0, focalLengthPixels = 0.0;

        double resolution = computeResolution(distance, focalLengthPixels);

        // Skip work if neither input moved meaningfully. ModifiedEvent fires
        // constantly; the query is expensive. (A tiny wobble of the mouse is
        // not a reason to re-read the file.)
        if (!force && HasLoaded && similarTo(box, resolution))
            return;

        std::cout << "\n============================================================\n";
        std::cout << std::fixed << std::setprecision(3);

        std::cout << "CAMERA\n";
        std::cout << "  distance to focal point : " << distance << "\n";

        if (focalLengthPixels > 0.0)
            std::cout << "  focal length            : " << focalLengthPixels
                      << " px\n";
        else
            std::cout << "  projection              : parallel\n";

        std::cout << "\nDERIVED QUERY\n";
        std::cout << "  visible box   : " << box.toPdalBounds() << "\n";
        std::cout << "                  " << box.width() << " x " << box.height()
                  << " units ("
                  << (100.0 * box.width() * box.height() /
                      std::max(FullExtent.width() * FullExtent.height(), 1e-9))
                  << " % of extent)\n";

        std::cout << "  resolution    : " << std::setprecision(5) << resolution
                  << "  (= " << TargetPixelError << " px * " << std::setprecision(3)
                  << distance << " / " << focalLengthPixels << ")\n";

        // ------------------------------------------------------------------
        // BUDGET, enforced from the index -- before any point data moves.
        //
        // chooseLevelForBudget costs every candidate level against this box
        // using the hierarchy we already hold, and returns the deepest one
        // that fits. It is arithmetic over a few hundred structs: no I/O, no
        // decompression, microseconds.
        //
        // It is safe because QueryCost::points counts whole nodes, which is an
        // UPPER BOUND on what PDAL returns after cropping to the box -- so a
        // level that fits by that measure is guaranteed to fit for real. It is
        // also conservative for the same reason, and can pick a level coarser
        // than strictly needed. Erring toward too few points is the right
        // default; the alternative is a dropped frame.
        // ------------------------------------------------------------------
        const copc::BudgetChoice choice = copc::chooseLevelForBudget(
            Octree.nodes, Octree.info,
            box.xmin, box.xmax, box.ymin, box.ymax,
            resolution, static_cast<uint64_t>(PointBudget));

        const int coarsenSteps = choice.wantedLevel - choice.chosenLevel;

        std::cout << "  index says    : level " << choice.wantedLevel
                  << " is what the camera wants\n";

        std::cout << "  budget picks  : level " << choice.chosenLevel << " -> "
                  << choice.cost.nodes << " of " << Octree.nodes.size()
                  << " nodes, " << choice.cost.dataBytes << " bytes, <= "
                  << choice.cost.points << " points\n";

        if (coarsenSteps > 0)
        {
            resolution = choice.resolution;

            std::cout << "  (coarsened " << coarsenSteps << " level(s) to stay "
                      << "under " << PointBudget << "; resolution now "
                      << std::setprecision(5) << resolution << ")\n";
        }

        std::size_t points = 0;
        double seconds = 0.0;

        // Exactly ONE query, at a level we already know fits.
        vtkSmartPointer<vtkPolyData> polyData =
            query(box, resolution, points, seconds);

        std::cout << "\nRESULT\n";
        std::cout << "  points loaded : " << points << "  ("
                  << std::setprecision(3)
                  << (100.0 * double(points) / double(std::max<uint64_t>(TotalPoints, 1)))
                  << " % of the file)\n";
        std::cout << "  load time     : " << seconds << " s\n";

        std::cout << "  queries made  : 1  (the level came from the index)\n";

        // Swap the geometry under the existing actor. No actor churn, no
        // camera reset -- the view stays exactly where the user put it.
        vtkPolyDataMapper *mapper =
            vtkPolyDataMapper::SafeDownCast(CloudActor->GetMapper());

        mapper->SetInputData(polyData);

        updateBox(box);
        updateHud(box, resolution, points, distance);

        LastBox = box;
        LastResolution = resolution;
        HasLoaded = true;

        Renderer->GetRenderWindow()->Render();
    }

private:
    // "Close enough that reloading would not change the picture."
    bool similarTo(const Box2D &box, double resolution) const
    {
        if (LastResolution <= 0.0)
            return false;

        const double ratio = resolution / LastResolution;

        // More than a 25 % change in requested spacing is worth a reload;
        // less is not, since it rarely even changes the octree depth.
        if (ratio < 0.8 || ratio > 1.25)
            return false;

        // Reload if the visible box has drifted by more than 20 % of its own
        // size in any direction.
        const double tolX = 0.2 * LastBox.width();
        const double tolY = 0.2 * LastBox.height();

        return std::abs(box.xmin - LastBox.xmin) < tolX &&
               std::abs(box.xmax - LastBox.xmax) < tolX &&
               std::abs(box.ymin - LastBox.ymin) < tolY &&
               std::abs(box.ymax - LastBox.ymax) < tolY;
    }

    void updateBox(const Box2D &box)
    {
        if (!BoxActor)
            return;

        vtkPolyDataMapper *mapper =
            vtkPolyDataMapper::SafeDownCast(BoxActor->GetMapper());

        vtkCubeSource *cube =
            vtkCubeSource::SafeDownCast(mapper->GetInputAlgorithm());

        if (cube)
            cube->SetBounds(box.xmin, box.xmax, box.ymin, box.ymax, MinZ, MaxZ);
    }

    void updateHud(const Box2D &box,
                   double resolution,
                   std::size_t points,
                   double distance)
    {
        if (!Hud)
            return;

        std::ostringstream os;

        os << std::fixed << std::setprecision(2)
           << "distance   " << distance << "\n"
           << "resolution " << std::setprecision(4) << resolution << "\n"
           << "box        " << std::setprecision(1) << box.width() << " x "
           << box.height() << "\n"
           << "points     " << points << " / " << TotalPoints << "\n"
           << "budget     " << PointBudget << "   error " << TargetPixelError
           << " px";

        Hud->SetInput(os.str().c_str());
    }
};

// ----------------------------------------------------------------------------
// Observers.
//
// EndInteractionEvent fires once, when the user releases the mouse. That is
// the right moment for a synchronous reload: the drag itself stays smooth
// because nothing blocks during it.
// ----------------------------------------------------------------------------
static void onEndInteraction(vtkObject *, unsigned long, void *clientData, void *)
{
    static_cast<CopcStreamer *>(clientData)->update(false);
}

static void onKeyPress(vtkObject *caller, unsigned long, void *clientData, void *)
{
    vtkRenderWindowInteractor *interactor =
        vtkRenderWindowInteractor::SafeDownCast(caller);

    CopcStreamer *streamer = static_cast<CopcStreamer *>(clientData);

    const std::string key = interactor->GetKeySym();

    if (key == "r")
    {
        std::cout << "\n[forced reload]\n";
        streamer->update(true);
    }
    else if (key == "b" && streamer->BoxActor)
    {
        streamer->BoxActor->SetVisibility(!streamer->BoxActor->GetVisibility());
        interactor->GetRenderWindow()->Render();
    }
}

// ============================================================================
// MAIN
// ============================================================================
int main(int argc, char *argv[])
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <file.copc.laz> "
                  << "[--budget N] [--error PIXELS]\n\n"
                  << "  --budget N       max points to hold at once (default 500000)\n"
                  << "  --error PIXELS   target on-screen point spacing (default 2.0)\n";
        return 1;
    }

    CopcStreamer streamer;

    streamer.Filename = argv[1];

    for (int i = 2; i < argc; ++i)
    {
        const std::string arg = argv[i];

        if (arg == "--budget" && i + 1 < argc)
            streamer.PointBudget =
                static_cast<std::size_t>(std::atoll(argv[++i]));
        else if (arg == "--error" && i + 1 < argc)
            streamer.TargetPixelError = std::atof(argv[++i]);
        else
        {
            std::cerr << "unknown argument: " << arg << "\n";
            return 1;
        }
    }

    try
    {
        // --------------------------------------------------------------------
        // Header peek: extent and point count, without reading points.
        // --------------------------------------------------------------------
        {
            pdal::StageFactory factory;

            pdal::Stage *reader = factory.createStage("readers.copc");

            if (!reader)
                throw std::runtime_error("readers.copc unavailable");

            pdal::Options options;
            options.add("filename", streamer.Filename);
            reader->setOptions(options);

            const pdal::QuickInfo qi = reader->preview();

            if (!qi.valid())
                throw std::runtime_error("cannot read header of " +
                                         streamer.Filename);

            streamer.FullExtent = {qi.m_bounds.minx, qi.m_bounds.maxx,
                                   qi.m_bounds.miny, qi.m_bounds.maxy};
            streamer.MinZ = qi.m_bounds.minz;
            streamer.MaxZ = qi.m_bounds.maxz;
            streamer.TotalPoints = qi.m_pointCount;
        }

        // Parse the octree index ourselves. PDAL walks the hierarchy on every
        // execute() but does not hand it to us, and we need it in memory to
        // cost a query before committing to it.
        streamer.Octree = copc::openIndex(streamer.Filename);

        std::cout << std::fixed << std::setprecision(3);
        std::cout << "\nFile          : " << streamer.Filename << "\n";
        std::cout << "Total points  : " << streamer.TotalPoints << "\n";
        std::cout << "Extent        : " << streamer.FullExtent.toPdalBounds()
                  << "\n";
        std::cout << "Octree        : " << streamer.Octree.nodes.size()
                  << " nodes, " << (streamer.Octree.maxLevel() + 1)
                  << " levels, root spacing " << streamer.Octree.info.spacing
                  << "\n";
        std::cout << "Point budget  : " << streamer.PointBudget << "\n";
        std::cout << "Target error  : " << streamer.TargetPixelError << " px\n";
        std::cout << "\nDrag / zoom / pan. Every mouse release re-derives the "
                     "query.\nKeys: r = reload, b = toggle query box, q = quit\n";

        // --------------------------------------------------------------------
        // Scene. The cloud actor starts empty and gets its geometry from the
        // first update().
        // --------------------------------------------------------------------

        vtkNew<vtkLookupTable> lut;
        lut->SetHueRange(0.667, 0.0);
        lut->SetTableRange(streamer.MinZ, streamer.MaxZ);
        lut->Build();

        vtkNew<vtkPolyData> empty;

        vtkNew<vtkPolyDataMapper> cloudMapper;
        cloudMapper->SetInputData(empty);
        cloudMapper->SetLookupTable(lut);
        cloudMapper->SetScalarRange(streamer.MinZ, streamer.MaxZ);
        cloudMapper->ScalarVisibilityOn();

        vtkNew<vtkActor> cloudActor;
        cloudActor->SetMapper(cloudMapper);
        cloudActor->GetProperty()->SetPointSize(2.0);

        vtkNew<vtkCubeSource> boxSource;
        boxSource->SetBounds(streamer.FullExtent.xmin, streamer.FullExtent.xmax,
                             streamer.FullExtent.ymin, streamer.FullExtent.ymax,
                             streamer.MinZ, streamer.MaxZ);

        vtkNew<vtkPolyDataMapper> boxMapper;
        boxMapper->SetInputConnection(boxSource->GetOutputPort());

        vtkNew<vtkActor> boxActor;
        boxActor->SetMapper(boxMapper);
        boxActor->GetProperty()->SetRepresentationToWireframe();
        boxActor->GetProperty()->SetColor(1.0, 0.6, 0.2);
        boxActor->GetProperty()->SetLineWidth(2.0);
        boxActor->PickableOff();

        vtkNew<vtkTextActor> hud;
        hud->GetTextProperty()->SetFontSize(16);
        hud->GetTextProperty()->SetColor(0.9, 0.9, 0.6);
        hud->SetPosition(12, 12);
        hud->SetInput("loading...");

        vtkNew<vtkRenderer> renderer;
        renderer->SetBackground(0.08, 0.10, 0.15);
        renderer->AddActor(cloudActor);
        renderer->AddActor(boxActor);
        renderer->AddActor2D(hud);

        vtkNew<vtkRenderWindow> window;
        window->AddRenderer(renderer);
        window->SetSize(1200, 800);
        window->SetWindowName("COPC camera-driven LOD streaming");

        vtkNew<vtkRenderWindowInteractor> interactor;
        interactor->SetRenderWindow(window);

        vtkNew<vtkInteractorStyleTrackballCamera> style;
        interactor->SetInteractorStyle(style);

        streamer.Renderer = renderer;
        streamer.CloudActor = cloudActor;
        streamer.BoxActor = boxActor;
        streamer.Hud = hud;

        // Frame the whole dataset, so the first query is a coarse overview.
        renderer->ResetCamera(streamer.FullExtent.xmin, streamer.FullExtent.xmax,
                              streamer.FullExtent.ymin, streamer.FullExtent.ymax,
                              streamer.MinZ, streamer.MaxZ);

        window->Render();

        // First load, before any interaction.
        streamer.update(true);

        vtkNew<vtkCallbackCommand> endInteraction;
        endInteraction->SetCallback(onEndInteraction);
        endInteraction->SetClientData(&streamer);

        style->AddObserver(vtkCommand::EndInteractionEvent, endInteraction);

        vtkNew<vtkCallbackCommand> keyPress;
        keyPress->SetCallback(onKeyPress);
        keyPress->SetClientData(&streamer);

        interactor->AddObserver(vtkCommand::KeyPressEvent, keyPress);

        interactor->Initialize();
        interactor->Start();
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }

    return 0;
}
