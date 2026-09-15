#include <pdal/StageFactory.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>

#include <algorithm>
#include <cmath>
#include <fstream>
#include <iostream>
#include <string>

// ============================================================
// STEP 1
// Generate our own known point cloud.
//
// Extent:
//      X = 0 ... 999
//      Y = 0 ... 999
//
// Density:
//      one point every 1 meter in X/Y
//
// Total:
//      1000 * 1000 = 1,000,000 points
// ============================================================

void createSyntheticCloud()
{
    std::ofstream file("synthetic.csv");

    file << "X,Y,Z\n";

    for (int y = 0; y < 1000; ++y)
    {
        for (int x = 0; x < 1000; ++x)
        {
            // Just make Z interesting.
            double z =
                20.0 *
                std::sin(x * 0.02) *
                std::cos(y * 0.02);

            file
                << x << ","
                << y << ","
                << z << "\n";
        }
    }

    std::cout
        << "Created synthetic.csv\n"
        << "Points : 1,000,000\n"
        << "X      : 0 ... 999\n"
        << "Y      : 0 ... 999\n"
        << "Spacing: 1 m input grid\n\n";
}

// ============================================================
// STEP 2
// Convert our CSV into COPC.
//
// readers.text
//      ↓
// writers.copc
//      ↓
// synthetic.copc.laz
//
// The COPC writer constructs the clustered octree.
// ============================================================

void createCopc()
{
    pdal::StageFactory factory;

    // ---------- Reader ----------

    pdal::Stage *reader =
        factory.createStage("readers.text");

    if (!reader)
    {
        throw std::runtime_error(
            "readers.text unavailable");
    }

    pdal::Options readerOptions;

    readerOptions.add(
        "filename",
        "synthetic.csv");

    readerOptions.add(
        "separator",
        ",");

    reader->setOptions(readerOptions);

    // ---------- COPC writer ----------

    pdal::Stage *writer =
        factory.createStage("writers.copc");

    if (!writer)
    {
        throw std::runtime_error(
            "writers.copc unavailable");
    }

    pdal::Options writerOptions;

    writerOptions.add(
        "filename",
        "synthetic.copc.laz");

    writer->setOptions(writerOptions);

    // Pipeline:
    //
    // readers.text → writers.copc

    writer->setInput(*reader);

    // Execute pipeline.

    pdal::PointTable table;

    writer->prepare(table);
    writer->execute(table);

    std::cout
        << "Created synthetic.copc.laz\n\n";
}

// ============================================================
// STEP 3
// Read a region from COPC.
//
// Same bounds every time.
//
// Only "resolution" changes.
// ============================================================

std::size_t queryCopc(double resolution)
{
    pdal::StageFactory factory;

    pdal::Stage *reader =
        factory.createStage("readers.copc");

    if (!reader)
    {
        throw std::runtime_error(
            "readers.copc unavailable");
    }

    pdal::Options options;

    options.add(
        "filename",
        "synthetic.copc.laz");

    // WHERE?
    //
    // X = 100 ... 200
    // Y = 300 ... 400

    options.add(
        "bounds",
        "([100,200],[300,400])");

    // HOW DETAILED?

    options.add(
        "resolution",
        resolution);

    reader->setOptions(options);

    pdal::PointTable table;

    reader->prepare(table);

    pdal::PointViewSet views =
        reader->execute(table);

    std::size_t total = 0;

    for (const auto &view : views)
        total += view->size();

    return total;
}

// ============================================================
// STEP 4
// Read the same bounds with NO resolution limit.
//
// This means:
//      retrieve full available detail.
// ============================================================

std::size_t queryFullResolution()
{
    pdal::StageFactory factory;

    pdal::Stage *reader =
        factory.createStage("readers.copc");

    if (!reader)
    {
        throw std::runtime_error(
            "readers.copc unavailable");
    }

    pdal::Options options;

    options.add(
        "filename",
        "synthetic.copc.laz");

    options.add(
        "bounds",
        "([100,200],[300,400])");

    // Notice:
    //
    // NO resolution option.

    reader->setOptions(options);

    pdal::PointTable table;

    reader->prepare(table);

    pdal::PointViewSet views =
        reader->execute(table);

    std::size_t total = 0;

    for (const auto &view : views)
        total += view->size();

    return total;
}

// ============================================================
// MAIN
// ============================================================

int main()
{
    try
    {
        // Create source data.

        createSyntheticCloud();

        // Convert it to COPC.

        createCopc();

        // --------------------------------------------
        // Same spatial region.
        // Different LOD requests.
        // --------------------------------------------

        std::cout
            << "Query bounds:\n"
            << "X = [100, 200]\n"
            << "Y = [300, 400]\n\n";

        std::size_t coarse =
            queryCopc(20.0);

        std::size_t medium =
            queryCopc(10.0);

        std::size_t fine =
            queryCopc(5.0);

        std::size_t finer =
            queryCopc(2.0);

        std::size_t full =
            queryFullResolution();

        // --------------------------------------------
        // Results
        // --------------------------------------------

        std::cout
            << "Resolution 20 m : "
            << coarse << " points\n";

        std::cout
            << "Resolution 10 m : "
            << medium << " points\n";

        std::cout
            << "Resolution  5 m : "
            << fine << " points\n";

        std::cout
            << "Resolution  2 m : "
            << finer << " points\n";

        std::cout
            << "Full resolution : "
            << full << " points\n";
    }
    catch (const std::exception &e)
    {
        std::cerr
            << "ERROR: "
            << e.what()
            << '\n';

        return 1;
    }

    return 0;
}