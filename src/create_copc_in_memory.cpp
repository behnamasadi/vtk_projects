#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/Options.hpp>

#include <iostream>
#include <memory>

// ------------------------------------------------------------
// 1. CREATE 1000 x 1000 POINTS AND WRITE THEM AS COPC
// ------------------------------------------------------------
void createCopc()
{
    pdal::PointTable table;

    // Tell PDAL which dimensions our points have.
    table.layout()->registerDim(pdal::Dimension::Id::X);
    table.layout()->registerDim(pdal::Dimension::Id::Y);
    table.layout()->registerDim(pdal::Dimension::Id::Z);

    // Create a PointView containing our points.
    pdal::PointViewPtr view =
        std::make_shared<pdal::PointView>(table);

    // Create:
    //
    // (0,0)   (1,0)   ... (999,0)
    // (0,1)   (1,1)   ... (999,1)
    // ...
    // (0,999)             (999,999)
    //
    for (int y = 0; y < 1000; ++y)
    {
        for (int x = 0; x < 1000; ++x)
        {
            pdal::PointId id = view->size();

            view->setField(
                pdal::Dimension::Id::X,
                id,
                static_cast<double>(x));

            view->setField(
                pdal::Dimension::Id::Y,
                id,
                static_cast<double>(y));

            view->setField(
                pdal::Dimension::Id::Z,
                id,
                0.0);
        }
    }

    std::cout
        << "Created points: "
        << view->size()
        << "\n";

    // Write as COPC.
    pdal::StageFactory factory;

    pdal::Stage *writer =
        factory.createStage("writers.copc");

    if (!writer)
    {
        std::cerr << "writers.copc is not available\n";
        return;
    }

    pdal::Options options;
    options.add("filename", "test.copc.laz");

    writer->setOptions(options);

    // A writer consumes a pipeline Stage, so expose the generated PointView
    // through an in-memory BufferReader source stage.
    pdal::BufferReader buffer;
    buffer.addView(view);
    writer->setInput(buffer);

    writer->prepare(table);
    writer->execute(table);

    std::cout << "Written: test.copc.laz\n";
}

// ------------------------------------------------------------
// 2. READ ONLY A SMALL REGION FROM THE COPC
// ------------------------------------------------------------
void readRegion()
{
    pdal::StageFactory factory;

    pdal::Stage *reader =
        factory.createStage("readers.copc");

    if (!reader)
    {
        std::cerr << "readers.copc is not available\n";
        return;
    }

    pdal::Options options;

    options.add(
        "filename",
        "test.copc.laz");

    options.add(
        "bounds",
        "([100,200],[300,400])");

    reader->setOptions(options);

    pdal::PointTable table;

    reader->prepare(table);

    pdal::PointViewSet views =
        reader->execute(table);

    for (const auto &view : views)
    {
        std::cout
            << "Points inside region: "
            << view->size()
            << "\n";

        // Print only first 20 points.
        pdal::PointId count =
            std::min<pdal::PointId>(
                20,
                view->size());

        for (pdal::PointId i = 0; i < count; ++i)
        {
            double x =
                view->getFieldAs<double>(
                    pdal::Dimension::Id::X,
                    i);

            double y =
                view->getFieldAs<double>(
                    pdal::Dimension::Id::Y,
                    i);

            double z =
                view->getFieldAs<double>(
                    pdal::Dimension::Id::Z,
                    i);

            std::cout
                << x << ", "
                << y << ", "
                << z << "\n";
        }
    }
}

// ------------------------------------------------------------
// MAIN
// ------------------------------------------------------------
int main()
{
    createCopc();

    std::cout
        << "\n-------------------------\n\n";

    readRegion();

    return 0;
}