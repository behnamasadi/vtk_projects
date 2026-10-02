#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>

// ---------------------------------------------------------------------------
// create_copc_advanced_options
//
// What PDAL's writers.copc lets you control, and what it keeps for itself.
//
// You control (shown below):
//   - source density before writing (here a 2 m input grid over 1000 m),
//   - output precision (scale/offset), CRS (a_srs), file metadata,
//   - reproducibility (fixed_seed) and writer threads.
//
// PDAL keeps (no writer option exists):
//   - root spacing / LOD ladder  (always rootCubeSize / 147),
//   - thinning rule, node chunking, hierarchy layout.
//
// Run:
//   ./create_copc_advanced_options [output.copc.laz]
//   ./copc_hierarchy_inspect advanced-options.copc.laz
// ---------------------------------------------------------------------------

namespace
{

constexpr int kExtent = 1000;
constexpr int kInputStep = 2; // Pre-COPC density control: 2 m input grid.
constexpr const char *kDefaultOutput = "advanced-options.copc.laz";

pdal::PointViewPtr makeInputView(pdal::PointTable &table)
{
    table.layout()->registerDim(pdal::Dimension::Id::X);
    table.layout()->registerDim(pdal::Dimension::Id::Y);
    table.layout()->registerDim(pdal::Dimension::Id::Z);

    pdal::PointViewPtr view = std::make_shared<pdal::PointView>(table);
    for (int y = 0; y < kExtent; y += kInputStep)
    {
        for (int x = 0; x < kExtent; x += kInputStep)
        {
            const pdal::PointId id = view->size();
            view->setField(pdal::Dimension::Id::X, id, static_cast<double>(x));
            view->setField(pdal::Dimension::Id::Y, id, static_cast<double>(y));
            view->setField(pdal::Dimension::Id::Z, id,
                25.0 * std::sin(0.02 * x) * std::cos(0.02 * y));
        }
    }
    return view;
}

void writeCopc(pdal::PointTable &table, const pdal::PointViewPtr &view,
    const std::string &filename)
{
    pdal::StageFactory factory;
    pdal::Stage *writer = factory.createStage("writers.copc");
    if (!writer)
        throw std::runtime_error("writers.copc is not available");

    pdal::Options options;
    options.add("filename", filename);

    // LAS coordinate representation: precision and integer encoding, not LOD.
    options.add("scale_x", 0.001);
    options.add("scale_y", 0.001);
    options.add("scale_z", 0.001);
    options.add("offset_x", "auto");
    options.add("offset_y", "auto");
    options.add("offset_z", "auto");

    // File metadata. EPSG:2056 is illustrative; use your source CRS.
    options.add("a_srs", "EPSG:2056");
    options.add("software_id", "vtk_projects COPC options example");
    options.add("pdal_metadata", true);
    options.add("pipeline", true);

    // fixed_seed makes PDAL's automatic sampling repeatable; it does NOT
    // prescribe sampling spacing, node size, or the hierarchy layout.
    options.add("fixed_seed", true);
    options.add("threads", 4);
    writer->setOptions(options);

    pdal::BufferReader source;
    source.addView(view);
    writer->setInput(source);
    writer->prepare(table);
    writer->execute(table);
}

std::size_t readBackCount(const std::string &filename)
{
    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createStage("readers.copc");
    if (!reader)
        throw std::runtime_error("readers.copc is not available");

    pdal::Options options;
    options.add("filename", filename);
    reader->setOptions(options);

    pdal::PointTable table;
    reader->prepare(table);
    const pdal::PointViewSet views = reader->execute(table);

    std::size_t total = 0;
    for (const auto &view : views)
        total += view->size();
    return total;
}

} // namespace

int main(int argc, char *argv[])
{
    try
    {
        const std::string filename =
            (argc > 1) ? argv[1] : kDefaultOutput;

        pdal::PointTable table;
        const pdal::PointViewPtr view = makeInputView(table);
        const std::size_t inputSize = view->size();

        writeCopc(table, view, filename);

        // Verify the round trip: a full-detail COPC read must return
        // every input point, whatever pyramid PDAL chose internally.
        const std::size_t readSize = readBackCount(filename);
        if (readSize != inputSize)
        {
            std::cerr << "ERROR: wrote " << inputSize << " points but read back "
                      << readSize << " from " << filename << "\n";
            return 1;
        }

        std::cout << "Wrote " << filename << " from " << inputSize
                  << " input points (" << kInputStep << " m input spacing).\n"
                  << "Read back " << readSize << " points (full detail).\n"
                  << "Inspect its PDAL-selected hierarchy with:\n"
                  << "  ./copc_hierarchy_inspect " << filename << "\n";
        return 0;
    }
    catch (const std::exception &e)
    {
        std::cerr << "ERROR: " << e.what() << "\n";
        return 1;
    }
}
