#include <limits>
#include <pdal/Dimension.hpp>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/BufferReader.hpp>
#include <pdal/io/LasWriter.hpp>
#include <vector>

int main() {

  using namespace pdal;

  Options options;
  options.add("filename", "myfile.las");

  pdal::PointTable table;

  // Define the layout of the point cloud and add dimensions
  table.layout()->registerDim(pdal::Dimension::Id::X);
  table.layout()->registerDim(pdal::Dimension::Id::Y);
  table.layout()->registerDim(pdal::Dimension::Id::Z);
  table.layout()->registerDim(pdal::Dimension::Id::Intensity);

  // Create a PointView with the defined schema
  pdal::PointViewPtr view(new pdal::PointView(table));

  // Populate the point cloud with points
  for (int i = 0; i < 100; ++i) // Generate 100 points for example
  {
    const double x = i;
    const double y = i * 2;
    const double z = i / 2.0;
    const uint16_t intensity =
        static_cast<uint16_t>(i * 10); // Example intensity calculation

    // Add a point to the point cloud
    view->setField(pdal::Dimension::Id::X, i, x);
    view->setField(pdal::Dimension::Id::Y, i, y);
    view->setField(pdal::Dimension::Id::Z, i, z);
    view->setField(pdal::Dimension::Id::Intensity, i, intensity);

    // view->setField(pdal::Dimension::Id::Intensity, i,
    //                std::numeric_limits<uint16_t>::max() - 1);
    // std::cout << std::numeric_limits<uint16_t>::max() - 1 << std::endl;
  }

  // Modify the intensity of the points, increase each point's intensity by 50
  for (pdal::PointId id = 0; id < view->size(); ++id) {
    uint16_t intensity =
        view->getFieldAs<uint16_t>(pdal::Dimension::Id::Intensity, id);

    std::cout << intensity << std::endl;

    intensity += 50; // Increase intensity
    view->setField(pdal::Dimension::Id::Intensity, id, intensity);
  }

  BufferReader reader;
  reader.addView(view);

  StageFactory factory;

  // StageFactory always "owns" stages it creates. They'll be destroyed with the
  // factory.
  Stage *writer = factory.createStage("writers.las");

  writer->setInput(reader);
  writer->setOptions(options);
  writer->prepare(table);
  writer->execute(table);

  return 0;
}
