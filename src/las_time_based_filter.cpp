#include <iostream>
#include <limits>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <vector>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

class SliderCallback : public vtkCommand {
public:
  static SliderCallback *New() { return new SliderCallback; }

  void Execute(vtkObject *caller, unsigned long, void *) override {
    vtkSliderWidget *sliderWidget = reinterpret_cast<vtkSliderWidget *>(caller);
    double sliderValue = static_cast<vtkSliderRepresentation *>(
                             sliderWidget->GetRepresentation())
                             ->GetValue();

    vtkSmartPointer<vtkPoints> filteredPoints =
        vtkSmartPointer<vtkPoints>::New();
    for (size_t i = 0; i < allPoints.size(); ++i) {
      if (allTimes[i] <= sliderValue) {
        filteredPoints->InsertNextPoint(allPoints[i].data());
      }
    }

    std::cout << "Filtered Points Count: "
              << filteredPoints->GetNumberOfPoints() << std::endl;

    vtkSmartPointer<vtkPolyData> filteredPolyData =
        vtkSmartPointer<vtkPolyData>::New();
    filteredPolyData->SetPoints(filteredPoints);

    // Ensure a glyph filter is used for the filtered points
    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(filteredPolyData);
    glyphFilter->Update();

    polyDataMapper->SetInputConnection(glyphFilter->GetOutputPort());
    renderWindow->Render();
  }

  vtkRenderer *renderer;
  vtkRenderWindow *renderWindow;
  vtkPolyDataMapper *polyDataMapper; // Add this
  std::vector<std::array<double, 3>> allPoints;
  std::vector<double> allTimes;

  SliderCallback()
      : renderer(nullptr), renderWindow(nullptr), polyDataMapper(nullptr) {}
};

void CallbackFunction(vtkObject *caller, long unsigned int eventId,
                      void *vtkNotUsed(clientData),
                      void *vtkNotUsed(callData)) {
  vtkRenderer *renderer = static_cast<vtkRenderer *>(caller);

  double timeInSeconds = renderer->GetLastRenderTimeInSeconds();
  double fps = 1.0 / timeInSeconds;
  std::cout << "FPS: " << fps << std::endl;

  std::cout << "Callback" << std::endl;
  std::cout << "eventId: " << eventId << std::endl;
}

int main() {
  // [PDAL loading code remains unchanged]
  // Load point cloud data using PDAL
  pdal::PointTable table;
  pdal::Options options;

  // options.add(
  //     "filename",
  //     "/home/behnam/workspace/vtk_projects/data/las_files/Palac_Moszna.laz");

  options.add("filename", "/home/behnam/map.las");

  pdal::StageFactory factory;
  pdal::Stage *reader = factory.createStage("readers.las");
  reader->setOptions(options);
  reader->prepare(table);
  pdal::PointViewSet pointViews = reader->execute(table);
  pdal::PointViewPtr pointView = *pointViews.begin();
  bool hasGpsTime = pointView->hasDim(pdal::Dimension::Id::GpsTime);

  std::vector<std::array<double, 3>> allPoints;
  std::vector<double> allTimes;
  double minTime = std::numeric_limits<double>::max();
  double maxTime = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    allPoints.push_back({x, y, z});

    if (hasGpsTime) {
      double time =
          pointView->getFieldAs<double>(pdal::Dimension::Id::GpsTime, id);
      allTimes.push_back(time);
      if (time < minTime)
        minTime = time;
      if (time > maxTime)
        maxTime = time;
    }
  }

  // Create a VTK points object from allPoints
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  for (const auto &point : allPoints) {
    points->InsertNextPoint(point.data());
  }

  // Create polydata and set the points
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);

  // Create a glyph filter to visualize the points
  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(polyData);
  glyphFilter->Update();

  // Create a mapper and actor for the initial point cloud
  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(glyphFilter->GetOutputPort());
  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Create a renderer and add the actor to it
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .2, .1);

  // Create a render window and interactor
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Configure the slider widget
  vtkSmartPointer<vtkSliderRepresentation2D> sliderRep =
      vtkSmartPointer<vtkSliderRepresentation2D>::New();
  sliderRep->SetMinimumValue(minTime); // Set to your minimum time value
  sliderRep->SetMaximumValue(maxTime); // Set to your maximum time value

  std::cout << "minTime: " << minTime << std::endl;
  std::cout << "maxTime: " << maxTime << std::endl;

  sliderRep->SetValue(minTime);
  sliderRep->SetTitleText("Time");
  sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint1Coordinate()->SetValue(0.1, 0.1);
  sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint2Coordinate()->SetValue(0.9, 0.1);
  sliderRep->SetSliderLength(0.02);
  sliderRep->SetSliderWidth(0.03);
  sliderRep->SetEndCapLength(0.01);
  sliderRep->SetEndCapWidth(0.03);
  sliderRep->SetTubeWidth(0.005);

  vtkSmartPointer<vtkSliderWidget> sliderWidget =
      vtkSmartPointer<vtkSliderWidget>::New();
  sliderWidget->SetInteractor(renderWindowInteractor);
  sliderWidget->SetRepresentation(sliderRep);
  sliderWidget->SetAnimationModeToAnimate();
  sliderWidget->EnabledOn();

  // Set up the callback and bind it to the slider widget
  vtkSmartPointer<SliderCallback> callback =
      vtkSmartPointer<SliderCallback>::New();
  callback->renderer = renderer;
  callback->renderWindow = renderWindow;
  callback->allPoints = allPoints;
  callback->allTimes = allTimes;
  sliderWidget->AddObserver(vtkCommand::InteractionEvent, callback);

  callback->polyDataMapper = mapper.GetPointer();

  // frame_rate_callback
  vtkNew<vtkCallbackCommand> frame_rate_callback;
  frame_rate_callback->SetCallback(CallbackFunction);
  renderer->AddObserver(vtkCommand::EndEvent, frame_rate_callback);

  // Start the visualization
  renderWindow->Render();
  renderWindowInteractor->Start();
}
