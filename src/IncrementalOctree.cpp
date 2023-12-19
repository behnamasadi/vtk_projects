#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkIncrementalOctreePointLocator.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOctreePointLocator.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>

#include <cmath>

namespace {
class vtkSliderCallback : public vtkCommand {
public:
  static vtkSliderCallback *New()

  {
    return new vtkSliderCallback;
  }
  vtkSliderCallback() : Octree(0), Level(0), PolyData(0), Renderer(0) {}

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    vtkSliderWidget *sliderWidget = reinterpret_cast<vtkSliderWidget *>(caller);
    this->Level = vtkMath::Round(static_cast<vtkSliderRepresentation *>(
                                     sliderWidget->GetRepresentation())
                                     ->GetValue());

    this->Octree->GenerateRepresentation(this->Level, this->PolyData);
    this->Renderer->Render();

    std::cout << "Number Of Cells representation  "
              << this->PolyData->GetNumberOfCells() << std::endl;

    // vtkSmartPointer<vtkPoints> points = this->PolyData->GetPoints();
    // if (points) {

    //   std::cout << "Number Of Points in Octree representation  "
    //             << points->GetNumberOfPoints() << std::endl;

    //     for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    //       double p[3];
    //       points->GetPoint(i, p);
    //       // Do something with the point coordinates p[0], p[1], p[2]
    //       std::cout << "Point Id: " << i << ": Point: (" << p[0] << ", " <<
    //       p[1]
    //                 << ", " << p[2] << ")" << std::endl;
    //     }
    // }
  }

  //   vtkOctreePointLocator *Octree;
  vtkIncrementalOctreePointLocator *Octree;
  int Level;
  vtkPolyData *PolyData;
  vtkRenderer *Renderer;
};
} // namespace

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // Create a point cloud

  vtkNew<vtkPoints> points;

  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);

  double cylanderRadius = 2.0;
  for (int i = 0; i < 1000000; i++) {
    double x, y, z, radius;
    x = randomSequence->GetRangeValue(4.0, 12.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-2.0, 2.0);
    randomSequence->Next();
    z = pow(cylanderRadius * cylanderRadius - y * y, 0.5);

    points->InsertNextPoint(x, y, -z);
  }

  std::cout << "There are " << points->GetNumberOfPoints() << " points."
            << std::endl;

  vtkNew<vtkPolyData> inputPolydata;
  inputPolydata->SetPoints(points);

  // vtkNew<vtkSphereSource> pointSource;
  // pointSource->SetPhiResolution(50);
  // pointSource->SetThetaResolution(50);
  vtkNew<vtkPolyDataMapper> pointsMapper;
  // pointsMapper->SetInputConnection(polydata);

  pointsMapper->SetInputData(inputPolydata);

  // pointSource->Update();
  vtkNew<vtkActor> pointsActor;
  pointsActor->SetMapper(pointsMapper);
  pointsActor->GetProperty()->SetInterpolationToFlat();
  pointsActor->GetProperty()->SetRepresentationToPoints();
  pointsActor->GetProperty()->SetColor(colors->GetColor4d("Yellow").GetData());

  // Create the tree
  // vtkNew<vtkOctreePointLocator> octree;
  vtkNew<vtkIncrementalOctreePointLocator> octree;
  //   octree->SetMaximumPointsPerRegion(50);
  // octree->SetDataSet(pointSource->GetOutput());
  octree->SetDataSet(inputPolydata);
  octree->BuildLocator();

  // octree->InitPointInsertion();

  cylanderRadius = 4.0;
  for (int i = 0; i < 1000; i++) {
    double x, y, z;
    x = randomSequence->GetRangeValue(12.0, 18.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(2.0, 4.0);
    randomSequence->Next();
    z = pow(cylanderRadius * cylanderRadius - y * y, 0.5);
    double newPoint[3] = {2 * x + 5, 2 * y + 5, z};
    // std::cout << newPoint[0] << "," << newPoint[1] << "," << newPoint[2]
    //           << "\n";

    std::cout << "point index: " << octree->InsertNextPoint(newPoint) << "\n";

    // vtkIdType pntId = -1;
    // octree->GetLeafContainer(octree->GetRoot(), newPoint);
    // ->InsertPoint(this->LocatorPoints, x, this->MaxPointsPerLeaf, &pntId, 2,
    // this->NumberOfNodes);

    // vtkIdType pntId;
    // int insert = 0;
    // octree->InsertPointWithoutChecking(newPoint, pntId, insert);
    // std::cout << i << "\n";
  }
  octree->BuildLocator();

  vtkNew<vtkPolyData> polydata;
  octree->GenerateRepresentation(0, polydata);

  vtkNew<vtkPolyDataMapper> octreeMapper;
  octreeMapper->SetInputData(polydata);

  vtkNew<vtkActor> octreeActor;
  octreeActor->SetMapper(octreeMapper);
  octreeActor->GetProperty()->SetInterpolationToFlat();
  octreeActor->GetProperty()->SetRepresentationToWireframe();
  octreeActor->GetProperty()->SetColor(
      colors->GetColor4d("SpringGreen").GetData());

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(pointsActor);
  renderer->AddActor(octreeActor);
  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

  // Render an image (lights and cameras are created automatically)
  renderWindow->SetWindowName("OctreeVisualize");
  renderWindow->SetSize(600, 600);
  renderWindow->Render();

  vtkNew<vtkSliderRepresentation2D> sliderRep;
  sliderRep->SetMinimumValue(0);
  sliderRep->SetMaximumValue(octree->GetLevel());
  sliderRep->SetValue(0);
  sliderRep->SetTitleText("Level");
  sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint1Coordinate()->SetValue(.2, .2);
  sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  sliderRep->GetPoint2Coordinate()->SetValue(.8, .2);
  sliderRep->SetSliderLength(0.075);
  sliderRep->SetSliderWidth(0.05);
  sliderRep->SetEndCapLength(0.05);
  sliderRep->GetTitleProperty()->SetColor(
      colors->GetColor3d("Beige").GetData());
  sliderRep->GetCapProperty()->SetColor(
      colors->GetColor3d("MistyRose").GetData());
  sliderRep->GetSliderProperty()->SetColor(
      colors->GetColor3d("LightBlue").GetData());
  sliderRep->GetSelectedProperty()->SetColor(
      colors->GetColor3d("Violet").GetData());

  vtkNew<vtkSliderWidget> sliderWidget;
  sliderWidget->SetInteractor(renderWindowInteractor);
  sliderWidget->SetRepresentation(sliderRep);
  sliderWidget->SetAnimationModeToAnimate();
  sliderWidget->EnabledOn();

  vtkNew<vtkSliderCallback> callback;
  callback->Octree = octree;
  callback->PolyData = polydata;
  callback->Renderer = renderer;

  sliderWidget->AddObserver(vtkCommand::InteractionEvent, callback);

  renderWindowInteractor->Initialize();
  renderWindow->Render();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}