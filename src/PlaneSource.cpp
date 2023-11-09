#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCubeSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkLineSource.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVectorText.h>
int main(int, char *[]) {

  // Create a plane source with grid lines
  vtkNew<vtkPlaneSource> planeSource;
  vtkNew<vtkCubeSource> cubeSource;

  cubeSource->SetXLength(2.0);
  cubeSource->SetYLength(1.5);
  cubeSource->SetZLength(4.0);

  cubeSource->Update();

  // Create a mapper and actor.
  vtkNew<vtkPolyDataMapper> cubeMapper;
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());

  vtkNew<vtkActor> cubeActor;
  cubeActor->SetMapper(cubeMapper);

  vtkNew<vtkTransform> transformCube;
  transformCube->Translate(2.0, 0.0, 0.0);
  cubeActor->SetUserTransform(transformCube);

  double bounds[6];
  // cubeActor->GetMapper()->GetBounds(bounds);

  cubeActor->GetBounds(bounds);

  for (const auto &bound : bounds)
    std::cout << bound << std::endl;

  vtkNew<vtkTransform> transformOrigin;
  vtkNew<vtkTransform> transformPoint1;
  vtkNew<vtkTransform> transformPoint2;

  vtkNew<vtkAxesActor> axesOrigin;
  vtkNew<vtkAxesActor> axesPoint1;
  vtkNew<vtkAxesActor> axesPoint2;

  planeSource->SetOrigin(0, 0, 0);
  transformOrigin->Translate(0.0, 0.0, 0.0);
  axesOrigin->SetUserTransform(transformOrigin);
  // axesOrigin->SetAxisLabels(false);
  axesOrigin->SetXAxisLabelText("O");
  axesOrigin->SetYAxisLabelText("");
  axesOrigin->SetZAxisLabelText("");

  planeSource->SetPoint1(10, 0, 0);
  transformPoint1->Translate(10.0, 0.0, 0.0);
  axesPoint1->SetUserTransform(transformPoint1);
  axesPoint1->SetXAxisLabelText("1");
  axesPoint1->SetYAxisLabelText("");
  axesPoint1->SetZAxisLabelText("");

  planeSource->SetPoint2(0, 0, 10);
  transformPoint2->Translate(0.0, 0.0, 10.0);
  axesPoint2->SetUserTransform(transformPoint2);
  axesPoint2->SetXAxisLabelText("2");
  axesPoint2->SetYAxisLabelText("");
  axesPoint2->SetZAxisLabelText("");

  double centerX = (bounds[0] + bounds[1]) / 2.0;
  double centerY = (bounds[2] + bounds[3]) / 2.0;
  double centerZ = (bounds[4] + bounds[5]) / 2.0;

  std::cout << "centerX: " << centerX << std::endl;

  std::cout << "centerY: " << centerY << std::endl;

  std::cout << "centerZ: " << centerZ << std::endl;

  // This is the with in X direction
  double length = bounds[1] - bounds[0];

  // This is the with in Y direction
  double width = bounds[3] - bounds[2];

  // This is the depth in Z direction
  double height = bounds[5] - bounds[4];

  double maxDimension = std::max(length, height);

  int grid_multiplier = 4;

  int squareSize = 1;

  int planeSourceXResolution = 10;

  planeSource->SetXResolution(
      planeSourceXResolution); // Set the number of grid lines in the X
                               // direction
  planeSource->SetYResolution(
      planeSourceXResolution); // Set the number of grid lines in the Y
                               // direction

  double gridHalfSize = squareSize * planeSourceXResolution / 2;

  planeSource->SetOrigin(centerX - gridHalfSize, 0, centerZ - gridHalfSize);
  planeSource->SetPoint1(centerX + gridHalfSize, 0, centerZ - gridHalfSize);
  planeSource->SetPoint2(centerX - gridHalfSize, 0, centerZ + gridHalfSize);

  // Map the plane's data to graphics primitives
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());

  // Create an actor for the plane and set its properties to display the grid
  // lines
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()
      ->SetRepresentationToWireframe();               // Display as wireframe
  planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to gray

  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);
  planeActor->PickableOff();

  // Set the grid size based on the unit of measurement
  const double gridSizeInMeters = 1.0;
  double gridSizeInCurrentUnit;

  // Default to meters
  gridSizeInCurrentUnit = gridSizeInMeters;
  // If you want to set the grid size to feet or inches, uncomment the desired
  // conversion
  // gridSizeInCurrentUnit = gridSizeInMeters * 3.28084; // For feet
  // gridSizeInCurrentUnit = gridSizeInMeters * 39.3701; // For inches

  planeSource->SetXResolution(10 * gridSizeInCurrentUnit);
  planeSource->SetYResolution(10 * gridSizeInCurrentUnit);

  // Create a renderer and a render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the plane actor to the renderer
  renderer->AddActor(planeActor);
  renderer->SetBackground(0.1, 0.1, 0.1); // Set background color to dark gray

  renderer->AddActor(axesOrigin);

  renderer->AddActor(axesPoint1);

  renderer->AddActor(axesPoint2);

  renderer->AddActor(cubeActor);

  renderer->UseFXAAOn();

  renderWindow->SetMultiSamples(4); // add this to your code.

  // Start the rendering loop
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
