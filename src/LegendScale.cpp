// #include <vtkActor.h>
// #include <vtkAxisActor2D.h>
// #include <vtkCubeSource.h>
// #include <vtkLegendScaleActor.h>
// #include <vtkNew.h>
// #include <vtkPlaneSource.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkTextProperty.h>

// int main(int, char *[]) {
//   vtkNew<vtkPlaneSource> planeSource;
//   planeSource->SetXResolution(
//       10); // Set the number of grid lines in the X direction
//   planeSource->SetYResolution(
//       10); // Set the number of grid lines in the Y direction

//   // Map the plane's data to graphics primitives
//   vtkNew<vtkPolyDataMapper> planeMapper;
//   planeMapper->SetInputConnection(planeSource->GetOutputPort());

//   // Create an actor for the plane and set its properties to display the grid
//   // lines
//   vtkNew<vtkActor> planeActor;
//   planeActor->SetMapper(planeMapper);
//   planeActor->GetProperty()
//       ->SetRepresentationToWireframe();               // Display as wireframe
//   planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to
//   gray

//   // Create a renderer and a render window
//   vtkNew<vtkRenderer> renderer;
//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add the plane actor to the renderer
//   renderer->AddActor(planeActor);
//   renderer->SetBackground(0.1, 0.1, 0.1);

//   // Create a cube actor for demonstration
//   vtkNew<vtkCubeSource> cubeSource;

//   double xMin, xMax, yMin, yMax, zMin, zMax;

//   xMin = -1;
//   xMax = 1;
//   yMin = -5;
//   yMax = 5;
//   zMin = -3;
//   zMax = 3;

//   cubeSource->SetBounds(xMin, xMax, yMin, yMax, zMin, zMax);

//   vtkNew<vtkPolyDataMapper> cubeMapper;
//   cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
//   vtkNew<vtkActor> cubeActor;
//   cubeActor->SetMapper(cubeMapper);
//   renderer->AddActor(cubeActor);

//   // Create a LegendScaleActor
//   vtkNew<vtkLegendScaleActor> legendScaleActor;

//   // Get the bounds of the cube actor
//   double bounds[6];
//   cubeActor->GetBounds(bounds);

//   // Compute the range in the x, y, and z dimensions
//   double xRange = bounds[1] - bounds[0];
//   double yRange = bounds[3] - bounds[2];
//   double zRange = bounds[5] - bounds[4];

//   std::cout << "xRange: " << xRange << std::endl;

//   // Adjust the properties of the LegendScaleActor based on the computed
//   range
//   // For this example, we'll set the length of the bottom axis based on the
//   // xRange
//   // legendScaleActor->SetBottomAxisLength(xRange);

//   legendScaleActor->GetBottomAxis()->SetRange(0, 5 * xRange);
//   legendScaleActor->GetBottomAxis()->SetTitle(
//       "---------------------------------- X Axis "
//       "----------------------------------");
//   legendScaleActor->GetBottomAxis()->SetNumberOfMinorTicks(10);
//   legendScaleActor->GetBottomAxis()->SetAdjustLabels(1);
//   legendScaleActor->GetLegendTitleProperty()->SetLineOffset(-25);
//   legendScaleActor->GetLegendLabelProperty()->SetLineOffset(-25);
//   legendScaleActor->GetLegendLabelProperty()->SetColor(0, 1, 0);
//   legendScaleActor->GetLegendTitleProperty()->SetColor(1, 0, 0);

//   legendScaleActor->GetLegendLabelProperty()->SetFontSize(
//       legendScaleActor->GetLegendLabelProperty()->GetFontSize() * 2);
//   legendScaleActor->GetLegendTitleProperty()->SetFontSize(
//       legendScaleActor->GetLegendTitleProperty()->GetFontSize() * 2);

//   legendScaleActor->SetBottomAxisVisibility(1); // Display the bottom    axis
//   // GetBottomAxis

//   // Configure the LegendScaleActor
//   //   legendScaleActor->SetBottomAxisVisibility(0);
//   legendScaleActor->SetTopAxisVisibility(0);
//   legendScaleActor->SetRightAxisVisibility(0);
//   legendScaleActor->SetLeftAxisVisibility(0);
//   //   legendScaleActor->SetLegendVisibility(1);     // Display the numerical
//   //   scale legendScaleActor->SetRightAxisVisibility(1);  // Display the
//   right
//   //   axis legendScaleActor->SetTopAxisVisibility(1);    // Display the top
//   //   axis legendScaleActor->SetLeftAxisVisibility(1);   // Display the left
//   //   axis

//   // Add the LegendScaleActor to the renderer
//   renderer->AddActor(legendScaleActor);

//   // Adjust the camera to fit the cube actor in the view
//   renderer->ResetCamera(cubeActor->GetBounds());

//   // ... [Rest of the rendering code]
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <vtkActor.h>
#include <vtkAxisActor2D.h>
#include <vtkCubeSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

// int main(int, char *[]) {
//   // Create a cube
//   vtkSmartPointer<vtkCubeSource> cube =
//   vtkSmartPointer<vtkCubeSource>::New(); cube->SetXLength(1.0);
//   cube->SetYLength(1.0);
//   cube->SetZLength(1.0);

//   // Map cube data to graphics primitives
//   vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
//       vtkSmartPointer<vtkPolyDataMapper>::New();
//   cubeMapper->SetInputConnection(cube->GetOutputPort());

//   // Create an actor for the cube
//   vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
//   cubeActor->SetMapper(cubeMapper);

//   // Create the legend scale actor
//   vtkSmartPointer<vtkLegendScaleActor> legend =
//       vtkSmartPointer<vtkLegendScaleActor>::New();
//   // legend->AllOn();

//   //   legend->GetBottomAxis()->SetMajorTickSize(1.0);  // Sets the major
//   tick
//   //   size legend->GetBottomAxis()->SetMinorTickSize(0.1);  // Sets the
//   minor
//   //   tick size legend->GetBottomAxis()->SetTickLength(10);      // Length
//   of
//   //   the ticks legend->GetBottomAxis()->NumberOfMinorTicksOn(); // Display
//   //   minor ticks legend->GetBottomAxis()->SetNumberOfMinorTicks(
//   //       9); // 9 minor ticks between major ticks to get 0.1, 0.2, ... ,
//   0.9

//   // legend->GetBottomAxis()->SetLabelFactor(0.2);
//   legend->SetLegendVisibility(1);

//   legend->GetBottomAxis()->GetMinorTickLength();
//   legend->GetBottomAxis()->GetNumberOfMinorTicks();
//   legend->GetBottomAxis()->GetNumberOfMinorTicksMaxValue();
//   legend->GetBottomAxis()->GetNumberOfMinorTicksMaxValue();

//   // Create a renderer and render window
//   vtkSmartPointer<vtkRenderer> renderer =
//   vtkSmartPointer<vtkRenderer>::New(); vtkSmartPointer<vtkRenderWindow>
//   renderWindow =
//       vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);

//   // Create a render window interactor
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//       vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add the actors to the scene
//   renderer->AddActor(cubeActor);
//   renderer->AddActor(legend);
//   renderer->SetBackground(.1, .2, .3); // Background color

//   // Reset the camera to show the full scene
//   renderer->ResetCamera();

//   // Start the rendering loop
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return 0;
// }

// #include <vtkLegendScaleActor.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>

// int main(int, char *[]) {

//   // Create a cube
//   vtkSmartPointer<vtkCubeSource> cube =
//   vtkSmartPointer<vtkCubeSource>::New(); cube->SetXLength(1.0);
//   cube->SetYLength(1.0);
//   cube->SetZLength(1.0);

//   // Map cube data to graphics primitives
//   vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
//       vtkSmartPointer<vtkPolyDataMapper>::New();
//   cubeMapper->SetInputConnection(cube->GetOutputPort());

//   // Create an actor for the cube
//   vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
//   cubeActor->SetMapper(cubeMapper);

//   // Create the legend scale actor as a bar scale
//   vtkSmartPointer<vtkLegendScaleActor> barScale =
//       vtkSmartPointer<vtkLegendScaleActor>::New();

//   // Use only the bottom axis
//   barScale->BottomAxisVisibilityOn();
//   barScale->TopAxisVisibilityOff();
//   barScale->LeftAxisVisibilityOff();
//   barScale->RightAxisVisibilityOff();

//   // Adjust properties for the bottom axis (e.g., tick marks, labels, etc.)
//   barScale->GetBottomAxis()->SetNumberOfMinorTicks(0); // No minor ticks
//   //   barScale->GetBottomAxis()->SetScaleFactor(1.0); // Major ticks every 1
//   //   unit
//   barScale->GetBottomAxis()->LabelVisibilityOn(); // Show labels

//   // Create a renderer and render window
//   vtkSmartPointer<vtkRenderer> renderer =
//   vtkSmartPointer<vtkRenderer>::New(); vtkSmartPointer<vtkRenderWindow>
//   renderWindow =
//       vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);

//   // Create a render window interactor
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//       vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add the bar scale to the scene
//   renderer->AddActor(barScale);
//   renderer->AddActor(cubeActor);

//   renderer->SetBackground(.1, .2, .3); // Background color

//   // Reset the camera to show the full scene
//   renderer->ResetCamera();

//   // Start the rendering loop
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return 0;
// }

#include <algorithm>
#include <sstream>
#include <vector>
#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkLegendBoxActor.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

double FindClosest(const std::vector<double> &values, double target) {
  double closest = values[0];
  double minDiff = std::abs(closest - target);
  for (double val : values) {
    double diff = std::abs(val - target);
    if (diff < minDiff) {
      closest = val;
      minDiff = diff;
    }
  }
  return closest;
}

int main(int, char *[]) {
  // Create a sample actor (a cube in this case)
  vtkSmartPointer<vtkCubeSource> cube = vtkSmartPointer<vtkCubeSource>::New();

  cube->SetXLength(5300.3);
  cube->SetYLength(237.26);
  cube->SetZLength(125.23);

  vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cube->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);

  // Compute the bounding box of the actor
  double bounds[6];
  cubeActor->GetBounds(bounds);
  double width = bounds[1] - bounds[0];
  double height = bounds[3] - bounds[2];
  double maxSize = std::max(width, height);

  // Define the desired sizes
  std::vector<double> desiredSizes = {1, 2, 5, 10, 20, 50, 100, 200, 500, 1000};
  double idealSize = maxSize * 2 / 20;
  double squareSize = FindClosest(desiredSizes, idealSize);

  // The grid is always 20x20
  int gridDimension = 20;
  double gridHalfSize = squareSize * gridDimension / 2;

  // Calculate the center of the actor's bounding box
  double centerX = (bounds[0] + bounds[1]) / 2;
  double centerY = (bounds[2] + bounds[3]) / 2;

  // Create a square grid with the actor at the center
  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  plane->SetOrigin(centerX - gridHalfSize, centerY - gridHalfSize, 0);
  plane->SetPoint1(centerX + gridHalfSize, centerY - gridHalfSize, 0);
  plane->SetPoint2(centerX - gridHalfSize, centerY + gridHalfSize, 0);
  plane->SetXResolution(gridDimension);
  plane->SetYResolution(gridDimension);

  vtkSmartPointer<vtkPolyDataMapper> planeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  planeMapper->SetInputConnection(plane->GetOutputPort());
  vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New();
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();

  // Create a scale bar using vtkLegendBoxActor
  vtkSmartPointer<vtkLegendBoxActor> legend =
      vtkSmartPointer<vtkLegendBoxActor>::New();
  legend->SetNumberOfEntries(1);
  std::ostringstream oss;
  oss << squareSize << " unit";
  legend->SetEntryString(0, oss.str().c_str());
  legend->GetPositionCoordinate()->SetCoordinateSystemToNormalizedDisplay();
  legend->GetPositionCoordinate()->SetValue(0.8, 0.05);
  legend->GetPosition2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  legend->GetPosition2Coordinate()->SetValue(0.9, 0.15);
  //   legend->GetEntryTextProperty(0)->SetJustificationToRight();

  // Create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // Create a render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors and legend to the scene
  renderer->AddActor(cubeActor);
  renderer->AddActor(planeActor);
  renderer->AddActor(legend);
  renderer->SetBackground(.1, .2, .3); // Background color

  // Reset the camera to show the full scene
  renderer->ResetCamera();

  // Start the rendering loop
  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}
