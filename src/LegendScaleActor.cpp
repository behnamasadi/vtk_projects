#include <vtkActor.h>
#include <vtkAxisActor2D.h>
#include <vtkCubeSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

int main(int, char *[]) {
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(
      10); // Set the number of grid lines in the X direction
  planeSource->SetYResolution(
      10); // Set the number of grid lines in the Y direction

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

  // Create a renderer and a render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the plane actor to the renderer
  renderer->AddActor(planeActor);
  renderer->SetBackground(0.1, 0.1, 0.1);

  // Create a cube actor for demonstration
  vtkNew<vtkCubeSource> cubeSource;

  double xMin, xMax, yMin, yMax, zMin, zMax;

  xMin = -1;
  xMax = 2;
  yMin = -2;
  yMax = 2;
  zMin = -3;
  zMax = 1;

  cubeSource->SetBounds(xMin, xMax, yMin, yMax, zMin, zMax);

  vtkNew<vtkPolyDataMapper> cubeMapper;
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkNew<vtkActor> cubeActor;
  cubeActor->SetMapper(cubeMapper);
  renderer->AddActor(cubeActor);

  // Create a LegendScaleActor
  vtkNew<vtkLegendScaleActor> legendScaleActor;

  // Get the bounds of the cube actor
  double bounds[6];
  cubeActor->GetBounds(bounds);

  // Compute the range in the x, y, and z dimensions
  double xRange = bounds[1] - bounds[0];
  double yRange = bounds[3] - bounds[2];
  double zRange = bounds[5] - bounds[4];

  std::cout << "xRange: " << xRange << std::endl;

  // Adjust the properties of the LegendScaleActor based on the computed range
  // For this example, we'll set the length of the bottom axis based on the
  // xRange
  // legendScaleActor->SetBottomAxisLength(xRange);

  legendScaleActor->GetBottomAxis()->SetRange(0, 5 * xRange);
  legendScaleActor->GetBottomAxis()->SetTitle(
      "---------------------------------- X Axis "
      "----------------------------------");
  legendScaleActor->GetBottomAxis()->SetNumberOfMinorTicks(10);
  legendScaleActor->GetBottomAxis()->SetAdjustLabels(1);
  legendScaleActor->GetLegendTitleProperty()->SetLineOffset(-25);
  legendScaleActor->GetLegendLabelProperty()->SetLineOffset(-25);
  legendScaleActor->GetLegendLabelProperty()->SetColor(0, 1, 0);
  legendScaleActor->GetLegendTitleProperty()->SetColor(1, 0, 0);

  legendScaleActor->GetLegendLabelProperty()->SetFontSize(
      legendScaleActor->GetLegendLabelProperty()->GetFontSize() * 2);
  legendScaleActor->GetLegendTitleProperty()->SetFontSize(
      legendScaleActor->GetLegendTitleProperty()->GetFontSize() * 2);

  legendScaleActor->SetBottomAxisVisibility(1); // Display the bottom    axis
  // GetBottomAxis

  // Configure the LegendScaleActor
  //   legendScaleActor->SetBottomAxisVisibility(0);
  legendScaleActor->SetTopAxisVisibility(0);
  legendScaleActor->SetRightAxisVisibility(0);
  legendScaleActor->SetLeftAxisVisibility(0);
  //   legendScaleActor->SetLegendVisibility(1);     // Display the numerical
  //   scale legendScaleActor->SetRightAxisVisibility(1);  // Display the right
  //   axis legendScaleActor->SetTopAxisVisibility(1);    // Display the top
  //   axis legendScaleActor->SetLeftAxisVisibility(1);   // Display the left
  //   axis

  // Add the LegendScaleActor to the renderer
  renderer->AddActor(legendScaleActor);

  // Adjust the camera to fit the cube actor in the view
  renderer->ResetCamera(cubeActor->GetBounds());

  // ... [Rest of the rendering code]
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
