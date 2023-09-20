#include <vtkActor.h>
#include <vtkAxisActor2D.h>
#include <vtkCubeSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

int main(int, char *[]) {
  vtkSmartPointer<vtkPlaneSource> planeSource =
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetXResolution(
      10); // Set the number of grid lines in the X direction
  planeSource->SetYResolution(
      10); // Set the number of grid lines in the Y direction

  // Map the plane's data to graphics primitives
  vtkSmartPointer<vtkPolyDataMapper> planeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  planeMapper->SetInputConnection(planeSource->GetOutputPort());

  // Create an actor for the plane and set its properties to display the grid
  // lines
  vtkSmartPointer<vtkActor> planeActor = vtkSmartPointer<vtkActor>::New();
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()
      ->SetRepresentationToWireframe();               // Display as wireframe
  planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to gray

  // Create a renderer and a render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the plane actor to the renderer
  renderer->AddActor(planeActor);
  renderer->SetBackground(0.1, 0.1, 0.1);

  // Create a cube actor for demonstration
  vtkSmartPointer<vtkCubeSource> cubeSource =
      vtkSmartPointer<vtkCubeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);
  renderer->AddActor(cubeActor);

  // Create a LegendScaleActor
  vtkSmartPointer<vtkLegendScaleActor> legendScaleActor =
      vtkSmartPointer<vtkLegendScaleActor>::New();

  // Get the bounds of the cube actor
  double bounds[6];
  cubeActor->GetBounds(bounds);

  // Compute the range in the x, y, and z dimensions
  double xRange = bounds[1] - bounds[0];
  double yRange = bounds[3] - bounds[2];
  double zRange = bounds[5] - bounds[4];

  // Adjust the properties of the LegendScaleActor based on the computed range
  // For this example, we'll set the length of the bottom axis based on the
  // xRange
  // legendScaleActor->SetBottomAxisLength(xRange);

  legendScaleActor->GetBottomAxis()->SetRange(0, xRange);
  legendScaleActor->GetBottomAxis()->SetTitle("X Axis");
  legendScaleActor->GetBottomAxis()->SetNumberOfMinorTicks(10);
  // GetBottomAxis

  // Configure the LegendScaleActor
  legendScaleActor->SetLegendVisibility(1);     // Display the numerical scale
  legendScaleActor->SetRightAxisVisibility(1);  // Display the right axis
  legendScaleActor->SetTopAxisVisibility(1);    // Display the top axis
  legendScaleActor->SetLeftAxisVisibility(1);   // Display the left axis
  legendScaleActor->SetBottomAxisVisibility(1); // Display the bottom axis

  // Add the LegendScaleActor to the renderer
  renderer->AddActor(legendScaleActor);

  // Adjust the camera to fit the cube actor in the view
  renderer->ResetCamera(cubeActor->GetBounds());

  // ... [Rest of the rendering code]
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
