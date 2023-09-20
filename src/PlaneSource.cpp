#include <vtkActor.h>
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
#include <vtkVectorText.h>

int main(int, char *[]) {
  // Create a plane source with grid lines
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

  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);
  planeActor->PickableOff();

  // Create a renderer and a render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the plane actor to the renderer
  renderer->AddActor(planeActor);
  renderer->SetBackground(0.1, 0.1, 0.1); // Set background color to dark gray

  // Create a LegendScaleActor
  vtkNew<vtkLegendScaleActor> legendScaleActor;

  // Configure the LegendScaleActor
  legendScaleActor->SetLegendVisibility(1);     // Display the numerical scale
  legendScaleActor->SetRightAxisVisibility(1);  // Display the right axis
  legendScaleActor->SetTopAxisVisibility(1);    // Display the top axis
  legendScaleActor->SetLeftAxisVisibility(1);   // Display the left axis
  legendScaleActor->SetBottomAxisVisibility(1); // Display the bottom axis

  // Add the LegendScaleActor to the renderer
  renderer->AddActor(legendScaleActor);

  // Start the rendering loop
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
