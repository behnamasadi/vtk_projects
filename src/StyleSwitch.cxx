#include <vtkActor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

/*
switching between four interactor styles:
- joystick actor, Type 'j'
- joystick camera,  Type  't'
- trackball actor, type 'c'
- trackball camera.  type  'a'
*/

int main(int, char *[]) {
  vtkNew<vtkNamedColors> namedColor;
  vtkNew<vtkSphereSource> sphereSource;

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<vtkActor> actor;
  // actor->PickableOff();
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(namedColor->GetColor3d("MistyRose").GetData());

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("StyleSwitch");

  // grid background
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(10);
  planeSource->SetYResolution(10);
  planeSource->SetOrigin(0, 0, 0);
  planeSource->SetPoint1(10, 0, 0);
  planeSource->SetPoint2(0, 10, 0);

  // Create a mapper and actor for the plane: show it as a wireframe
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();
  planeActor->GetProperty()->SetColor(
      namedColor->GetColor3d("Silver").GetData());

  // planeActor->PickableOff();

  renderer->AddActor(planeActor);

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(namedColor->GetColor3d("SlateGray").GetData());

  // Render
  renderWindow->Render();

  vtkNew<vtkInteractorStyleSwitch> style;

  renderWindowInteractor->SetInteractorStyle(style);

  // Begin mouse interaction
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
