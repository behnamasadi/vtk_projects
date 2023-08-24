#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCameraActor.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

int main(int, char *[]) {
  // Create a cone
  vtkSmartPointer<vtkConeSource> coneSource =
      vtkSmartPointer<vtkConeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> coneMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  coneMapper->SetInputConnection(coneSource->GetOutputPort());
  vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
  coneActor->SetMapper(coneMapper);

  // Create a render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(coneActor);

  // Create a camera and its frustum representation
  vtkSmartPointer<vtkCamera> camera = vtkSmartPointer<vtkCamera>::New();
  vtkSmartPointer<vtkCameraActor> cameraActor =
      vtkSmartPointer<vtkCameraActor>::New();
  cameraActor->SetCamera(camera);

  renderer->AddActor(cameraActor);
  renderer->SetActiveCamera(camera);

  // Set the camera's position and look at the origin
  camera->SetPosition(0, -10, 5);
  camera->Azimuth(30);
  camera->Elevation(30);
  camera->SetFocalPoint(0, 0, 0);
  camera->SetViewUp(0, 0, 1);
  camera->SetClippingRange(5, 15);
  camera->SetViewAngle(30);

  // Adjust the frustum's appearance
  cameraActor->GetProperty()->SetColor(1.0, 1.0, 0.0); // Yellow color

  renderer->ResetCamera();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
