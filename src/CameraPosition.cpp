#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

void CameraCallback(vtkObject *caller, long unsigned int vtkNotUsed(eventId),
                    void *vtkNotUsed(clientData), void *vtkNotUsed(callData)) {

  std::cout << caller->GetClassName() << " modified" << std::endl;

  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  // Print the interesting stuff.
  std::cout << "  auto camera = renderer->GetActiveCamera();" << std::endl;
  std::cout << "  camera->SetPosition(" << camera->GetPosition()[0] << ", "
            << camera->GetPosition()[1] << ", " << camera->GetPosition()[2]
            << ");" << std::endl;
  std::cout << "  camera->SetFocalPoint(" << camera->GetFocalPoint()[0] << ", "
            << camera->GetFocalPoint()[1] << ", " << camera->GetFocalPoint()[2]
            << ");" << std::endl;
  std::cout << "  camera->SetViewUp(" << camera->GetViewUp()[0] << ", "
            << camera->GetViewUp()[1] << ", " << camera->GetViewUp()[2] << ");"
            << std::endl;
  std::cout << "  camera->SetDistance(" << camera->GetDistance() << ");"
            << std::endl;
  std::cout << "  camera->SetClippingRange(" << camera->GetClippingRange()[0]
            << ", " << camera->GetClippingRange()[1] << ");" << std::endl;
  std::cout << std::endl;
}

int main() {
  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkConeSource> cone;
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkRenderer> renderer;

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("MistyRose").GetData());

  renderer->AddActor(actor);

  vtkNew<vtkRenderWindow> renWin;

  // This is automatically set when the renderer is created by MakeRenderer. The
  // user probably shouldn't ever need to call this method
  // renderer->SetRenderWindow(renWin);

  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());
  renWin->AddRenderer(renderer);

  vtkNew<vtkCallbackCommand> cameraCallbackCommand;

  cameraCallbackCommand->SetCallback(CameraCallback);
  renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent,
                                           cameraCallbackCommand);

  vtkNew<vtkRenderWindowInteractor> iRen;
  iRen->SetRenderWindow(renWin);
  iRen->Start();
}