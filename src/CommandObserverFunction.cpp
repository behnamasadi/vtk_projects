#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAppendFilter.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

void func(vtkObject *caller, long unsigned int eventId, void *clientData,
          void *callData) {

  std::cout << "Click callback" << std::endl;
  std::cout << "Event: " << vtkCommand::GetStringFromEventId(eventId)
            << std::endl;

  // Get the interactor like this:
  auto *iren = static_cast<vtkRenderWindowInteractor *>(caller);
}

int main() {

  vtkNew<vtkConeSource> cone;
  cone->Update();

  vtkNew<vtkCubeSource> cube;
  cube->Update();

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkPolyDataMapper> cubeMapper;
  cubeMapper->SetInputConnection(cube->GetOutputPort());

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  vtkNew<vtkActor> cubeActor;
  cubeActor->SetMapper(cubeMapper);

  vtkNew<vtkRenderer> ren;
  ren->AddActor(coneActor);
  ren->AddActor(cubeActor);

  cubeActor->SetPosition(-1, 2, 0.5);

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(ren);

  renWin->Render();

  vtkNew<vtkCallbackCommand> keypressCallback;
  keypressCallback->SetCallback(func);
  vtkNew<vtkRenderWindowInteractor> i_Ren;

  i_Ren->AddObserver(vtkCommand::LeftButtonPressEvent, keypressCallback);
  i_Ren->AddObserver(vtkCommand::RightButtonPressEvent, keypressCallback);
  i_Ren->AddObserver(vtkCommand::KeyPressEvent, keypressCallback);

  i_Ren->SetRenderWindow(renWin);
  i_Ren->Initialize();

  vtkNew<vtkInteractorStyleTrackballActor> style;
  i_Ren->SetInteractorStyle(style);
  i_Ren->Start();
}
