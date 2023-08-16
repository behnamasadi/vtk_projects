#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleJoystickActor.h>
#include <vtkInteractorStyleSwitch.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObject.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <vtkRenderer.h>

enum class INTERACTION_MODE { CAMERA, ACTOR };

class InteractorStyleSwitch : vtkInteractorStyleSwitch {

public:
  static InteractorStyleSwitch *New();
  void SetDefaultRenderer(vtkRenderer *) override;
  void SetCurrentRenderer(vtkRenderer *) override;
  void SetInteractor(vtkRenderWindowInteractor *) override;
  void OnChar() override;
};

void InteractorStyleSwitch::OnChar() {}

void func(vtkObject *caller, unsigned long eid, void *clientdata,
          void *calldata) {
  auto renderer = reinterpret_cast<vtkRenderer *>(caller);

  std::cout << renderer->GetActiveCamera()->GetPosition()[0] << ", "
            << renderer->GetActiveCamera()->GetPosition()[1] << ", "
            << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
}

struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {

    auto renderer = reinterpret_cast<vtkRenderer *>(caller);

    std::cout << renderer->GetActiveCamera()->GetPosition()[0] << ", "
              << renderer->GetActiveCamera()->GetPosition()[1] << ", "
              << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
  }
};

/*
struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {

    auto renderer = reinterpret_cast<vtkRenderer *>(caller);

    std::cout << renderer->GetActiveCamera()->GetPosition()[0] << ", "
              << renderer->GetActiveCamera()->GetPosition()[1] << ", "
              << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
  }
};

*/

int main(int argc, char **argv) {
  vtkNew<vtkConeSource> cone;
  cone->SetRadius(10);
  cone->SetHeight(10);
  cone->Update();

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(coneActor);

  vtkNew<vtkRenderWindow> renWin;

  renWin->AddRenderer(renderer);

  vtkNew<vtkCallbackCommand> keypressCallback;
  keypressCallback->SetCallback(func);

  renderer->AddObserver(vtkCommand::StartEvent, keypressCallback);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  iren->SetInteractorStyle(style);

  iren->Initialize();
  iren->Start();

  // for (int i = 0; i < 360; ++i) {
  //   // Render the image
  //   renWin->Render();

  //   // Rotate the camera about the view up vector centered at the focal
  //   point.

  //   renderer->GetActiveCamera()->Azimuth(1);

  //   // renderer->GetActiveCamera()->Elevation(1);
  // }
}