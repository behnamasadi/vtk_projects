#include <QQuickVTKItem.h>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCameraOrientationWidget.h>
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

struct myvtkItem : QQuickVTKItem {

  struct userData : public vtkObject {
    static userData *New();
    vtkTypeMacro(userData, vtkObject);
  };

public:
  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {
    vtkNew<userData> data;
    return data;
  }
};

void func(vtkObject *caller, unsigned long eid, void *clientdata,
          void *calldata) {
  auto renderer = reinterpret_cast<vtkRenderer *>(caller);

  std::cout << "Position: " << renderer->GetActiveCamera()->GetPosition()[0]
            << ", " << renderer->GetActiveCamera()->GetPosition()[1] << ", "
            << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;

  double focalPoint[3];
  renderer->GetActiveCamera()->GetFocalPoint(focalPoint[0], focalPoint[1],
                                             focalPoint[2]);
  std::cout << "FocalPoint: " << focalPoint[0] << ", " << focalPoint[1] << ","
            << focalPoint[2] << std::endl;
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

  vtkNew<vtkAxesActor> axes;
  // axes->SetUserTransform();
  // axes->SetCoordinateSystem()

  vtkNew<vtkCameraOrientationWidget> gizmo;

  vtkNew<vtkBillboardTextActor3D> textActor;
  textActor->SetInput("foo");

  textActor->SetPosition(10, 1, 1);

  vtkNew<vtkConeSource> cone;
  cone->SetRadius(1);
  cone->SetHeight(1);
  cone->Update();

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(coneActor);

  renderer->AddActor(textActor);
  renderer->AddActor(axes);

  vtkNew<vtkRenderWindow> renWin;

  renWin->AddRenderer(renderer);

  vtkNew<vtkCallbackCommand> keypressCallback;
  keypressCallback->SetCallback(func);

  renderer->AddObserver(vtkCommand::StartEvent, keypressCallback);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  // vtkNew<vtkInteractorStyleTrackballActor> style;

  // vtkNew<InteractorStyleSwitch> style;

  iren->SetInteractorStyle(style);

  gizmo->SetParentRenderer(renderer);
  gizmo->On();
  gizmo->SetInteractor(iren);

  // renderer->GetActiveCamera()->SetViewUp(0, 1, 0);

  double focalPoint[3] = {1, 1, 2};
  renderer->GetActiveCamera()->SetViewUp(1, 0, 0);
  // renderer->GetActiveCamera()->SetFocalDistance();
  renderer->GetActiveCamera()->SetFocalPoint(focalPoint[0], focalPoint[1],
                                             focalPoint[2]);

  // renderer->GetActiveCamera()->GetFrustumPlanes()

  renderer->GetActiveCamera()->GetFocalPoint(focalPoint[0], focalPoint[1],
                                             focalPoint[2]);

  std::cout << "FocalPoint: " << focalPoint[0] << ", " << focalPoint[1] << ","
            << focalPoint[2] << std::endl;

  iren->Initialize();
  iren->Start();
}