#include "InteractorStyleSwitch.hpp"
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
#include <vtkSphereSource.h>

// int main(int argc, char **argv) {

//   vtkNew<vtkAxesActor> axes;
//   // axes->SetUserTransform();
//   // axes->SetCoordinateSystem()

//   vtkNew<vtkCameraOrientationWidget> gizmo;

//   vtkNew<vtkBillboardTextActor3D> textActor;
//   textActor->SetInput("foo");

//   textActor->SetPosition(10, 1, 1);

//   vtkNew<vtkConeSource> cone;
//   cone->SetRadius(1);
//   cone->SetHeight(1);
//   cone->Update();

//   vtkNew<vtkPolyDataMapper> coneMapper;
//   coneMapper->SetInputConnection(cone->GetOutputPort());

//   vtkNew<vtkActor> coneActor;
//   coneActor->SetMapper(coneMapper);

//   vtkNew<vtkRenderer> renderer;
//   renderer->AddActor(coneActor);

//   renderer->AddActor(textActor);
//   renderer->AddActor(axes);

//   vtkNew<vtkRenderWindow> renWin;

//   renWin->AddRenderer(renderer);

//   vtkNew<vtkRenderWindowInteractor> iren;
//   iren->SetRenderWindow(renWin);

//   vtkNew<vtkInteractorStyleTrackballCamera> style;
//   // vtkNew<vtkInteractorStyleTrackballActor> style;

//   // vtkNew<InteractorStyleSwitch> style;

//   iren->SetInteractorStyle(style);

//   iren->Initialize();
//   iren->Start();
// }

void func(vtkObject *caller, long unsigned int eventId, void *clientData,
          void *callData) {

  // std::cout << "----" << caller->GetClassName() << std::endl;

  vtkRenderer *ren = dynamic_cast<vtkRenderer *>(caller);

  double position[3], focalPoint[3];
  ren->GetActiveCamera()->GetPosition(position);

  std::cout << "position: " << position[0] << "," << position[1] << ","
            << position[2] << std::endl;

  ren->GetActiveCamera()->GetFocalPoint(focalPoint);

  std::cout << "focalPoint: " << focalPoint[0] << "," << focalPoint[1] << ","
            << focalPoint[2] << std::endl;
}

int main() {
  vtkNew<vtkSphereSource> sphere;
  vtkNew<vtkConeSource> cone;

  vtkNew<vtkAxesActor> axes;

  sphere->SetRadius(1);
  sphere->SetCenter(-1, -1, -1);

  cone->SetCenter(2, 0, 0);

  vtkNew<vtkActor> coneActor;
  vtkNew<vtkActor> sphereActor;

  vtkNew<vtkPolyDataMapper> coneMapper;
  vtkNew<vtkPolyDataMapper> sphereMapper;

  coneMapper->SetInputConnection(cone->GetOutputPort());
  sphereMapper->SetInputConnection(sphere->GetOutputPort());

  coneActor->SetMapper(coneMapper);

  sphereActor->SetMapper(sphereMapper);

  vtkNew<vtkRenderer> ren;

  // ren->GetActiveCamera()->SetPosition(10, 10, 10);
  // ren->GetActiveCamera()->SetFocalPoint(2, 2, 2);

  ren->GetActiveCamera()->SetViewUp(0, 1, 0);
  // ren->GetActiveCamera()->OrthogonalizeViewUp();
  // ren->GetActiveCamera()->ResetClippingRange();

  vtkNew<vtkCallbackCommand> callbackCommand;
  callbackCommand->SetCallback(func);

  ren->AddObserver(vtkCommand::StartEvent, callbackCommand);

  ren->AddActor(coneActor);
  ren->AddActor(sphereActor);
  ren->AddActor(axes);

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(ren);
  vtkNew<vtkRenderWindowInteractor> i_ren;

  // vtkNew<vtkInteractorStyleTrackballActor> style;

  vtkNew<vtkInteractorStyleTrackballCamera> style;

  // vtkNew<InteractorStyleSwitch> style;

  style->SetCurrentRenderer(ren);

  i_ren->SetInteractorStyle(style);

  // style->SetCurrentStyleToTrackballActor();

  i_ren->SetRenderWindow(renWin);
  i_ren->Start();
}