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

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  // vtkNew<vtkInteractorStyleTrackballActor> style;

  // vtkNew<InteractorStyleSwitch> style;

  iren->SetInteractorStyle(style);

  iren->Initialize();
  iren->Start();
}