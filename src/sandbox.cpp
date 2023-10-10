#include "InteractorStyleSwitch.hpp"
#include <QQuickVTKItem.h>
#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkArcSource.h>
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
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

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

  ////////////////////////// adding  trackball //////////////////////////

  // Create arcs for visual representation of the trackball
  vtkSmartPointer<vtkArcSource> arcSource =
      vtkSmartPointer<vtkArcSource>::New();
  arcSource->SetPoint1(0, 1, 0);
  arcSource->SetPoint2(0, -1, 0);
  arcSource->SetCenter(0, 0, 0);
  arcSource->Update();

  // Create the X, Y and Z arcs using transformation
  vtkSmartPointer<vtkTransform> transformX =
      vtkSmartPointer<vtkTransform>::New();
  transformX->RotateY(90);

  vtkSmartPointer<vtkTransform> transformY =
      vtkSmartPointer<vtkTransform>::New();
  // No need for rotation on Y as it's the default orientation of arc

  vtkSmartPointer<vtkTransform> transformZ =
      vtkSmartPointer<vtkTransform>::New();
  transformZ->RotateX(90);

  // Apply the transformations
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterX =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilterX->SetTransform(transformX);
  transformFilterX->SetInputConnection(arcSource->GetOutputPort());

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterZ =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilterZ->SetTransform(transformZ);
  transformFilterZ->SetInputConnection(arcSource->GetOutputPort());

  // Append all arcs together
  vtkSmartPointer<vtkAppendPolyData> appendFilter =
      vtkSmartPointer<vtkAppendPolyData>::New();
  appendFilter->AddInputConnection(arcSource->GetOutputPort());
  appendFilter->AddInputConnection(transformFilterX->GetOutputPort());
  appendFilter->AddInputConnection(transformFilterZ->GetOutputPort());
  appendFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> arcMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  arcMapper->SetInputConnection(appendFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> arcActor = vtkSmartPointer<vtkActor>::New();
  arcActor->SetMapper(arcMapper);
  arcActor->GetProperty()->SetColor(1.0, 0.5, 0.0); // Color the arcs

  // Add actors to the render

  vtkNew<vtkRenderer> ren;

  ren->AddActor(arcActor);
  ren->SetBackground(0.1, 0.1, 0.1);

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