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

// int main() {
//   vtkNew<vtkSphereSource> sphere;
//   vtkNew<vtkConeSource> cone;

//   vtkNew<vtkAxesActor> axes;

//   sphere->SetRadius(1);
//   sphere->SetCenter(-1, -1, -1);

//   cone->SetCenter(2, 0, 0);

//   vtkNew<vtkActor> coneActor;
//   vtkNew<vtkActor> sphereActor;

//   vtkNew<vtkPolyDataMapper> coneMapper;
//   vtkNew<vtkPolyDataMapper> sphereMapper;

//   coneMapper->SetInputConnection(cone->GetOutputPort());
//   sphereMapper->SetInputConnection(sphere->GetOutputPort());

//   coneActor->SetMapper(coneMapper);

//   sphereActor->SetMapper(sphereMapper);

//   vtkNew<vtkRenderer> ren;

//   // ren->GetActiveCamera()->SetPosition(10, 10, 10);
//   // ren->GetActiveCamera()->SetFocalPoint(2, 2, 2);

//   ren->GetActiveCamera()->SetViewUp(0, 1, 0);
//   // ren->GetActiveCamera()->OrthogonalizeViewUp();
//   // ren->GetActiveCamera()->ResetClippingRange();

//   vtkNew<vtkCallbackCommand> callbackCommand;
//   callbackCommand->SetCallback(func);

//   ren->AddObserver(vtkCommand::StartEvent, callbackCommand);

//   ren->AddActor(coneActor);
//   ren->AddActor(sphereActor);
//   ren->AddActor(axes);

//   vtkNew<vtkRenderWindow> renWin;
//   renWin->AddRenderer(ren);
//   vtkNew<vtkRenderWindowInteractor> i_ren;

//   // vtkNew<vtkInteractorStyleTrackballActor> style;

//   vtkNew<vtkInteractorStyleTrackballCamera> style;

//   // vtkNew<InteractorStyleSwitch> style;

//   style->SetCurrentRenderer(ren);

//   i_ren->SetInteractorStyle(style);

//   // style->SetCurrentStyleToTrackballActor();

//   i_ren->SetRenderWindow(renWin);
//   i_ren->Start();
// }
#include <vtkActor.h>
#include <vtkPLYReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkMaskPoints.h>
#include <vtkMath.h>
#include <vtkRenderer.h>

#include <vtkLight.h>

class LoDCallback : public vtkCommand {
public:
  static LoDCallback *New() { return new LoDCallback; }

  vtkRenderer *Renderer;
  vtkMaskPoints *MaskPoints;
  double
      Centroid[3]; // Assume you have computed the centroid of the point cloud

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    vtkCamera *camera = Renderer->GetActiveCamera();
    double camPos[3];
    camera->GetPosition(camPos);

    double distance = vtkMath::Distance2BetweenPoints(camPos, Centroid);
    distance = sqrt(distance);

    int ratio = static_cast<int>(
        distance / 10.0); // This is an example; adjust the factor as necessary
    ratio = std::max(1, ratio); // Ensure we don't go below 1

    std::cout << ratio << std::endl;

    MaskPoints->SetOnRatio(ratio);
    MaskPoints->Modified(); // Important to force an update
  }
};

int main() {
  vtkSmartPointer<vtkPLYReader> reader = vtkSmartPointer<vtkPLYReader>::New();
  reader->SetFileName("/home/behnam/map.ply");

  vtkSmartPointer<vtkMaskPoints> mask = vtkSmartPointer<vtkMaskPoints>::New();
  mask->SetInputConnection(reader->GetOutputPort());
  mask->RandomModeOn();

  vtkSmartPointer<vtkVertexGlyphFilter> vertexFilter =
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  vertexFilter->SetInputConnection(reader->GetOutputPort());
  vertexFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(vertexFilter->GetOutputPort());

  //   if (reader->GetOutput() == nullptr ||
  //       reader->GetOutput()->GetNumberOfCells() == 0) {
  //     std::cerr << "Error reading PLY file." << std::endl;
  //     return -1;
  //   }

  //   vtkSmartPointer<vtkPolyDataMapper> mapper =
  //       vtkSmartPointer<vtkPolyDataMapper>::New();
  //   mapper->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(0.1, 0.1, 0.1); // Background color

  renderer->SetAmbient(0.5, 0.5, 0.5); // R, G, B values for ambient light

  actor->GetProperty()->SetAmbient(1.0); // Fully respond to ambient light

  //   renderer->AutomaticLightCreationOff();

  //   vtkSmartPointer<vtkLight> ambientLight =
  //   vtkSmartPointer<vtkLight>::New();
  //   //ambientLight->SetLightTypeToAmbient();
  //   ambientLight->SetIntensity(0.5); // Adjust the intensity as needed
  //   renderer->AddLight(ambientLight);

  //   vtkSmartPointer<vtkLight> light1 = vtkSmartPointer<vtkLight>::New();
  //   light1->SetPosition(1, 1, 1);
  //   light1->SetFocalPoint(0, 0, 0);
  //   renderer->AddLight(light1);

  //   vtkSmartPointer<vtkLight> light2 = vtkSmartPointer<vtkLight>::New();
  //   light2->SetPosition(-1, -1, -1);
  //   light2->SetFocalPoint(0, 0, 0);
  //   renderer->AddLight(light2);

  //   actor->GetProperty()->SetAmbient(0.6);
  //   actor->GetProperty()->SetDiffuse(0.5);
  //   actor->GetProperty()->SetSpecular(0.1);

  // You can add more lights as needed from various directions.

  double bounds[6];
  reader->Update();
  reader->GetOutput()->GetBounds(bounds);
  double centroid[3];
  centroid[0] = (bounds[0] + bounds[1]) / 2.0;
  centroid[1] = (bounds[2] + bounds[3]) / 2.0;
  centroid[2] = (bounds[4] + bounds[5]) / 2.0;

  vtkSmartPointer<LoDCallback> lodCallback =
      vtkSmartPointer<LoDCallback>::New();
  lodCallback->Renderer = renderer;
  lodCallback->MaskPoints = mask;
  for (int i = 0; i < 3; ++i) {
    lodCallback->Centroid[i] = centroid[i];
  }

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);
  // interactor->AddObserver(vtkCommand::RenderEvent, lodCallback);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}