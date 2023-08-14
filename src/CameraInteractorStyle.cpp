#include "CameraInteractorStyle.hpp"

#include <algorithm>
#include <vtkAngleWidget.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkObjectFactory.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTextProperty.h>
#include <vtkTimeStamp.h>
#include <vtkTransform.h>
#include <vtkWidgetEvent.h>

CameraInteractorStyle::CameraInteractorStyle() {
  m_lastPickedActor = nullptr;
  m_lastPickedProperty = vtkProperty::New();
  m_previousPosition[0] = 0;
  m_previousPosition[1] = 0;

  m_numberOfClicks = 0;
  m_resetPixelDistance = 5;

  // CurrentRenderer->GetActiveCamera()->SetViewUp(0, 1, 0);
}
CameraInteractorStyle::~CameraInteractorStyle() {
  m_lastPickedProperty->Delete();
}

void CameraInteractorStyle::OnKeyPress() {
  // Get the keypress
  vtkRenderWindowInteractor *rwi = Interactor;
  std::string key = rwi->GetKeySym();

  std::transform(key.begin(), key.end(), key.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  // Output the key that was pressed
  std::cout << "CameraInteractorStyle::OnKeyPress() " << key << std::endl;

  // Handle an arrow key
  if (key == "up") {
    std::cout << "The up arrow was pressed." << std::endl;
  }

  if (key == "delete") {
    std::cout << "The delete key was pressed." << std::endl;
  }

  // Forward events
  // vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CameraInteractorStyle::OnChar() {
  // Get the keypress
  vtkRenderWindowInteractor *rwi = Interactor;
  std::string key = rwi->GetKeySym();

  std::transform(key.begin(), key.end(), key.begin(),
                 [](unsigned char c) { return std::tolower(c); });

  // Output the key that was pressed
  std::cout << "OnChar() " << key << std::endl;

  // Handle an arrow key
  if (key == "up") {
    std::cout << "The up arrow was pressed." << std::endl;
  }

  if (key == "delete") {
    std::cout << "The delete key was pressed." << std::endl;
  }

  // Forward events
  // vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CameraInteractorStyle::Spin() {
  std::cout << "------------------- Spin ----------------------" << std::endl;
  vtkInteractorStyleTrackballCamera::Spin();
}

void CameraInteractorStyle::Dolly() {

  //   right click down + <- ->
  //   camera should rotate in its place, to do that we calculate the azimuth
  //   that rotate teh camera around We should store the old camera position,
  //   apply azimuth, then we calculate forward vector from camera to the
  //   focal point, then from previously stored camera position we go forward
  //   in the direction of forward vector and calculate the new camera focal
  //   point

  std::cout << "Dolly." << std::endl;

  vtkTimeStamp timeStamp;
  std::cout << "Time stamp: " << timeStamp << std::endl;
  timeStamp.Modified();
  std::cout << "Time stamp: " << timeStamp << std::endl;

  std::cout << " RightButtonDown " << std::endl;

  if (CurrentRenderer == nullptr) {
    return;
  }
  vtkRenderWindowInteractor *rwi = Interactor;

  double *center = CurrentRenderer->GetCenter();

  int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

  std::cout << "dx: " << dx << " dy: " << dy << std::endl;

  const int *size = CurrentRenderer->GetRenderWindow()->GetSize();

  double delta_elevation = +20.0 / size[1];
  double delta_azimuth = -20.0 / size[0];

  double rxf = dx * delta_azimuth * MotionFactor;
  double ryf = dy * delta_elevation * MotionFactor;

  vtkCamera *camera = CurrentRenderer->GetActiveCamera();
  double previousCameraPosition[3];
  double *cameraPosition;

  cameraPosition = camera->GetPosition();

  previousCameraPosition[0] = cameraPosition[0];
  previousCameraPosition[1] = cameraPosition[1];
  previousCameraPosition[2] = cameraPosition[2];

  std::cout << "Previous Camera Position: " << previousCameraPosition[0]
            << " , " << previousCameraPosition[1] << " , "
            << previousCameraPosition[2] << std::endl;

  double *previousDirectionOfProjection = camera->GetDirectionOfProjection();

  std::cout << "Previous Direction Of Projection: "
            << previousDirectionOfProjection[0] << ", "
            << previousDirectionOfProjection[1] << " , "
            << previousDirectionOfProjection[2] << std::endl;

  std::cout << "Previous Distance: " << camera->GetDistance() << std::endl;

  camera->Azimuth(rxf);

  double distance = camera->GetDistance();

  double *directionOfProjection;
  double newDirectionOfProjection[3];
  directionOfProjection = camera->GetDirectionOfProjection();

  newDirectionOfProjection[0] = directionOfProjection[0];
  newDirectionOfProjection[1] = directionOfProjection[1];
  newDirectionOfProjection[2] = directionOfProjection[2];

  camera->SetFocalPoint(
      previousCameraPosition[0] + distance * newDirectionOfProjection[0],
      previousCameraPosition[1] + distance * newDirectionOfProjection[1],
      previousCameraPosition[2] + distance * newDirectionOfProjection[2]);
  camera->SetViewUp(0, 1, 0);
  camera->OrthogonalizeViewUp();

  /////////////// up/ down + right click ///////////////

  directionOfProjection = camera->GetDirectionOfProjection();
  double *focalPoint = camera->GetFocalPoint();

  double previousFocalPoint[3];
  previousFocalPoint[0] = focalPoint[0];
  previousFocalPoint[1] = focalPoint[1];
  previousFocalPoint[2] = focalPoint[2];

  newDirectionOfProjection[0] = directionOfProjection[0];
  newDirectionOfProjection[1] = directionOfProjection[1];
  newDirectionOfProjection[2] = directionOfProjection[2];

  camera->SetFocalPoint(
      previousFocalPoint[0] + ryf * newDirectionOfProjection[0],
      previousFocalPoint[1] + ryf * newDirectionOfProjection[1],
      previousFocalPoint[2] + ryf * newDirectionOfProjection[2]);

  camera->SetPosition(
      previousCameraPosition[0] + ryf * newDirectionOfProjection[0],
      previousCameraPosition[1] + ryf * newDirectionOfProjection[1],
      previousCameraPosition[2] + ryf * newDirectionOfProjection[2]);

  if (AutoAdjustCameraClippingRange) {
    CurrentRenderer->ResetCameraClippingRange();
  }

  if (rwi->GetLightFollowCamera()) {
    CurrentRenderer->UpdateLightsGeometryToFollowCamera();
  }

  rwi->Render();

  // vtkInteractorStyleTrackballCamera::Dolly();
}

void CameraInteractorStyle::Pan() {
  std::cout << "Pan." << std::endl;
  vtkInteractorStyleTrackballCamera::Pan();
}

void CameraInteractorStyle::EnvironmentRotate() {
  std::cout << "EnvironmentRotate." << std::endl;
}

void CameraInteractorStyle::Rotate() {
  std::cout << "****************************************** " << std::endl;

  if (CurrentRenderer == nullptr) {
    return;
  }
  vtkRenderWindowInteractor *rwi = Interactor;

  double *center = CurrentRenderer->GetCenter();

  int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

  std::cout << "dx: " << dx << " dy: " << dy << std::endl;

  const int *size = CurrentRenderer->GetRenderWindow()->GetSize();

  double delta_elevation = -20.0 / size[1];
  double delta_azimuth = -20.0 / size[0];

  double rxf = dx * delta_azimuth * MotionFactor;
  double ryf = dy * delta_elevation * MotionFactor;

  vtkCamera *camera = CurrentRenderer->GetActiveCamera();
  double *cameraPosition = camera->GetPosition();

  std::cout << "cameraPosition: " << cameraPosition[0] << " , "
            << cameraPosition[1] << " , " << cameraPosition[2] << std::endl;

  double *cameraGetFocalPoint = camera->GetFocalPoint();

  std::cout << "cameraGetFocalPoint: " << cameraGetFocalPoint[0] << " , "
            << cameraGetFocalPoint[1] << " , " << cameraGetFocalPoint[2]
            << std::endl;

  camera->SetViewUp(0, 1, 0);
  // camera->SetViewUp(0, 1, 0);
  camera->OrthogonalizeViewUp();

  double *viewUpVector = camera->GetViewUp();

  std::cout << "viewUpVector: " << viewUpVector[0] << " , " << viewUpVector[1]
            << " , " << viewUpVector[2] << std::endl;

  camera->Azimuth(rxf);
  camera->Elevation(ryf);

  // camera->Pro

  std::cout << "rxf: " << rxf << std::endl;
  std::cout << "ryf: " << ryf << std::endl;

  // camera->OrthogonalizeViewUp();

  if (AutoAdjustCameraClippingRange) {
    CurrentRenderer->ResetCameraClippingRange();
  }

  if (rwi->GetLightFollowCamera()) {
    CurrentRenderer->UpdateLightsGeometryToFollowCamera();
  }

  rwi->Render();
}

void CameraInteractorStyle::OnLeftButtonDown() {
  std::cout << "Pressed left mouse button." << std::endl;

  // HighLightActor
  std::cout << "HighLight Actor" << std::endl;
  int *clickPos = GetInteractor()->GetEventPosition();
  std::cout << clickPos[0] << "," << clickPos[1] << std::endl;

  vtkNew<vtkPropPicker> picker;
  if (GetCurrentRenderer())
    picker->Pick(clickPos[0], clickPos[1], 0, GetCurrentRenderer());

  // If we picked something before, reset its property
  if (m_lastPickedActor) {
    m_lastPickedActor->GetProperty()->DeepCopy(m_lastPickedProperty);
  }

  m_lastPickedActor = picker->GetActor();

  if (m_lastPickedActor) {
    // Save the property of the picked actor so that we can
    // restore it next time

    m_lastPickedProperty->DeepCopy(m_lastPickedActor->GetProperty());

    // Highlight the picked actor by changing its properties
    m_lastPickedActor->GetProperty()->SetColor(
        m_namedColors->GetColor3d("Red").GetData());
    m_lastPickedActor->GetProperty()->SetDiffuse(1.0);
    m_lastPickedActor->GetProperty()->SetSpecular(0.0);
    m_lastPickedActor->GetProperty()->EdgeVisibilityOn();
  }

  ////////////double click /////////////////

  m_numberOfClicks++;
  int pickPosition[2];
  GetInteractor()->GetEventPosition(pickPosition);

  int xdist = pickPosition[0] - m_previousPosition[0];
  int ydist = pickPosition[1] - m_previousPosition[1];

  std::cout << "xdist: " << xdist << std::endl;
  std::cout << "ydist: " << ydist << std::endl;

  m_previousPosition[0] = pickPosition[0];
  m_previousPosition[1] = pickPosition[1];

  int moveDistance = (int)sqrt((double)(xdist * xdist + ydist * ydist));

  std::cout << "moveDistance: " << moveDistance << std::endl;

  // Reset numClicks - If mouse moved further than resetPixelDistance
  if (moveDistance > m_resetPixelDistance) {
    m_numberOfClicks = 1;
  }

  if (m_numberOfClicks == 2) {
    std::cout << "Double clicked." << std::endl;

    // move camera focal point to here
    // picker->GetPickPosition();
    // //picker->GetProp3D
    // //picker->Getpick

    std::cout << "Picking pixel: " << GetInteractor()->GetEventPosition()[0]
              << " " << GetInteractor()->GetEventPosition()[1] << std::endl;
    GetInteractor()->GetPicker()->Pick(
        GetInteractor()->GetEventPosition()[0],
        GetInteractor()->GetEventPosition()[1],
        0, // always zero.
        GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
    double picked[3];
    Interactor->GetPicker()->GetPickPosition(picked);
    std::cout << "Picked value: " << picked[0] << " " << picked[1] << " "
              << picked[2] << std::endl;

    m_numberOfClicks = 0;

    vtkCamera *camera = CurrentRenderer->GetActiveCamera();
    camera->SetFocalPoint(picked);
  }

  // Forward events
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

vtkStandardNewMacro(CameraInteractorStyle);
