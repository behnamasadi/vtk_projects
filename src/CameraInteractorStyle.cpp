#include "CameraInteractorStyle.hpp"

#include <QLoggingCategory>
#include <algorithm>
#include <vtkAngleWidget.h>
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

Q_DECLARE_LOGGING_CATEGORY(CAMERA_INTERACTOR_STYLE)
Q_LOGGING_CATEGORY(CAMERA_INTERACTOR_STYLE, "CAMERA_INTERACTOR_STYLE",
                   QtInfoMsg);

CameraInteractorStyle::CameraInteractorStyle() {
  m_lastPickedActor = nullptr;
  m_lastPickedProperty = vtkProperty::New();
  m_previousPosition[0] = 0;
  m_previousPosition[1] = 0;

  m_numberOfClicks = 0;
  m_resetPixelDistance = 5;
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
  qCDebug(CAMERA_INTERACTOR_STYLE) << "CameraInteractorStyle::OnKeyPress() ";

  // Handle an arrow key
  if (key == "up") {
    qCDebug(CAMERA_INTERACTOR_STYLE) << "The up arrow was pressed.";
  }

  if (key == "delete") {
    qCDebug(CAMERA_INTERACTOR_STYLE) << "The delete key was pressed.";
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
  qCDebug(CAMERA_INTERACTOR_STYLE) << "OnChar() " << key.c_str();

  // Handle an arrow key
  if (key == "up") {
    qCDebug(CAMERA_INTERACTOR_STYLE) << "The up arrow was pressed.";
  }

  if (key == "delete") {
    qCDebug(CAMERA_INTERACTOR_STYLE) << "The delete key was pressed.";
  }

  // Forward events
  // vtkInteractorStyleTrackballCamera::OnKeyPress();
}

void CameraInteractorStyle::Spin() {
  qCDebug(CAMERA_INTERACTOR_STYLE) << "CameraInteractorStyle::Spin()";
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

  qCDebug(CAMERA_INTERACTOR_STYLE) << "Dolly.";

  vtkTimeStamp timeStamp;
  qCDebug(CAMERA_INTERACTOR_STYLE) << "Time stamp: " << timeStamp;
  timeStamp.Modified();
  qCDebug(CAMERA_INTERACTOR_STYLE) << "Time stamp: " << timeStamp;

  qCDebug(CAMERA_INTERACTOR_STYLE) << " RightButtonDown ";

  if (CurrentRenderer == nullptr) {
    return;
  }
  vtkRenderWindowInteractor *rwi = Interactor;

  double *center = CurrentRenderer->GetCenter();

  int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

  qCDebug(CAMERA_INTERACTOR_STYLE) << "dx: " << dx << " dy: " << dy;

  const int *size = CurrentRenderer->GetRenderWindow()->GetSize();

  double delta_elevation = +10.0 / size[1];
  double delta_azimuth = -10.0 / size[0];

  double rxf = dx * delta_azimuth * MotionFactor;
  double ryf = dy * delta_elevation * MotionFactor;

  vtkCamera *camera = CurrentRenderer->GetActiveCamera();
  double previousCameraPosition[3];
  double *cameraPosition;

  cameraPosition = camera->GetPosition();

  previousCameraPosition[0] = cameraPosition[0];
  previousCameraPosition[1] = cameraPosition[1];
  previousCameraPosition[2] = cameraPosition[2];

  qCDebug(CAMERA_INTERACTOR_STYLE)
      << "Previous Camera Position: " << previousCameraPosition[0] << " , "
      << previousCameraPosition[1] << " , " << previousCameraPosition[2];

  double *previousDirectionOfProjection = camera->GetDirectionOfProjection();

  qCDebug(CAMERA_INTERACTOR_STYLE) << "Previous Direction Of Projection: "
                                   << previousDirectionOfProjection[0] << ", "
                                   << previousDirectionOfProjection[1] << " , "
                                   << previousDirectionOfProjection[2];

  qCDebug(CAMERA_INTERACTOR_STYLE)
      << "Previous Distance: " << camera->GetDistance();

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
  qCDebug(CAMERA_INTERACTOR_STYLE) << "Pan.";
  vtkInteractorStyleTrackballCamera::Pan();
}

void CameraInteractorStyle::EnvironmentRotate() {
  qCDebug(CAMERA_INTERACTOR_STYLE) << "EnvironmentRotate.";
}

void CameraInteractorStyle::Rotate() {
  qCDebug(CAMERA_INTERACTOR_STYLE) << "CameraInteractorStyle::Rotate() ";

  if (CurrentRenderer == nullptr) {
    return;
  }
  vtkRenderWindowInteractor *rwi = Interactor;

  double *center = CurrentRenderer->GetCenter();

  int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
  int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

  qCDebug(CAMERA_INTERACTOR_STYLE) << "dx: " << dx << " dy: " << dy;

  const int *size = CurrentRenderer->GetRenderWindow()->GetSize();

  double delta_elevation = -20.0 / size[1];
  double delta_azimuth = -20.0 / size[0];

  double rxf = dx * delta_azimuth * MotionFactor;
  double ryf = dy * delta_elevation * MotionFactor;

  vtkCamera *camera = CurrentRenderer->GetActiveCamera();
  double *cameraPosition = camera->GetPosition();

  qCDebug(CAMERA_INTERACTOR_STYLE)
      << "cameraPosition: " << cameraPosition[0] << " , " << cameraPosition[1]
      << " , " << cameraPosition[2];

  double *cameraGetFocalPoint = camera->GetFocalPoint();

  qCDebug(CAMERA_INTERACTOR_STYLE)
      << "cameraGetFocalPoint: " << cameraGetFocalPoint[0] << " , "
      << cameraGetFocalPoint[1] << " , " << cameraGetFocalPoint[2];

  camera->SetViewUp(0, 1, 0);
  camera->OrthogonalizeViewUp();

  double *viewUpVector = camera->GetViewUp();

  qCDebug(CAMERA_INTERACTOR_STYLE)
      << "viewUpVector: " << viewUpVector[0] << " , " << viewUpVector[1]
      << " , " << viewUpVector[2];

  camera->Azimuth(rxf);
  camera->Elevation(ryf);

  // camera->Pro

  qCDebug(CAMERA_INTERACTOR_STYLE) << "rxf: " << rxf;
  qCDebug(CAMERA_INTERACTOR_STYLE) << "ryf: " << ryf;

  if (AutoAdjustCameraClippingRange) {
    CurrentRenderer->ResetCameraClippingRange();
  }

  if (rwi->GetLightFollowCamera()) {
    CurrentRenderer->UpdateLightsGeometryToFollowCamera();
  }

  rwi->Render();
}

void CameraInteractorStyle::OnLeftButtonDown() {
  qCDebug(CAMERA_INTERACTOR_STYLE) << "Pressed left mouse button.";

  // HighLightActor
  qCDebug(CAMERA_INTERACTOR_STYLE) << "HighLight Actor";
  int *clickPos = GetInteractor()->GetEventPosition();
  qCDebug(CAMERA_INTERACTOR_STYLE) << clickPos[0] << "," << clickPos[1];

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

  qCDebug(CAMERA_INTERACTOR_STYLE) << "xdist: " << xdist;
  qCDebug(CAMERA_INTERACTOR_STYLE) << "ydist: " << ydist;

  m_previousPosition[0] = pickPosition[0];
  m_previousPosition[1] = pickPosition[1];

  int moveDistance = (int)sqrt((double)(xdist * xdist + ydist * ydist));

  qCDebug(CAMERA_INTERACTOR_STYLE) << "moveDistance: " << moveDistance;

  // Reset numClicks - If mouse moved further than resetPixelDistance
  if (moveDistance > m_resetPixelDistance) {
    m_numberOfClicks = 1;
  }

  if (m_numberOfClicks == 2) {
    qCDebug(CAMERA_INTERACTOR_STYLE) << "Double clicked.";

    // move camera focal point to here
    // picker->GetPickPosition();
    // //picker->GetProp3D
    // //picker->Getpick

    qCDebug(CAMERA_INTERACTOR_STYLE)
        << "Picking pixel: " << GetInteractor()->GetEventPosition()[0] << " "
        << GetInteractor()->GetEventPosition()[1];
    GetInteractor()->GetPicker()->Pick(
        GetInteractor()->GetEventPosition()[0],
        GetInteractor()->GetEventPosition()[1],
        0, // always zero.
        GetInteractor()->GetRenderWindow()->GetRenderers()->GetFirstRenderer());
    double picked[3];
    Interactor->GetPicker()->GetPickPosition(picked);
    qCDebug(CAMERA_INTERACTOR_STYLE) << "Picked value: " << picked[0] << " "
                                     << picked[1] << " " << picked[2];

    m_numberOfClicks = 0;

    vtkCamera *camera = CurrentRenderer->GetActiveCamera();
    camera->SetFocalPoint(picked);
  }

  // Forward events
  vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
}

vtkStandardNewMacro(CameraInteractorStyle);
