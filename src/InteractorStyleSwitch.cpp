#include "InteractorStyleSwitch.hpp"

#include <algorithm>
#include <math.h>
#include <typeinfo>
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

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(InteractorStyleSwitch);

// constructor
InteractorStyleSwitch::InteractorStyleSwitch() {

  // setting camera/ actor
  m_trackballActorStyle = ActorManipulationInteractorStyle::New();
  m_trackballCameraStyle = CameraInteractorStyle::New();
  m_interactionMode = INTERACTION_MODE::CAMERA;
  m_currentStyle = nullptr;

  m_txtModeIndicator->SetInput("Camera Mode");
  m_txtModeIndicatorProperty = m_txtModeIndicator->GetTextProperty();
  m_txtModeIndicatorProperty->SetFontFamilyToArial();
  m_txtModeIndicatorProperty->BoldOn();
  m_txtModeIndicatorProperty->SetFontSize(36);
  m_txtModeIndicatorProperty->ShadowOn();
  m_txtModeIndicatorProperty->SetShadowOffset(4, 4);
  m_txtModeIndicatorProperty->SetColor(
      m_namedColors->GetColor3d("Cornsilk").GetData());
  m_txtModeIndicator->SetDisplayPosition(20, 30);
  m_iren = vtkRenderWindowInteractor::New();

  // SetCurrentRenderer();
  // SetCurrentStyle();
}

InteractorStyleSwitch::~InteractorStyleSwitch() {

  m_trackballActorStyle->Delete();
  m_trackballActorStyle = nullptr;

  m_trackballCameraStyle->Delete();
  m_trackballCameraStyle = nullptr;
}

void InteractorStyleSwitch::SetAutoAdjustCameraClippingRange(
    vtkTypeBool value) {
  if (value == AutoAdjustCameraClippingRange) {
    return;
  }

  if (value < 0 || value > 1) {
    vtkErrorMacro("Value must be between 0 and 1 for"
                  << " SetAutoAdjustCameraClippingRange");
    return;
  }

  AutoAdjustCameraClippingRange = value;
  m_trackballActorStyle->SetAutoAdjustCameraClippingRange(value);
  m_trackballCameraStyle->SetAutoAdjustCameraClippingRange(value);

  Modified();
}

void InteractorStyleSwitch::SetCurrentStyleToTrackballActor() {

  m_interactionMode = INTERACTION_MODE::ACTOR;
  SetCurrentStyle();
}

void InteractorStyleSwitch::SetCurrentStyleToTrackballCamera() {
  m_interactionMode = INTERACTION_MODE::CAMERA;
  SetCurrentStyle();
}

void placeAnglePoint(vtkObject *caller, unsigned long eid, void *clientData,
                     void *calldata) {
  double p1[3], p2[3], center[3];

  auto angleWidget = reinterpret_cast<vtkAngleWidget *>(caller);

  vtkSmartPointer<InteractorStyleSwitch> interactorStyle =
      static_cast<InteractorStyleSwitch *>(clientData);

  interactorStyle->SetAbstractWidget(angleWidget);

  vtkSmartPointer<vtkAngleRepresentation3D> rep;

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetCenterDisplayPosition(center);

  std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
  std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
  std::cout << "center: " << center[0] << ", " << center[1] << ", " << center[2]
            << std::endl;

  vtkNew<vtkPointPicker> pointPicker;
  angleWidget->GetInteractor()->SetPicker(pointPicker);

  if (pointPicker->Pick(p1, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(p2, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(center, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetCenterRepresentation()
        ->SetWorldPosition(data);
  }

  interactorStyle->m_interactionMode = INTERACTION_MODE::CAMERA;
  interactorStyle->m_txtModeIndicator->SetInput("Camera Mode");
  interactorStyle->SetCurrentStyle();
}

void placePoint(vtkObject *caller, unsigned long eid, void *clientData,
                void *calldata) {
  double p1[3], p2[3];

  // auto distanceWidget = reinterpret_cast<vtkDistanceWidget *>(clientData);
  auto distanceWidget = reinterpret_cast<vtkDistanceWidget *>(caller);

  vtkSmartPointer<InteractorStyleSwitch> interactorStyle =
      static_cast<InteractorStyleSwitch *>(clientData);

  interactorStyle->SetAbstractWidget(distanceWidget);
  vtkSmartPointer<vtkDistanceRepresentation3D> rep;

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  std::cout << "placePoint p1: " << p1[0] << ", " << p1[1] << ", " << p1[2]
            << std::endl;
  std::cout << "placePoint p2: " << p2[0] << ", " << p2[1] << ", " << p2[2]
            << std::endl;

  vtkNew<vtkPointPicker> pointPicker;
  distanceWidget->GetInteractor()->SetPicker(pointPicker);

  double data1[3];
  if (pointPicker->Pick(p1, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data1);
    std::cout << "point: " << data1[0] << ", " << data1[1] << ", " << data1[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data1);
  }

  double data2[3];
  if (pointPicker->Pick(p2, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data2);
    std::cout << "point: " << data2[0] << ", " << data2[1] << ", " << data2[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data2);
  }

  double sumSquared = std::pow(data1[0] - data2[0], 2) +
                      std::pow(data1[1] - data2[1], 2) +
                      std::pow(data1[2] - data2[2], 2);
  double distance = std::pow(sumSquared, 0.5);
  std::cout << "distance: " << distance << std::endl;

  interactorStyle->m_interactionMode = INTERACTION_MODE::CAMERA;
  interactorStyle->m_txtModeIndicator->SetInput("Camera Mode");

  interactorStyle->SetCurrentStyle();
}

void InteractorStyleSwitch::OnChar() {

  switch (m_iren->GetKeyCode()) {
  case 'c':
  case 'C':
    m_interactionMode = INTERACTION_MODE::CAMERA;
    EventCallbackCommand->SetAbortFlag(1);
    m_txtModeIndicator->SetInput("Camera Mode");

    //->RemoveObserver(observerId);

    break;
  case 'h':
  case 'H':
    std::cout << "Handling Mode" << std::endl;
    m_txtModeIndicator->SetInput("Handling Mode");

    m_interactionMode = INTERACTION_MODE::ACTOR;
    EventCallbackCommand->SetAbortFlag(1);

    break;
  case 'd':
  case 'D':
    m_txtModeIndicator->SetInput("Deletion Mode");

    if (m_abstractWidget) {
      std::cout << "deleting last selected widget" << std::endl;

      m_abstractWidget->Off();
      m_abstractWidget = nullptr;

    } else {
      std::cout << "no widget to delete " << std::endl;
    }
    break;
  case 'a':
  case 'A': {
    m_txtModeIndicator->SetInput("Angle Measurement Mode");

    vtkAngleWidget *angleWidget;
    angleWidget = vtkAngleWidget::New();
    angleWidget->SetInteractor(m_iren);
    angleWidget->CreateDefaultRepresentation();

    vtkSmartPointer<vtkCallbackCommand> placeAnglePointCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    placeAnglePointCallback->SetCallback(placeAnglePoint);
    unsigned long observerId = angleWidget->AddObserver(
        vtkCommand::EndInteractionEvent, placeAnglePointCallback);

    observerIds.push_back(observerId);

    placeAnglePointCallback->SetClientData((void *)this);

    vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
        vtkSmartPointer<vtkPointHandleRepresentation3D>::New();

    vtkSmartPointer<vtkAngleRepresentation3D> rep;
    rep = vtkSmartPointer<vtkAngleRepresentation3D>::New();
    rep->SetHandleRepresentation(handle);
    angleWidget->SetRepresentation(rep);
    angleWidget->On();
  }

  break;

  case 'm':
  case 'M': {
    m_txtModeIndicator->SetInput("Measurement Mode");
    vtkDistanceWidget *distanceWidget;
    distanceWidget = vtkDistanceWidget::New();
    distanceWidget->SetInteractor(m_iren);
    std::cout << "m_iren type name: " << typeid(m_iren).name() << "\n";

    vtkSmartPointer<vtkCallbackCommand> placePointCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    placePointCallback->SetCallback(placePoint);

    unsigned long observerId = distanceWidget->AddObserver(
        vtkCommand::EndInteractionEvent, placePointCallback);

    observerIds.push_back(observerId);

    placePointCallback->SetClientData((void *)this);

    vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
        vtkSmartPointer<vtkPointHandleRepresentation3D>::New();
    vtkSmartPointer<vtkDistanceRepresentation3D> rep;
    rep = vtkSmartPointer<vtkDistanceRepresentation3D>::New();
    rep->SetHandleRepresentation(handle);
    distanceWidget->SetRepresentation(rep);
    rep->SetMaximumNumberOfRulerTicks(2);
    distanceWidget->On();
    break;
  }

  case 's':
  case 'S': {
    m_txtModeIndicator->SetInput("Scaling Mode");
    vtkDistanceWidget *distanceWidget;
    distanceWidget = vtkDistanceWidget::New();
    distanceWidget->SetInteractor(m_iren);

    vtkSmartPointer<vtkCallbackCommand> placePointCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    placePointCallback->SetCallback(placePoint);
    distanceWidget->AddObserver(vtkCommand::EndInteractionEvent,
                                placePointCallback);

    placePointCallback->SetClientData((void *)this);

    vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
        vtkSmartPointer<vtkPointHandleRepresentation3D>::New();
    vtkSmartPointer<vtkDistanceRepresentation3D> rep;
    rep = vtkSmartPointer<vtkDistanceRepresentation3D>::New();
    rep->SetHandleRepresentation(handle);
    distanceWidget->SetRepresentation(rep);
    rep->SetMaximumNumberOfRulerTicks(2);
    distanceWidget->On();
    break;
  }

  default:
    std::cout << "InteractorStyleSwitch::OnChar() " << m_iren->GetKeyCode()
              << std::endl;
  }

  SetCurrentStyle();
}

// void InteractorStyleSwitch::OnKeyPress() {

//   // Get the keypress
//   vtkRenderWindowInteractor *rwi = Interactor;
//   std::string key = rwi->GetKeySym();

//   std::transform(key.begin(), key.end(), key.begin(),
//                  [](unsigned char c) { return std::tolower(c); });

//   // Output the key that was pressed
//   std::cout << "InteractorStyleSwitch::OnKeyPress(): " << key << std::endl;
// }

void InteractorStyleSwitch::SetCurrentStyle() {
  if (m_interactionMode == INTERACTION_MODE::CAMERA) {
    if (m_currentStyle != m_trackballCameraStyle) {
      if (m_currentStyle) {
        m_currentStyle->SetInteractor(nullptr);
      }
      m_currentStyle = m_trackballCameraStyle;
    }

  } else if (m_interactionMode == INTERACTION_MODE::ACTOR) {
    if (m_currentStyle != m_trackballActorStyle) {
      if (m_currentStyle) {
        m_currentStyle->SetInteractor(nullptr);
      }
      m_currentStyle = m_trackballActorStyle;
    }
  }
  if (m_currentStyle) {
    m_currentStyle->SetInteractor(m_iren);
    m_currentStyle->SetTDxStyle(TDxStyle);
  }
  CurrentRenderer->AddActor(m_txtModeIndicator);
  std::cout << "InteractorStyleSwitch::SetCurrentStyle" << std::endl;
}

void InteractorStyleSwitch::SetInteractor(vtkRenderWindowInteractor *iren) {
  std::cout << "(((((((((((((((((((((((((((())))))))))))))))))))))))))))"
            << std::endl;
  if (iren == m_iren) {
    return;
  }
  // if we already have an Interactor then stop observing it
  if (m_iren) {
    m_iren->RemoveObserver(EventCallbackCommand);
    // Interactor->RemoveObserver(EventCallbackCommand);
    std::cout << "m_iren->RemoveObserver" << std::endl;
  }
  m_iren = iren;
  m_trackballCameraStyle->SetInteractor(m_iren);
  // Interactor = iren;
  // add observers for each of the events handled in ProcessEvents
  if (iren) {
    iren->AddObserver(vtkCommand::CharEvent, EventCallbackCommand, Priority);

    iren->AddObserver(vtkCommand::DeleteEvent, EventCallbackCommand, Priority);

    std::cout << "-------************-----------" << std::endl;
  }
  SetCurrentStyle();
}

vtkRenderWindowInteractor *InteractorStyleSwitch::GetInteractor() {
  return m_iren;
}

void InteractorStyleSwitch::PrintSelf(ostream &os, vtkIndent indent) {
  Superclass::PrintSelf(os, indent);
  os << indent << "m_currentStyle " << m_currentStyle << "\n";
  if (m_currentStyle) {
    vtkIndent next_indent = indent.GetNextIndent();
    os << next_indent << m_currentStyle->GetClassName() << "\n";
    m_currentStyle->PrintSelf(os, indent.GetNextIndent());
  }
}

void InteractorStyleSwitch::SetDefaultRenderer(vtkRenderer *renderer) {
  vtkInteractorStyle::SetDefaultRenderer(renderer);
  m_trackballActorStyle->SetDefaultRenderer(renderer);
  m_trackballCameraStyle->SetDefaultRenderer(renderer);
}

void InteractorStyleSwitch::SetCurrentRenderer(vtkRenderer *renderer) {
  vtkInteractorStyle::SetCurrentRenderer(renderer);
  m_trackballActorStyle->SetCurrentRenderer(renderer);
  m_trackballCameraStyle->SetCurrentRenderer(renderer);
}

void InteractorStyleSwitch::SetAbstractWidget(
    vtkSmartPointer<vtkAbstractWidget> abstractWidget) {
  m_abstractWidget = abstractWidget;
}

vtkSmartPointer<vtkAbstractWidget> InteractorStyleSwitch::GetAbstractWidget() {
  return m_abstractWidget;
}

VTK_ABI_NAMESPACE_END