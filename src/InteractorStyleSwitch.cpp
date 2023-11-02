#include "InteractorStyleSwitch.hpp"
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkFollower.h>
#include <vtkMapper.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>

VTK_ABI_NAMESPACE_BEGIN
vtkStandardNewMacro(InteractorStyleSwitch);

Q_LOGGING_CATEGORY(INTERACTOR_STYLE_SWITCH, "INTERACTOR_STYLE_SWITCH",
                   QtInfoMsg);

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

// void placeAnglePoint(vtkObject *caller, unsigned long eid, void *clientData,
//                      void *calldata) {
//   double p1[3], p2[3], center[3];

//   auto angleWidget = reinterpret_cast<vtkAngleWidget *>(caller);

//   vtkSmartPointer<InteractorStyleSwitch> interactorStyle =
//       static_cast<InteractorStyleSwitch *>(clientData);

//   interactorStyle->SetAbstractWidget(angleWidget);

//   vtkSmartPointer<vtkAngleRepresentation3D> rep;

//   static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//       ->GetPoint1DisplayPosition(p1);

//   static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//       ->GetPoint2DisplayPosition(p2);

//   static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//       ->GetCenterDisplayPosition(center);

//   qCDebug(INTERACTOR_STYLE_SWITCH) << "p1: " << p1[0] << ", " << p1[1] << ",
//   " << p1[2] ; qCDebug(INTERACTOR_STYLE_SWITCH) << "p2: " << p2[0] << ", " <<
//   p2[1] << ", " << p2[2] ;
// qCDebug(INTERACTOR_STYLE_SWITCH) << "center: " << center[0] << ", " <<
// center[1] << ", " << center[2];

//   vtkNew<vtkPointPicker> pointPicker;
//   angleWidget->GetInteractor()->SetPicker(pointPicker);

//   if (pointPicker->Pick(p1, angleWidget->GetCurrentRenderer())) {
//     double data[3];
//     pointPicker->GetPickPosition(data);
//     qCDebug(INTERACTOR_STYLE_SWITCH) << "point: " << data[0] << ", " <<
//     data[1] << ", " << data[2];

//     static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//         ->GetPoint1Representation()
//         ->SetWorldPosition(data);
//   }

//   if (pointPicker->Pick(p2, angleWidget->GetCurrentRenderer())) {
//     double data[3];
//     pointPicker->GetPickPosition(data);
//     qCDebug(INTERACTOR_STYLE_SWITCH) << "point: " << data[0] << ", " <<
//     data[1] << ", " << data[2]     << ;

//     static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//         ->GetPoint2Representation()
//         ->SetWorldPosition(data);
//   }

//   if (pointPicker->Pick(center, angleWidget->GetCurrentRenderer())) {
//     double data[3];
//     pointPicker->GetPickPosition(data);
//     qCDebug(INTERACTOR_STYLE_SWITCH) << "point: " << data[0] << ", " <<
//     data[1] << ", " << data[2]               ;

//     static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
//         ->GetCenterRepresentation()
//         ->SetWorldPosition(data);
//   }

//   interactorStyle->m_interactionMode = INTERACTION_MODE::CAMERA;
//   interactorStyle->m_txtModeIndicator->SetInput("Camera Mode");
//   interactorStyle->SetCurrentStyle();
// }

void placePoint(vtkObject *caller, unsigned long eid, void *clientData,
                void *calldata) {
  double p1[3], p2[3];

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

  qCDebug(INTERACTOR_STYLE_SWITCH)
      << "placePoint p1: " << p1[0] << ", " << p1[1] << ", " << p1[2];
  qCDebug(INTERACTOR_STYLE_SWITCH)
      << "placePoint p2: " << p2[0] << ", " << p2[1] << ", " << p2[2];

  vtkNew<vtkPointPicker> pointPicker;
  distanceWidget->GetInteractor()->SetPicker(pointPicker);

  double data1[3];
  if (pointPicker->Pick(p1, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data1);
    qCDebug(INTERACTOR_STYLE_SWITCH)
        << "point: " << data1[0] << ", " << data1[1] << ", " << data1[2];

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data1);
  }

  double data2[3];
  if (pointPicker->Pick(p2, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data2);
    qCDebug(INTERACTOR_STYLE_SWITCH)
        << "point: " << data2[0] << ", " << data2[1] << ", " << data2[2];

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data2);
  }

  double sumSquared = std::pow(data1[0] - data2[0], 2) +
                      std::pow(data1[1] - data2[1], 2) +
                      std::pow(data1[2] - data2[2], 2);
  double distance = std::pow(sumSquared, 0.5);
  qCDebug(INTERACTOR_STYLE_SWITCH) << "distance: " << distance;

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

    // m_iren->RemoveObserver()

    // m_iren->RemoveObserver(EventCallbackCommand);
    // m_iren->RemoveObserver(KeyPressCallbackCommand);
    // m_iren->RemoveObserver(vtkCommand::EndInteractionEvent);

    // for (const auto &observer : m_iren->GetObserverMediator() ) {
    // }
    // {
    //   vtkIndent indent;
    //   m_iren->PrintSelf(std::cout, indent);
    // }

    break;
  case 'h':
  case 'H':
    qCDebug(INTERACTOR_STYLE_SWITCH) << "Handling Mode";
    m_txtModeIndicator->SetInput("Handling Mode");

    m_interactionMode = INTERACTION_MODE::ACTOR;
    EventCallbackCommand->SetAbortFlag(1);

    break;

  case 'd':
  case 'D':
    // m_txtModeIndicator->SetInput("Deletion Mode");

    for (auto const &distanceWidget : m_distanceWidgets) {
      distanceWidget->Off();
    }

    // if (m_abstractWidget) {

    //   m_abstractWidget->Off();
    //   m_abstractWidget = nullptr;

    // } else {
    //  qCDebug(INTERACTOR_STYLE_SWITCH) << "no widget to delete " ;
    // }

    m_interactionMode = INTERACTION_MODE::CAMERA;
    m_txtModeIndicator->SetInput("Camera Mode");

    break;
    // case 'a':
    // case 'A': {
    //   m_txtModeIndicator->SetInput("Angle Measurement Mode");

    //   vtkAngleWidget *angleWidget;
    //   angleWidget = vtkAngleWidget::New();
    //   angleWidget->SetInteractor(m_iren);
    //   angleWidget->CreateDefaultRepresentation();

    //   vtkSmartPointer<vtkCallbackCommand> placeAnglePointCallback =
    //       vtkSmartPointer<vtkCallbackCommand>::New();
    //   placeAnglePointCallback->SetCallback(placeAnglePoint);
    //   angleWidget->AddObserver(vtkCommand::EndInteractionEvent,
    //                            placeAnglePointCallback);

    //   placeAnglePointCallback->SetClientData((void *)this);

    //   vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
    //       vtkSmartPointer<vtkPointHandleRepresentation3D>::New();

    //   vtkSmartPointer<vtkAngleRepresentation3D> rep;
    //   rep = vtkSmartPointer<vtkAngleRepresentation3D>::New();
    //   rep->SetHandleRepresentation(handle);
    //   angleWidget->SetRepresentation(rep);
    //   angleWidget->On();
    // }

    break;

  case 'm':
  case 'M': {

    if (m_interactionMode == INTERACTION_MODE::MEASUREMENT) {
      break;
    }

    // m_iren->RemoveAllObservers();
    m_interactionMode = INTERACTION_MODE::MEASUREMENT;
    m_txtModeIndicator->SetInput("Measurement Mode");
    vtkSmartPointer<vtkDistanceWidget> distanceWidget;
    distanceWidget = vtkDistanceWidget::New();
    distanceWidget->SetInteractor(m_iren);

    vtkSmartPointer<vtkCallbackCommand> placePointCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    placePointCallback->SetCallback(placePoint);

    distanceWidget->AddObserver(vtkCommand::EndInteractionEvent,
                                placePointCallback);

    m_distanceWidgets.push_back(distanceWidget);

    placePointCallback->SetClientData((void *)this);

    vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
        vtkSmartPointer<vtkPointHandleRepresentation3D>::New();
    vtkSmartPointer<vtkDistanceRepresentation3D> rep;
    rep = vtkSmartPointer<vtkDistanceRepresentation3D>::New();
    rep->SetHandleRepresentation(handle);

    distanceWidget->CreateDefaultRepresentation();

    // distanceWidget->GetRepresentation();

    // handle->AllOff();
    // distanceWidget->SetRepresentation(rep);
    // rep->RulerModeOff();

    // rep->GetGlyphActor()->SetScale(0, 0, 0);

    // rep->GetLabelActor()->GetProperty()->SetDisplayLocationToForeground();

    // rep->GetLabelActor()->GetProperty()->SetRenderingOrder(vtkProperty::VTK_RENDERING_TRANSPARENT);

    rep->SetNumberOfRulerTicks(0);

    rep->GetLabelActor()->SetScale(15, 15, 15);

    distanceWidget->On();
    break;
  }

  case 's':
  case 'S': {

    if (m_interactionMode == INTERACTION_MODE::SCALING) {
      break;
    }
    m_interactionMode = INTERACTION_MODE::SCALING;
    if (m_scalingWidget)
      m_scalingWidget->Off();
    m_txtModeIndicator->SetInput("Scaling Mode");

    m_scalingWidget = vtkDistanceWidget::New();
    m_scalingWidget->SetInteractor(m_iren);

    vtkSmartPointer<vtkCallbackCommand> placePointCallback =
        vtkSmartPointer<vtkCallbackCommand>::New();
    placePointCallback->SetCallback(placePoint);
    m_scalingWidget->AddObserver(vtkCommand::EndInteractionEvent,
                                 placePointCallback);

    placePointCallback->SetClientData((void *)this);

    vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
        vtkSmartPointer<vtkPointHandleRepresentation3D>::New();
    vtkSmartPointer<vtkDistanceRepresentation3D> rep;
    rep = vtkSmartPointer<vtkDistanceRepresentation3D>::New();
    rep->SetHandleRepresentation(handle);
    m_scalingWidget->SetRepresentation(rep);
    rep->SetMaximumNumberOfRulerTicks(0);

    rep->RulerModeOff();

    rep->GetLineProperty()->SetColor(
        m_namedColors->GetColor3d("Gold").GetData());

    rep->GetLabelProperty()->SetColor(
        m_namedColors->GetColor3d("Navy").GetData());

    m_scalingWidget->On();
    break;
  }

  default:
    qCDebug(INTERACTOR_STYLE_SWITCH)
        << "InteractorStyleSwitch::OnChar() " << m_iren->GetKeyCode();
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
//   qCDebug(INTERACTOR_STYLE_SWITCH) << "InteractorStyleSwitch::OnKeyPress(): "
//   << key ;
// }

void InteractorStyleSwitch::SetCurrentStyle() {
  if (m_interactionMode == INTERACTION_MODE::CAMERA) {
    if (m_currentStyle != m_trackballCameraStyle) {
      if (m_currentStyle) {
        m_currentStyle->SetInteractor(nullptr);
      }
      m_currentStyle = m_trackballCameraStyle;
      m_txtModeIndicator->SetInput("Camera Mode");
    }

  } else if (m_interactionMode == INTERACTION_MODE::ACTOR) {
    if (m_currentStyle != m_trackballActorStyle) {
      if (m_currentStyle) {
        m_currentStyle->SetInteractor(nullptr);
      }
      m_currentStyle = m_trackballActorStyle;
      m_txtModeIndicator->SetInput("Handling Mode");
    }
  }
  if (m_currentStyle) {
    m_currentStyle->SetInteractor(m_iren);
    m_currentStyle->SetTDxStyle(TDxStyle);
  }
  CurrentRenderer->AddActor(m_txtModeIndicator);
}

void InteractorStyleSwitch::SetInteractor(vtkRenderWindowInteractor *iren) {

  if (iren == m_iren) {
    return;
  }
  // if we already have an Interactor then stop observing it
  if (m_iren) {
    m_iren->RemoveObserver(EventCallbackCommand);
  }
  m_iren = iren;
  m_trackballCameraStyle->SetInteractor(m_iren);
  // add observers for each of the events handled in ProcessEvents
  if (iren) {
    iren->AddObserver(vtkCommand::CharEvent, EventCallbackCommand, Priority);
    iren->AddObserver(vtkCommand::DeleteEvent, EventCallbackCommand, Priority);
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