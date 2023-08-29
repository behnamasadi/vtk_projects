#include "ActorManipulationInteractorStyle.hpp"
#include <QLoggingCategory>

Q_DECLARE_LOGGING_CATEGORY(ACTOR_INTERACTOR_STYLE)
Q_LOGGING_CATEGORY(ACTOR_INTERACTOR_STYLE, "ACTOR_INTERACTOR_STYLE", QtInfoMsg);

ActorManipulationInteractorStyle::ActorManipulationInteractorStyle() {
  m_lastPickedActor = nullptr;
  m_lastPickedProperty = vtkProperty::New();
}

ActorManipulationInteractorStyle::~ActorManipulationInteractorStyle() {
  m_lastPickedProperty->Delete();
}

void ActorManipulationInteractorStyle::OnLeftButtonDown() {
  qCDebug(ACTOR_INTERACTOR_STYLE) << "Pressed left mouse button.";

  int x = Interactor->GetEventPosition()[0];
  int y = Interactor->GetEventPosition()[1];

  FindPokedRenderer(x, y);
  FindPickedActor(x, y);

  if (CurrentRenderer == nullptr || InteractionProp == nullptr) {
    return;
  }

  vtkNew<vtkMatrix4x4> m;
  InteractionProp->GetMatrix(m);
  // qCDebug(CAMERA_INTERACTOR_STYLE) << "Matrix: " << endl << *m;

  // Forward events
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
}

void ActorManipulationInteractorStyle::OnLeftButtonUp() {
  qCDebug(ACTOR_INTERACTOR_STYLE) << "Released left mouse button.";

  vtkNew<vtkMatrix4x4> m;
  if (InteractionProp != nullptr) {
    InteractionProp->GetMatrix(m);
    // qCDebug(CAMERA_INTERACTOR_STYLE) << "Matrix: " << endl << *m;
  }

  // Forward events
  vtkInteractorStyleTrackballActor::OnLeftButtonUp();
}

void ActorManipulationInteractorStyle::OnMiddleButtonDown() {
  int x = Interactor->GetEventPosition()[0];
  int y = Interactor->GetEventPosition()[1];

  FindPokedRenderer(x, y);
  FindPickedActor(x, y);
  if (CurrentRenderer == nullptr || InteractionProp == nullptr) {
    return;
  }

  vtkNew<vtkMatrix4x4> m;
  InteractionProp->GetMatrix(m);
  // qCDebug(ACTOR_INTERACTOR_STYLE) << "Matrix: " << endl << *m ;

  GrabFocus(EventCallbackCommand);
  if (Interactor->GetControlKey()) {
    StartDolly();
  } else {
    StartPan();
  }
}

void ActorManipulationInteractorStyle::OnMiddleButtonUp() {
  switch (State) {
  case VTKIS_DOLLY:
    EndDolly();
    break;

  case VTKIS_PAN:
    EndPan();
    break;
  }

  if (Interactor) {
    ReleaseFocus();
  }
}

vtkStandardNewMacro(ActorManipulationInteractorStyle);
