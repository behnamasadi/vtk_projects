#include "ActorManipulationInteractorStyle.hpp"

ActorManipulationInteractorStyle::ActorManipulationInteractorStyle() {
  m_lastPickedActor = nullptr;
  m_lastPickedProperty = vtkProperty::New();
}

ActorManipulationInteractorStyle::~ActorManipulationInteractorStyle() {
  m_lastPickedProperty->Delete();
}

void ActorManipulationInteractorStyle::OnLeftButtonDown() {
  // std::cout << "Pressed left mouse button." << std::endl;

  int x = Interactor->GetEventPosition()[0];
  int y = Interactor->GetEventPosition()[1];

  FindPokedRenderer(x, y);
  FindPickedActor(x, y);

  if (CurrentRenderer == nullptr || InteractionProp == nullptr) {
    return;
  }

  vtkNew<vtkMatrix4x4> m;
  InteractionProp->GetMatrix(m);
  std::cout << "Matrix: " << endl << *m << std::endl;

  // Forward events
  vtkInteractorStyleTrackballActor::OnLeftButtonDown();
}

void ActorManipulationInteractorStyle::OnLeftButtonUp() {
  // std::cout << "Released left mouse button." << std::endl;

  vtkNew<vtkMatrix4x4> m;
  if (InteractionProp != nullptr) {
    InteractionProp->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;
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
  std::cout << "Matrix: " << endl << *m << std::endl;

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
