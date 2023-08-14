#pragma once

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

class ActorManipulationInteractorStyle
    : public vtkInteractorStyleTrackballActor {
public:
  static ActorManipulationInteractorStyle *New();
  vtkTypeMacro(ActorManipulationInteractorStyle,
               vtkInteractorStyleTrackballActor);

  ActorManipulationInteractorStyle();

  virtual ~ActorManipulationInteractorStyle();

  virtual void OnLeftButtonDown();

  virtual void OnLeftButtonUp();

  void OnMiddleButtonDown();
  void OnMiddleButtonUp();

private:
  vtkSmartPointer<vtkActor> m_lastPickedActor;
  vtkSmartPointer<vtkProperty> m_lastPickedProperty;
};
