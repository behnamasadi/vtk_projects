#pragma once

#include "ActorManipulationInteractorStyle.hpp"
#include "CameraInteractorStyle.hpp"
#include <list>
#include <vtkActor.h>
#include <vtkAngleRepresentation3D.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkInteractionStyleModule.h> // For export macro
#include <vtkInteractorStyleSwitchBase.h>
#include <vtkNamedColors.h>
#include <vtkPropPicker.h>
#include <vtkSmartPointer.h>
#include <vtkTextActor.h>
#include <vtkWorldPointPicker.h>

/*
class InteractorStyleSwitch
{
- ActorManipulationInteractorStyle * TrackballActor;
- CameraInteractorStyle * TrackballCamera;

  + void SetCurrentStyleToTrackballActor();
  + void SetCurrentStyleToTrackballCamera();

  + void SetDefaultRenderer(vtkRenderer *) override;
  + void SetCurrentRenderer(vtkRenderer *) override;

  + void SetInteractor(vtkRenderWindowInteractor *iren) override;
  + vtkRenderWindowInteractor *GetInteractor();

}


InteractorStyleSwitch o-- ActorManipulationInteractorStyle
InteractorStyleSwitch o-- CameraInteractorStyle

class vtkInteractorStyleSwitchBase



vtkInteractorStyleSwitchBase  <|-- InteractorStyleSwitch


class vtkInteractorStyleTrackballActor
class vtkInteractorStyleTrackballCamera


class ActorManipulationInteractorStyle
class CameraInteractorStyle

vtkInteractorStyleTrackballCamera   <|--  CameraInteractorStyle
vtkInteractorStyleTrackballActor  <|-- ActorManipulationInteractorStyle




*/

enum class INTERACTION_MODE { CAMERA, ACTOR };

// VTK_ABI_NAMESPACE_BEGIN

class InteractorStyleSwitch : public vtkInteractorStyleSwitchBase {
public:
  static InteractorStyleSwitch *New();
  vtkTypeMacro(InteractorStyleSwitch, vtkInteractorStyleSwitchBase);
  void PrintSelf(ostream &os, vtkIndent indent) override;

  void SetAbstractWidget(vtkSmartPointer<vtkAbstractWidget> abstractWidget);
  vtkSmartPointer<vtkAbstractWidget> GetAbstractWidget();
  void SetAutoAdjustCameraClippingRange(vtkTypeBool value) override;

  void SetInteractor(vtkRenderWindowInteractor *iren) override;
  vtkRenderWindowInteractor *GetInteractor() override;

  void SetCurrentStyleToTrackballActor();
  void SetCurrentStyleToTrackballCamera();

  void SetDefaultRenderer(vtkRenderer *) override;
  void SetCurrentRenderer(vtkRenderer *) override;

  void OnChar() override;
  // void OnKeyPress() override;
  vtkGetObjectMacro(m_currentStyle, vtkInteractorStyle);
  vtkNew<vtkTextActor> m_txtModeIndicator;
  void SetCurrentStyle();

  std::vector<unsigned long> observerIds;

  vtkNew<vtkRenderWindowInteractor> m_iren;

protected:
  InteractorStyleSwitch();
  ~InteractorStyleSwitch() override;

private:
  InteractorStyleSwitch(const InteractorStyleSwitch &) = delete;
  void operator=(const InteractorStyleSwitch &) = delete;

  // small text on the screen to inform the user the mode

  vtkSmartPointer<vtkTextProperty> m_txtModeIndicatorProperty;

  // abstract widget for distance and angle measurement
  vtkSmartPointer<vtkAbstractWidget> m_abstractWidget = nullptr;
  vtkNew<vtkNamedColors> m_namedColors;

  // style for camera interaction
  vtkSmartPointer<ActorManipulationInteractorStyle> m_trackballActorStyle;

  // style for actor interaction
  vtkSmartPointer<CameraInteractorStyle> m_trackballCameraStyle;

  // current style, could be any of above
  vtkSmartPointer<vtkInteractorStyle> m_currentStyle;

public:
  INTERACTION_MODE m_interactionMode;
};

// VTK_ABI_NAMESPACE_END
