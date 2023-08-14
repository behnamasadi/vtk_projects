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
#include <vtkNamedColors.h>
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

class CameraInteractorStyle : public vtkInteractorStyleTrackballCamera {
private:
  vtkSmartPointer<vtkActor> m_lastPickedActor;
  vtkSmartPointer<vtkProperty> m_lastPickedProperty;
  vtkNew<vtkNamedColors> m_namedColors;

  unsigned int m_numberOfClicks;
  int m_previousPosition[2];
  int m_resetPixelDistance;

public:
  static CameraInteractorStyle *New();
  vtkTypeMacro(CameraInteractorStyle, vtkInteractorStyleTrackballCamera);

  CameraInteractorStyle();
  virtual ~CameraInteractorStyle();

  virtual void OnKeyPress() override;

  virtual void OnChar() override;

  virtual void Spin() override;

  virtual void Dolly() override;

  virtual void Pan() override;

  virtual void EnvironmentRotate() override;

  virtual void Rotate() override;

  virtual void OnLeftButtonDown() override;
};
