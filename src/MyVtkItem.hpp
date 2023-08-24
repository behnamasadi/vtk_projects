#include <QtQml/QQmlApplicationEngine>

#include <QtQuick/QQuickWindow>

#include <QtGui/QGuiApplication>
#include <QtGui/QSurfaceFormat>

#include <QQuickVTKItem.h>
#include <QVTKRenderWindowAdapter.h>

#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include <vtkInteractorStyleJoystickCamera.h>
#include <vtkInteractorStyleRubberBandZoom.h>
#include <vtkInteractorStyleTerrain.h>

#include "CameraInteractorStyle.hpp"

#include "InteractorStyleSwitch.hpp"
#include <QVTKInteractor.h>
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

struct MyVtkItem : public QQuickVTKItem {

  Q_OBJECT
public:
  struct Data : vtkObject {
    static Data *New();
    vtkTypeMacro(Data, vtkObject);

    vtkNew<vtkConeSource> cone;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
    vtkNew<QVTKInteractor> iRen;
  };

  void handleMouseClick(int x, int y);

  // using vtkUserData = vtkSmartPointer<vtkObject>;
  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {
    // Create a cone pipeline and add it to the view

    vtkNew<Data> vtk;

    vtk->actor->SetMapper(vtk->mapper);
    vtk->mapper->SetInputConnection(vtk->cone->GetOutputPort());

    vtk->renderer->AddActor(vtk->actor);
    vtk->renderer->ResetCamera();
    vtk->renderer->SetBackground(0.0, 1.0, 1.0);
    vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
    vtk->renderer->SetGradientBackground(true);

    renderWindow->AddRenderer(vtk->renderer);
    renderWindow->SetMultiSamples(16);

    // vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
    //     vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

    // vtkSmartPointer<vtkInteractorStyleRubberBandZoom> style =
    //     vtkSmartPointer<vtkInteractorStyleRubberBandZoom>::New();

    // vtkSmartPointer<vtkInteractorStyleTerrain> style =
    //     vtkSmartPointer<vtkInteractorStyleTerrain>::New();

    // vtkSmartPointer<vtkInteractorStyleJoystickCamera> style =
    //     vtkSmartPointer<vtkInteractorStyleJoystickCamera>::New();

    // vtkSmartPointer<CameraInteractorStyle> style =
    //     vtkSmartPointer<CameraInteractorStyle>::New();

    vtkSmartPointer<InteractorStyleSwitch> style =
        vtkSmartPointer<InteractorStyleSwitch>::New();

    style->SetCurrentRenderer(vtk->renderer);

    renderWindow->AddRenderer(vtk->renderer);
    vtk->iRen->SetInteractorStyle(style);

    renderWindow->SetInteractor(vtk->iRen);

    return vtk;
  }

  // using vtkUserData = vtkSmartPointer<vtkObject>;

  Q_INVOKABLE void resetCamera() {
    dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
      /*

      auto *vtk = Data::SafeDownCast(userData);
      double s;
      s = 1.1;
      vtk->actor->SetScale(vtk->actor->GetScale()[0] * s);

      std::cout << "Scale: " << vtk->actor->GetScale()[0] << ","
                << vtk->actor->GetScale()[1] << "," << vtk->actor->GetScale()[2]
                << std::endl;
      */

      Data *vtk = Data::SafeDownCast(userData);

      double s;
      s = 1.1;
      vtk->actor->SetScale(vtk->actor->GetScale()[0] * s);

      std::cout << "Scale: " << vtk->actor->GetScale()[0] << ","
                << vtk->actor->GetScale()[1] << "," << vtk->actor->GetScale()[2]
                << std::endl;

      vtkMatrix4x4 *m = vtk->actor->GetMatrix();

      std::cout << "actor matrix: \n";
      m->Print(std::cout);
      scheduleRender();
    });
  }
};
