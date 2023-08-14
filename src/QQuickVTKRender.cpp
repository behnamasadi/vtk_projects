#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickVTKInteractorAdapter.h>
#include <QQuickVTKItem.h>
#include <QQuickVTKRenderItem.h>

#include <QQuickVTKRenderWindow.h>
#include <QQuickWindow>
#include <QUrl>
#include <algorithm>
#include <vtkActor.h>
#include <vtkAngleWidget.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkInteractorStyleRubberBandZoom.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkObjectFactory.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
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

#include "ActorManipulationInteractorStyle.hpp"
#include "CameraInteractorStyle.hpp"

/*
https://discourse.vtk.org/t/best-way-to-implement-vtk-9-1-with-qt6-using-qml-and-qt-quick/6958
https://discourse.vtk.org/t/qml-example/7247/4
https://kitware.github.io/vtk-examples/site/CxxHowTo/
https://vtk.org/doc/nightly/html/classQQuickVTKRenderItem.html
https://discourse.vtk.org/t/is-it-possible-to-visualize-the-point-cloud-in-qvtkopenglwidget/8534/4

*/

int main(int argc, char *argv[]) {

  QQuickVTKRenderWindow::setupGraphicsBackend();
  QGuiApplication app(argc, argv);

  QQmlApplicationEngine engine;

  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl("qrc:/qml/QQuickVTKRender.qml"));

  QObject *topLevel = engine.rootObjects().value(0);
  QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

  window->show();

  // Fetch the QQuick window using the standard object name set up in the
  // constructor

  QQuickVTKRenderItem *qquickvtkItem =
      topLevel->findChild<QQuickVTKRenderItem *>("ConeView");

  //   QQuickVTKItem *qquickvtkItem =
  //       topLevel->findChild<QQuickVTKItem *>("ConeView");

  // qquickvtkItem->keyReleaseEvent();

  // qquickvtkItem->setKeepMouseGrab

  vtkNew<vtkActor> actor;

  vtkNew<vtkPolyDataMapper> mapper;
  vtkNew<vtkConeSource> cone;
  mapper->SetInputConnection(cone->GetOutputPort());

  actor->SetMapper(mapper);
  qquickvtkItem->renderer()->AddActor(actor);
  qquickvtkItem->renderer()->ResetCamera();
  qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
  qquickvtkItem->renderer()->SetBackground2(0.7, 0.7, 0.7);
  qquickvtkItem->renderer()->SetGradientBackground(true);

  // qquickvtkItem->renderWindow()->interactorAdapter()->QueueKeyEvent()

  // https://discourse.vtk.org/t/vtk9-1-for-qml-with-qquickvtkrenderitem-have-some-issues/7601

  //   vtkSmartPointer<vtkInteractorStyleTerrain> style =
  //       vtkSmartPointer<vtkInteractorStyleTerrain>::New();

  //   vtkSmartPointer<vtkInteractorStyleRubberBandZoom> style =
  //       vtkSmartPointer<vtkInteractorStyleRubberBandZoom>::New();

  // vtkSmartPointer<ActorManipulationInteractorStyle> style =
  //     vtkSmartPointer<ActorManipulationInteractorStyle>::New();

  vtkSmartPointer<CameraInteractorStyle> style =
      vtkSmartPointer<CameraInteractorStyle>::New();

  vtkNew<vtkRenderWindowInteractor> iRen;
  qquickvtkItem->renderWindow()
      ->renderWindow()
      ->GetInteractor()
      ->SetInteractorStyle(style);
  style->SetDefaultRenderer(qquickvtkItem->renderer());

  qquickvtkItem->setAcceptHoverEvents(true);

  qquickvtkItem->renderWindow()->setAcceptHoverEvents(true);
  qquickvtkItem->update();

  app.exec();
}

// https://fossies.org/linux/VTK/GUISupport/QtQuick/QQuickVTKRenderItem.h