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

// #include "../vtk/ActorManipulationInteractorStyle.hpp"
//#include "../vtk/CameraInteractorStyle.hpp"

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

#include "MyVtkItem.hpp"
vtkStandardNewMacro(MyVtkItem::Data);

int main(int argc, char *argv[]) {
  QQuickVTKItem::setGraphicsApi();

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QGuiApplication app(argc, argv);

  qmlRegisterType<MyVtkItem>("com.vtk.example", 1, 0, "MyVtkItem");

  QQmlApplicationEngine engine;

  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl(QStringLiteral("qrc:/qml/QtQuickCone.qml")));

  if (engine.rootObjects().isEmpty())
    return -1;

  return app.exec();
}
