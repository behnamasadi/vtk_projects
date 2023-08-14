#include <vtkActor.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickVTKItem.h>
#include <QQuickVTKRenderItem.h>
#include <QQuickVTKRenderWindow.h>
#include <QQuickWindow>
#include <QUrl>

int main(int argc, char *argv[]) {

  QQuickVTKRenderWindow::setupGraphicsBackend();
  QGuiApplication app(argc, argv);
  QQmlApplicationEngine engine;
  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl("qrc:/qml/pointcloud.qml"));

  QObject *topLevel = engine.rootObjects().value(0);
  QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

  window->show();

  // Fetch the QQuick window using the standard object name set up in the
  // constructor
  QQuickVTKRenderItem *qquickvtkItem =
      topLevel->findChild<QQuickVTKRenderItem *>("View");

  // QQuickVTKItem *qquickvtkItem = topLevel->findChild<QQuickVTKItem
  // *>("View");

  vtkNew<vtkPointSource> pointSource;

  pointSource->SetNumberOfPoints(1000000);
  pointSource->SetRadius(1.0);
  pointSource->Update();

  vtkPolyData *points = pointSource->GetOutput();
  // Get a point point in the set
  double inSet[3];
  points->GetPoint(25, inSet);

  vtkNew<vtkPolyDataMapper> cleanedMapper;
  cleanedMapper->SetInputConnection(pointSource->GetOutputPort());

  // Create a cone pipeline and add it to the view
  vtkNew<vtkActor> actor;

  actor->SetMapper(cleanedMapper);

  qquickvtkItem->renderer()->AddActor(actor);
  qquickvtkItem->renderer()->ResetCamera();
  qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
  qquickvtkItem->renderer()->SetBackground2(0.7, 0.7, 0.7);
  qquickvtkItem->renderer()->SetGradientBackground(true);
  qquickvtkItem->update();
  app.exec();
}
