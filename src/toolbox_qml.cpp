#include <vtkActor.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>

#include <QApplication>
#include <QQmlApplicationEngine>
#include <QQuickVTKRenderItem.h>
#include <QQuickVTKRenderWindow.h>
#include <QQuickWindow>
#include <QUrl>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/common/shapes.h>

#include <QApplication>
#include <QQmlApplicationEngine>
// #include <QVTKRenderWindowInteractor.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkRenderer.h>

int main(int argc, char *argv[]) {

  /*

    QQuickVTKRenderWindow::setupGraphicsBackend();
    QGuiApplication app(argc, argv);

    QQmlApplicationEngine engine;
    engine.addImportPath("/home/behnam/usr/lib/qml");
    // engine.addImportPath("/home/behnam/foo/lib/qml");

    engine.load(QUrl("qrc:/qml/toolbox.qml"));

    QObject *topLevel = engine.rootObjects().value(0);
    QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

    window->show();

    // Fetch the QQuick window using the standard object name set up in the
    // constructor
    QQuickVTKRenderItem *qquickvtkItem =
        topLevel->findChild<QQuickVTKRenderItem *>("View");

    // Create a cone pipeline and add it to the view
    vtkNew<vtkActor> actor;
    vtkNew<vtkPolyDataMapper> mapper;

    qquickvtkItem->renderer()->ResetCamera();
    qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
    qquickvtkItem->renderer()->SetBackground2(0.7, 0.7, 0.7);
    qquickvtkItem->renderer()->SetGradientBackground(true);
    qquickvtkItem->update();
    app.exec();
    */

  QApplication app(argc, argv);

  // Create a QML engine
  QQmlApplicationEngine engine;
  engine.load(QUrl(QStringLiteral(
      "qrc:/qml/toolbox.qml"))); // Adjust the path to your QML file

  // Create a VTK render window and renderer
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
      vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(renderer);

  // Create a QVTKRenderWindowInteractor
  // QVTKRenderWindowInteractor *vtkWidget = new QVTKRenderWindowInteractor();
  // vtkWidget->SetRenderWindow(renderWindow);

  // Set up your main window layout and add the QML and VTK components
  // ...

  // Show your main window
  // ...

  return app.exec();
}
