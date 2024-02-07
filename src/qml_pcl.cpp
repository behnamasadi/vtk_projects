#include "QQuickVTKRenderItem.h"
#include "QQuickVTKRenderWindow.h"

#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkAxisActor.h>
#include <vtkCleanPolyData.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkStructuredGrid.h>
#include <vtkTransform.h>

#include <QApplication>
#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QTimer>
#include <QUrl>

#include <pcl/io/vtk_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/common/shapes.h>

// https://pointclouds.org/documentation/vtk__lib__io_8hpp_source.html
// https://pointclouds.org/documentation/vtk__lib__io_8h_source.html
// https://vtk.org/doc/nightly/html/classQQmlVTKPlugin.html#details

int main(int argc, char *argv[]) {

  QQuickVTKRenderWindow::setupGraphicsBackend();
  QGuiApplication app(argc, argv);

  QQmlApplicationEngine engine;

  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl("qrc:/qml/qml_pcl.qml"));

  QObject *topLevel = engine.rootObjects().value(0);
  QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

  window->show();

  // Fetch the QQuick window using the standard object name set up in the
  // constructor
  QQuickVTKRenderItem *qquickvtkItem =
      topLevel->findChild<QQuickVTKRenderItem *>("View");

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;

  for (int i = 0; i < 10000; i++) {

    point.x = rand() * 10 - 5;
    point.y = rand() * 10 - 5;
    point.z = rand() * 10 - 5;
    cloud.points.push_back(point);
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

  pcl::io::pointCloudTovtkPolyData(cloud, polydata);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polydata);

  vtkNew<vtkActor> actor;
  //  vtkNew<vtkAxisActor> actor;

  actor->SetMapper(mapper);
  // actor->DrawGridlinesOnlyOn();
  // actor->DrawGridlinesOn();

  vtkNew<vtkTransform> transform;
  transform->Translate(1.0, 0.0, 0.0);

  vtkNew<vtkAxesActor> axes;

  // The axes are positioned with a user transform
  axes->SetUserTransform(transform);

  // properties of the axes labels can be set as follows
  // this sets the x axis label to red
  // axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(
  //   colors->GetColor3d("Red").GetData());

  // the actual text of the axis label can be changed:
  // axes->SetXAxisLabelText("test");

  qquickvtkItem->renderer()->AddActor(actor);
  qquickvtkItem->renderer()->AddActor(axes);

  qquickvtkItem->renderer()->ResetCamera();
  qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
  qquickvtkItem->renderer()->SetBackground2(0.7, 0.7, 0.7);
  qquickvtkItem->renderer()->SetGradientBackground(true);
  qquickvtkItem->update();
  app.exec();
}
