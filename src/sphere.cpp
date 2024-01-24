#include "QQuickVTKRenderItem.h"
#include "QQuickVTKRenderWindow.h"
#include "vtkActor.h"
#include "vtkColorTransferFunction.h"
#include "vtkConeSource.h"
#include "vtkGenericOpenGLRenderWindow.h"
#include "vtkGlyph3DMapper.h"
#include "vtkNew.h"
#include "vtkPNGWriter.h"
#include "vtkPiecewiseFunction.h"
#include "vtkPolyDataMapper.h"
#include "vtkProperty.h"
#include "vtkRenderer.h"
// #include "vtkSmartVolumeMapper.h"
#include "vtkSphereSource.h"
#include "vtkTestUtilities.h"
#include "vtkTesting.h"
#include "vtkVolume.h"
#include "vtkVolumeProperty.h"
#include "vtkWindowToImageFilter.h"
#include "vtkXMLImageDataReader.h"

#include <vtkActor.h>
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

#include <QApplication>
#include <QDebug>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QTimer>
#include <QUrl>

#include <pcl/features/normal_3d.h>
#include <pcl/io/pcd_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/point_types.h>
#include <pcl/surface/gp3.h>
#include <pcl/visualization/common/shapes.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/visualization/common/shapes.h>

int main(int argc, char *argv[]) {

  QQuickVTKRenderWindow::setupGraphicsBackend();
  QGuiApplication app(argc, argv);

  QQmlApplicationEngine engine;

  engine.addImportPath("/home/behnam/usr/lib/qml");
  // engine.addImportPath("/home/behnam/foo/lib/qml");

  engine.load(QUrl("qrc:/qml/sphere.qml"));

  QObject *topLevel = engine.rootObjects().value(0);
  QQuickWindow *window = qobject_cast<QQuickWindow *>(topLevel);

  window->show();

  // Fetch the QQuick window using the standard object name set up in the
  // constructor
  QQuickVTKRenderItem *qquickvtkItem =
      topLevel->findChild<QQuickVTKRenderItem *>("View");

  ////////////////////////////////////////////////////////////////////////////////////////////
  // The following are given (or computed using sample consensus techniques --
  // see SampleConsensusModelSphere)
  Eigen::Vector3f sphere_center;

  sphere_center[0] = 1;
  sphere_center[1] = 1;
  sphere_center[2] = 1;

  pcl::ModelCoefficients sphere_coeff;
  sphere_coeff.values.resize(4); // We need 4 values
  sphere_coeff.values[0] = sphere_center[0];
  sphere_coeff.values[1] = sphere_center[1];
  sphere_coeff.values[2] = sphere_center[2];

  double radius = 2;
  int resolution = 200;

  sphere_coeff.values[3] = radius;

  vtkSmartPointer<vtkDataSet> data =
      pcl::visualization::createSphere(sphere_coeff, resolution);
  vtkNew<vtkPointSource> pointSource;

  pointSource->AddInputData(data);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(pointSource->GetOutputPort());

  vtkNew<vtkActor> actor;

  actor->SetMapper(mapper);

  qquickvtkItem->renderer()->AddActor(actor);
  qquickvtkItem->renderer()->ResetCamera();
  qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
  qquickvtkItem->renderer()->SetBackground2(0.7, 0.7, 0.7);
  qquickvtkItem->renderer()->SetGradientBackground(true);
  qquickvtkItem->update();
  app.exec();
}
