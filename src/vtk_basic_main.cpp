// #include <QLoggingCategory>
// #include <QQuickVTKItem.h>
// #include <QQuickWindow>
// #include <QVTKRenderWindowAdapter.h>
// #include <QtGui/QGuiApplication>
// #include <QtGui/QSurfaceFormat>
// #include <QtQml/QQmlApplicationEngine>
// #include <QtQuick/QQuickWindow>
// #include <pdal/Options.hpp>
// #include <pdal/PointTable.hpp>
// #include <pdal/PointView.hpp>
// #include <pdal/StageFactory.hpp>
// #include <pdal/io/LasReader.hpp>
// #include <vtkActor.h>
// #include <vtkCallbackCommand.h>
// #include <vtkCommand.h>
// #include <vtkConeSource.h>
// #include <vtkInteractorStyleTrackballCamera.h>
// #include <vtkMinimalStandardRandomSequence.h>
// #include <vtkNamedColors.h>
// #include <vtkNew.h>
// #include <vtkPointData.h>
// #include <vtkPoints.h>
// #include <vtkPolyData.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkPropPicker.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSliderRepresentation2D.h>
// #include <vtkSliderWidget.h>
// #include <vtkSmartPointer.h>
// #include <vtkSphereSource.h>
// #include <vtkVertexGlyphFilter.h>

// void CallbackFunction(vtkObject *caller, long unsigned int eventId,
//                       void *vtkNotUsed(clientData),
//                       void *vtkNotUsed(callData)) {
//   vtkRenderer *renderer = static_cast<vtkRenderer *>(caller);

//   double timeInSeconds = renderer->GetLastRenderTimeInSeconds();
//   double fps = 1.0 / timeInSeconds;
//   std::cout << "FPS: " << fps << std::endl;

//   std::cout << "Callback" << std::endl;
//   std::cout << "eventId: " << eventId << std::endl;
// }
// struct MyVtkItem : public QQuickVTKItem {

//   vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {

//     // // Create a cone pipeline and add it to the view
//     // vtkNew<vtkConeSource> cone;

//     // vtkNew<vtkPolyDataMapper> mapper;
//     // mapper->SetInputConnection(cone->GetOutputPort());

//     // vtkNew<vtkActor> actor;
//     // actor->SetMapper(mapper);

//     // Read LAS File Using PDAL

//     pdal::PointTable table;
//     pdal::Options options;

//     // options.add(
//     //     "filename",
//     //
//     "/home/behnam/workspace/vtk_projects/data/las_files/Palac_Moszna.laz");

//     options.add("filename", "/home/behnam/map.las");

//     pdal::StageFactory factory;
//     pdal::Stage *reader = factory.createStage("readers.las");
//     reader->setOptions(options);
//     reader->prepare(table);

//     pdal::PointViewSet pointViews = reader->execute(table);
//     pdal::PointViewPtr pointView = *pointViews.begin();

//     // Convert PDAL Data to VTK Format
//     // Check if the GpsTime dimension is available
//     bool hasGpsTime = pointView->hasDim(pdal::Dimension::Id::GpsTime);

//     vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
//     points->SetNumberOfPoints(pointView->size());

//     // points->Resize()
//     std::cout << " points->GetNumberOfPoints(): " <<
//     points->GetNumberOfPoints()
//               << std::endl;

//     for (pdal::PointId id = 0; id < pointView->size(); ++id) {
//       float x = pointView->getFieldAs<float>(pdal::Dimension::Id::X, id);
//       float y = pointView->getFieldAs<float>(pdal::Dimension::Id::Y, id);
//       float z = pointView->getFieldAs<float>(pdal::Dimension::Id::Z, id);
//       const float p[3] = {x, y, z};
//       points->InsertPoint(id, p);
//     }

//     vtkSmartPointer<vtkPolyData> polyData =
//     vtkSmartPointer<vtkPolyData>::New(); polyData->SetPoints(points);

//     vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
//         vtkSmartPointer<vtkVertexGlyphFilter>::New();
//     glyphFilter->SetInputData(polyData);
//     glyphFilter->Update();

//     // Visualize with VTK

//     vtkSmartPointer<vtkPolyDataMapper> mapper =
//         vtkSmartPointer<vtkPolyDataMapper>::New();
//     mapper->SetInputConnection(glyphFilter->GetOutputPort());

//     vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//     actor->SetMapper(mapper);

//     vtkNew<vtkRenderer> renderer;
//     renderer->AddActor(actor);
//     renderer->ResetCamera();
//     // renderer->SetBackground(0.0, 1.0, 1.0);
//     // renderer->SetBackground2(1.0, 0.0, 0.0);
//     // renderer->SetGradientBackground(true);

//     renderWindow->AddRenderer(renderer);
//     // renderWindow->SetMultiSamples(16);

//     vtkNew<vtkCallbackCommand> callback;

//     callback->SetCallback(CallbackFunction);
//     renderer->AddObserver(vtkCommand::EndEvent, callback);

//     return nullptr;
//   }

//   bool event(QEvent *event) override {

//     qDebug() << "-------------------------";

//     QEvent::Type type = event->type();

//     switch (type) {
//     case QEvent::None:
//       qDebug() << "None";
//     case QEvent::Timer:
//       qDebug() << "Timer";
//     case QEvent::MouseButtonPress:
//       qDebug() << "MouseButtonPress";
//     case QEvent::MouseButtonRelease:
//       qDebug() << "MouseButtonRelease";
//     case QEvent::MouseButtonDblClick:
//       qDebug() << "MouseButtonDblClick";
//     case QEvent::MouseMove:
//       qDebug() << "MouseMove";
//     case QEvent::KeyPress:
//       qDebug() << "KeyPress";
//     case QEvent::KeyRelease:
//       qDebug() << "KeyRelease";
//     case QEvent::FocusIn:
//       qDebug() << "FocusIn";
//     case QEvent::FocusOut:
//       qDebug() << "FocusOut";
//     case QEvent::Enter:
//       qDebug() << "Enter";
//     case QEvent::Leave:
//       qDebug() << "Leave";
//     case QEvent::Paint:
//       qDebug() << "Paint";
//     case QEvent::Move:
//       qDebug() << "Move";
//     case QEvent::Resize:
//       qDebug() << "Resize";
//     case QEvent::Close:
//       qDebug() << "Close";
//     // Add other cases as needed
//     default:
//       qDebug() << QString("Unknown(%1)").arg(static_cast<int>(type));
//     }

//     return QQuickVTKItem::event(event);
//   }
//   QSGNode *updatePaintNode(QSGNode *q, UpdatePaintNodeData *u) override {

//     std::cout << "updatePaintNode" << std::endl;
//     return QQuickVTKItem::updatePaintNode(q, u);
//   }

//   bool isTextureProvider() const override {
//     std::cout << "isTextureProvider" << std::endl;

//     return QQuickVTKItem::isTextureProvider();
//   }
//   // QSGTextureProvider* textureProvider() const override{}
//   // void releaseResources() override{}
// };

// int main(int argc, char *argv[]) {
//   QQuickVTKItem::setGraphicsApi();

// #if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
//   QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
// #endif

//   QGuiApplication app(argc, argv);

//   // QQuickWindow::setPersistentSceneGraph(false);
//   // QQuickWindow::setPersistentGraphics(false);

//   //   // Set the scene graph backend to software rendering (optional,
//   depending
//   //   on your needs)
//   // QQuickWindow::setSceneGraphBackend(QSGRendererInterface::Software);

//   // QQuickWindow::setPersistentSceneGraph(false);

//   // // Set the render loop to "OnDemand" to avoid continuous rendering
//   // QQuickWindow::setRenderLoop(QQuickWindow::RenderLoop::OnDemand);

//   qmlRegisterType<MyVtkItem>("com.vtk.example", 1, 0, "MyVtkItem");

//   QQmlApplicationEngine engine;
//   //   engine.load(QUrl(QStringLiteral("qml/vtk_basic.qml")));
//   engine.addImportPath("/home/behnam/usr/lib/qml");
//   engine.load(QUrl("qrc:/qml/vtk_basic.qml"));
//   if (engine.rootObjects().isEmpty())
//     return -1;

//   // Check if the engine has loaded the root objects correctly
//   if (engine.rootObjects().isEmpty())
//     return -1;

//   // Retrieve the root object (typically a QQuickWindow)
//   QObject *rootObject = engine.rootObjects().first();
//   QQuickWindow *window = qobject_cast<QQuickWindow *>(rootObject);

//   // Apply the persistent scene graph and graphics settings
//   if (window) {
//     std::cout << "settings" << std::endl;
//     window->setPersistentSceneGraph(false);
//     window->setPersistentOpenGLContext(false);

//     QObject::connect(window, &QQuickWindow::beforeRendering, [=]() {
//       // QMessageBox msgBox;
//       // msgBox.setText(QString::number(0));
//       // msgBox.exec();
//       std::cout << "beforeRendering" << std::endl;
//       ;
//     });

//     // window->setPersistentGraphics(false);
//   }

//   return app.exec();
// }

// // 567dadd312b21204d993858892dadad5561815c3

// ./MinimalQtVTKApp
// ./vtk_basic
// ./multi_interactor
// ./las_time_based_filter
// ./FrameRate

#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQuickWindow>
#include <QSGRendererInterface>

#include <QVTKRenderWindowAdapter.h>

#include "MyVtkItem.hpp"

int main(int argc, char *argv[]) {
#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QQuickWindow::setSceneGraphBackend(QSGRendererInterface::OpenGLRhi);
#else
  QQuickWindow::setGraphicsApi(QSGRendererInterface::OpenGLRhi);
#endif
  QSurfaceFormat::setDefaultFormat(QVTKRenderWindowAdapter::defaultFormat());

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif
  QGuiApplication app(argc, argv);

  qmlRegisterType<MyVtkItem>("Vtk", 1, 0, "MyVtkItem");

  QQmlApplicationEngine engine;
  const QUrl url(QStringLiteral("qrc:/qml/vtk_basic.qml"));
  QObject::connect(
      &engine, &QQmlApplicationEngine::objectCreated, &app,
      [url](QObject *obj, const QUrl &objUrl) {
        if (!obj && url == objUrl)
          QCoreApplication::exit(-1);
      },
      Qt::QueuedConnection);
  engine.load(url);

  return app.exec();
}
