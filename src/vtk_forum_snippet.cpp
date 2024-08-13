#include <QQuickVTKItem.h>
#include <QQuickWindow>
#include <QVTKRenderWindowAdapter.h>
#include <QtGui/QGuiApplication>
#include <QtGui/QSurfaceFormat>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickWindow>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>

void CallbackFunction(vtkObject *caller, long unsigned int eventId,
                      void *vtkNotUsed(clientData),
                      void *vtkNotUsed(callData)) {
  vtkRenderer *renderer = static_cast<vtkRenderer *>(caller);

  double timeInSeconds = renderer->GetLastRenderTimeInSeconds();
  double fps = 1.0 / timeInSeconds;
  std::cout << "FPS: " << fps << std::endl;

  std::cout << "Callback" << std::endl;
  std::cout << "eventId: " << eventId << std::endl;
}

struct MyVtkItem : public QQuickVTKItem {

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {

    // Create a cone pipeline and add it to the view
    vtkNew<vtkConeSource> cone;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(cone->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);

    vtkNew<vtkRenderer> renderer;
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderWindow->AddRenderer(renderer);
    vtkNew<vtkCallbackCommand> callback;
    callback->SetCallback(CallbackFunction);
    renderer->AddObserver(vtkCommand::EndEvent, callback);
    return nullptr;
  }

  bool event(QEvent *event) override {

    qDebug() << "-------------------------";

    QEvent::Type type = event->type();

    switch (type) {
    case QEvent::None:
      qDebug() << "None";
    case QEvent::Timer:
      qDebug() << "Timer";
    case QEvent::MouseButtonPress:
      qDebug() << "MouseButtonPress";
    case QEvent::MouseButtonRelease:
      qDebug() << "MouseButtonRelease";
    case QEvent::MouseButtonDblClick:
      qDebug() << "MouseButtonDblClick";
    case QEvent::MouseMove:
      qDebug() << "MouseMove";
    case QEvent::KeyPress:
      qDebug() << "KeyPress";
    case QEvent::KeyRelease:
      qDebug() << "KeyRelease";
    case QEvent::FocusIn:
      qDebug() << "FocusIn";
    case QEvent::FocusOut:
      qDebug() << "FocusOut";
    case QEvent::Enter:
      qDebug() << "Enter";
    case QEvent::Leave:
      qDebug() << "Leave";
    case QEvent::Paint:
      qDebug() << "Paint";
    case QEvent::Move:
      qDebug() << "Move";
    case QEvent::Resize:
      qDebug() << "Resize";
    case QEvent::Close:
      qDebug() << "Close";
    // Add other cases as needed
    default:
      qDebug() << QString("Unknown(%1)").arg(static_cast<int>(type));
    }

    return QQuickVTKItem::event(event);
  }
  QSGNode *updatePaintNode(QSGNode *q, UpdatePaintNodeData *u) override {

    std::cout << "updatePaintNode" << std::endl;
    return QQuickVTKItem::updatePaintNode(q, u);
  }

  bool isTextureProvider() const override {
    std::cout << "isTextureProvider" << std::endl;

    return QQuickVTKItem::isTextureProvider();
  }
  // QSGTextureProvider* textureProvider() const override{}
  // void releaseResources() override{}
};

int main(int argc, char *argv[]) {
  QQuickVTKItem::setGraphicsApi();

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QGuiApplication app(argc, argv);
  qmlRegisterType<MyVtkItem>("com.vtk.example", 1, 0, "MyVtkItem");

  QQmlApplicationEngine engine;
  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl("qrc:/qml/vtk_basic.qml"));
  if (engine.rootObjects().isEmpty())
    return -1;

  return app.exec();
}

// 567dadd312b21204d993858892dadad5561815c3