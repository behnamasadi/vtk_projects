//#include "vtk_basic.hpp"

#include <QQuickVTKItem.h>
#include <QVTKRenderWindowAdapter.h>
#include <QtGui/QGuiApplication>
#include <QtGui/QSurfaceFormat>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickWindow>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
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
struct MyVtkItem : QQuickVTKItem {

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {
    // // Create a cone pipeline and add it to the view
    // vtkNew<vtkConeSource> cone;

    // vtkNew<vtkPolyDataMapper> mapper;
    // mapper->SetInputConnection(cone->GetOutputPort());

    // vtkNew<vtkActor> actor;
    // actor->SetMapper(mapper);

    // Read LAS File Using PDAL

    pdal::PointTable table;
    pdal::Options options;

    // options.add(
    //     "filename",
    //     "/home/behnam/workspace/vtk_projects/data/las_files/Palac_Moszna.laz");

    options.add("filename", "/home/behnam/map.las");

    pdal::StageFactory factory;
    pdal::Stage *reader = factory.createStage("readers.las");
    reader->setOptions(options);
    reader->prepare(table);

    pdal::PointViewSet pointViews = reader->execute(table);
    pdal::PointViewPtr pointView = *pointViews.begin();

    // Convert PDAL Data to VTK Format
    // Check if the GpsTime dimension is available
    bool hasGpsTime = pointView->hasDim(pdal::Dimension::Id::GpsTime);

    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    points->SetNumberOfPoints(pointView->size());

    // points->Resize()
    std::cout << " points->GetNumberOfPoints(): " << points->GetNumberOfPoints()
              << std::endl;

    for (pdal::PointId id = 0; id < pointView->size(); ++id) {
      float x = pointView->getFieldAs<float>(pdal::Dimension::Id::X, id);
      float y = pointView->getFieldAs<float>(pdal::Dimension::Id::Y, id);
      float z = pointView->getFieldAs<float>(pdal::Dimension::Id::Z, id);
      const float p[3] = {x, y, z};
      points->InsertPoint(id, p);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(points);

    vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
        vtkSmartPointer<vtkVertexGlyphFilter>::New();
    glyphFilter->SetInputData(polyData);
    glyphFilter->Update();

    // Visualize with VTK

    vtkSmartPointer<vtkPolyDataMapper> mapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    mapper->SetInputConnection(glyphFilter->GetOutputPort());

    vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
    actor->SetMapper(mapper);

    vtkNew<vtkRenderer> renderer;
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(0.0, 1.0, 1.0);
    renderer->SetBackground2(1.0, 0.0, 0.0);
    renderer->SetGradientBackground(true);

    renderWindow->AddRenderer(renderer);
    // renderWindow->SetMultiSamples(16);

    vtkNew<vtkCallbackCommand> callback;

    callback->SetCallback(CallbackFunction);
    renderer->AddObserver(vtkCommand::EndEvent, callback);
    // renderer->AddObserver(vtkCommand::RenderEvent, callback);

    return nullptr;
  }
};

int main(int argc, char *argv[]) {
  QQuickVTKItem::setGraphicsApi();

#if QT_VERSION < QT_VERSION_CHECK(6, 0, 0)
  QCoreApplication::setAttribute(Qt::AA_EnableHighDpiScaling);
#endif

  QGuiApplication app(argc, argv);

  qmlRegisterType<MyVtkItem>("com.vtk.example", 1, 0, "MyVtkItem");

  QQmlApplicationEngine engine;
  //   engine.load(QUrl(QStringLiteral("qml/vtk_basic.qml")));
  engine.addImportPath("/home/behnam/usr/lib/qml");
  engine.load(QUrl("qrc:/qml/vtk_basic.qml"));
  if (engine.rootObjects().isEmpty())
    return -1;

  return app.exec();
}
