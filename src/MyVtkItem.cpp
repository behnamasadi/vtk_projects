#include "MyVtkItem.hpp"

#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

#include <vtkActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

struct MyVtkData : vtkObject {
  static MyVtkData *New();
  vtkTypeMacro(MyVtkData, vtkObject);

  // Place all your persistant VTK objects here
};

vtkStandardNewMacro(MyVtkData);

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

QQuickVtkItem::vtkUserData
MyVtkItem::initializeVTK(vtkRenderWindow *renderWindow) {
  auto vtk = vtkNew<MyVtkData>();

  //////////////////////////////////////////////////////////////////////////////////

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

  ////////////////////////////////////////////////////////////////////////////

  vtkNew<vtkNamedColors> colors;

  int numberOfSpheres = 10;

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(actor);

  vtkNew<vtkCallbackCommand> callback;

  callback->SetCallback(CallbackFunction);
  renderer->AddObserver(vtkCommand::EndEvent, callback);
  // remove    vtkNew<vtkRenderWindow> renderWindow;
  //   renderWindow->SetSize(640, 480);
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("HighlightPickedActor");

  return vtk;
}
