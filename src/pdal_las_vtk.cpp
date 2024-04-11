#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>

#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkVertexGlyphFilter.h>

#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>

void displayLASFileUsingPDAL() {

  // Read LAS File Using PDAL

  pdal::PointTable table;
  pdal::Options options;
  options.add(
      "filename",
      "/home/behnam/workspace/vtk_projects/data/las_files/Palac_Moszna.laz");

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

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);

    if (hasGpsTime) {
      double time =
          pointView->getFieldAs<double>(pdal::Dimension::Id::GpsTime, id);
      // Process time as needed, for example, store it in an array or print it
      // std::cout << time << std::endl;
    }
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

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->SetBackground(.3, .2, .1);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();
  renderWindowInteractor->Start();
}

void readLASFileUsingPDAL() {
  pdal::Options options;
  options.add(
      "filename",
      "/home/behnam/workspace/vtk_projects/data/las_files/Palac_Moszna.laz");
  pdal::LasReader reader;
  reader.setOptions(options);
  pdal::PointTable table;

  reader.prepare(table);
  pdal::PointViewSet viewSet = reader.execute(table);

  for (auto const &view : viewSet) {
    for (size_t i = 0; i < view->size(); ++i) {
      double x = view->getFieldAs<double>(pdal::Dimension::Id::X, i);
      double y = view->getFieldAs<double>(pdal::Dimension::Id::Y, i);
      double z = view->getFieldAs<double>(pdal::Dimension::Id::Z, i);
      // Use x, y, z as needed
    }
  }
}

int main() { displayLASFileUsingPDAL(); }