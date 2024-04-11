#include <iostream>
#include <limits>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <vtkActor.h>
#include <vtkCellData.h>
#include <vtkColorTransferFunction.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkFloatArray.h>
#include <vtkGlyph3D.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLookupTable.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkObjectFactory.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkScalarBarActor.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkStructuredGrid.h>
#include <vtkStructuredGridGeometryFilter.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

int lasHeightMapDisplay(int, char *[]) {

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

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  double zMin = std::numeric_limits<double>::max(),
         zMax = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);
    if (z < zMin)
      zMin = z;
    if (z > zMax)
      zMax = z;
  }

  vtkSmartPointer<vtkPolyData> pointPolyData =
      vtkSmartPointer<vtkPolyData>::New();
  pointPolyData->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(pointPolyData);
  glyphFilter->Update();

  // Create a lookup table to map point colors
  vtkSmartPointer<vtkLookupTable> lookupTable =
      vtkSmartPointer<vtkLookupTable>::New();

  lookupTable->SetRange(
      zMin, zMax); // Set the range based on the min and max z-values

  lookupTable->SetHueRange(0.667,
                           0); // Set hue range from blue (0.667) to red (0)

  lookupTable->Build();

  // Generate the colors for each point based on the Z value
  vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(
      3); // 3 components (R, G, B), we can use 4 for RGBA
  colors->SetName("Colors");

  double p[3], dColor[3];
  unsigned char color[3];
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    points->GetPoint(i, p);
    double z = p[2]; // Get the z-value

    lookupTable->GetColor(z, dColor); // Get the color as double[3]

    // Convert double color to unsigned char
    color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
    color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
    color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

    colors->InsertNextTypedTuple(color);
  }

  // Attach the colors to the point poly data
  pointPolyData->GetPointData()->SetScalars(colors);

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

  return EXIT_SUCCESS;
}

/*

namespace {

// Define interaction style
class MyInteractorStyle : public vtkInteractorStyleTrackballActor {
public:
  static MyInteractorStyle *New();
  vtkTypeMacro(MyInteractorStyle, vtkInteractorStyleTrackballActor);

  virtual void OnLeftButtonDown() {
    std::cout << "Pressed left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballActor::OnLeftButtonDown();
  }

  virtual void OnLeftButtonUp() {
    std::cout << "Released left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }

private:
  vtkSmartPointer<vtkActor> Actor;
};
vtkStandardNewMacro(MyInteractorStyle);

} // namespace

int main(int, char *[]) {

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

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  double zMin = std::numeric_limits<double>::max(),
         zMax = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);
    if (z < zMin)
      zMin = z;
    if (z > zMax)
      zMax = z;
  }

  vtkSmartPointer<vtkPolyData> pointPolyData =
      vtkSmartPointer<vtkPolyData>::New();
  pointPolyData->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(pointPolyData);
  glyphFilter->Update();

  // Create a lookup table to map point colors
  vtkSmartPointer<vtkLookupTable> lookupTable =
      vtkSmartPointer<vtkLookupTable>::New();

  lookupTable->SetRange(
      zMin, zMax); // Set the range based on the min and max z-values

  lookupTable->SetHueRange(0.667,
                           0); // Set hue range from blue (0.667) to red (0)

  lookupTable->Build();

  // Generate the colors for each point based on the Z value
  vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(
      3); // 3 components (R, G, B), we can use 4 for RGBA
  colors->SetName("Colors");

  double p[3], dColor[3];
  unsigned char color[3];
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    points->GetPoint(i, p);
    double z = p[2]; // Get the z-value

    lookupTable->GetColor(z, dColor); // Get the color as double[3]

    // Convert double color to unsigned char
    color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
    color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
    color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

    colors->InsertNextTypedTuple(color);
  }

  // Attach the colors to the point poly data
  pointPolyData->GetPointData()->SetScalars(colors);

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyphFilter->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // Create a renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("RotateActor");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<MyInteractorStyle> style;
  style->SetActor(actor);

  renderWindowInteractor->SetInteractorStyle(style);

  // Add the actor to the scene
  renderer->AddActor(actor);

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
*/

namespace {

class MyInteractorStyle : public vtkInteractorStyleTrackballActor {
public:
  static MyInteractorStyle *New();
  vtkTypeMacro(MyInteractorStyle, vtkInteractorStyleTrackballActor);

  virtual void OnLeftButtonDown() {
    std::cout << "Pressed left mouse button." << std::endl;
    vtkInteractorStyleTrackballActor::OnLeftButtonDown();
  }

  virtual void OnLeftButtonUp() {
    std::cout << "Released left mouse button." << std::endl;
    vtkInteractorStyleTrackballActor::OnLeftButtonUp();

    RecalculateColors();
  }

  void SetPoints(vtkSmartPointer<vtkPoints> points) { this->Points = points; }
  void SetLookupTable(vtkSmartPointer<vtkLookupTable> lut) {
    this->LookupTable = lut;
  }
  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }

private:
  vtkSmartPointer<vtkActor> Actor;
  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkLookupTable> LookupTable;

  void RecalculateColors() {
    if (!this->Actor || !this->Points || !this->LookupTable)
      return;

    vtkSmartPointer<vtkUnsignedCharArray> colors =
        vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    double p[3], dColor[3];
    unsigned char color[3];
    for (vtkIdType i = 0; i < this->Points->GetNumberOfPoints(); i++) {
      this->Points->GetPoint(i, p);
      this->LookupTable->GetColor(p[2], dColor); // Use Z for color mapping

      color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
      color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
      color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

      colors->InsertNextTypedTuple(color);
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    polyData->SetPoints(this->Points);
    polyData->GetPointData()->SetScalars(colors);

    vtkPolyDataMapper *polyDataMapper =
        vtkPolyDataMapper::SafeDownCast(this->Actor->GetMapper());
    if (polyDataMapper) {
      polyDataMapper->SetInputData(polyData);
      polyDataMapper->Update();
    } else {
      std::cerr << "Error: Mapper is not a vtkPolyDataMapper!" << std::endl;
    }
  }
};

vtkStandardNewMacro(MyInteractorStyle);

} // namespace

int main(int, char *[]) {

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

  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  double zMin = std::numeric_limits<double>::max(),
         zMax = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);
    if (z < zMin)
      zMin = z;
    if (z > zMax)
      zMax = z;
  }

  vtkSmartPointer<vtkPolyData> pointPolyData =
      vtkSmartPointer<vtkPolyData>::New();
  pointPolyData->SetPoints(points);

  vtkSmartPointer<vtkVertexGlyphFilter> glyphFilter =
      vtkSmartPointer<vtkVertexGlyphFilter>::New();
  glyphFilter->SetInputData(pointPolyData);
  glyphFilter->Update();

  // Create a lookup table to map point colors
  vtkSmartPointer<vtkLookupTable> lookupTable =
      vtkSmartPointer<vtkLookupTable>::New();

  lookupTable->SetRange(
      zMin, zMax); // Set the range based on the min and max z-values

  lookupTable->SetHueRange(0.667,
                           0); // Set hue range from blue (0.667) to red (0)

  lookupTable->Build();

  // Generate the colors for each point based on the Z value
  vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(
      3); // 3 components (R, G, B), we can use 4 for RGBA
  colors->SetName("Colors");

  double p[3], dColor[3];
  unsigned char color[3];
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    points->GetPoint(i, p);
    double z = p[2]; // Get the z-value

    lookupTable->GetColor(z, dColor); // Get the color as double[3]

    // Convert double color to unsigned char
    color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
    color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
    color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

    colors->InsertNextTypedTuple(color);
  }

  // Attach the colors to the point poly data
  pointPolyData->GetPointData()->SetScalars(colors);

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyphFilter->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // Create a renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("RotateActor");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<MyInteractorStyle> style;
  style->SetActor(actor);
  style->SetPoints(points); // Pass points to the interactor style
  style->SetLookupTable(
      lookupTable); // Pass the lookup table to the interactor style

  renderWindowInteractor->SetInteractorStyle(style);

  // Add the actor to the scene
  renderer->AddActor(actor);

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}