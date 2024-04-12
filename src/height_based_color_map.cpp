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
    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    RecalculateColors();
    vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  virtual void OnMiddleButtonUp() {
    std::cout << "Released middle mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    vtkInteractorStyleTrackballActor::OnMiddleButtonUp();
  }

  void SetPoints(vtkSmartPointer<vtkPoints> points) { this->Points = points; }

  void SetPointPolyData(vtkSmartPointer<vtkPolyData> pointPolyData) {
    this->PointPolyData = pointPolyData;
  }

  void SetLookupTable(vtkSmartPointer<vtkLookupTable> lut) {
    this->LookupTable = lut;
  }
  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }

private:
  vtkSmartPointer<vtkActor> Actor;
  vtkSmartPointer<vtkPoints> Points;
  vtkSmartPointer<vtkLookupTable> LookupTable;
  vtkSmartPointer<vtkPolyData> PointPolyData;

  void RecalculateColors() {
    if (!this->Actor || !this->Points || !this->LookupTable)
      return;

    vtkNew<vtkTransform> transform;
    transform->SetMatrix(this->Actor->GetMatrix());

    // Variables to hold the minimum and maximum z values
    double minZ = std::numeric_limits<double>::max();
    double maxZ = std::numeric_limits<double>::lowest();
    // Iterate through all the points to find min and max z values

    double yMin = std::numeric_limits<double>::max(),
           yMax = std::numeric_limits<double>::lowest();

    // Generate the colors for each point based on the Z value
    vtkSmartPointer<vtkUnsignedCharArray> colors =
        vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(
        3); // 3 components (R, G, B), we can use 4 for RGBA
    colors->SetName("Colors");

    for (vtkIdType i = 0; i < this->Points->GetNumberOfPoints(); i++) {
      double p[3];                  // Array to hold the point coordinates
      this->Points->GetPoint(i, p); // Get the i-th point's coordinates

      transform->TransformPoint(
          p, p); // Apply the actor's current transformation matrix

      double z = p[2]; // Get the z-value

      double dColor[3];
      unsigned char color[3];

      LookupTable->GetColor(z, dColor); // Get the color as double[3]

      // Convert double color to unsigned char
      color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
      color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
      color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

      colors->InsertNextTypedTuple(color);

      // this->Points->SetPoint(i, p);

      // Compare and update min and max z values
      if (p[2] < minZ) {
        minZ = p[2];
      }
      if (p[2] > maxZ) {
        maxZ = p[2];
      }
    }

    // Attach the colors to the point poly data
    PointPolyData->GetPointData()->SetScalars(colors);

    // // Print the results
    std::cout << "Minimum z-value: " << minZ << std::endl;
    std::cout << "Maximum z-value: " << maxZ << std::endl;

    std::cout << "maxZ-minZ: " << maxZ - minZ << std::endl;
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

  double yMin = std::numeric_limits<double>::max(),
         yMax = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);
    if (z < zMin)
      zMin = z;
    if (z > zMax)
      zMax = z;

    if (y < yMin)
      yMin = y;
    if (y > yMax)
      yMax = y;
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

  // lookupTable->SetRange(
  //     zMin, zMax); // Set the range based on the min and max z-values

  lookupTable->SetRange(
      yMin, yMax); // Set the range based on the min and max z-values

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
    // double z = p[2]; // Get the z-value
    double y = p[1]; // Get the z-value

    // lookupTable->GetColor(z, dColor); // Get the color as double[3]
    lookupTable->GetColor(y, dColor); // Get the color as double[3]

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
  style->SetPoints(points);
  style->SetLookupTable(lookupTable);
  style->SetPointPolyData(pointPolyData);

  renderWindowInteractor->SetInteractorStyle(style);

  // Add the actor to the scene
  renderer->AddActor(actor);

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}