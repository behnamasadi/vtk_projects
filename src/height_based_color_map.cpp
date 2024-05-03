#include <iostream>
#include <limits>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkCaptionActor2D.h>
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
#include <vtkOrientationMarkerWidget.h>
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
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkUnsignedCharArray.h>
#include <vtkVertexGlyphFilter.h>

namespace {

class MyInteractorStyle : public vtkInteractorStyleTrackballActor {
public:
  static MyInteractorStyle *New();
  vtkTypeMacro(MyInteractorStyle, vtkInteractorStyleTrackballActor);

  virtual void OnLeftButtonUp() {
    std::cout << "Released left mouse button." << std::endl;

    // RecalculateColors();
    calculateColors();
    vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  void SetPointPolyData(vtkSmartPointer<vtkPolyData> pointPolyData) {
    this->ppData = pointPolyData;
  }
  void SetLookupTable(vtkSmartPointer<vtkLookupTable> lut) { this->lut = lut; }
  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }

private:
  vtkSmartPointer<vtkActor> Actor;
  vtkSmartPointer<vtkLookupTable> lut;
  vtkSmartPointer<vtkPolyData> ppData;

  void RecalculateColors() {
    if (!this->Actor || !this->ppData || !this->lut)
      return;

    vtkNew<vtkMatrix4x4> matrix;
    vtkNew<vtkTransform> transform;

    this->Actor->GetMatrix(matrix);
    std::cout << "Matrix: " << std::endl << *matrix << std::endl;

    transform->SetMatrix(matrix); // Use actor's current transformation matrix

    vtkSmartPointer<vtkUnsignedCharArray> colors =
        vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors->SetNumberOfComponents(3);
    colors->SetName("Colors");

    double yMin = std::numeric_limits<double>::max(),
           yMax = std::numeric_limits<double>::lowest();

    double xMin = std::numeric_limits<double>::max(),
           xMax = std::numeric_limits<double>::lowest();

    double zMin = std::numeric_limits<double>::max(),
           zMax = std::numeric_limits<double>::lowest();

    vtkSmartPointer<vtkPoints> transformedPoints =
        vtkSmartPointer<vtkPoints>::New();

    for (vtkIdType i = 0; i < this->ppData->GetPoints()->GetNumberOfPoints();
         i++) {
      double point[3], transformedPoint[3];
      this->ppData->GetPoints()->GetPoint(i, point);

      double x, y, z;

      transform->TransformPoint(point, transformedPoint);
      transformedPoints->InsertNextPoint(transformedPoint);

      x = transformedPoint[0];
      y = transformedPoint[1];
      z = transformedPoint[2];

      if (y < yMin)
        yMin = y;
      if (y > yMax)
        yMax = y;

      if (z < zMin)
        zMin = z;
      if (z > zMax)
        zMax = z;

      if (x < xMin)
        xMin = x;
      if (x > xMax)
        xMax = x;
    }

    std::cout << "yMin, yMax: " << std::fixed << yMin - yMin << ","
              << yMax - yMin << std::endl;

    this->lut->SetRange(yMin - yMin, yMax - yMin);
    this->lut->SetHueRange(0.667, 0); // Blue to red hue range
    this->lut->Build();

    for (vtkIdType i = 0; i < transformedPoints->GetNumberOfPoints(); i++) {
      double point[3];
      transformedPoints->GetPoint(i, point);

      double dColor[3];

      this->lut->GetColor(point[1] - yMin, dColor);

      unsigned char color[3] = {static_cast<unsigned char>(255.0 * dColor[0]),
                                static_cast<unsigned char>(255.0 * dColor[1]),
                                static_cast<unsigned char>(255.0 * dColor[2])};

      colors->InsertNextTypedTuple(color);
    }

    this->ppData->GetPointData()->SetScalars(colors);
  }

  void calculateColors() {
    if (!this->Actor || !this->ppData || !this->lut)
      return;

    vtkNew<vtkMatrix4x4> matrix;
    vtkNew<vtkTransform> transform;

    this->Actor->GetMatrix(matrix);
    std::cout << "Matrix: " << std::endl << *matrix << std::endl;

    transform->SetMatrix(matrix); // Use actor's current transformation matrix

    vtkNew<vtkFloatArray> scalars;

    this->ppData->GetPointData()->SetScalars(scalars);

    // apply the Color Map for height-based coloring
    vtkNew<vtkColorTransferFunction> colorTransferFunction;
    colorTransferFunction->AddRGBPoint(0.0, 1, 0, 0); // Red
    colorTransferFunction->AddRGBPoint(0.5, 0, 1, 0); // Green
    colorTransferFunction->AddRGBPoint(1.0, 0, 0, 1); // Blue

    double yMin = std::numeric_limits<double>::max(),
           yMax = std::numeric_limits<double>::lowest();

    double xMin = std::numeric_limits<double>::max(),
           xMax = std::numeric_limits<double>::lowest();

    double zMin = std::numeric_limits<double>::max(),
           zMax = std::numeric_limits<double>::lowest();

    vtkSmartPointer<vtkPoints> transformedPoints =
        vtkSmartPointer<vtkPoints>::New();

    for (vtkIdType i = 0; i < this->ppData->GetPoints()->GetNumberOfPoints();
         i++) {
      double point[3], transformedPoint[3];
      this->ppData->GetPoints()->GetPoint(i, point);

      double x, y, z;

      transform->TransformPoint(point, transformedPoint);
      transformedPoints->InsertNextPoint(transformedPoint);

      x = transformedPoint[0];
      y = transformedPoint[1];
      z = transformedPoint[2];

      if (y < yMin)
        yMin = y;
      if (y > yMax)
        yMax = y;

      if (z < zMin)
        zMin = z;
      if (z > zMax)
        zMax = z;

      if (x < xMin)
        xMin = x;
      if (x > xMax)
        xMax = x;
    }

    std::cout << "yMin, yMax: " << std::fixed << yMin - yMin << ","
              << yMax - yMin << std::endl;

    for (vtkIdType i = 0; i < transformedPoints->GetNumberOfPoints(); i++) {
      double point[3];
      transformedPoints->GetPoint(i, point);

      double normalizedY =
          (point[1] - yMin) / (yMax - yMin); // Normalize Y value

      scalars->InsertTuple1(i, normalizedY);
    }

    this->ppData->GetPointData()->SetScalars(scalars);
    this->Actor->GetProperty()->SetInterpolationToFlat();
  }
};

vtkStandardNewMacro(MyInteractorStyle);

} // namespace

int main(int, char *[]) {
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
  double yMin = std::numeric_limits<double>::max(),
         yMax = std::numeric_limits<double>::lowest();

  double xMin = std::numeric_limits<double>::max(),
         xMax = std::numeric_limits<double>::lowest();

  double zMin = std::numeric_limits<double>::max(),
         zMax = std::numeric_limits<double>::lowest();

  for (pdal::PointId id = 0; id < pointView->size(); ++id) {
    double x = pointView->getFieldAs<double>(pdal::Dimension::Id::X, id);
    double y = pointView->getFieldAs<double>(pdal::Dimension::Id::Y, id);
    double z = pointView->getFieldAs<double>(pdal::Dimension::Id::Z, id);
    points->InsertNextPoint(x, y, z);

    if (y < yMin)
      yMin = y;
    if (y > yMax)
      yMax = y;

    if (z < zMin)
      zMin = z;
    if (z > zMax)
      zMax = z;

    if (x < xMin)
      xMin = x;
    if (x > xMax)
      xMax = x;
  }

  double p[3];
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    points->GetPoint(i, p);
    p[0] = p[0] - xMin;
    p[1] = p[1] - yMin;
    p[2] = p[2] - zMin;
    points->SetPoint(i, p);
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
  lookupTable->SetRange(yMin - yMin, yMax - yMin); // from min to max

  lookupTable->SetHueRange(0.667, 0); // from blue to red
  lookupTable->Build();

  std::cout << "yMin, yMax: " << std::fixed << yMin - yMin << "," << yMax - yMin
            << std::endl;

  vtkSmartPointer<vtkUnsignedCharArray> colors =
      vtkSmartPointer<vtkUnsignedCharArray>::New();
  colors->SetNumberOfComponents(3);
  colors->SetName("Colors");

  double dColor[3];
  unsigned char color[3];
  for (vtkIdType i = 0; i < points->GetNumberOfPoints(); i++) {
    points->GetPoint(i, p);
    double y = p[1];

    lookupTable->GetColor(y, dColor);

    color[0] = static_cast<unsigned char>(255.0 * dColor[0]);
    color[1] = static_cast<unsigned char>(255.0 * dColor[1]);
    color[2] = static_cast<unsigned char>(255.0 * dColor[2]);

    colors->InsertNextTypedTuple(color);
  }

  pointPolyData->GetPointData()->SetScalars(colors);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyphFilter->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("Point Cloud Viewer");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<MyInteractorStyle> style;
  style->SetActor(actor);
  style->SetLookupTable(lookupTable);
  style->SetPointPolyData(pointPolyData);

  renderWindowInteractor->SetInteractorStyle(style);
  renderer->AddActor(actor);
  renderer->SetBackground(0.1, 0.2, 0.3); // Dark blue background

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}