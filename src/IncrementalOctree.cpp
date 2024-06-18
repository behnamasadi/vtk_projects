#include <vtkActor.h>
#include <vtkCellIterator.h>
#include <vtkCommand.h>
#include <vtkGlyph3D.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOctreePointLocator.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkVertexGlyphFilter.h>

#include <cmath>

namespace {
class vtkSliderCallback : public vtkCommand {
public:
  static vtkSliderCallback *New()

  {
    return new vtkSliderCallback;
  }
  vtkSliderCallback() : Octree(0), Level(0), PolyData(0), Renderer(0) {}

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    vtkSliderWidget *sliderWidget = reinterpret_cast<vtkSliderWidget *>(caller);
    this->Level = vtkMath::Round(static_cast<vtkSliderRepresentation *>(
                                     sliderWidget->GetRepresentation())
                                     ->GetValue());

    this->Octree->GenerateRepresentation(this->Level, this->PolyData);

    std::cout << "Level: " << this->Level << std::endl;

    std::cout << "NumberOfPoints :" << this->PolyData->GetNumberOfPoints()
              << std::endl;
    this->Renderer->Render();
  }

  vtkOctreePointLocator *Octree;
  int Level;
  vtkPolyData *PolyData;
  vtkRenderer *Renderer;
};
} // namespace

void cellIterator(vtkDataSet *ds) {
  vtkCellIterator *it = ds->NewCellIterator();

  for (it->InitTraversal(); !it->IsDoneWithTraversal(); it->GoToNextCell()) {

    // VTK_QUAD = 9, VTK_TRIANGLE=5 complete list at vtkCellType.h, for ref:
    // https://vtk.org/doc/nightly/html/vtkCellType_8h_source.html
    std::cout << "Cell type is: " << it->GetCellType() << std::endl;

    vtkIdList *pointIds = it->GetPointIds();
    std::cout << "point ids are: ";
    for (vtkIdType *pIds_iter = pointIds->begin(); pIds_iter != pointIds->end();
         pIds_iter++) {
      std::cout << *pIds_iter << " ";
    }
    std::cout << "\n";

    // vtkPoints: represent and manipulate 3D points
    vtkPoints *points = it->GetPoints();

    for (auto i = 0; i < points->GetNumberOfPoints(); i++) {
      double *point = points->GetPoint(i);
      std::cout << "point at i=" << i << ": " << point[0] << " " << point[1]
                << " " << point[2] << "\n";
    }
    std::cout << "\n";

    vtkNew<vtkGenericCell> cell;
    it->GetCell(cell);
  }
  it->Delete();
}

int main(int, char *[]) {
  // vtkNew<vtkNamedColors> colors;

  /*
    // Create a point cloud
    vtkNew<vtkSphereSource> pointSource;
    pointSource->SetPhiResolution(50);
    pointSource->SetThetaResolution(50);
    vtkNew<vtkPolyDataMapper> pointsMapper;
    pointsMapper->SetInputConnection(pointSource->GetOutputPort());
    pointSource->Update();
  */

  // vtkNew<vtkPoints> points;

  // vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  // randomSequence->SetSeed(8775070);

  // double cylanderRadius = 2.0;
  // for (int i = 0; i < 10000000; i++) {
  //   double x, y, z, radius;
  //   x = randomSequence->GetRangeValue(4.0, 12.0);
  //   randomSequence->Next();
  //   y = randomSequence->GetRangeValue(-2.0, 2.0);
  //   randomSequence->Next();
  //   z = pow(cylanderRadius * cylanderRadius - y * y, 0.5);

  //   points->InsertNextPoint(x, y, -z);
  // }

  // vtkNew<vtkPolyData> inputPolydata;
  // inputPolydata->SetPoints(points);
  // vtkNew<vtkPolyDataMapper> pointsMapper;

  // vtkNew<vtkActor> pointsActor;
  // pointsActor->SetMapper(pointsMapper);
  // pointsActor->GetProperty()->SetInterpolationToFlat();
  // pointsActor->GetProperty()->SetRepresentationToPoints();
  // pointsActor->GetProperty()->SetColor(colors->GetColor4d("Yellow").GetData());

  // // Create the tree
  // vtkNew<vtkOctreePointLocator> octree;
  // octree->SetMaximumPointsPerRegion(5);
  // octree->SetDataSet(inputPolydata);
  // octree->BuildLocator();

  // vtkNew<vtkPolyData> polydata;
  // octree->GenerateRepresentation(0, polydata);

  // vtkNew<vtkPolyDataMapper> octreeMapper;
  // octreeMapper->SetInputData(polydata);

  // vtkNew<vtkActor> octreeActor;
  // octreeActor->SetMapper(octreeMapper);
  // octreeActor->GetProperty()->SetInterpolationToFlat();
  // octreeActor->GetProperty()->SetRepresentationToWireframe();
  // octreeActor->GetProperty()->SetColor(
  //     colors->GetColor4d("SpringGreen").GetData());

  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkPoints> originalPoints;

  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);

  double cylanderRadius = 2.0;
  for (int i = 0; i < 20000;
       i++) { // Reduced number of points for faster rendering
    double x, y, z;
    x = randomSequence->GetRangeValue(4.0, 12.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-2.0, 2.0);
    randomSequence->Next();
    z = sqrt(cylanderRadius * cylanderRadius - y * y);

    originalPoints->InsertNextPoint(x, y, -z);
  }

  vtkNew<vtkPolyData> originalPointsPolydata;
  originalPointsPolydata->SetPoints(originalPoints);
  // vtkNew<vtkPolyDataMapper> pointsMapper;
  // pointsMapper->SetInputData(inputPolydata);

  // vtkNew<vtkActor> pointsActor;
  // pointsActor->SetMapper(pointsMapper);
  // pointsActor->GetProperty()->SetInterpolationToFlat();
  // pointsActor->GetProperty()->SetRepresentationToPoints();
  // pointsActor->GetProperty()->SetColor(colors->GetColor4d("Yellow").GetData());

  // Create the tree
  vtkNew<vtkOctreePointLocator> octree;
  // Maximum number of points per spatial region.  Default is 100.
  octree->SetMaximumPointsPerRegion(5);
  octree->SetDataSet(originalPointsPolydata);
  octree->BuildLocator();

  vtkNew<vtkPolyData> octreePolydata;
  octree->GenerateRepresentation(0, octreePolydata);

  cellIterator(octreePolydata);

  vtkIdType numRegions = octree->GetNumberOfLeafNodes();

  std::cout << "num Regions: " << numRegions << std::endl;

  vtkNew<vtkPoints> octreePointsCenters;

  for (vtkIdType i = 0; i < numRegions; ++i) {
    double bounds[6];
    octree->GetRegionBounds(i, bounds);

    // Calculate the center of the region
    double center[3];
    center[0] = (bounds[0] + bounds[1]) / 2.0;
    center[1] = (bounds[2] + bounds[3]) / 2.0;
    center[2] = (bounds[4] + bounds[5]) / 2.0;

    // Insert the center point into the points array
    // std::cout << "center: " << center[0] << "," << center[1] << "," <<
    // center[2]
    //           << std::endl;
    octreePointsCenters->InsertNextPoint(center);
  }

  vtkNew<vtkPolyData> octreePointsCenterPolydata;

  // pointsPolydata->SetPoints(points);

  // Set the points to the polydata
  octreePointsCenterPolydata->SetPoints(octreePointsCenters);

  // Use a vertex glyph filter to visualize the points
  vtkNew<vtkVertexGlyphFilter> octreePointsCenterVertexFilter;

  octreePointsCenterVertexFilter->SetInputData(octreePointsCenterPolydata);
  octreePointsCenterVertexFilter->Update();

  vtkNew<vtkVertexGlyphFilter> originalPointsVertexFilter;

  originalPointsVertexFilter->SetInputData(originalPointsPolydata);
  originalPointsVertexFilter->Update();

  // Mapper
  vtkNew<vtkPolyDataMapper> octreePointsCenterMapper;
  octreePointsCenterMapper->SetInputConnection(
      octreePointsCenterVertexFilter->GetOutputPort());

  vtkNew<vtkPolyDataMapper> originalPointsMapper;
  originalPointsMapper->SetInputConnection(
      originalPointsVertexFilter->GetOutputPort());

  // Actor
  vtkNew<vtkActor> actorOctreePointsCenter;
  actorOctreePointsCenter->SetMapper(octreePointsCenterMapper);
  actorOctreePointsCenter->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color
  actorOctreePointsCenter->GetProperty()->SetPointSize(
      5); // Increase point size for better visibility

  vtkNew<vtkActor> vtkActorInputPolydata;
  vtkActorInputPolydata->SetMapper(originalPointsMapper);
  vtkActorInputPolydata->GetProperty()->SetColor(0.0, 0.0, 1.0); // Blue color
  vtkActorInputPolydata->GetProperty()->SetPointSize(
      2); // Increase point size for better visibility

  // Renderer
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actorOctreePointsCenter);
  renderer->AddActor(vtkActorInputPolydata);

  renderer->SetBackground(0.1, 0.2, 0.4); // Background color

  // Render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);

  // Render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  // vtkNew<vtkPolyDataMapper> octreeMapper;
  // // octreeMapper->SetInputData(polydata);
  // octreeMapper->SetInputData(pointsPolydata);

  // vtkNew<vtkActor> octreeActor;
  // octreeActor->SetMapper(octreeMapper);
  // octreeActor->GetProperty()->SetInterpolationToFlat();
  // octreeActor->GetProperty()->SetRepresentationToWireframe();
  // octreeActor->GetProperty()->SetColor(
  //     colors->GetColor4d("SpringGreen").GetData());

  // // A renderer and render window
  // vtkNew<vtkRenderer> renderer;
  // vtkNew<vtkRenderWindow> renderWindow;
  // renderWindow->AddRenderer(renderer);

  // // An interactor
  // vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  // renderWindowInteractor->SetRenderWindow(renderWindow);

  // // Add the actors to the scene
  // renderer->AddActor(pointsActor);
  // renderer->AddActor(octreeActor);
  // renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

  // // Render an image (lights and cameras are created automatically)
  // renderWindow->SetWindowName("OctreeVisualize");
  // renderWindow->SetSize(600, 600);
  // renderWindow->Render();

  // vtkNew<vtkSliderRepresentation2D> sliderRep;
  // sliderRep->SetMinimumValue(0);
  // sliderRep->SetMaximumValue(octree->GetLevel());
  // sliderRep->SetValue(0);
  // sliderRep->SetTitleText("Level");
  // sliderRep->GetPoint1Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  // sliderRep->GetPoint1Coordinate()->SetValue(.2, .2);
  // sliderRep->GetPoint2Coordinate()->SetCoordinateSystemToNormalizedDisplay();
  // sliderRep->GetPoint2Coordinate()->SetValue(.8, .2);
  // sliderRep->SetSliderLength(0.075);
  // sliderRep->SetSliderWidth(0.05);
  // sliderRep->SetEndCapLength(0.05);
  // sliderRep->GetTitleProperty()->SetColor(
  //     colors->GetColor3d("Beige").GetData());
  // sliderRep->GetCapProperty()->SetColor(
  //     colors->GetColor3d("MistyRose").GetData());
  // sliderRep->GetSliderProperty()->SetColor(
  //     colors->GetColor3d("LightBlue").GetData());
  // sliderRep->GetSelectedProperty()->SetColor(
  //     colors->GetColor3d("Violet").GetData());

  // vtkNew<vtkSliderWidget> sliderWidget;
  // sliderWidget->SetInteractor(renderWindowInteractor);
  // sliderWidget->SetRepresentation(sliderRep);
  // sliderWidget->SetAnimationModeToAnimate();
  // sliderWidget->EnabledOn();

  // vtkNew<vtkSliderCallback> callback;
  // callback->Octree = octree;
  // callback->PolyData = polydata;
  // callback->Renderer = renderer;

  // sliderWidget->AddObserver(vtkCommand::InteractionEvent, callback);

  // renderWindowInteractor->Initialize();
  // renderWindow->Render();

  // renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}