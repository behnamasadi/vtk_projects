#include <pcl/io/vtk_lib_io.h>
#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkOutlineFilter.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

int main(int, char *[]) {
  // Create a sphere
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->SetCenter(+5, 2, -1);
  sphereSource->Update();

  // Create a cone
  vtkNew<vtkConeSource> coneSource;
  coneSource->Update();

  // create a cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;
  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);

  double cylanderRadius = 2.0;
  for (int i = 0; i < 1000; i++) {
    double x, y, z, radius;
    x = randomSequence->GetRangeValue(4.0, 12.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-2.0, 2.0);
    randomSequence->Next();
    z = pow(cylanderRadius * cylanderRadius - y * y, 0.5);

    point.x = x;
    point.y = y;
    point.z = z;

    cloud.points.push_back(point);

    point.x = x;
    point.y = y;
    point.z = -z;
    cloud.points.push_back(point);
  }

  vtkNew<vtkPolyData> polydata;
  pcl::io::pointCloudTovtkPolyData(cloud, polydata);

  // Combine the sphere and cone vtkPolyData
  vtkNew<vtkAppendPolyData> appendFilter;
  appendFilter->AddInputData(sphereSource->GetOutput());
  appendFilter->AddInputData(coneSource->GetOutput());
  appendFilter->AddInputData(polydata);

  appendFilter->Update();

  // Generate outline for the combined vtkPolyData
  vtkNew<vtkOutlineFilter> combinedOutlineFilter;
  combinedOutlineFilter->SetInputConnection(appendFilter->GetOutputPort());
  combinedOutlineFilter->Update();

  // Map the combined outline data to graphics primitives
  vtkNew<vtkPolyDataMapper> combinedOutlineMapper;
  vtkNew<vtkPolyDataMapper> sphereMapper;
  vtkNew<vtkPolyDataMapper> coneMapper;
  vtkNew<vtkPolyDataMapper> pointcloudMapper;

  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  coneMapper->SetInputConnection(coneSource->GetOutputPort());
  combinedOutlineMapper->SetInputConnection(
      combinedOutlineFilter->GetOutputPort());

  pointcloudMapper->SetInputData(polydata);

  vtkNew<vtkActor> sphereActor;
  vtkNew<vtkActor> coneActor;
  vtkNew<vtkActor> combinedOutlineActor;
  vtkNew<vtkActor> pointcloudActor;

  pointcloudActor->SetMapper(pointcloudMapper);
  pointcloudActor->GetProperty()->SetPointSize(12);

  coneActor->SetMapper(sphereMapper);
  sphereActor->SetMapper(coneMapper);
  combinedOutlineActor->SetMapper(combinedOutlineMapper);

  // Setup rendering
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(combinedOutlineActor);
  renderer->AddActor(coneActor);
  renderer->AddActor(sphereActor);
  renderer->AddActor(pointcloudActor);

  renderer->SetBackground(0.1, 0.1, 0.1);

  double bounds[6];
  combinedOutlineFilter->GetOutput()->GetBounds(bounds);

  std::cout << "Min Point: (" << bounds[0] << ", " << bounds[2] << ", "
            << bounds[4] << ")" << std::endl;
  std::cout << "Max Point: (" << bounds[1] << ", " << bounds[3] << ", "
            << bounds[5] << ")" << std::endl;

  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(20);
  planeSource->SetYResolution(20);

  double x_min, x_max, y_min, y_max, z_min, z_max;

  x_min = bounds[0];
  x_max = bounds[1];
  y_min = bounds[2];
  y_max = bounds[3];
  z_min = bounds[4];
  z_max = bounds[5];

  // planeSource->SetOrigin(bounds[0], 0, -bounds[4]);
  // planeSource->SetPoint1(bounds[0], 0, bounds[4]);
  // planeSource->SetPoint2(bounds[1], 0, bounds[5]);

  // planeSource->SetOrigin(bounds[0], bounds[2], bounds[4]);
  // planeSource->SetPoint1(bounds[1], bounds[2], bounds[4]);
  // planeSource->SetPoint2(bounds[0], bounds[3], bounds[4]);
  // planeSource->Update();

  double width = bounds[1] - bounds[0];
  double height = bounds[3] - bounds[2];
  double maxDimension = std::max(width, height);

  // Factor to make the plane bigger, e.g., 2 makes it twice as big
  double factor = 4.0;

  double adjustedDimension = maxDimension * factor;
  double centerX = (bounds[0] + bounds[1]) / 2.0;
  double centerY = (bounds[2] + bounds[3]) / 2.0;

  // Create a square plane that covers the bounding box of the object and is
  // bigger by a given factor
  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  planeSource->SetOrigin(centerX - adjustedDimension / 2,
                         centerY - adjustedDimension / 2, bounds[4]);
  planeSource->SetPoint1(centerX + adjustedDimension / 2,
                         centerY - adjustedDimension / 2, bounds[4]);
  planeSource->SetPoint2(centerX - adjustedDimension / 2,
                         centerY + adjustedDimension / 2, bounds[4]);
  planeSource->Update();

  // Create a mapper and actor for the plane: show it as a wireframe
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();
  //   planeActor->GetProperty()->SetColor(
  //       namedColor->GetColor3d("Silver").GetData());
  planeActor->PickableOff();
  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);
  // renderer->GetActiveCamera()->SetViewUp(0, 1, 0);

  // adding axes
  vtkNew<vtkAxesActor> axesActor;
  axesActor->AxisLabelsOff();
  // axesActor->SetOrigin(2, 2, -bounds[4]);
  // axesActor->SetPosition(5, 5, 5);
  // axesActor->SetUserTransform();

  vtkNew<vtkTransform> transform;
  transform->Translate(centerX - adjustedDimension / 2,
                       centerY - adjustedDimension / 2, bounds[4]);

  // axesActor->SetUserTransform(transform);

  renderer->AddActor(planeActor);
  renderer->AddActor(axesActor);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  // vtkNew<vtkInteractorStyleTrackballActor> style;

  // vtkNew<InteractorStyleSwitch> style;

  renderWindowInteractor->SetInteractorStyle(style);

  // vtkBoundingBox boundingBox;

  // boundingBox.AddBounds(bounds);

  // // Print bounding box details
  // double minPoint[3], maxPoint[3];
  // boundingBox.GetMinPoint(minPoint[0], minPoint[1], minPoint[2]);
  // boundingBox.GetMaxPoint(maxPoint[0], maxPoint[1], maxPoint[2]);
  // std::cout << "Min Point: (" << minPoint[0] << ", " << minPoint[1] << ", "
  //           << minPoint[2] << ")" << std::endl;
  // std::cout << "Max Point: (" << maxPoint[0] << ", " << maxPoint[1] << ", "
  //           << maxPoint[2] << ")" << std::endl;

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
