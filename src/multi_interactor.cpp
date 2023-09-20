#include <pcl/io/vtk_lib_io.h>
#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCamera.h>
#include <vtkCameraOrientationWidget.h>
#include <vtkCellArray.h>
#include <vtkConeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkImageData.h>

#include <vtkMatrix4x4.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProgrammableSource.h>
#include <vtkProperty.h>
#include <vtkPyramid.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTetra.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertexGlyphFilter.h>

#include <algorithm>
#include <cctype>
#include <string>

#include "InteractorStyleSwitch.hpp"

int main() {
  vtkNew<vtkNamedColors> namedColor;

  // grid background
  int grid_size = 10;
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(20);
  planeSource->SetYResolution(20);
  planeSource->SetOrigin(0, 0, 0);
  planeSource->SetPoint1(grid_size, 0, 0);
  planeSource->SetPoint2(0, 0, grid_size);

  // Create a mapper and actor for the plane: show it as a wireframe
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();
  planeActor->GetProperty()->SetColor(
      namedColor->GetColor3d("Silver").GetData());
  planeActor->PickableOff();
  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);

  // adding axes
  vtkNew<vtkAxesActor> axesActor;
  axesActor->AxisLabelsOff();
  // axesActor->AddPosition(grid_size / 2, grid_size / 2, 0);
  // axesActor->SetPosition(grid_size / 2, grid_size / 2, 0);

  // adding a cone
  vtkNew<vtkConeSource> coneSource;
  coneSource->Update();
  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(coneSource->GetOutputPort());
  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);
  coneActor->RotateY(45);
  coneActor->SetPosition(2, 2, 3);
  coneActor->GetProperty()->SetColor(namedColor->GetColor3d("Gold").GetData());
  coneActor->GetProperty()->SetAmbient(1.0);
  coneActor->GetProperty()->SetDiffuse(0.0);

  // renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(axesActor);
  renderer->AddActor(planeActor);
  renderer->AddActor(coneActor);
  renderer->GetActiveCamera()->SetViewUp(1, 0, 0);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetPosition(1, 1, 1);
  renderer->SetBackground(namedColor->GetColor3d("DarkGreen").GetData());

  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);

  // Spheres
  int numberOfSpheres = 10;
  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);
  for (int i = 0; i < numberOfSpheres; ++i) {
    vtkNew<vtkSphereSource> source;
    double x, y, z, radius;
    // random position and radius
    x = randomSequence->GetRangeValue(-5.0, 5.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-5.0, 5.0);
    randomSequence->Next();
    z = randomSequence->GetRangeValue(-5.0, 5.0);
    randomSequence->Next();
    radius = randomSequence->GetRangeValue(0.5, 1.0);
    randomSequence->Next();
    source->SetRadius(radius);
    source->SetCenter(x, y, z);
    source->SetPhiResolution(11);
    source->SetThetaResolution(21);
    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(source->GetOutputPort());
    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);
    double r, g, b;
    r = randomSequence->GetRangeValue(0.4, 1.0);
    randomSequence->Next();
    g = randomSequence->GetRangeValue(0.4, 1.0);
    randomSequence->Next();
    b = randomSequence->GetRangeValue(0.4, 1.0);
    randomSequence->Next();
    actor->GetProperty()->SetDiffuseColor(r, g, b);
    actor->GetProperty()->SetDiffuse(0.8);
    actor->GetProperty()->SetSpecular(0.5);
    actor->GetProperty()->SetSpecularColor(
        namedColor->GetColor3d("White").GetData());
    actor->GetProperty()->SetSpecularPower(30.0);
    renderer->AddActor(actor);
  }

  // adding point cloud

  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::PointXYZ point;

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

  vtkNew<vtkPolyDataMapper> pointcloudMapper;
  pointcloudMapper->SetInputData(polydata);
  // pointcloudMapper->SetScalarRange(zMin, zMax)

  vtkNew<vtkActor> pointcloudActor;
  pointcloudActor->SetMapper(pointcloudMapper);
  pointcloudActor->GetProperty()->SetPointSize(12);
  // pointcloudActor->GetProperty()->SetRepresentation()

  renderer->AddActor(pointcloudActor);

  // renderWindow
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(300, 300);
  renderWindow->SetWindowName("multi_interactor");
  renderWindow->AddRenderer(renderer);

  // Interactor Style
  vtkNew<InteractorStyleSwitch> style;
  style->SetCurrentRenderer(renderer);
  // style->
  // style->SetActor(coneActor);

  // Window Interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);
  renderWindowInteractor->SetInteractorStyle(style);

  // Camera
  vtkNew<vtkCameraOrientationWidget> camOrientManipulator;
  camOrientManipulator->SetParentRenderer(renderer);
  camOrientManipulator->On();

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
}

/*
Pan shift + left click rotation
Dolly  right click back and forth
Spin ctrl + left click rotation
*/