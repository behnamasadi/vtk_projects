#include <pcl/io/vtk_lib_io.h>
#include <vtkActor.h>
#include <vtkAngleRepresentation3D.h>
#include <vtkAngleWidget.h>
#include <vtkCallbackCommand.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

void func(vtkObject *caller, unsigned long eid, void *clientdata,
          void *calldata) {
  double p1[3], p2[3], center[3];

  auto angleWidget = reinterpret_cast<vtkAngleWidget *>(caller);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetCenterDisplayPosition(center);

  std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
  std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
  std::cout << "center: " << center[0] << ", " << center[1] << ", " << center[2]
            << std::endl;

  vtkNew<vtkPointPicker> pointPicker;
  angleWidget->GetInteractor()->SetPicker(pointPicker);

  if (pointPicker->Pick(p1, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(p2, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(center, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetCenterRepresentation()
        ->SetWorldPosition(data);
  }
}

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(renderer);
  renWin->SetWindowName("AngleWidget");

  // An interactor
  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  // Adding a cube
  vtkNew<vtkPolyData> polydata;

  pcl::PointCloud<pcl::PointXYZ> cloud;

  std::vector<pcl::PointXYZ> points = {{0, 0, 0}, {0, 0, 1}, {0, 1, 0},
                                       {1, 1, 0}, {1, 0, 0}, {1, 1, 1}};
  for (auto const &point : points) {
    cloud.push_back(point);
  }

  pcl::io::pointCloudTovtkPolyData(cloud, polydata);

  vtkNew<vtkPolyDataMapper> pointcloudMapper;
  pointcloudMapper->SetInputData(polydata);

  vtkNew<vtkActor> cloudActor;
  cloudActor->SetMapper(pointcloudMapper);
  cloudActor->GetProperty()->SetPointSize(12);

  // AngleWidget
  vtkNew<vtkAngleWidget> angleWidget;
  angleWidget->SetInteractor(iren);
  angleWidget->CreateDefaultRepresentation();

  // Callback
  vtkNew<vtkCallbackCommand> placeAnglePointCallback;
  placeAnglePointCallback->SetCallback(func);
  angleWidget->AddObserver(vtkCommand::EndInteractionEvent,
                           placeAnglePointCallback);

  // Representation3D
  vtkNew<vtkPointHandleRepresentation3D> handle;
  vtkNew<vtkAngleRepresentation3D> rep;

  rep->SetHandleRepresentation(handle);
  angleWidget->SetRepresentation(rep);

  // Render
  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());
  renderer->AddActor(cloudActor);

  renWin->Render();
  iren->Initialize();
  renWin->Render();
  angleWidget->On();
  iren->Start();

  return EXIT_SUCCESS;
}
