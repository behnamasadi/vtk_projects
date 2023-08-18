#include <pcl/io/vtk_lib_io.h>
#include <vtkCallbackCommand.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

void func(vtkObject *caller, unsigned long eid, void *clientdata,
          void *calldata) {
  double p1[3], p2[3];
  auto distanceWidget = reinterpret_cast<vtkDistanceWidget *>(caller);
  // renderer->GetInteractive

  //   static_cast<vtkDistanceRepresentation3D *>(
  //       distanceWidget->GetRepresentation())
  //       ->GetPoint1DisplayPosition

  //           // std::cout << "-------------" << std::endl;

  vtkSmartPointer<vtkDistanceRepresentation3D> rep;

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
  std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;

  vtkNew<vtkPointPicker> pointPicker;
  distanceWidget->GetInteractor()->SetPicker(pointPicker);

  if (pointPicker->Pick(p1, distanceWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(p2, distanceWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data);
  }
}

int main() {

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

  // renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(cloudActor);
  cloudActor->GetProperty()->SetPointSize(12);

  // RenderWindow
  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(renderer);

  // Interactor
  vtkNew<vtkRenderWindowInteractor> iRen;
  iRen->SetRenderWindow(renWin);

  // DistanceWidget
  vtkNew<vtkPointHandleRepresentation3D> handle;
  vtkNew<vtkDistanceRepresentation3D> rep;
  rep->SetHandleRepresentation(handle);
  vtkNew<vtkDistanceWidget> widget;
  widget->SetInteractor(iRen);
  widget->SetRepresentation(rep);
  widget->SetWidgetStateToManipulate();
  widget->EnabledOn();
  widget->ProcessEventsOn();

  vtkNew<vtkCallbackCommand> callback;
  callback->SetCallback(func);
  widget->AddObserver(vtkCommand::EndInteractionEvent, callback);

  renderer->ResetCamera();
  renWin->Render();
  iRen->Initialize();
  renWin->Render();
  widget->On();
  iRen->Start();

  return 0;
}