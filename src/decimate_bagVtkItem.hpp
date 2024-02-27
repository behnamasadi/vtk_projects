
#include <QQuickVTKItem.h>
#include <QVTKInteractor.h>
#include <pcl/io/vtk_lib_io.h>
#include <vtkActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

struct MyVtkItem : public QQuickVTKItem {

  Q_OBJECT
public:
  struct Data : vtkObject {

    static Data *New();
    vtkTypeMacro(Data, vtkObject);

    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
    vtkNew<QVTKInteractor> iRen;
  };

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {

    // Adding a cube
    vtkNew<vtkPolyData> polydata;
    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ point;

    for (int i = 0; i < 1000000; i++) {

      point.x = rand() * 10 - 5;
      point.y = rand() * 10 - 5;
      point.z = rand() * 10 - 5;
      cloud.points.push_back(point);
    }

    vtkNew<Data> vtk;

    pcl::io::pointCloudTovtkPolyData(cloud, polydata);

    vtkNew<vtkPolyDataMapper> pointcloudMapper;
    pointcloudMapper->SetInputData(polydata);

    vtkNew<vtkActor> cloudActor;
    cloudActor->SetMapper(pointcloudMapper);

    vtk->actor->SetMapper(vtk->mapper);

    vtk->renderer->AddActor(cloudActor);
    cloudActor->GetProperty()->SetPointSize(12);

    vtk->renderer->AddActor(vtk->actor);
    vtk->renderer->ResetCamera();
    vtk->renderer->SetBackground(0.0, 1.0, 0.0);
    vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
    vtk->renderer->SetGradientBackground(true);

    renderWindow->AddRenderer(vtk->renderer);
    renderWindow->SetMultiSamples(16);

    renderWindow->SetInteractor(vtk->iRen);

    return vtk;
  }
};
