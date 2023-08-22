#include <pcl/io/vtk_lib_io.h>
#include <vtkBoundingBox.h>
#include <vtkConeSource.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>

int main(int argc, char **argv) {
  double p0[3] = {1, 1, 1};

  vtkNew<vtkConeSource> cone;
  cone->SetRadius(2);
  cone->SetHeight(4);
  cone->Update();

  vtkNew<vtkPolyDataMapper> mapper;

  mapper->SetInputConnection(cone->GetOutputPort());
  vtkBoundingBox boundingBox;

  double coneBounds[6];
  cone->GetOutput()->GetBounds(coneBounds);

  // boundingBox.SetBounds(coneBounds);
  boundingBox.AddBounds(coneBounds);

  // adding point cloud

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

  vtkNew<vtkPolyDataMapper> pointcloudMapper;
  pointcloudMapper->SetInputData(polydata);

  double cloudBounds[6];
  polydata->GetBounds(cloudBounds);

  boundingBox.AddBounds(cloudBounds);

  // Print bounding box details
  double minPoint[3], maxPoint[3];
  boundingBox.GetMinPoint(minPoint[0], minPoint[1], minPoint[2]);
  boundingBox.GetMaxPoint(maxPoint[0], maxPoint[1], maxPoint[2]);
  std::cout << "Min Point: (" << minPoint[0] << ", " << minPoint[1] << ", "
            << minPoint[2] << ")" << std::endl;
  std::cout << "Max Point: (" << maxPoint[0] << ", " << maxPoint[1] << ", "
            << maxPoint[2] << ")" << std::endl;

  return EXIT_SUCCESS;
}
