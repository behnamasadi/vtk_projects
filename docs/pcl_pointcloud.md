```cpp
pcl::PointCloud<pcl::PointXYZ> cloud;
pcl::PointXYZ point;

for (int i = 0; i < 10000; i++) {

point.x = rand() * 10 - 5;
point.y = rand() * 10 - 5;
point.z = rand() * 10 - 5;
cloud.points.push_back(point);
}

vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();

pcl::io::pointCloudTovtkPolyData(cloud, polydata);

vtkNew<vtkPolyDataMapper> mapper;
mapper->SetInputData(polydata);
```
