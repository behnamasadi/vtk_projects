## vtkThresholdPoints
`vtkThresholdPoints` is a filter that extracts points from a dataset that satisfy a threshold criterion. The criterion can take three forms:
 1) greater than a particular value
2) less than a particular value
3) between a particular value. 

The output of the filter is polygonal data.


create some points:
```cpp
vtkNew<vtkPoints> points;
points->InsertNextPoint(0, 0, 0);
.
.
.
points->InsertNextPoint(4, 4, 4);
```

Setup position index:

```cpp
vtkNew<vtkIntArray> index;
index->SetNumberOfComponents(1);
index->SetName("index");
index->InsertNextValue(0);
.
.
.
index->InsertNextValue(4);
```

Associate some data with these points:

```cpp
vtkNew<vtkPolyData> polydata;
polydata->SetPoints(points);
polydata->GetPointData()->AddArray(index);
```


We want the points whose associated index value is less than  `3`:

```cpp
vtkNew<vtkThresholdPoints> threshold;
threshold->SetInputData(polydata);
threshold->ThresholdByLower(3);
threshold->SetInputArrayToProcess(0, 0, 0, vtkDataObject::FIELD_ASSOCIATION_POINTS, "index");
threshold->Update();
```

Get the thresholded data: 
```cpp
vtkPolyData *thresholdedPolydata = threshold->GetOutput();
```

[code](../src/ThresholdPoints.cpp)
