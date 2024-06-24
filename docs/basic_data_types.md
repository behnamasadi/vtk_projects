
### vtkFloatArray

`vtkFloatArray`: is an array of values of type float. It provides methods for insertion and retrieval of values and will automatically resize itself to hold new data.


```cpp
    vtkSmartPointer<vtkFloatArray> vectors = vtkSmartPointer<vtkFloatArray>::New();
    vectors->SetNumberOfComponents(3);  // vectors in 3D
    vectors->InsertNextTuple3(1.0, 0.0, 0.0);
    vectors->InsertNextTuple3(0.0, 1.0, 0.0);

```

Another example:

```cpp
  // Create a new vtkFloatArray
  vtkSmartPointer<vtkFloatArray> floatArray =
      vtkSmartPointer<vtkFloatArray>::New();

  // Set the name of the array (optional)
  floatArray->SetName("Example Float Array");

  // Add some floating point values to the array
  floatArray->InsertNextValue(1.1);
  floatArray->InsertNextValue(2.2);
  floatArray->InsertNextValue(3.3);

  // Retrieve and print the values from the array
  for (vtkIdType i = 0; i < floatArray->GetNumberOfTuples(); ++i) {
    double value;
    floatArray->GetTuple(i, &value);
    std::cout << "Value " << i << ": " << value << std::endl;
  }
```
   


### vtkPoints

`vtkPoints`: represent and manipulate 3D points (an array of vx-vy-vz triplets accessible by (point or cell) id.)

```cpp
  points->InsertPoint(0, 0, 0, 0);
  points->InsertPoint(1, 1, 0, 0);
  points->InsertPoint(2, 1, 1, 0);
  points->InsertPoint(3, 0, 1, 0);
  
  //Insert point into next available slot and returns id of slot.
  points->InsertNextPoint(0, 0, 1);

```

### vtkIntArray
same as above for int.

### vtkPointData
`vtkPointData`: represent and manipulate point attribute data i.e. color, temperature.


### vtkPolyData
`vtkPolyData`: concrete dataset represents vertices, lines, polygons, and triangle strips

```cpp
 vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);

  // Create scalar data for these points
  vtkSmartPointer<vtkFloatArray> scalars =
      vtkSmartPointer<vtkFloatArray>::New();
  scalars->InsertNextValue(0.0);
  scalars->InsertNextValue(0.5);
  scalars->InsertNextValue(1.0);

  // Create a polydata object and set points and scalars
  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points);
  polyData->GetPointData()->SetScalars(scalars);
```

### vtkPointSource
`vtkPointSource` creates a user-specified number of points within a specified radius about a specified center point. It is also possible to generate random points only on the surface of the sphere; or a exponential distribution weighted towards the center point. The output `PolyData` has the specified number of points and a single cell - a `vtkPolyVertex` cell referencing all of the points.

```cpp
vtkNew<vtkPointSource> sphere;
  sphere->SetCenter(1.0, 2.0, 1.0);
  sphere->SetNumberOfPoints(100);
  sphere->SetLambda(-1);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphere->GetOutputPort());
```

[code](../src/point_source_example.cpp)


Refs: [1](https://vtk.org/doc/nightly/html/classvtkPointSource.html#a78dc8ac26ecfd8a3a326517e9c09491a)



### vtkPolyVertex


[code](../src/poly_vertex.cpp)


