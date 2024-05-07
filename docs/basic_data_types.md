
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




`vtkPoints`: represent and manipulate 3D points (an array of vx-vy-vz triplets accessible by (point or cell) id.)

```cpp
  points->InsertPoint(0, 0, 0, 0);
  points->InsertPoint(1, 1, 0, 0);
  points->InsertPoint(2, 1, 1, 0);
  points->InsertPoint(3, 0, 1, 0);
  points->InsertNextPoint(0, 0, 1);

```


`vtkPointData`: represent and manipulate point attribute data i.e. color, temperature.



`vtkPolyData`: concrete dataset represents vertices, lines, polygons, and triangle strips

```cpp
vtkNew<vtkPolyData> polyData;
polyData->SetPoints(points);
```









