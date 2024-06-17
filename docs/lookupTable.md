`vtkLookupTable` is used to map scalar values to colors. 


## vtkLookupTable vs vtkColorTransferFunction
`vtkLookupTable` and `vtkColorTransferFunction` are used to map scalar values to colors for data visualization, but they serve different purposes and have different capabilities. 


`vtkLookupTable` is used to map scalar values to colors. This mapping is essential for scientific visualization where different data values are represented by different colors, aiding in the interpretation and understanding of complex data sets. The `vtkLookupTable` provides a flexible and efficient way to manage color assignments based on scalar values, and is widely used in conjunction with scalar data associated with points, lines, and surfaces.

### Purpose of vtkLookupTable
The primary functions of `vtkLookupTable` are:
- **Mapping Scalars to Colors**: Convert numerical scalar values into visual colors.
- **Color Gradients**: Define a gradient or range of colors that correspond to the minimum and maximum values of the scalar data.
- **Custom Color Mapping**: Allow customization of the color map, including the ability to set discrete colors for specific scalar ranges.

### Example of vtkLookupTable in C++
Here’s a practical example that demonstrates how to create and use a `vtkLookupTable` in a VTK application. In this example, we'll map scalar values of a set of points to colors using a lookup table and visualize them:

```cpp
// Create a set of points
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);

  // Set the id at location i
  vtkNew<vtkTriangle> triangle;
  triangle->GetPointIds()->SetId(0, 0);
  triangle->GetPointIds()->SetId(1, 1);
  triangle->GetPointIds()->SetId(2, 2);

  // Create a vertex list (cell type) to connect the points,
  vtkNew<vtkCellArray> triangles;
  triangles->InsertNextCell(triangle);

  // Create scalar data for these points
  vtkNew<vtkFloatArray> scalars;
  scalars->InsertNextValue(0.0);
  scalars->InsertNextValue(0.25);
  scalars->InsertNextValue(1.0);

  // Create a polydata object and set points and scalars
  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);
  polyData->GetPointData()->SetScalars(scalars);

  // Create a lookup table and define its properties
  vtkSmartPointer<vtkLookupTable> lookupTable =
      vtkSmartPointer<vtkLookupTable>::New();

  lookupTable->SetRange(0.0, 1.0); // Scalar range
  lookupTable->Build();            // Build the lookup table

  std::cout << "points->GetNumberOfPoints():" << points->GetNumberOfPoints()
            << std::endl;

  for (vtkIdType id = 0; id < points->GetNumberOfPoints(); id++) {
    double color[3];

    lookupTable->GetColor(id, color);
    std::cout << "id: " << id << " color: " << color[0] << "," << color[1]
              << "," << color[2] << std::endl;
  }

  // Create a mapper and set its scalar range and lookup table
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);
  mapper->SetLookupTable(lookupTable);
  mapper->SetScalarRange(0.0, 1.0);
  mapper->SetInputData(polyData);
```

[source](../src/lookupTable.cpp)
