### vtkCell

`vtkCell` is an abstract class that specifies the interfaces for data cells. 

### vtkTriangle

a cell that represents a triangle, `vtkTriangle` is a concrete implementation of `vtkCell` to represent a triangle located in 3-space

### vtkCellArray
object to represent cell connectivity

```cpp
 vtkNew<vtkPoints> points;

  /*

  Y----------------------▸
  |
  |
  |
  |
  |
  |
  ▾
  0                             2
  (0, 0, 0)                     (0, 1, 0)



  1                             3
  (1, 0, 0)                     (1, 0, 0)

  */

  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);
  points->InsertNextPoint(1, 1, 0);

  vtkNew<vtkTriangle> triangle1;

  triangle1->GetPointIds()->SetId(0, 0);
  triangle1->GetPointIds()->SetId(1, 1);
  triangle1->GetPointIds()->SetId(2, 3);

  vtkNew<vtkTriangle> triangle2;
  triangle2->GetPointIds()->SetId(0, 0);
  triangle2->GetPointIds()->SetId(1, 2);
  triangle2->GetPointIds()->SetId(2, 3);

  vtkNew<vtkCellArray> triangles;
  triangles->InsertNextCell(triangle1);
  triangles->InsertNextCell(triangle2);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  std::cout << "There are " << polyData->GetNumberOfCells() << " cells."
            << std::endl;

  std::cout << "There are " << polyData->GetNumberOfPoints() << " points."
            << std::endl;

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);
```



```
 auto iter = vtk::TakeSmartPointer(triangles->NewIterator());
  // auto iter = triangles->NewIterator();

  for (iter->GoToFirstCell(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    vtkIdType npts;
    const vtkIdType *pts;
    iter->GetCurrentCell(npts, pts);

    std::cout << "Connectivity: ";
    for (vtkIdType i = 0; i < npts; ++i) {
      std::cout << pts[i] << " ";
    }
    std::cout << std::endl;

    // Assuming the cell array is made of triangles, the offset is 3 * cell
    // index. For other cell types or mixed cell arrays, you would need a
    // different method to get offsets.
    vtkIdType offset = iter->GetCurrentCellId() * 3;
    std::cout << "Offset: " << offset << std::endl;
  }
```

Offsets and Connectivity:

```
Offsets:      {0, 3}
Connectivity: {0, 1, 2, 0, 2, 3}
```

Refs: [1](https://vtk.org/doc/nightly/html/classvtkCellArrayIterator.html)


[source](../src/AddCell.cxx)
  
