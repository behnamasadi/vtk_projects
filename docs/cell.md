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


### Cell Connectivity
Cell Connectivity and Offsets (`triangles` is a `vtkCellArray` ):

```
 auto iter = vtk::TakeSmartPointer(cellArray->NewIterator());
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


### Iteration on cells

First let have look at inheritance diagram: 
```uml
vtkDataSet
    ▴
    |
    |
vtkPointSet
    ▴
    |
    |
vtkPolyData
```

Now Iteration
```cpp

  vtkDataSet *ds;
  vtkCellIterator *it = ds->NewCellIterator();

  for (it->InitTraversal(); !it->IsDoneWithTraversal(); it->GoToNextCell()) {

    // VTK_QUAD = 9, VTK_TRIANGLE=5, etc
    std::cout << "Cell type is: " << it->GetCellType() << std::endl;

    vtkIdList *pointIds = it->GetPointIds();
    std::cout << "point ids are: ";
    for (vtkIdType *pIds_iter = pointIds->begin(); pIds_iter != pointIds->end();
         pIds_iter++) {
      std::cout << *pIds_iter << " ";
    }

    // vtkPoints: represent and manipulate 3D points
    vtkPoints *points = it->GetPoints();

    for (auto i = 0; i < points->GetNumberOfPoints(); i++) {
      double *point = points->GetPoint(i);
      std::cout << "point at i=" << i << ": " << point[0] << " " << point[1] << " " << point[2] << "\n";
    }

    vtkNew<vtkGenericCell> cell;
    it->GetCell(cell);
  }

```

complete list of [CellType](https://vtk.org/doc/nightly/html/vtkCellType_8h_source.html) defined at `vtkCellType.h`
    


Refs: [1](https://vtk.org/doc/nightly/html/classvtkCellArrayIterator.html)


[source](../src/AddCell.cxx)
  
