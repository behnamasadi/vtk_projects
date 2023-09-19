# AppendFilter

The `vtkAppendFilter` is a filter in VTK that combines multiple data sets into a single one. It's commonly used in situations where you have multiple data sources or geometry that you want to treat or render as a single entity. Here's when you might want to use `vtkAppendFilter`:

1. **Combining Geometry**: Suppose you have multiple geometric shapes, each represented by its own data set (like multiple `vtkPolyData` objects). If you want to combine these shapes into a single geometry for further processing or visualization, you would use `vtkAppendFilter`.

2. **Multiple Data Sources**: In scientific visualization, it's not uncommon to have data coming from multiple sources or simulations. If you want to visualize all these data sets together, you can append them into a single data set using `vtkAppendFilter`.

3. **Pipeline Simplification**: If you have a visualization pipeline where multiple data sets are being processed in parallel using similar operations, it might be more efficient to combine the data sets first and then apply the operation. This can simplify the pipeline and reduce redundant operations.

4. **Interactions and Animations**: If you're creating an interaction or animation where multiple objects move together or share some properties, it can be beneficial to combine them into a single data set.

5. **Data Cleanup and Post-processing**: After combining data, it might be easier to apply cleanup operations, like removing duplicate points or cells.

Here's a simple example of how to use `vtkAppendFilter`:

```cpp
vtkSmartPointer<vtkSphereSource> sphereSource1 = vtkSmartPointer<vtkSphereSource>::New();
sphereSource1->SetCenter(0, 0, 0);
sphereSource1->Update();

vtkSmartPointer<vtkSphereSource> sphereSource2 = vtkSmartPointer<vtkSphereSource>::New();
sphereSource2->SetCenter(2, 0, 0);
sphereSource2->Update();

vtkSmartPointer<vtkAppendFilter> appendFilter = vtkSmartPointer<vtkAppendFilter>::New();
appendFilter->AddInputData(sphereSource1->GetOutput());
appendFilter->AddInputData(sphereSource2->GetOutput());
appendFilter->Update();
```

In this example, two spheres are created and their data is combined using `vtkAppendFilter`. The result is a single data set containing both spheres. This combined data set can be rendered or processed further as needed.


```cpp
vtkNew<vtkSphereSource> sphereSource1;
  sphereSource1->SetCenter(0, 0, 0);
  sphereSource1->Update();

  vtkNew<vtkSphereSource> sphereSource2;
  sphereSource2->SetCenter(2, 0, 0);
  sphereSource2->Update();

  vtkNew<vtkAppendFilter> appendFilter;
  appendFilter->AddInputData(sphereSource1->GetOutput());
  appendFilter->AddInputData(sphereSource2->GetOutput());
  appendFilter->Update();

  auto combined = appendFilter->GetOutput();
  std::cout << "There are " << combined->GetNumberOfPoints()
            << " points combined." << std::endl;

  vtkNew<vtkDataSetMapper> mapper;
  mapper->SetInputConnection(appendFilter->GetOutputPort());
```

[code](../src/AppendFilter.cpp)

Refs: [1](https://examples.vtk.org/site/Cxx/Filtering/AppendFilter/)

