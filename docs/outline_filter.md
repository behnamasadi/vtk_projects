# vtkOutlineFilter
create wireframe outline for an arbitrary data set or composite dataset. An outline consists of the twelve edges of the dataset bounding box

```cpp
  vtkNew<vtkConeSource> source;
  vtkNew<vtkOutlineFilter> outline;
  outline->SetInputConnection(source->GetOutputPort());
  vtkNew<vtkPolyDataMapper> outlineMapper;
  outlineMapper->SetInputConnection(outline->GetOutputPort());
  vtkNew<vtkActor> outlineActor;
  outlineActor->SetMapper(outlineMapper);
```
