```cpp
vtkNew<vtkPlaneSource> planeSource;
planeSource->SetXResolution(10);
planeSource->SetYResolution(10);
planeSource->SetOrigin(0, 0, 0);
planeSource->SetPoint1(10, 0, 0);
planeSource->SetPoint2(0, 10, 0);

// Create a mapper and actor for the plane: show it as a wireframe
vtkNew<vtkPolyDataMapper> planeMapper;
planeMapper->SetInputConnection(planeSource->GetOutputPort());
vtkNew<vtkActor> planeActor;
planeActor->SetMapper(planeMapper);
planeActor->GetProperty()->SetRepresentationToWireframe();
planeActor->GetProperty()->SetColor(colors->GetColor3d("Silver").GetData());

planeActor->PickableOff();
```
