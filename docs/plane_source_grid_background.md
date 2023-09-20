```cpp
vtkNew<vtkPlaneSource> planeSource;
planeSource->SetXResolution(
    10); // Set the number of grid lines in the X direction
planeSource->SetYResolution(
    10); // Set the number of grid lines in the Y direction

// Map the plane's data to graphics primitives
vtkNew<vtkPolyDataMapper> planeMapper;
planeMapper->SetInputConnection(planeSource->GetOutputPort());

  // Create an actor for the plane and set its properties to display the grid
  // lines
vtkNew<vtkActor> planeActor;
planeActor->SetMapper(planeMapper);
planeActor->GetProperty()
    ->SetRepresentationToWireframe();               // Display as wireframe
planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to gray

planeActor->GetProperty()->SetAmbient(1.0);
planeActor->GetProperty()->SetDiffuse(0.0);

planeActor->PickableOff();
```
