
```cpp
vtkNew<vtkCamera> camera;
camera->SetPosition(0, 0, 100);
camera->SetFocalPoint(0, 0, 0);
camera->Azimuth(30);
camera->Elevation(30);
camera->SetViewUp(0, 0, 1);
camera->SetClippingRange(5, 15);
camera->SetViewAngle(30);

// Create a renderer, render window, and interactor
vtkNew<vtkRenderer> renderer;
renderer->SetActiveCamera(camera);
```

or

```cpp
vtkCamera* camera = nullptr;
camera = renderer->GetActiveCamera();
camera->Azimuth(30);
camera->Elevation(30);
```

[code](../src/Camera.cxx)
