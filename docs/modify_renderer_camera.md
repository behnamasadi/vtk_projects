
```cpp
vtkNew<vtkCamera> camera;
camera->SetPosition(0, 0, 100);
camera->SetFocalPoint(0, 0, 0);

// Create a renderer, render window, and interactor
vtkNew<vtkRenderer> renderer;

renderer->SetActiveCamera(camera);

//renderer->GetActiveCamera()
```

or

```cpp
vtkCamera* camera = nullptr;
camera = renderer->GetActiveCamera();
camera->Azimuth(30);
camera->Elevation(30);
```

[code](../src/Camera.cxx)
