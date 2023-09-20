# CameraOrientationWidget

The widget representation has shafts and little spheres with text on them. The spheres always follow the camera.

```cpp
vtkNew<vtkCameraOrientationWidget> gizmo;
vtkNew<vtkRenderWindowInteractor> iren;

gizmo->SetParentRenderer(renderer);
gizmo->On();
gizmo->SetInteractor(iren);
```

[code](../src/CameraOrientationWidgetGizmo.cxx)
