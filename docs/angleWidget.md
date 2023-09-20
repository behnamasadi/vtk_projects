## AngleWidget

```
vtkNew<vtkRenderer> renderer;
vtkNew<vtkRenderWindow> renWin;
renWin->AddRenderer(renderer);

// An interactor
vtkNew<vtkRenderWindowInteractor> iren;
iren->SetRenderWindow(renWin);

vtkNew<vtkAngleWidget> angleWidget;
angleWidget->SetInteractor(iren);
angleWidget->CreateDefaultRepresentation();

// Render
renWin->Render();
iren->Initialize();
renWin->Render();
angleWidget->On();
iren->Start();
```

code: [AngleWidget](../src/AngleWidget.cxx)

Changing the Representation of AngleWidget

```
vtkNew<vtkAngleRepresentation2D> rep;
rep->ArcVisibilityOff();
angleWidget->CreateDefaultRepresentation();
angleWidget->SetRepresentation(rep);
```

and you can set it:

```
rep->SetPoint1DisplayPosition(pos1);
rep->SetPoint2DisplayPosition(pos2);
rep->SetCenterDisplayPosition(center);
rep->Ray1VisibilityOn();
rep->Ray2VisibilityOn();
rep->ArcVisibilityOn();
```

If you want to have callback over the picking points:

```cpp
// Callback
vtkNew<vtkCallbackCommand> placeAnglePointCallback;
placeAnglePointCallback->SetCallback(func);
angleWidget->AddObserver(vtkCommand::EndInteractionEvent,
                   placeAnglePointCallback);

// Representation3D
vtkNew<vtkPointHandleRepresentation3D> handle;
vtkNew<vtkAngleRepresentation3D> rep;

rep->SetHandleRepresentation(handle);
angleWidget->SetRepresentation(rep);
```

The call back:

```cpp
void func(vtkObject *caller, unsigned long eid, void *clientdata,
          void *calldata) {
  double p1[3], p2[3], center[3];

  auto angleWidget = reinterpret_cast<vtkAngleWidget *>(caller);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
      ->GetCenterDisplayPosition(center);

  std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2] << std::endl;
  std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2] << std::endl;
  std::cout << "center: " << center[0] << ", " << center[1] << ", " << center[2]
            << std::endl;

  vtkNew<vtkPointPicker> pointPicker;
  angleWidget->GetInteractor()->SetPicker(pointPicker);

  if (pointPicker->Pick(p1, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(p2, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data);
  }

  if (pointPicker->Pick(center, angleWidget->GetCurrentRenderer())) {
    double data[3];
    pointPicker->GetPickPosition(data);
    std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
              << std::endl;

    static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
        ->GetCenterRepresentation()
        ->SetWorldPosition(data);
  }
}
```

[AngleWidget2D](../src/AngleWidget2D.cxx)
