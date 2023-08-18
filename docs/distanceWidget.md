# DistanceWidget3D

```cpp
// AngleWidget
vtkNew<vtkAngleWidget> angleWidget;
angleWidget->SetInteractor(iren);
angleWidget->CreateDefaultRepresentation();

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



code: [DistanceWidget](../src/DistanceWidget.cxx), [DistanceWidget3D](../src/DistanceWidget3D.cpp)


