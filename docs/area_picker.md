## Area Picker

```
void PickCallbackFunction(vtkObject* caller,
                          long unsigned int vtkNotUsed(eventId),
                          void* vtkNotUsed(clientData),
                          void* vtkNotUsed(callData))
{
  std::cout << "Pick." << std::endl;
  vtkAreaPicker* areaPicker = static_cast<vtkAreaPicker*>(caller);
  vtkProp3DCollection* props = areaPicker->GetProp3Ds();
  props->InitTraversal();

  for (vtkIdType i = 0; i < props->GetNumberOfItems(); i++)
  {
    vtkProp3D* prop = props->GetNextProp3D();
    std::cout << "Picked prop: " << prop << std::endl;
  }
}
```

In your main

```cpp
vtkNew<vtkAreaPicker> areaPicker;

vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
renderWindowInteractor->SetRenderWindow(renderWindow);
renderWindowInteractor->SetPicker(areaPicker);
```

For `vtkInteractorStyleRubberBandPick` - use 'r' and left-mouse to draw a selection box used to pick

```cpp
vtkNew<vtkInteractorStyleRubberBandPick> style;
```

For `vtkInteractorStyleTrackballCamera` - use 'p' to pick at the current mouse position

```cpp
vtkNew<vtkInteractorStyleTrackballCamera> style;

style->SetCurrentRenderer(renderer);
renderWindowInteractor->SetInteractorStyle(style);


vtkNew<vtkCallbackCommand> pickCallback;
pickCallback->SetCallback(PickCallbackFunction);

areaPicker->AddObserver(vtkCommand::EndPickEvent, pickCallback);
```

[code](../src/AreaPicking.cxx)
