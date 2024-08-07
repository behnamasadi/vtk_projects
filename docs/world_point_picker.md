## World Point Picker

```cpp
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle *New();
  vtkTypeMacro(MouseInteractorStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnLeftButtonDown() override {
    std::cout << "Picking pixel: " << this->Interactor->GetEventPosition()[0]
              << " " << this->Interactor->GetEventPosition()[1] << std::endl;
    this->Interactor->GetPicker()->Pick(this->Interactor->GetEventPosition()[0],
                                        this->Interactor->GetEventPosition()[1],
                                        0, // always zero.
                                        this->Interactor->GetRenderWindow()
                                            ->GetRenderers()
                                            ->GetFirstRenderer());
    double picked[3];
    this->Interactor->GetPicker()->GetPickPosition(picked);
    std::cout << "Picked value: " << picked[0] << " " << picked[1] << " "
              << picked[2] << std::endl;
    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }
};
```

In your main

```cpp
vtkNew<vtkWorldPointPicker> worldPointPicker;

vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
renderWindowInteractor->SetPicker(worldPointPicker);
renderWindowInteractor->SetRenderWindow(renderWindow);

vtkNew<MouseInteractorStyle> style;
renderWindowInteractor->SetInteractorStyle(style);

```

[code](../src/WorldPointPicker.cxx)
