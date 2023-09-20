## Prop (Actor) Picker

```cpp
class MouseInteractorHighLightActor : public vtkInteractorStyleTrackballCamera
{
public:
  virtual void OnLeftButtonDown() override
  {

    int* clickPos = this->GetInteractor()->GetEventPosition();

    // Pick from this location.
    vtkNew<vtkPropPicker> picker;
    picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
    this->LastPickedActor = picker->GetActor();

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

private:
  vtkActor* LastPickedActor;
};

vtkStandardNewMacro(MouseInteractorHighLightActor);
}
```

In your main:

```cpp
vtkNew<MouseInteractorHighLightActor> style;
style->SetDefaultRenderer(renderer);

renderWindowInteractor->SetInteractorStyle(style);
```

[code](../src/HighlightPickedActor.cxx)
