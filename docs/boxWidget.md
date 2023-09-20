```cpp
struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {
    vtkNew<vtkTransform> t;

    auto widget = reinterpret_cast<vtkBoxWidget *>(caller);

    widget->GetTransform(t);
    widget->GetProp3D()->SetUserTransform(t);
  }
};
```

```cpp
vtkNew<vtkRenderWindowInteractor> iren;
iren->SetRenderWindow(renWin);

vtkNew<MyCallback> mycallback;

vtkNew<vtkBoxWidget> boxWidget;

boxWidget->SetInteractor(iren);
boxWidget->SetProp3D(actor);

boxWidget->AddObserver(vtkCommand::InteractionEvent, mycallback);
boxWidget->On();
```

[code](../src/boxWidget.cpp)

Refs: [1](https://examples.vtk.org/site/Cxx/Tutorial/Tutorial_Step6/)
