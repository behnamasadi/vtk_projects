## RotateActor

```cpp
class MyInteractorStyle : public vtkInteractorStyleTrackballActor {
public:
  static MyInteractorStyle *New();
  vtkTypeMacro(MyInteractorStyle, vtkInteractorStyleTrackballActor);

  virtual void OnLeftButtonDown() {
    std::cout << "Pressed left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballActor::OnLeftButtonDown();

    
  }

  virtual void OnLeftButtonUp() {
    std::cout << "Released left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }

private:
  vtkSmartPointer<vtkActor> Actor;
};
```

or you could 

```cpp
virtual void OnLeftButtonDown() override
{

    int* clickPos = this->GetInteractor()->GetEventPosition();

    // Pick from this location.
    vtkNew<vtkPropPicker> picker;
    picker->Pick(clickPos[0], clickPos[1], 0, this->GetDefaultRenderer());
    this->LastPickedActor = picker->GetActor();
}    
```
also:

```cpp
vtkNew<vtkTransform> transform;
transform->PostMultiply();
transform->Translate(10.0, 0.0, 0.0);
actor->SetUserTransform(transform);
```

In your main:

```cpp
vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
renderWindowInteractor->SetRenderWindow(renderWindow);

vtkNew<MyInteractorStyle> style;
style->SetActor(actor);

renderWindowInteractor->SetInteractorStyle(style);
```  

 
    

[code](../vtk/RotateActor.cxx)

