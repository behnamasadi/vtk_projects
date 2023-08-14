```cpp
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera
{
public:
  static KeyPressInteractorStyle* New();
  vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnKeyPress() override
  {
    // Get the keypress
    vtkRenderWindowInteractor* rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    // Output the key that was pressed
    std::cout << "Pressed " << key << std::endl;

    // Handle an arrow key
    if (key == "Up")
    {
      std::cout << "The up arrow was pressed." << std::endl;
    }

    // Handle a "normal" key
    if (key == "a")
    {
      std::cout << "The a key was pressed." << std::endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }
};
vtkStandardNewMacro(KeyPressInteractorStyle);
```

In your main 

```cpp
vtkNew<vtkRenderer> renderer;
vtkNew<vtkRenderWindow> renderWindow;
renderWindow->AddRenderer(renderer);
renderWindow->SetWindowName("KeypressEvents");

// An interactor
vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
renderWindowInteractor->SetRenderWindow(renderWindow);

vtkNew<KeyPressInteractorStyle> style;
renderWindowInteractor->SetInteractorStyle(style);
style->SetCurrentRenderer(renderer);
```


