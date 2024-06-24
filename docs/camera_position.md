## Camera Position

In your main:

```cpp
vtkNew<vtkCallbackCommand> cameraCallbackCommand;
cameraCallbackCommand->SetCallback(CameraCallback);
renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent, cameraCallbackCommand);
```

Call back function:
```cpp
void CameraCallback(vtkObject *caller, long unsigned int vtkNotUsed(eventId),
                    void *vtkNotUsed(clientData), void *vtkNotUsed(callData)) {

  std::cout << caller->GetClassName() << " modified" << std::endl;

  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  // Print the interesting stuff.
  std::cout << "  auto camera = renderer->GetActiveCamera();" << std::endl;
  std::cout << "  camera->SetPosition(" << camera->GetPosition()[0] << ", "
            << camera->GetPosition()[1] << ", " << camera->GetPosition()[2]
            << ");" << std::endl;
  std::cout << "  camera->SetFocalPoint(" << camera->GetFocalPoint()[0] << ", "
            << camera->GetFocalPoint()[1] << ", " << camera->GetFocalPoint()[2]
            << ");" << std::endl;
  std::cout << "  camera->SetViewUp(" << camera->GetViewUp()[0] << ", "
            << camera->GetViewUp()[1] << ", " << camera->GetViewUp()[2] << ");"
            << std::endl;
  std::cout << "  camera->SetDistance(" << camera->GetDistance() << ");"
            << std::endl;
  std::cout << "  camera->SetClippingRange(" << camera->GetClippingRange()[0]
            << ", " << camera->GetClippingRange()[1] << ");" << std::endl;
  std::cout << std::endl;
}
```


Refs: [1](https://examples.vtk.org/site/Cxx/Snippets/CameraPosition/)  
[code](../src/CameraPosition.cpp)







```cpp
class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }



  virtual void OnMouseWheelForward() override {
    std::cout << "OnMouseWheelForward" << std::endl;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;
      }
    }

    std::cout << "renderer->GetActors()->GetNumberOfItems():"
              << renderer->GetActors()->GetNumberOfItems() << std::endl;

    for (int i = 0; i < renderer->GetActors()->GetNumberOfItems(); i++) {

      std::cout << "renderer->GetActors()[i].GetClassName():"
                << renderer->GetActors()[i].GetClassName() << std::endl;
    }

    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
  }

  virtual void OnMouseWheelBackward() override {
    std::cout << "OnMouseWheelBackward" << std::endl;

    vtkRenderWindowInteractor *rwi = Interactor;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      //   camera->GetFrustumPlanes()
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;
      }
    }

    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
  }
  
    virtual void OnChar() override {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = Interactor;
    std::string key = rwi->GetKeySym();

    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Output the key that was pressed
    std::cout << "OnChar() " << key.c_str() << std::endl;

    // Handle an arrow key
    if (key == "up") {
      std::cout << "The up arrow was pressed." << std::endl;
    }

    if (key == "delete") {
      std::cout << "The delete key was pressed." << std::endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }
    virtual void Dolly() override {

    //   right click down + <- ->
    //   camera should rotate in its place, to do that we calculate the azimuth
    //   that rotate teh camera around We should store the old camera position,
    //   apply azimuth, then we calculate forward vector from camera to the
    //   focal point, then from previously stored camera position we go forward
    //   in the direction of forward vector and calculate the new camera focal
    //   point
    std::cout << "Dolly" << std::endl;

    vtkInteractorStyleTrackballCamera::Dolly();
  }
};
```


in main

```cpp
 vtkCamera *camera = renderer->GetActiveCamera();
  auto viewAngle = camera->GetViewAngle();
  std::cout << viewAngle << std::endl;

  auto clippingRange = camera->GetClippingRange();

  std::cout << clippingRange[0] << "," << clippingRange[1] << std::endl;

  camera->SetPosition(150, 0, 150);
  camera->SetFocalPoint(0, 0, 0);
  camera->Azimuth(30);
  camera->Elevation(30);
  camera->SetViewUp(0, 0, 1);
  camera->SetClippingRange(5, 15);
  camera->SetViewAngle(30);
```


[source](../src/camera_call_back_events.cpp), and [here](../src/CameraInteractorStyle.cpp)
