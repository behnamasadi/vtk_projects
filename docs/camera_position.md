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
