

```cpp
struct CallbackData {
  vtkActor *lowResActor;
  vtkActor *highResActor;
  vtkRenderer *renderer;
};
```
call back to hide/ show high/ low resolution actor:

```cpp
void CameraModifiedCallback(vtkObject *caller, long unsigned int eventId,
                            void *clientData, void *callData) {
  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  CallbackData *data = static_cast<CallbackData *>(clientData);

  vtkActor *lowResActor = data->lowResActor;
  vtkActor *highResActor = data->highResActor;
  vtkRenderer *renderer = data->renderer;

  // Get the distance of the camera from the focal point
  double distance = camera->GetDistance();

  // Define your threshold distance here
  double thresholdDistance = 2.5;

  // Switch visibility based on the distance
  if (distance > thresholdDistance) {
    lowResActor->SetVisibility(1);
    highResActor->SetVisibility(0);
  } else {
    lowResActor->SetVisibility(0);
    highResActor->SetVisibility(1);
  }

  renderer->GetRenderWindow()->Render();
}
```

main

```cpp
 // Create two point sources
  vtkNew<vtkPointSource> lowResPointSource;
  lowResPointSource->SetNumberOfPoints(5000);

  vtkNew<vtkPointSource> highResPointSource;
  highResPointSource->SetNumberOfPoints(50000000);

  // Create mappers
  vtkNew<vtkPolyDataMapper> lowResMapper;
  lowResMapper->SetInputConnection(lowResPointSource->GetOutputPort());

  vtkNew<vtkPolyDataMapper> highResMapper;
  highResMapper->SetInputConnection(highResPointSource->GetOutputPort());

  // Create actors
  vtkNew<vtkActor> lowResActor;
  lowResActor->SetMapper(lowResMapper);

  vtkNew<vtkActor> highResActor;
  highResActor->SetMapper(highResMapper);

  // Initially set low resolution actor visible and high resolution actor
  // invisible
  lowResActor->SetVisibility(1);
  highResActor->SetVisibility(0);

  // Create a renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(lowResActor);
  renderer->AddActor(highResActor);
  renderer->SetBackground(0, 0, 0); // Background color

  // Get the camera
  vtkCamera *camera = renderer->GetActiveCamera();

  // Create a callback command
  vtkSmartPointer<vtkCallbackCommand> callback =
      vtkSmartPointer<vtkCallbackCommand>::New();
  callback->SetCallback(CameraModifiedCallback);

  // Set up callback data
  CallbackData data;
  data.lowResActor = lowResActor;
  data.highResActor = highResActor;
  data.renderer = renderer;

  callback->SetClientData(&data);

  // Add the observer to the camera
  camera->AddObserver(vtkCommand::ModifiedEvent, callback);

  // Render and start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();
```

  
