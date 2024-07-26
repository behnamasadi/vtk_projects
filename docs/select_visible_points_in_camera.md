
## vtkSelectVisiblePoints

vtkSelectVisiblePoints is a filter that selects points based on whether they are visible or not. Visibility is determined by accessing the z-buffer of a rendering window. (The position of each input point is converted into display coordinates, and then the z-value at that point is obtained. If within the user-specified tolerance, the point is considered visible.)



Refs: [1](https://vtk.org/doc/nightly/html/classvtkSelectVisiblePoints.html#details)



The heart of the code

```cpp
  vtkNew<vtkSelectVisiblePoints> selectVisiblePoints;
  selectVisiblePoints->SetInputConnection(highResPointSource->GetOutputPort());
  selectVisiblePoints->SetRenderer(renderer);
  selectVisiblePoints->Update();
```  


you main:

```cpp

  vtkNew<vtkPointSource> highResPointSource;
  highResPointSource->SetNumberOfPoints(500000);

  // Create mappers

  vtkNew<vtkPolyDataMapper> highResMapper;
  highResMapper->SetInputConnection(highResPointSource->GetOutputPort());

  vtkNew<vtkActor> highResActor;
  highResActor->SetMapper(highResMapper);

  // Create a renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene

  renderer->AddActor(highResActor);
  renderer->SetBackground(0, 0, 0); // Background color

  // Get the camera
  vtkCamera *camera = renderer->GetActiveCamera();

  // Create a callback command
  vtkNew<vtkCallbackCommand> callback;
  callback->SetCallback(CameraModifiedCallback);

  // Set up client data
  void *data[2] = {renderer, highResPointSource};
  callback->SetClientData(data);

  // Add the observer to the camera
  camera->AddObserver(vtkCommand::ModifiedEvent, callback);

  // Render and start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();
```
data structure for passing data in SetClientData

```cpp
struct CallbackData {
  vtkActor *highResActor;
  vtkRenderer *renderer;
};
```

```cpp
void CameraModifiedCallback(vtkObject *caller, long unsigned int eventId,
                            void *clientData, void *callData) {
  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  vtkRenderer *renderer =
      static_cast<vtkRenderer *>(reinterpret_cast<void **>(clientData)[0]);
  vtkPointSource *highResPointSource =
      static_cast<vtkPointSource *>(reinterpret_cast<void **>(clientData)[1]);

  std::cout << "highResPointSource->GetNumberOfPoints()"
            << highResPointSource->GetNumberOfPoints() << std::endl;

  // Set up the vtkSelectVisiblePoints filter
  vtkNew<vtkSelectVisiblePoints> selectVisiblePoints;
  selectVisiblePoints->SetInputConnection(highResPointSource->GetOutputPort());
  selectVisiblePoints->SetRenderer(renderer);
  selectVisiblePoints->Update();

  // Get the number of visible points
  vtkIdType numberOfVisiblePoints =
      selectVisiblePoints->GetOutput()->GetNumberOfPoints();

  std::cout << "Number of visible points: " << numberOfVisiblePoints
            << std::endl;

  renderer->GetRenderWindow()->Render();
}
```


[source](../src/select_visible_points.cpp)
