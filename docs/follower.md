`vtkFollower` remains facing the camera

```cpp

vtkNew<vtkVectorText> textSource;
textSource->SetText("Hello");


vtkNew<vtkPolyDataMapper> mapper;
mapper->SetInputConnection(textSource->GetOutputPort());

vtkNew<vtkFollower> follower;
follower->SetMapper(mapper);


renderer->AddActor(follower);
follower->SetCamera(renderer->GetActiveCamera());
```

[code](../src/Follower.cxx)
