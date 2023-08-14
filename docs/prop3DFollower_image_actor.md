


```cpp
vtkNew<vtkFollower> follower;

follower->SetMapper(mapper);
follower->SetPosition(1, 2, 3);


vtkNew<vtkPNGReader> pnmReader;
pnmReader->SetFileName("../images/maps-marker.png");

vtkNew<vtkImageActor> ia;
ia->GetMapper()->SetInputConnection(pnmReader->GetOutputPort());
ia->SetScale(0.01, 0.01, 0.01);

vtkNew<vtkProp3DFollower> p3dFollower;

p3dFollower->SetProp3D(ia);
```

[code](../vtk/vtkProp3DFollower.cxx)

