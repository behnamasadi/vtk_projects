The basic setup of `source -> mapper -> actor -> renderer -> renderwindow`

```cpp

vtkNew<vtkConeSource> cone;
vtkNew<vtkPolyDataMapper> mapper;
mapper->SetInputConnection(cone->GetOutputPort());

vtkNew<vtkRenderer> renderer;

vtkNew<vtkActor> actor;
actor->SetMapper(mapper);


renderer->AddActor(actor);

vtkNew<vtkRenderWindow> renWin;



renWin->AddRenderer(renderer);

for (int i = 0; i < 360; ++i) {
	// Render the image
	renWin->Render();

	// Rotate the camera about the view up vector centered at the focal point.
	renderer->GetActiveCamera()->Azimuth(1);
}
```

[code](../vtk/the_basic_setup.cpp)


Refs: [1](https://examples.vtk.org/site/Cxx/Tutorial/Tutorial_Step1)
