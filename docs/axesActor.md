

```cpp
vtkNew<vtkTransform> transform;
transform->Translate(1.0, 0.0, 0.0);

vtkNew<vtkAxesActor> axes;

// The axes are positioned with a user transform
axes->SetUserTransform(transform);

// properties of the axes labels can be set as follows
// this sets the x axis label to red
// axes->GetXAxisCaptionActor2D()->GetCaptionTextProperty()->SetColor(
//   colors->GetColor3d("Red").GetData());

// the actual text of the axis label can be changed:
// axes->SetXAxisLabelText("test");

renderer->AddActor(axes);
```

[code](../vtk/Axes.cxx)

