### vtkGlyph3D

`glyph`: a "glyph" refers to a small graphical object used to represent data points visually. Glyphs can be any geometric shape such as spheres, arrows, cones, or cubes, and are typically used to represent more complex data attributes like direction, magnitude, or other scalar properties of data at specific points in a dataset.

```cpp
vtkNew<vtkPoints> points;

  // location of  glyph in 3d space
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(10, 1, 1);
  points->InsertNextPoint(0, 1, 0);
  points->InsertNextPoint(1, 5, 0);
  points->InsertNextPoint(0, 0, 1);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);

  vtkNew<vtkCubeSource> cubeSource;

  vtkNew<vtkGlyph3D> glyph3D;
  glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
  glyph3D->SetInputData(polyData);
  glyph3D->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyph3D->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
```
[source](../src/glyph3D.cpp)


### vtkVertexGlyphFilter

This filter throws away all of the cells in the input and replaces them with a vertex on each point. The intended use of this filter is roughly equivalent to the vtkGlyph3D filter, except this filter is specifically for data that has many vertices, making the rendered result faster and less cluttered than the glyph filter. This filter may take a graph or point set as input.

