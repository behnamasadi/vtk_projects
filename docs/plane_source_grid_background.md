# vtkPlaneSource

The plane is defined by specifying three points: the origin (`SetOrigin`) and two other points (`SetPoint1` and `SetPoint2`). These three points define a parallelogram in space.

1. **SetOrigin**: This method sets the location of the origin of the plane. The origin is one corner of the parallelogram and serves as a reference point for the plane. In 3D space, this point is typically defined by three coordinates (x, y, z).

2. **SetPoint1**: This method sets the position of a point that, together with the origin, defines one of the axes of the plane. The vector from the origin to this point represents one side of the parallelogram and determines the orientation and length of that side.

3. **SetPoint2**: This method sets the position of another point that defines the second axis of the plane when combined with the origin. The vector from the origin to this point represents the adjacent side of the parallelogram and determines the orientation and length of that side.

When you define the origin and these two points, you're effectively defining two vectors on the plane:

- The vector from `SetOrigin` to `SetPoint1` (Vector A)
- The vector from `SetOrigin` to `SetPoint2` (Vector B)

These vectors span a parallelogram in the plane. The cross product of these two vectors gives the normal of the plane, which is perpendicular to the plane's surface.

Here's a visual representation to help you understand:

```
     SetPoint2
        +
       /|
      / |
     /  |
    /   |
   /    |
  +-----+
Origin  SetPoint1
```

In this diagram, the `+` signs represent the points. The origin is where you start, and you define the plane by specifying how far along and in what direction you move to get to `SetPoint1` and `SetPoint2`. The edges of the parallelogram are the vectors from the origin to each of these points.

When you use these methods to define a plane in VTK, the `vtkPlaneSource` automatically computes the necessary parameters to render the plane within the bounds defined by these three points. The plane itself is not limited to the parallelogram; that's just the portion that will be rendered by VTK when you use this source in a pipeline. The actual plane is infinite, but for practical visualization purposes, we usually visualize only a finite section.



```cpp
vtkNew<vtkPlaneSource> planeSource;
planeSource->SetXResolution(
    10); // Set the number of grid lines in the X direction
planeSource->SetYResolution(
    10); // Set the number of grid lines in the Y direction

// Map the plane's data to graphics primitives
vtkNew<vtkPolyDataMapper> planeMapper;
planeMapper->SetInputConnection(planeSource->GetOutputPort());

  // Create an actor for the plane and set its properties to display the grid
  // lines
vtkNew<vtkActor> planeActor;
planeActor->SetMapper(planeMapper);
planeActor->GetProperty()
    ->SetRepresentationToWireframe();               // Display as wireframe
planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to gray

planeActor->GetProperty()->SetAmbient(1.0);
planeActor->GetProperty()->SetDiffuse(0.0);

planeActor->PickableOff();
```
