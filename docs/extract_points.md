## vtkExtractPoints

`vtkExtractPoints` removes points that are either inside or outside of a `vtkImplicitFunction`. Implicit functions in VTK defined as function of the form `f(x,y,z)=c`, where values `c<=0` are interior values of the implicit function. Typical examples include planes, spheres, cylinders, cones, etc. plus boolean combinations of these functions. (This operation presumes closure on the set, so points on the boundary are also considered to be inside.)

Refs: [1](https://vtk.org/doc/nightly/html/classvtkExtractPoints.html#details)
