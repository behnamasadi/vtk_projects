## Frustum Source
Source like circle source, 

[code](../src/CameraActor.cxx)

## Frustum Culling
Frustum culling is a technique used in computer graphics and game development to improve rendering performance. It involves not rendering objects that are outside of the camera's field of view (or "frustum").



## Camera Frustum Planes
The camera's field of view in 3D graphics can be visualized as a pyramid with its top cut off. This shape is called a "frustum", and it represents the volume of space that is visible from the camera's perspective. Anything outside of this volume is not visible on the screen and therefore doesn't need to be rendered.
```cpp
void vtkCamera::GetFrustumPlanes (double aspect,double 	planes[24] )
```

Get the plane equations that bound the view frustum. The plane normals point inward. The planes array contains six plane equations of the form `Ax+By+Cz+D=0`, the first four values are (A,B,C,D) which repeats for each of the planes. The planes are given in the following order: -x,+x,-y,+y,-z,+z. Warning: it means left,right,bottom,top,far,near (NOT near,far) The aspect of the viewport is needed to correctly compute the planes. aspect is ignored if UseExplicitAspectRatio is true.


