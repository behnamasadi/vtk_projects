# Frustum
The camera's field of view in 3D graphics can be visualized as a pyramid with its top cut off. This shape is called a "frustum", and it represents the volume of space that is visible from the camera's perspective. Anything outside of this volume is not visible on the screen and therefore doesn't need to be rendered.

[code](../src/CameraActor.cxx)

# Frustum Culling
Frustum culling is a technique used in computer graphics and game development to improve rendering performance. It involves not rendering objects that are outside of the camera's field of view (or "frustum").
