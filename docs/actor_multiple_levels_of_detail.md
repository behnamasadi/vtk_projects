# vtkLODActor


`vtkLODActor` is an actor that stores multiple levels of detail (LOD) and can automatically switch between them. It selects which level of detail to use based on how much time it has been allocated to render.


There are three levels of detail by default.
1. The top level is just the normal data. 
2. The lowest level of detail is a simple bounding box outline of the actor. 
3. The middle level of detail is a point cloud of a fixed number of points that have been randomly sampled from the mapper's input data. 

These two lower levels of detail are accomplished by creating instances of a vtkOutlineFilter (low-res) and vtkMaskPoints (medium-res). Additional levels of detail can be add using the AddLODMapper() method.
