To remove an actor from a `vtkRenderer` in VTK, you can use the `RemoveActor` method provided by the `vtkRenderer` class.

Here's a simple example:

```cpp
vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();

// Add actor to renderer
renderer->AddActor(actor);

// Later in the code, when you want to remove the actor:
renderer->RemoveActor(actor);
```

After calling `RemoveActor`, the actor will no longer be part of the rendering process. If you render the scene after removing the actor, the actor will not be visible.

Remember, the `vtkRenderer` doesn't take ownership of the actor, so you still have to manage the memory (or rely on `vtkSmartPointer` to do that automatically). If you were not using `vtkSmartPointer` and created the actor with a `new` keyword, you'd need to call `delete` on the actor after removing it from the renderer to ensure no memory leaks. However, with `vtkSmartPointer`, the memory management is handled for you.
