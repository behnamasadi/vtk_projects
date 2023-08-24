Cleaning up all the `EventCallbackCommand` objects or handlers in a VTK-based application usually involves removing the observers that have been attached to the VTK objects and then releasing the callback commands themselves.

Typically, when an event callback is added to a VTK object, an observer ID is returned. You can use this ID to remove the observer later.

Here's a general procedure:

1. When adding an observer, store the returned observer ID:
```cpp
vtkSmartPointer<vtkCallbackCommand> myCallback = vtkSmartPointer<vtkCallbackCommand>::New();
myCallback->SetCallback(MyFunction);

unsigned long observerId = someVtkObject->AddObserver(vtkCommand::SomeEvent, myCallback);
```

2. When you want to remove this observer, use the stored ID:
```cpp
someVtkObject->RemoveObserver(observerId);
```

3. Finally, ensure that all references to the callback command are released. If you're using `vtkSmartPointer`, this will be handled automatically once the reference goes out of scope or is explicitly set to `nullptr`.

For a complete cleanup, you would follow this procedure for all VTK objects and events in your application.

If you have many observers and callbacks, it might be a good idea to manage them systematically, for instance, by using containers (like `std::map` or `std::vector`) to store observer IDs, associated VTK objects, and callback commands. This way, you can loop through the container to systematically remove observers and clean up.
