## Callback
There are times when VTK will know that something has happened and it will send out a command notifying all observers of the event. To catch these commands:

First Create a function with this signature: 
```cpp
void func(vtkObject*, unsigned long eid, void* clientdata, void *calldata)
```

Now you have two options, 
1. Create a `vtkCallbackCommand` and set its callback to the function you have just created:

```cpp
vtkSmartPointer<vtkCallbackCommand> keypressCallback =  vtkSmartPointer<vtkCallbackCommand>::New();
keypressCallback->SetCallback ( func );
```

2. Alternatively, you can create a class that inherit from `vtkCommand`  and reimplement  `virtual void Execute(vtkObject *caller, unsigned long, void*)`

```cpp
struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {

    auto caller = reinterpret_cast<vtkRenderer *>(caller);

};
```


Refs: [1](https://vtk.org/Wiki/VTK/Tutorials/Callbacks), [2](https://vtk.org/doc/nightly/html/classvtkCommand.html)

### caller
The `vtkObject` "caller" is a pointer to the observer and it can be cast to the type of observer. for instance if you add `renderer->AddObserver` then you can `auto renderer = reinterpret_cast<vtkRenderer *>(caller);`

### clientdata
clientdata provides a way to provide access to data that will be necessary in the callback function, you can set anything

### calldata
calldata is data that may be sent with the callback. For example, when the ProgressEvent event is sent, it sends the progress value as calldata. 
## Observer

VTK uses a command/observer design pattern. That is, observers watch for particular events that any vtkObject (or subclass) may invoke on itself. For example, the vtkRenderer invokes a "StartEvent" as it begins to render. 

```cpp
struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {

    auto renderer = reinterpret_cast<vtkRenderer *>(caller);

    std::cout << renderer->GetActiveCamera()->GetPosition()[0] << ", "
              << renderer->GetActiveCamera()->GetPosition()[1] << ", "
              << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
  }
};


renderer->AddObserver(vtkCommand::StartEvent, mycallback);


```


[code](../vtk/command_observer_for_events.cpp)


Refs: [1](https://examples.vtk.org/site/Cxx/Tutorial/Tutorial_Step2)


Another way:

```cpp
void PickCallbackFunction(vtkObject* caller,
                          long unsigned int vtkNotUsed(eventId),
                          void* vtkNotUsed(clientData),
                          void* vtkNotUsed(callData))
{
  std::cout << "Pick." << std::endl;
  vtkAreaPicker* areaPicker = static_cast<vtkAreaPicker*>(caller);
  vtkProp3DCollection* props = areaPicker->GetProp3Ds();
  props->InitTraversal();

  for (vtkIdType i = 0; i < props->GetNumberOfItems(); i++)
  {
    vtkProp3D* prop = props->GetNextProp3D();
    std::cout << "Picked prop: " << prop << std::endl;
  }
}
```

and in your code:

```
vtkNew<vtkCallbackCommand> pickCallback;
pickCallback->SetCallback(PickCallbackFunction);
areaPicker->AddObserver(vtkCommand::EndPickEvent, pickCallback);
```

[code](../vtk/AreaPicking.cxx)





