```cpp
vtkNew<vtkActor> actor;
actor->SetPosition(0, 0, 0);

vtkNew<vtkBillboardTextActor3D> textActor;
textActor->SetInput("");
textActor->SetPosition(actor->GetPosition());

renderer->AddActor(actor);
renderer->AddActor(textActor);
```

If you want to use a callback, do this:

```cpp
vtkNew<vtkCallbackCommand> actorCallback;
actorCallback->SetCallback(ActorCallback);
actorCallback->SetClientData(textActor);
actor->AddObserver(vtkCommand::ModifiedEvent, actorCallback);
```
definition of callback:

```cpp
void ActorCallback(vtkObject *caller, long unsigned int vtkNotUsed(eventId),
                   void *clientData, void *vtkNotUsed(callData)) {
  auto textActor = static_cast<vtkBillboardTextActor3D *>(clientData);
  auto actor = static_cast<vtkActor *>(caller);
  std::ostringstream label;
  label << std::setprecision(3) << actor->GetPosition()[0] << ", "
        << actor->GetPosition()[1] << ", " << actor->GetPosition()[2]
        << std::endl;
  textActor->SetPosition(actor->GetPosition());
  textActor->SetInput(label.str().c_str());
}
```

[code](../vtk/BillboardTextActor3D.cxx)

