# ActorCollection

```
  // Create an actor collection and add actors
  vtkNew<vtkActorCollection> actorCollection;
  actorCollection->AddItem(sphereActor);
  actorCollection->AddItem(cubeActor);


  vtkNew<vtkTransform> transform;
  transform->PostMultiply(); // This is the key line.
  transform->Translate(0.0, 0, 0);

  // very important, Apply the transform to all actors in the collection so if we move one, all other actors get moved
  actorCollection->InitTraversal();
  vtkActor *actor;
  while ((actor = actorCollection->GetNextActor()) != nullptr) {
    actor->SetUserTransform(transform);
    renderer->AddActor(actor);
  }


  // adding more
  actorCollection->AddItem(sphereActor);
  actorCollection->GetLastActor()->SetUserTransform(transform);
  renderer->AddActor(actorCollection->GetLastActor());
```

[code](../src/TransformActorCollection.cxx)

Refs: [1](https://examples.vtk.org/site/Cxx/Visualization/TransformActorCollection/)

An other approach is is using observer:

```cpp
// Custom command to sync transformations
class SyncTransformationsCommand : public vtkCommand {
public:
  static SyncTransformationsCommand *New() {
    return new SyncTransformationsCommand;
  }

  virtual void Execute(vtkObject *caller, unsigned long, void *) override {

    if (SourceActor && TargetActor) {

      vtkMatrix4x4 *currentMatrix = SourceActor->GetMatrix();
      //   if (!previousMatrix->DeepCopy(currentMatrix)) {
      //     TargetActor->SetUserMatrix(currentMatrix);
      // if (!currentMatrix) {
      previousMatrix->DeepCopy(currentMatrix);
      TargetActor->SetUserMatrix(currentMatrix);
      vtkIndent indent;
      std::cout << "currentMatrix: " << std::endl;
      currentMatrix->PrintSelf(std::cout, indent.GetNextIndent());

      //}
    }
  }

  void SetSourceActor(vtkActor *actor) {
    SourceActor = actor;
    previousMatrix->DeepCopy(actor->GetMatrix());
  }

  vtkActor *SourceActor = nullptr;
  vtkActor *TargetActor = nullptr;
  vtkNew<vtkMatrix4x4> previousMatrix;
};
```

in your cpp code:

```cpp
  vtkNew<vtkInteractorStyleTrackballActor> style;
  renderWindowInteractor->SetInteractorStyle(style);

  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

  // Add observer to synchronize transformations
  vtkNew<SyncTransformationsCommand> syncCommand;
  syncCommand->SourceActor = actor1;
  syncCommand->TargetActor = actor2;
  style->AddObserver(vtkCommand::InteractionEvent, syncCommand);
```

[code](../src/SyncTransformations.cpp)


# Assembly

To group multiple VTK widgets and actors such that moving one of them results in all of them moving together, you can use the vtkAssembly class. The vtkAssembly class is a composite data structure that allows you to group multiple actors together and treat them as a single entity.

```cpp
vtkNew<vtkAssembly> assembly;
assembly->AddPart(actor1);
assembly->AddPart(actor2);
assembly->RotateZ(45);
renderer->AddActor(assembly);
```

[code](../src/Assembly.cpp)
