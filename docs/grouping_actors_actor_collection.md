# ActorCollection



```
  // Create an actor collection and add actors
  vtkNew<vtkActorCollection> actorCollection;
  actorCollection->AddItem(sphereActor);
  actorCollection->AddItem(cubeActor);

  // Create a common transform
  vtkNew<vtkTransform> transform;
  transform->RotateY(
      0); // Example transformation: Rotate 45 degrees about the Y axis

  // Apply the transform to all actors in the collection
  actorCollection->InitTraversal();
  vtkActor *actor;
  while ((actor = actorCollection->GetNextActor()) != nullptr) {
    actor->SetUserTransform(transform);
  }
```

[code](../src/TransformActorCollection.cxx)  




Refs: [1](https://examples.vtk.org/site/Cxx/Visualization/TransformActorCollection/)
