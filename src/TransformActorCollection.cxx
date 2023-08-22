// #include <vtkActor.h>
// #include <vtkActorCollection.h>
// #include <vtkConeSource.h>
// #include <vtkCubeSource.h>
// #include <vtkInteractorStyleTrackballActor.h>
// #include <vtkNamedColors.h>
// #include <vtkNew.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSphereSource.h>
// #include <vtkTransform.h>

// int main(int, char *[]) {
//   vtkNew<vtkNamedColors> colors;

//   vtkNew<vtkRenderer> renderer;

//   // Create a cone.
//   vtkNew<vtkConeSource> coneSource;
//   coneSource->SetHeight(3);

//   vtkNew<vtkPolyDataMapper> coneMapper;
//   coneMapper->SetInputConnection(coneSource->GetOutputPort());
//   vtkNew<vtkActor> coneActor;
//   coneActor->SetMapper(coneMapper);
//   coneActor->GetProperty()->SetColor(colors->GetColor3d("MistyRose").GetData());

//   // Create a cube.
//   vtkNew<vtkCubeSource> cubeSource;

//   vtkNew<vtkPolyDataMapper> cubeMapper;
//   cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
//   vtkNew<vtkActor> cubeActor;
//   cubeActor->SetMapper(cubeMapper);
//   cubeActor->GetProperty()->SetColor(colors->GetColor3d("Cornsilk").GetData());

//   vtkNew<vtkTransform> transform;
//   transform->PostMultiply(); // This is the key line.
//   transform->Translate(5.0, 0, 0);

//   // Create a sphere.
//   vtkNew<vtkSphereSource> sphereSource;

//   vtkNew<vtkPolyDataMapper> sphereMapper;
//   sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
//   vtkNew<vtkActor> sphereActor;
//   sphereActor->SetMapper(sphereMapper);
//   // sphereActor->SetUserTransform(transform);
//   sphereActor->GetProperty()->SetColor(
//       colors->GetColor3d("Lavender").GetData());

//   renderer->AddActor(sphereActor);

//   vtkNew<vtkActorCollection> actorCollection;
//   actorCollection->AddItem(cubeActor);
//   actorCollection->AddItem(coneActor);
//   //   actorCollection->AddItem(sphereActor);
//   actorCollection->InitTraversal();

//   transform->Translate(-0.0, 0, 0);
//   // actorCollection->SetUserTransform(transform);

//   for (vtkIdType i = 0; i < actorCollection->GetNumberOfItems(); i++) {
//     vtkActor *actor = actorCollection->GetNextActor();
//     actor->SetUserTransform(transform);
//     renderer->AddActor(actor);
//   }

//   // Create a renderer, render window, and interactor.

//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetWindowName("TransformActorCollection");

//   vtkNew<vtkInteractorStyleTrackballActor> style;

//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);
//   renderWindowInteractor->SetInteractorStyle(style);

//   // Add the actor to the scene.
//   renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

//   // Render and interact.
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkCubeSource.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

int main(int, char *[]) {
  // Create a sphere and cube source
  vtkNew<vtkSphereSource> sphereSource;
  vtkNew<vtkCubeSource> cubeSource;
  cubeSource->SetCenter(2, 0, 0); // Offset for visualization

  // Map the sources to actors
  vtkNew<vtkPolyDataMapper> sphereMapper;
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkNew<vtkActor> sphereActor;
  sphereActor->SetMapper(sphereMapper);

  vtkNew<vtkPolyDataMapper> cubeMapper;
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkNew<vtkActor> cubeActor;
  cubeActor->SetMapper(cubeMapper);

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

  // Render
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballActor> style;
  renderWindowInteractor->SetInteractorStyle(style);

  renderer->AddActor(sphereActor);
  renderer->AddActor(cubeActor);
  renderer->SetBackground(0.1, 0.1, 0.1); // Dark background

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
