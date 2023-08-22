// #include <vtkActor.h>
// #include <vtkCallbackCommand.h>
// #include <vtkInteractorStyleTrackballActor.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSmartPointer.h>
// #include <vtkSphereSource.h>
// #include <vtkTransform.h>

// // The callback that applies the same transform to all actors
// void TransformCallbackFunc(vtkObject *caller, unsigned long eventId,
//                            void *clientData, void *callData) {

//   std::cout << "***" << std::endl;
//   vtkActor *primaryActor = static_cast<vtkActor *>(caller);
//   vtkTransform *transform = vtkTransform::New();
//   transform->SetMatrix(primaryActor->GetMatrix());

//   std::vector<vtkActor *> *otherActors =
//       static_cast<std::vector<vtkActor *> *>(clientData);
//   for (vtkActor *actor : *otherActors) {
//     actor->SetUserTransform(transform);
//   }

//   transform->Delete();
// }

// int main(int, char *[]) {
//   // Create some objects (e.g., spheres for simplicity)
//   vtkNew<vtkSphereSource> sphereSource1;
//   sphereSource1->SetRadius(2);
//   sphereSource1->Update();

//   vtkNew<vtkPolyDataMapper> mapper1;
//   mapper1->SetInputConnection(sphereSource1->GetOutputPort());
//   vtkNew<vtkActor> actor1;
//   actor1->SetMapper(mapper1);

//   vtkNew<vtkSphereSource> sphereSource2;
//   sphereSource2->SetCenter(2, 0, 0);
//   vtkNew<vtkPolyDataMapper> mapper2;
//   mapper2->SetInputConnection(sphereSource2->GetOutputPort());
//   vtkNew<vtkActor> actor2;
//   actor2->SetMapper(mapper2);

//   // This vector will hold actors that should be transformed with the primary
//   // actor
//   std::vector<vtkActor *> otherActors;
//   otherActors.push_back(actor2);

//   // Setup renderer, render window, and interactor
//   vtkNew<vtkRenderer> renderer;
//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add actors to the renderer
//   renderer->AddActor(actor1);
//   renderer->AddActor(actor2);

//   // Setup interactor style
//   vtkNew<vtkInteractorStyleTrackballActor> style;
//   renderWindowInteractor->SetInteractorStyle(style);

//   // Add the observer
//   vtkNew<vtkCallbackCommand> transformCallback;
//   transformCallback->SetCallback(TransformCallbackFunc);
//   transformCallback->SetClientData(&otherActors);
//   actor1->AddObserver(vtkCommand::EndInteractionEvent, transformCallback);
//   renderWindowInteractor->Initialize();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

// The callback that applies the same transform to all actors
void TransformCallbackFunc(vtkObject *caller, unsigned long eventId,
                           void *clientData, void *callData) {
  vtkActor *primaryActor = static_cast<vtkActor *>(caller);
  vtkTransform *transform = vtkTransform::New();
  transform->SetMatrix(primaryActor->GetMatrix());

  std::vector<vtkActor *> *otherActors =
      static_cast<std::vector<vtkActor *> *>(clientData);
  for (vtkActor *actor : *otherActors) {
    actor->SetUserTransform(transform);
  }

  transform->Delete();
}

int main(int, char *[]) {
  // Create some objects (e.g., spheres for simplicity)
  vtkSmartPointer<vtkSphereSource> sphereSource1 =
      vtkSmartPointer<vtkSphereSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> mapper1 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper1->SetInputConnection(sphereSource1->GetOutputPort());
  vtkSmartPointer<vtkActor> actor1 = vtkSmartPointer<vtkActor>::New();
  actor1->SetMapper(mapper1);

  vtkSmartPointer<vtkSphereSource> sphereSource2 =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource2->SetCenter(2, 0, 0);
  vtkSmartPointer<vtkPolyDataMapper> mapper2 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper2->SetInputConnection(sphereSource2->GetOutputPort());
  vtkSmartPointer<vtkActor> actor2 = vtkSmartPointer<vtkActor>::New();
  actor2->SetMapper(mapper2);

  // This vector will hold actors that should be transformed with the primary
  // actor
  std::vector<vtkActor *> otherActors;
  otherActors.push_back(actor2);

  // Setup renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add actors to the renderer
  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

  // Setup interactor style
  vtkSmartPointer<vtkInteractorStyleTrackballActor> style =
      vtkSmartPointer<vtkInteractorStyleTrackballActor>::New();
  renderWindowInteractor->SetInteractorStyle(style);

  // Add the observer
  vtkSmartPointer<vtkCallbackCommand> transformCallback =
      vtkSmartPointer<vtkCallbackCommand>::New();
  transformCallback->SetCallback(TransformCallbackFunc);
  transformCallback->SetClientData(&otherActors);
  actor1->AddObserver(vtkCommand::EndInteractionEvent, transformCallback);

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
