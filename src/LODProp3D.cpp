#include <vtkCallbackCommand.h>
#include <vtkLODProp3D.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

// namespace {
// void RefreshCallback(vtkObject *vtkNotUsed(caller),
//                      long unsigned int vtkNotUsed(eventId), void *clientData,
//                      void *vtkNotUsed(callData));
// }

// int main(int, char *[]) {
//   vtkNew<vtkNamedColors> colors;

//   // High res sphere.
//   vtkNew<vtkSphereSource> highResSphereSource;
//   int res = 1000;
//   highResSphereSource->SetThetaResolution(res);
//   highResSphereSource->SetPhiResolution(res);
//   highResSphereSource->Update();

//   vtkNew<vtkPolyDataMapper> highResMapper;
//   highResMapper->SetInputConnection(highResSphereSource->GetOutputPort());

//   // Low res sphere.
//   vtkNew<vtkSphereSource> lowResSphereSource;

//   vtkNew<vtkPolyDataMapper> lowResMapper;
//   lowResMapper->SetInputConnection(lowResSphereSource->GetOutputPort());

//   vtkNew<vtkProperty> propertyLowRes;
//   propertyLowRes->SetDiffuseColor(
//       colors->GetColor3d("BlanchedAlmond").GetData());
//   propertyLowRes->SetInterpolationToFlat();

//   vtkNew<vtkProperty> propertyHighRes;
//   propertyHighRes->SetDiffuseColor(colors->GetColor3d("MistyRose").GetData());
//   propertyHighRes->SetInterpolationToFlat();

//   vtkNew<vtkLODProp3D> prop;
//   prop->AddLOD(lowResMapper, propertyLowRes, 0.0);
//   prop->AddLOD(highResMapper, propertyHighRes, 0.0);

//   std::cout << "There are " << prop->GetNumberOfLODs() << " LODs" <<
//   std::endl;

//   // A renderer and render window.
//   vtkNew<vtkRenderer> renderer;
//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetWindowName("LODProp3D");

// //   prop->SetAllocatedRenderTime(1e-6, renderer);
//     prop->SetAllocatedRenderTime(1e-12, renderer);

//   // An interactor
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add the actors to the scene.
//   renderer->AddActor(prop);
//   renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

//   vtkNew<vtkCallbackCommand> refreshCallback;
//   refreshCallback->SetCallback(RefreshCallback);
//   refreshCallback->SetClientData(prop);

//   renderWindow->AddObserver(vtkCommand::ModifiedEvent, refreshCallback);

//   renderWindow->Render();

//   // Begin mouse interaction.
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

// namespace {
// void RefreshCallback(vtkObject *vtkNotUsed(caller),
//                      long unsigned int vtkNotUsed(eventId), void *clientData,
//                      void *vtkNotUsed(callData)) {
//   auto lodProp = static_cast<vtkLODProp3D *>(clientData);
//   std::cout << "Last rendered LOD ID: " << lodProp->GetLastRenderedLODID()
//             << std::endl;
// }
// } // namespace

// #include <vtkActor.h>
// #include <vtkCallbackCommand.h>
// #include <vtkCommand.h>
// #include <vtkLODProp3D.h>
// #include <vtkNamedColors.h>
// #include <vtkNew.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSphereSource.h>

// namespace {
// void RefreshCallback(vtkObject *vtkNotUsed(caller),
//                      long unsigned int vtkNotUsed(eventId), void *clientData,
//                      void *vtkNotUsed(callData));
// void InteractionEndCallback(vtkObject *caller,
//                             long unsigned int vtkNotUsed(eventId),
//                             void *clientData, void *vtkNotUsed(callData));
// } // namespace

// int main(int, char *[]) {
//   vtkNew<vtkNamedColors> colors;

//   // High res sphere.
//   vtkNew<vtkSphereSource> highResSphereSource;
//   int res = 10000;
//   highResSphereSource->SetThetaResolution(res);
//   highResSphereSource->SetPhiResolution(res);
//   highResSphereSource->Update();

//   vtkNew<vtkPolyDataMapper> highResMapper;
//   highResMapper->SetInputConnection(highResSphereSource->GetOutputPort());

//   // Low res sphere.
//   vtkNew<vtkSphereSource> lowResSphereSource;

//   vtkNew<vtkPolyDataMapper> lowResMapper;
//   lowResMapper->SetInputConnection(lowResSphereSource->GetOutputPort());

//   vtkNew<vtkProperty> propertyLowRes;
//   propertyLowRes->SetDiffuseColor(
//       colors->GetColor3d("BlanchedAlmond").GetData());
//   propertyLowRes->SetInterpolationToFlat();

//   vtkNew<vtkProperty> propertyHighRes;
//   propertyHighRes->SetDiffuseColor(colors->GetColor3d("MistyRose").GetData());
//   propertyHighRes->SetInterpolationToFlat();

//   vtkNew<vtkLODProp3D> prop;
//   prop->AddLOD(lowResMapper, propertyLowRes, 0.0);
//   prop->AddLOD(highResMapper, propertyHighRes, 0.0);
//   prop->SetAutomaticLODSelection(true);

//   std::cout << "There are " << prop->GetNumberOfLODs() << " LODs" <<
//   std::endl;

//   // A renderer and render window.
//   vtkNew<vtkRenderer> renderer;
//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetWindowName("LODProp3D");

//   prop->SetAllocatedRenderTime(1e-6, renderer);

//   // An interactor
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Add the actors to the scene.
//   renderer->AddActor(prop);
//   renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

//   // Interaction end callback
//   vtkNew<vtkCallbackCommand> interactionEndCallback;
//   interactionEndCallback->SetCallback(InteractionEndCallback);
//   interactionEndCallback->SetClientData(prop);
//   renderWindowInteractor->AddObserver(vtkCommand::EndInteractionEvent,
//                                       interactionEndCallback);

//   renderWindow->Render();

//   // Begin mouse interaction.
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

// namespace {
// void RefreshCallback(vtkObject *vtkNotUsed(caller),
//                      long unsigned int vtkNotUsed(eventId), void *clientData,
//                      void *vtkNotUsed(callData)) {
//   auto lodProp = static_cast<vtkLODProp3D *>(clientData);
//   std::cout << "Last rendered LOD ID: " << lodProp->GetLastRenderedLODID()
//             << std::endl;
// }

// void InteractionEndCallback(vtkObject *caller,
//                             long unsigned int vtkNotUsed(eventId),
//                             void *clientData, void *vtkNotUsed(callData)) {
//   auto lodProp = static_cast<vtkLODProp3D *>(clientData);
//   std::cout << "Interaction ended, switching to high-res model." <<
//   std::endl;
//   // lodProp->SetLOD(1); // Assuming high-res model is at index 1

//   lodProp->SetAllocatedRenderTime(1.0,
//                                   nullptr); // Ensure the high-res LOD is
//                                   used
//   static_cast<vtkRenderWindowInteractor
//   *>(caller)->GetRenderWindow()->Render();
// }
// } // namespace

#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkLODProp3D.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

namespace {
void RefreshCallback(vtkObject *vtkNotUsed(caller),
                     long unsigned int vtkNotUsed(eventId), void *clientData,
                     void *vtkNotUsed(callData));
void InteractionEndCallback(vtkObject *caller,
                            long unsigned int vtkNotUsed(eventId),
                            void *clientData, void *vtkNotUsed(callData));
} // namespace

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // High res sphere.
  vtkNew<vtkSphereSource> highResSphereSource;
  int res = 1000;
  highResSphereSource->SetThetaResolution(res);
  highResSphereSource->SetPhiResolution(res);
  highResSphereSource->Update();

  vtkNew<vtkPolyDataMapper> highResMapper;
  highResMapper->SetInputConnection(highResSphereSource->GetOutputPort());

  // Low res sphere.
  vtkNew<vtkSphereSource> lowResSphereSource;

  vtkNew<vtkPolyDataMapper> lowResMapper;
  lowResMapper->SetInputConnection(lowResSphereSource->GetOutputPort());

  vtkNew<vtkProperty> propertyLowRes;
  propertyLowRes->SetDiffuseColor(
      colors->GetColor3d("BlanchedAlmond").GetData());
  propertyLowRes->SetInterpolationToFlat();

  vtkNew<vtkProperty> propertyHighRes;
  propertyHighRes->SetDiffuseColor(colors->GetColor3d("MistyRose").GetData());
  propertyHighRes->SetInterpolationToFlat();

  vtkNew<vtkLODProp3D> prop;
  prop->AddLOD(lowResMapper, propertyLowRes, 0.0);
  prop->AddLOD(highResMapper, propertyHighRes, 0.0);

  std::cout << "There are " << prop->GetNumberOfLODs() << " LODs" << std::endl;

  // A renderer and render window.
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("LODProp3D");

  prop->SetAllocatedRenderTime(1e-6, renderer);

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene.
  renderer->AddActor(prop);
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  vtkNew<vtkCallbackCommand> refreshCallback;
  refreshCallback->SetCallback(RefreshCallback);
  refreshCallback->SetClientData(prop);

  renderWindow->AddObserver(vtkCommand::ModifiedEvent, refreshCallback);

  // Interaction end callback
  vtkNew<vtkCallbackCommand> interactionEndCallback;
  interactionEndCallback->SetCallback(InteractionEndCallback);
  interactionEndCallback->SetClientData(prop);
  renderWindowInteractor->AddObserver(vtkCommand::EndInteractionEvent,
                                      interactionEndCallback);

  renderWindow->Render();

  // Begin mouse interaction.
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

namespace {
void RefreshCallback(vtkObject *vtkNotUsed(caller),
                     long unsigned int vtkNotUsed(eventId), void *clientData,
                     void *vtkNotUsed(callData)) {
  auto lodProp = static_cast<vtkLODProp3D *>(clientData);
  std::cout << "Last rendered LOD ID: " << lodProp->GetLastRenderedLODID()
            << std::endl;
}

void InteractionEndCallback(vtkObject *caller,
                            long unsigned int vtkNotUsed(eventId),
                            void *clientData, void *vtkNotUsed(callData)) {
  auto lodProp = static_cast<vtkLODProp3D *>(clientData);
  std::cout << "Interaction ended, switching to high-res model." << std::endl;

  // Adjust the allocated render time to prioritize the high-res LOD
  lodProp->SetAllocatedRenderTime(1.0,
                                  nullptr); // Ensure the high-res LOD is used
  static_cast<vtkRenderWindowInteractor *>(caller)->GetRenderWindow()->Render();
}
} // namespace
