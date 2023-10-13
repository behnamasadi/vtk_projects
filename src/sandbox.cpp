#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkConeSource.h>
#include <vtkFollower.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkLookupTable.h>
#include <vtkMatrix4x4.h>
#include <vtkNew.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProp3DFollower.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>

int main(int, char *[]) {
  // Create a simple sphere
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->Update();

  vtkNew<vtkConeSource> coneSource;
  coneSource->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> sphereMapper;
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(coneSource->GetOutputPort());

  vtkNew<vtkActor> sphereActor;
  sphereActor->SetMapper(sphereMapper);

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  // coneActor->SetPosition(5, 2, 3);

  // Create a renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor and scalar bar to the scene
  // renderer->AddActor(sphereActor);
  // renderer->AddActor(coneActor);

  vtkNew<vtkTransform> transform;
  vtkNew<vtkMatrix4x4> m;

  m->Identity();
  m->SetElement(0, 0, .3);
  m->SetElement(1, 1, .3);
  m->SetElement(2, 2, .3);

  m->SetElement(0, 3, 10);
  m->SetElement(1, 3, 3);

  transform->SetMatrix(m);
  // transform->Scale(5, 5, 5);
  double scale[3];

  transform->GetScale(scale);

  std::cout << "-----------" << scale[0] << "," << scale[1] << "," << scale[2]
            << std::endl;

  sphereActor->GlobalWarningDisplayOn();
  // sphereActor->SetUserTransform(transform);

  // vtkNew<renderWindowInteractor> i_ren;

  // // actor->SetUserMatrix(m);

  // // Render and interact

  /*
    vtkNew<vtkPNGReader> pnmReader;
    pnmReader->SetFileName("../../images/maps-marker.png");

    vtkNew<vtkImageActor> ia;
    ia->GetMapper()->SetInputConnection(pnmReader->GetOutputPort());
    ia->SetScale(0.01, 0.01, 0.01);

    vtkNew<vtkProp3DFollower> p3dFollower;

    p3dFollower->SetProp3D(ia);
    p3dFollower->SetCamera(renderer->GetActiveCamera());

    vtkNew<vtkAssembly> assembly;
    assembly->AddPart(sphereActor);
    assembly->AddPart(coneActor);
    assembly->AddPart(p3dFollower);

    renderer->AddActor(assembly);
    // renderer->AddActor(p3dFollower);

    renderWindow->Render();

    */

  vtkNew<vtkPNGReader> pnmReader;
  pnmReader->SetFileName("../../images/maps-marker.png");

  vtkNew<vtkImageActor> ia;
  ia->GetMapper()->SetInputConnection(pnmReader->GetOutputPort());
  ia->SetScale(0.01, 0.01, 0.01);

  vtkNew<vtkProp3DFollower> p3dFollower;

  p3dFollower->SetProp3D(ia);
  p3dFollower->SetCamera(renderer->GetActiveCamera());

  vtkNew<vtkAssembly> assembly;
  assembly->AddPart(sphereActor);
  assembly->AddPart(coneActor);

  renderer->AddActor(assembly);
  renderer->AddActor(p3dFollower); // Add the follower directly to the renderer

  assembly->SetUserMatrix(m);

  // Ensure the camera settings and update the view
  renderer->ResetCamera();
  renderWindow->Render();

  vtkNew<vtkInteractorStyleTrackballActor> style;

  // renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
