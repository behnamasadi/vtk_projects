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

#include <vtkActor.h>
#include <vtkCubeSource.h>
#include <vtkLegendScaleActor.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

int main(int, char *[]) {
  // Create a cube.
  vtkSmartPointer<vtkCubeSource> cubeSource =
      vtkSmartPointer<vtkCubeSource>::New();

  cubeSource->SetXLength(1.0);
  cubeSource->SetYLength(1.0);
  cubeSource->SetZLength(1.0);
  cubeSource->Update();

  // Create a mapper and actor.
  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(cubeSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // Create a renderer, render window, and interactor.
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene.
  renderer->AddActor(actor);

  // Create the legend scale actor.
  vtkSmartPointer<vtkLegendScaleActor> legendScaleActor =
      vtkSmartPointer<vtkLegendScaleActor>::New();

  renderer->AddActor(legendScaleActor); // Add scale actor to the renderer.

  // Render and interact.
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
