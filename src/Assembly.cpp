#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

int main(int, char *[]) {
  // Create two simple actors
  vtkNew<vtkSphereSource> sphereSource1;
  vtkNew<vtkPolyDataMapper> mapper1;
  mapper1->SetInputConnection(sphereSource1->GetOutputPort());
  vtkNew<vtkActor> actor1;
  actor1->SetMapper(mapper1);
  actor1->SetPosition(0, 0, 0);

  vtkNew<vtkSphereSource> sphereSource2;
  vtkNew<vtkPolyDataMapper> mapper2;
  mapper2->SetInputConnection(sphereSource2->GetOutputPort());
  vtkNew<vtkActor> actor2;
  actor2->SetMapper(mapper2);
  actor2->SetPosition(
      1, 0, 0); // Position it slightly to the right of the first sphere

  // Create an assembly and add the actors to it
  vtkNew<vtkAssembly> assembly;
  assembly->AddPart(actor1);
  assembly->AddPart(actor2);

  // Now, any transformation applied to the assembly will affect both actors
  assembly->RotateZ(45); // Rotate the assembly around the Z-axis

  // Visualization pipeline
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the assembly to the renderer
  renderer->AddActor(assembly);
  renderer->SetBackground(0.1, 0.1, 0.1);

  renderWindow->Render();

  vtkNew<vtkInteractorStyleTrackballActor> style;
  renderWindowInteractor->SetInteractorStyle(style);

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
