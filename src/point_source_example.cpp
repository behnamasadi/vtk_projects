
#include <vtkActor.h>
#include <vtkInteractorStyleTrackball.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {

  vtkNew<vtkPointSource> sphere;
  sphere->SetCenter(1.0, 2.0, 1.0);
  sphere->SetNumberOfPoints(100);
  // sphere->SetLambda(-1);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphere->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> ren;
  ren->AddActor(actor);

  vtkNew<vtkRenderWindow> win_ren;
  win_ren->AddRenderer(ren);
  win_ren->Start();

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(win_ren);

  iren->Start();
}