
#include "InteractorStyleSwitch.hpp"
#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {
  vtkNew<vtkConeSource> cone;
  vtkNew<vtkPolyDataMapper> mapper;
  vtkNew<vtkActor> actor;

  mapper->SetInputConnection(cone->GetOutputPort());
  actor->SetMapper(mapper);
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> winRen;
  vtkNew<vtkRenderWindowInteractor> iRen;

  vtkNew<InteractorStyleSwitch> style;

  style->SetCurrentRenderer(renderer);

  renderer->AddActor(actor);
  winRen->AddRenderer(renderer);
  iRen->SetInteractorStyle(style);

  winRen->SetInteractor(iRen);

  iRen->Start();
}
