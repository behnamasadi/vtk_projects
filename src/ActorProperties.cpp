#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {

  vtkNew<vtkNamedColors> namedColor;
  vtkNew<vtkConeSource> cone;
  cone->SetHeight(2);
  vtkNew<vtkPolyDataMapper> mapper;

  mapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkActor> coneActor;

  coneActor->GetProperty()->SetColor(namedColor->GetColor3d("Gold").GetData());
  coneActor->GetProperty()->SetAmbient(1.0);
  coneActor->GetProperty()->SetDiffuse(0.0);
  coneActor->GetProperty()->SetRepresentationToWireframe();
  coneActor->GetProperty()->SetDiffuseColor(
      namedColor->GetColor3d("Red").GetData());

  coneActor->GetProperty()->SetOpacity(0.2);

  coneActor->GetProperty()->SetSpecular(0.3);
  coneActor->GetProperty()->SetSpecularPower(30);

  coneActor->SetMapper(mapper);
  vtkNew<vtkRenderWindowInteractor> iren;
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkInteractorStyleTrackballCamera> style;
  vtkNew<vtkRenderWindow> render_window;

  renderer->AddActor(coneActor);
  render_window->AddRenderer(renderer);
  render_window->SetInteractor(iren);

  iren->SetInteractorStyle(style);
  iren->Initialize();
  iren->Start();
}
