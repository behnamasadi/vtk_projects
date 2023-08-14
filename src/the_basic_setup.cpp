#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

int main() {
  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkConeSource> cone;
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkRenderer> renderer;

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("MistyRose").GetData());

  renderer->AddActor(actor);

  vtkNew<vtkRenderWindow> renWin;

  // This is automatically set when the renderer is created by MakeRenderer. The
  // user probably shouldn't ever need to call this method
  // renderer->SetRenderWindow(renWin);

  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());
  renWin->AddRenderer(renderer);

  for (int i = 0; i < 360; ++i) {
    // Render the image
    renWin->Render();

    // Rotate the camera about the view up vector centered at the focal point.
    renderer->GetActiveCamera()->Azimuth(1);
  }
}