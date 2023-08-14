#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {

    auto renderer = reinterpret_cast<vtkRenderer *>(caller);

    std::cout << renderer->GetActiveCamera()->GetPosition()[0] << ", "
              << renderer->GetActiveCamera()->GetPosition()[1] << ", "
              << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
  }
};

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

  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

  vtkNew<MyCallback> mycallback;

  renderer->AddObserver(vtkCommand::StartEvent, mycallback);

  renWin->AddRenderer(renderer);

  for (int i = 0; i < 360; ++i) {
    // Render the image
    renWin->Render();

    // Rotate the camera about the view up vector centered at the focal point.
    renderer->GetActiveCamera()->Azimuth(1);
    // renderer->GetActiveCamera()->Elevation(1);
  }
}