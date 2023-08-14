#include <vtkActor.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

struct MyCallback : public vtkCommand {

  static MyCallback *New() { return new MyCallback; }

  void Execute(vtkObject *caller, unsigned long event,
               void *callData) override {
    vtkNew<vtkTransform> t;

    auto widget = reinterpret_cast<vtkBoxWidget *>(caller);

    widget->GetTransform(t);
    widget->GetProp3D()->SetUserTransform(t);
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
  renWin->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);

  vtkNew<MyCallback> mycallback;

  vtkNew<vtkBoxWidget> boxWidget;

  boxWidget->SetInteractor(iren);
  boxWidget->SetProp3D(actor);

  boxWidget->AddObserver(vtkCommand::InteractionEvent, mycallback);
  boxWidget->On();
  iren->Initialize();
  iren->Start();
}