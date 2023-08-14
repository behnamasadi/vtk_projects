// https://kitware.github.io/vtk-examples/site/Cxx/Tutorial/Tutorial_Step5/
#include <vtkActor.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

/*
source -> mapper -> actor -> renderer -> rendererWindow

Source:  source object, i.e. vtkConeSource
Mapper: map the source  data into graphics primitives
Actor: an actor represent the object. The actor orchestrates rendering  of the
mapper's graphics primitives. An actor also refers to properties via a
vtkProperty instance, and includes an internal transformation matrix. We set
this actor's mapper to be coneMapper which we created above.

Renderer:  A renderer is like a viewport. It is part or all of a window on the
screen and it is responsible for drawing the actors it has.  We also set the
background color here.

RenderWindow: render window which will show up on the screen.
*/

/**/
namespace {
// Callback for the interaction.
class vtkMyCallback : public vtkCommand {
public:
  static vtkMyCallback *New() { return new vtkMyCallback; }
  void Execute(vtkObject *caller, unsigned long, void *) override {
    // Note the use of reinterpret_cast to cast the caller to the expected type.
    auto renderer = reinterpret_cast<vtkRenderer *>(caller);

    std::cout << renderer->GetActiveCamera()->GetPosition()[0] << " "
              << renderer->GetActiveCamera()->GetPosition()[1] << " "
              << renderer->GetActiveCamera()->GetPosition()[2] << std::endl;
  }
  vtkMyCallback() = default;
};
} // namespace
int main(int argc, char **argv) {

  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkConeSource> cone;
  cone->SetHeight(1.0);
  cone->SetRadius(0.5);
  cone->SetResolution(10);

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);
  coneActor->GetProperty()->SetColor(0.2, 0.63, 0.79);
  coneActor->GetProperty()->SetDiffuse(0.7);
  coneActor->GetProperty()->SetSpecular(0.4);
  coneActor->GetProperty()->SetSpecularPower(20);

  vtkNew<vtkRenderer> coneRenderer1;
  coneRenderer1->SetBackground(colors->GetColor3d("MidnightBlue").GetData());
  coneRenderer1->SetViewport(0.0, 0.0, 0.5, 1.0);
  coneRenderer1->AddActor(coneActor);

  vtkNew<vtkRenderer> coneRenderer2;
  coneRenderer2->SetViewport(0.5, 0.0, 1.0, 1.0);
  coneRenderer2->AddActor(coneActor);

  vtkNew<vtkMyCallback> mo1;
  coneRenderer1->AddObserver(vtkCommand::StartEvent, mo1);
  coneRenderer1->AddObserver(vtkCommand::StartEvent, mo1);

  vtkNew<vtkRenderWindow> coneRenderWindow;

  coneRenderWindow->AddRenderer(coneRenderer1);
  coneRenderWindow->AddRenderer(coneRenderer2);

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(coneRenderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;

  interactor->Initialize();
  interactor->Start();

  //   for (int i = 0; i < 360; ++i) {
  //     // Render the image
  //     coneRenderWindow->Render();
  //     // Rotate the active camera by one degree.
  //     coneRenderer1->GetActiveCamera()->Azimuth(1);
  //     coneRenderer2->GetActiveCamera()->Azimuth(1);
  //   }
  return EXIT_SUCCESS;
}
