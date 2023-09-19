#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkCubeSource.h>
#include <vtkIndent.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

// Custom command to sync transformations
class SyncTransformationsCommand : public vtkCommand {
public:
  static SyncTransformationsCommand *New() {
    return new SyncTransformationsCommand;
  }

  virtual void Execute(vtkObject *caller, unsigned long, void *) override {

    if (SourceActor && TargetActor) {

      vtkMatrix4x4 *currentMatrix = SourceActor->GetMatrix();
      //   if (!previousMatrix->DeepCopy(currentMatrix)) {
      //     TargetActor->SetUserMatrix(currentMatrix);
      // if (!currentMatrix) {
      previousMatrix->DeepCopy(currentMatrix);
      TargetActor->SetUserMatrix(currentMatrix);
      vtkIndent indent;
      std::cout << "currentMatrix: " << std::endl;
      currentMatrix->PrintSelf(std::cout, indent.GetNextIndent());

      //}
    }
  }

  void SetSourceActor(vtkActor *actor) {
    SourceActor = actor;
    previousMatrix->DeepCopy(actor->GetMatrix());
  }

  vtkActor *SourceActor = nullptr;
  vtkActor *TargetActor = nullptr;
  vtkNew<vtkMatrix4x4> previousMatrix;
};

int main(int, char *[]) {
  vtkNew<vtkCubeSource> cubeSource1;
  vtkNew<vtkCubeSource> cubeSource2;

  vtkNew<vtkPolyDataMapper> mapper1;
  mapper1->SetInputConnection(cubeSource1->GetOutputPort());
  vtkNew<vtkPolyDataMapper> mapper2;
  mapper2->SetInputConnection(cubeSource2->GetOutputPort());

  vtkNew<vtkActor> actor1;
  actor1->SetMapper(mapper1);
  vtkNew<vtkActor> actor2;
  actor2->SetMapper(mapper2);
  actor2->SetPosition(2, 0,
                      0); // Offset the second cube for visualization  purposes

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballActor> style;
  renderWindowInteractor->SetInteractorStyle(style);

  renderer->AddActor(actor1);
  renderer->AddActor(actor2);

  // Add observer to synchronize transformations
  vtkNew<SyncTransformationsCommand> syncCommand;
  syncCommand->SourceActor = actor1;
  syncCommand->TargetActor = actor2;
  style->AddObserver(vtkCommand::InteractionEvent, syncCommand);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}