#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

// int main(int, char *[]) {
//   vtkNew<vtkNamedColors> colors;

//   // A renderer and render window
//   vtkNew<vtkRenderer> renderer;
//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetWindowName("DistanceWidget");

//   renderer->SetBackground(colors->GetColor3d("Navy").GetData());

//   // An interactor
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   vtkNew<vtkDistanceWidget> distanceWidget;
//   distanceWidget->SetInteractor(renderWindowInteractor);
//   distanceWidget->CreateDefaultRepresentation();
//   static_cast<vtkDistanceRepresentation3D *>(
//       distanceWidget->GetRepresentation())
//       ->SetLabelFormat("%-#6.3g mm");

//   // distanceWidget->GetRepresentation()->Delete();
//   // distanceWidget->Delete();

//   // Render an image (lights and cameras are created automatically)
//   renderWindow->Render();

//   renderWindowInteractor->Initialize();
//   renderWindow->Render();
//   distanceWidget->On();
//   // distanceWidget->EndDistanceInteraction();

//   // Begin mouse interaction
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <vtkCommand.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

// Define a callback class
class DistanceWidgetCallback : public vtkCommand {
public:
  static DistanceWidgetCallback *New() { return new DistanceWidgetCallback; }

  virtual void Execute(vtkObject *caller, unsigned long eventId, void *) {
    vtkDistanceWidget *distanceWidget =
        reinterpret_cast<vtkDistanceWidget *>(caller);
    vtkDistanceRepresentation3D *rep =
        static_cast<vtkDistanceRepresentation3D *>(
            distanceWidget->GetRepresentation());

    if (eventId == vtkCommand::StartInteractionEvent) {
      // Hide the label at the beginning of the interaction
      // rep->SetLabelVisibility(0);
      // double scale[3] = {0, 0, 0};
      // rep->SetLabelScale(scale);
      // rep->SetVisibility(0);

      rep->SetLabelFormat("");

      std::cout << "StartInteractionEvent" << std::endl;
    } else if (eventId == vtkCommand::EndInteractionEvent) {
      // Show the label once the distance is measured (after the second click)
      // rep->SetVisibility(1);
      // double scale[3] = {1, 1, 1};
      // rep->SetLabelScale(scale);

      // rep->SetLabelFormat("%-#6.3g mm");
      rep->SetLabelFormat("%-#6.3g");
      std::cout << "EndInteractionEvent" << std::endl;
    }
  }
};

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("DistanceWidget");

  renderer->SetBackground(colors->GetColor3d("Navy").GetData());

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkDistanceWidget> distanceWidget;
  distanceWidget->SetInteractor(renderWindowInteractor);
  distanceWidget->CreateDefaultRepresentation();

  vtkNew<DistanceWidgetCallback> callback;
  distanceWidget->AddObserver(vtkCommand::StartInteractionEvent, callback);
  distanceWidget->AddObserver(vtkCommand::EndInteractionEvent, callback);

  // Initialize the interaction
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  distanceWidget->On();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
