
#include <vtkCommand.h>
#include <vtkImageData.h>
#include <vtkLogoRepresentation.h>
#include <vtkLogoWidget.h>
#include <vtkPNGReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

// // Callback function for the click event
// class LogoClickCallback : public vtkCommand {
// public:
//   static LogoClickCallback *New() { return new LogoClickCallback; }

//   virtual void Execute(vtkObject *caller, unsigned long, void *) {
//     std::cout << "Logo clicked!" << std::endl;
//     // Implement additional actions here if needed
//   }
// };

// Callback function for the click event
class LogoClickCallback : public vtkCommand {
public:
  vtkLogoWidget *LogoWidget;

  static LogoClickCallback *New() { return new LogoClickCallback; }

  void SetLogoWidget(vtkLogoWidget *logoWidget) {
    this->LogoWidget = logoWidget;
  }

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    std::cout << "foo" << std::endl;

    vtkRenderWindowInteractor *interactor =
        static_cast<vtkRenderWindowInteractor *>(caller);

    int x, y;
    interactor->GetEventPosition(x, y);

    std::cout << "x,y" << x << "," << y << std::endl;

    // Check if the click is within the logo widget bounds
    vtkLogoRepresentation *logoRep =
        static_cast<vtkLogoRepresentation *>(LogoWidget->GetRepresentation());
    if (logoRep->ComputeInteractionState(x, y) != 0) {
      std::cout << "Logo clicked!" << std::endl;
      // Implement additional actions here if needed
    }
  }
};
int main() {
  // Set up the main VTK components
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  interactor->SetRenderWindow(renderWindow);

  // Load the logo image
  vtkSmartPointer<vtkPNGReader> reader = vtkSmartPointer<vtkPNGReader>::New();
  reader->SetFileName("/home/behnam/Pictures/qml_vtk.png");
  reader->Update();

  // Set up the logo widget
  vtkNew<vtkLogoRepresentation> logoRepresentation;

  logoRepresentation->SetImage(reader->GetOutput());
  vtkNew<vtkLogoWidget> logoWidget;
  logoWidget->SetInteractor(interactor);
  logoWidget->SetRepresentation(logoRepresentation);
  //   logoWidget->KeyPressActivationOff();
  //   logoWidget->ManagesCursorOff();
  logoWidget->On();

  // Add click event handler
  // interactor->AddObserver(vtkCommand::LeftButtonPressEvent, clickCallback);

  // logoWidget->AddObserver(vtkCommand::LeftButtonPressEvent, clickCallback);

  vtkNew<LogoClickCallback> clickCallback;
  clickCallback->SetLogoWidget(logoWidget);
  interactor->AddObserver(vtkCommand::LeftButtonPressEvent, clickCallback);

  // Start the interaction
  renderWindow->Render();
  interactor->Start();

  return 0;
}
