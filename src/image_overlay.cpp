// #include <vtkImageActor.h>
// #include <vtkImageReader2.h>
// #include <vtkImageReader2Factory.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSmartPointer.h>

// int main() {
//   // Set up the main VTK components
//   vtkSmartPointer<vtkRenderer> mainRenderer =
//       vtkSmartPointer<vtkRenderer>::New();
//   vtkSmartPointer<vtkRenderWindow> renderWindow =
//       vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(mainRenderer);
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//       vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Load the overlay image
//   vtkSmartPointer<vtkImageReader2Factory> readerFactory =
//       vtkSmartPointer<vtkImageReader2Factory>::New();
//   vtkSmartPointer<vtkImageReader2> imageReader =
//       readerFactory->CreateImageReader2("/home/behnam/Pictures/nvidia.jpg");
//   imageReader->SetFileName("/home/behnam/Pictures/nvidia.jpg");
//   imageReader->Update();

//   // Create and configure the image actor
//   vtkSmartPointer<vtkImageActor> imageActor =
//       vtkSmartPointer<vtkImageActor>::New();
//   imageActor->SetInputData(imageReader->GetOutput());
//   // Set properties like position and opacity as needed
//   // e.g., imageActor->SetOpacity(0.5);

//   // Add the image actor to the renderer
//   mainRenderer->AddActor(imageActor);

//   // Start the rendering process
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return 0;
// }
#include <vtkActor.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowCollection.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>

class CustomInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static CustomInteractorStyle *New();
  vtkTypeMacro(CustomInteractorStyle, vtkInteractorStyleTrackballCamera);

  void SetImageActor(vtkImageActor *actor) { this->ImageActor = actor; }

  virtual void OnLeftButtonDown() override {
    // Get click position
    int *clickPos = this->GetInteractor()->GetEventPosition();

    // Convert click position to world coordinates
    vtkSmartPointer<vtkCoordinate> coordinate =
        vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetCoordinateSystemToDisplay();
    coordinate->SetValue(clickPos[0], clickPos[1], 0);
    double *worldPos =
        coordinate->GetComputedWorldValue(this->GetDefaultRenderer());

    // Check if the click is within the bounds of the image actor
    double *bounds = ImageActor->GetBounds();
    if (worldPos[0] >= bounds[0] && worldPos[0] <= bounds[1] &&
        worldPos[1] >= bounds[2] && worldPos[1] <= bounds[3]) {
      std::cout << "Image Actor Clicked!" << std::endl;
      // Handle the click event on the image actor
      return;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

protected:
  CustomInteractorStyle() : ImageActor(NULL) {}

  vtkImageActor *ImageActor;
};

vtkStandardNewMacro(CustomInteractorStyle);

// Callback function for the click event
class ClickEventCallback : public vtkCommand {
public:
  static ClickEventCallback *New() { return new ClickEventCallback; }

  void SetImageActor(vtkImageActor *actor) { this->ImageActor = actor; }

  virtual void Execute(vtkObject *caller, unsigned long eventId,
                       void *callData) {
    vtkRenderWindowInteractor *interactor =
        static_cast<vtkRenderWindowInteractor *>(caller);
    int x, y;
    interactor->GetEventPosition(x, y);

    vtkRenderer *renderer =
        interactor->GetRenderWindow()->GetRenderers()->GetFirstRenderer();
    vtkSmartPointer<vtkCoordinate> coordinate =
        vtkSmartPointer<vtkCoordinate>::New();
    coordinate->SetCoordinateSystemToDisplay();
    coordinate->SetValue(x, y, 0);

    double *world = coordinate->GetComputedWorldValue(renderer);
    if (ImageActor->GetBounds()[0] <= world[0] &&
        world[0] <= ImageActor->GetBounds()[1] &&
        ImageActor->GetBounds()[2] <= world[1] &&
        world[1] <= ImageActor->GetBounds()[3]) {
      std::cout << "Image clicked!" << std::endl;
      // Implement additional actions here
    }
  }

protected:
  ClickEventCallback() : ImageActor(NULL) {}

  vtkImageActor *ImageActor;
};

int main() {
  // Load the image
  vtkSmartPointer<vtkImageReader2Factory> readerFactory =
      vtkSmartPointer<vtkImageReader2Factory>::New();
  vtkSmartPointer<vtkImageReader2> reader =
      readerFactory->CreateImageReader2("/home/behnam/Pictures/qml_vtk.png");
  reader->SetFileName("/home/behnam/Pictures/qml_vtk.png");
  reader->Update();

  // Create image actor
  vtkSmartPointer<vtkImageActor> imageActor =
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->SetInputData(reader->GetOutput());

  // Set up renderer and window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(imageActor);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();

  vtkSmartPointer<CustomInteractorStyle> customStyle =
      vtkSmartPointer<CustomInteractorStyle>::New();
  customStyle->SetDefaultRenderer(renderer);
  customStyle->SetImageActor(imageActor);
  interactor->SetInteractorStyle(customStyle);

  interactor->SetRenderWindow(renderWindow);

  // Attach click event handler
  vtkSmartPointer<ClickEventCallback> clickCallback =
      vtkSmartPointer<ClickEventCallback>::New();
  clickCallback->SetImageActor(imageActor);
  interactor->AddObserver(vtkCommand::LeftButtonPressEvent, clickCallback);

  // Start interaction
  renderWindow->Render();
  interactor->Start();

  return 0;
}
