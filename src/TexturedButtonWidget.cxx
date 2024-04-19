
#include <array>
#include <vtkActor.h>
#include <vtkButtonWidget.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkEllipticalButtonSource.h>
#include <vtkImageActor.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProp3DButtonRepresentation.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkSuperquadricSource.h>
#include <vtkTexture.h>
#include <vtkTexturedButtonRepresentation2D.h>

std::string img1 = "/home/behnam/workspace/vtk_projects/images/HQ.png";
std::string img2 = "/home/behnam/workspace/vtk_projects/images/maps-marker.png";

/*

// Callback for the interaction
class vtkButtonCallback : public vtkCommand {
public:
  static vtkButtonCallback *New() { return new vtkButtonCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void *) override {
    vtkButtonWidget *buttonWidget = reinterpret_cast<vtkButtonWidget *>(caller);
    vtkTexturedButtonRepresentation2D *rep =
        dynamic_cast<vtkTexturedButtonRepresentation2D *>(
            buttonWidget->GetRepresentation());
    int state = rep->GetState();
    if (state == 0) {
      imageActor->SetInputData(image1);
    } else {
      imageActor->SetInputData(image2);
    }
    renderWindow->Render();
  }
  vtkButtonCallback()
      : imageActor(nullptr), image1(nullptr), image2(nullptr),
        renderWindow(nullptr) {}
  vtkImageActor *imageActor;
  vtkImageData *image1, *image2;
  vtkRenderWindow *renderWindow;
};

class vtkButtonCallback2d : public vtkCommand {
public:
  static vtkButtonCallback2d *New() {
    return new vtkButtonCallback2d;
    std::cout << "new vtkButtonCallback` \n";
  }

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    auto buttonWidget = reinterpret_cast<vtkButtonWidget *>(caller);
    auto rep = reinterpret_cast<vtkProp3DButtonRepresentation *>(
        buttonWidget->GetRepresentation());
    //        int state = rep->GetState();
    //        this->Actor->GetProperty()->SetColor(
    //            reinterpret_cast<vtkActor
    //            *>(rep->GetButtonProp(state))->GetProperty()->GetColor()
    //        );
    //        std::cout << "State: " << state << "\n";

    std::cout << "clicked: \n";
  }
};

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // Load images
  vtkNew<vtkImageReader2Factory> readerFactory;

  vtkSmartPointer<vtkImageReader2> reader1 = readerFactory->CreateImageReader2(
      "/home/behnam/workspace/vtk_projects/images/Icons_Explore/"
      "Basicmode-grey.png");
  reader1->SetFileName("/home/behnam/workspace/vtk_projects/images/"
                       "Icons_Explore/Basicmode-grey.png");
  reader1->Update();

  vtkSmartPointer<vtkImageReader2> reader2 = readerFactory->CreateImageReader2(
      "/home/behnam/workspace/vtk_projects/images/Icons_Explore/"
      "Basicmode-white.png");
  reader2->SetFileName("/home/behnam/workspace/vtk_projects/images/"
                       "Icons_Explore/Basicmode-white.png");
  reader2->Update();

  // Create some geometry
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->Update();

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("MistyRose").GetData());

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("TexturedButtonWidget");

  // Create image actor
  // vtkNew<vtkImageActor> imageActor;
  // imageActor->SetInputData(reader1->GetOutput());
  // renderer->AddActor(imageActor);

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(colors->GetColor3d("MidnightBLue").GetData());

  renderWindow->SetSize(640 * 2, 480 * 2);
  renderWindow->Render(); // Add the actors to the scene
  renderer->AddActor(actor);
  renderer->SetBackground(colors->GetColor3d("MidnightBLue").GetData());

  // // Create the button
  // vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation2d =
  //     vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();
  // buttonRepresentation2d->SetNumberOfStates(1);
  // buttonRepresentation2d->SetPlaceFactor(1);

  // Create a button widget
  vtkNew<vtkTexturedButtonRepresentation2D> buttonRepresentation;
  buttonRepresentation->SetNumberOfStates(2);
  buttonRepresentation->SetButtonTexture(0, reader1->GetOutput()); // State 0
  buttonRepresentation->SetButtonTexture(1, reader2->GetOutput()); // State 1

  vtkNew<vtkButtonWidget> buttonWidget;
  buttonWidget->SetInteractor(renderWindowInteractor);
  buttonWidget->SetRepresentation(buttonRepresentation);

  // Setup callback
  vtkSmartPointer<vtkButtonCallback> callback =
      vtkSmartPointer<vtkButtonCallback>::New();
  callback->imageActor = imageActor;
  callback->image1 = reader1->GetOutput();
  callback->image2 = reader2->GetOutput();
  callback->renderWindow = renderWindow;

  buttonWidget->AddObserver(vtkCommand::StateChangedEvent, callback);

  vtkNew<vtkCoordinate> upperRight;
  upperRight->SetCoordinateSystemToNormalizedDisplay();
  upperRight->SetValue(0.2, 0.5);

  double bds[6];
  double sz = 200.0;
  bds[0] = upperRight->GetComputedDisplayValue(renderer)[0] - sz;
  bds[1] = bds[0] + sz;
  bds[2] = upperRight->GetComputedDisplayValue(renderer)[1] - sz;
  bds[3] = bds[2] + sz;
  bds[4] = bds[5] = 0.0;

  for (const auto &b : bds)
    std::cout << "............" << b << std::endl;

  // Initialize and start the interaction
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  buttonWidget->On();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
*/

#include <vtkActor.h>
#include <vtkButtonWidget.h>
#include <vtkCommand.h>
#include <vtkImageData.h>
#include <vtkImageReader2.h>
#include <vtkImageReader2Factory.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTexture.h>
#include <vtkTexturedButtonRepresentation2D.h>

// Callback for the interaction
class vtkButtonCallback : public vtkCommand {
public:
  static vtkButtonCallback *New() { return new vtkButtonCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void *) override {
    vtkButtonWidget *buttonWidget = reinterpret_cast<vtkButtonWidget *>(caller);
    vtkTexturedButtonRepresentation2D *rep =
        dynamic_cast<vtkTexturedButtonRepresentation2D *>(
            buttonWidget->GetRepresentation());
    int state = rep->GetState();
    if (state == 0) {
      sphereActor->SetTexture(texture1);
    } else {
      sphereActor->SetTexture(texture2);
    }
    renderWindow->Render();
  }
  vtkButtonCallback()
      : sphereActor(nullptr), texture1(nullptr), texture2(nullptr),
        renderWindow(nullptr) {}
  vtkActor *sphereActor;
  vtkTexture *texture1, *texture2;
  vtkRenderWindow *renderWindow;
};

int main() {
  // Create a render window and renderer
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Load textures
  vtkSmartPointer<vtkImageReader2Factory> readerFactory =
      vtkSmartPointer<vtkImageReader2Factory>::New();
  vtkSmartPointer<vtkImageReader2> reader1 =
      readerFactory->CreateImageReader2(img1.c_str());
  reader1->SetFileName(img1.c_str());
  reader1->Update();

  vtkSmartPointer<vtkImageReader2> reader2 =
      readerFactory->CreateImageReader2(img2.c_str());
  reader2->SetFileName(img2.c_str());
  reader2->Update();

  vtkSmartPointer<vtkTexture> texture1 = vtkSmartPointer<vtkTexture>::New();
  texture1->SetInputData(reader1->GetOutput());

  vtkSmartPointer<vtkTexture> texture2 = vtkSmartPointer<vtkTexture>::New();
  texture2->SetInputData(reader2->GetOutput());

  // Create sphere actor
  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
  sphereActor->SetMapper(sphereMapper);
  sphereActor->SetTexture(texture1); // Initial texture
  renderer->AddActor(sphereActor);

  // Create a button widget
  vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation =
      vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();
  buttonRepresentation->SetNumberOfStates(2);

  vtkSmartPointer<vtkButtonWidget> buttonWidget =
      vtkSmartPointer<vtkButtonWidget>::New();
  buttonWidget->SetInteractor(renderWindowInteractor);
  buttonWidget->SetRepresentation(buttonRepresentation);

  vtkNew<vtkCoordinate> upperRight;
  upperRight->SetCoordinateSystemToNormalizedDisplay();
  upperRight->SetValue(0.2, 0.5);

  double bds[6];
  double sz = 200.0;
  bds[0] = upperRight->GetComputedDisplayValue(renderer)[0] - sz;
  bds[1] = bds[0] + sz;
  bds[2] = upperRight->GetComputedDisplayValue(renderer)[1] - sz;
  bds[3] = bds[2] + sz;
  bds[4] = bds[5] = 0.0;

  // Setup callback
  vtkSmartPointer<vtkButtonCallback> callback =
      vtkSmartPointer<vtkButtonCallback>::New();
  callback->sphereActor = sphereActor;
  callback->texture1 = texture1;
  callback->texture2 = texture2;
  callback->renderWindow = renderWindow;

  buttonWidget->AddObserver(vtkCommand::StateChangedEvent, callback);

  // Initialize and start the interaction
  renderWindow->Render();
  renderWindowInteractor->Initialize();
  buttonWidget->On();
  renderWindowInteractor->Start();

  return 0;
}
