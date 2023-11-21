
#include <array>
#include <vtkActor.h>
#include <vtkButtonWidget.h>
#include <vtkCommand.h>
#include <vtkCoordinate.h>
#include <vtkEllipticalButtonSource.h>
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

namespace {
void CreateImage(vtkImageData *image, std::string const &color1,
                 std::string const &color2);
}

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

vtkSmartPointer<vtkActor> CreateButtonActor(const char *textureFile) {
  vtkNew<vtkImageReader2Factory> readerFactory;
  vtkSmartPointer<vtkImageReader2> imageReader;
  imageReader.TakeReference(readerFactory->CreateImageReader2(textureFile));
  imageReader->SetFileName(textureFile);
  imageReader->Update();

  // Aspect ratio of image
  int dims[3];
  imageReader->GetOutput()->GetDimensions(dims);
  double aspect = static_cast<double>(dims[0]) / static_cast<double>(dims[1]);

  vtkNew<vtkTexture> texture;
  texture->SetInputConnection(imageReader->GetOutputPort());

  vtkNew<vtkEllipticalButtonSource> ellipticalButtonSource;
  ellipticalButtonSource->SetCircumferentialResolution(50);
  ellipticalButtonSource->SetShoulderResolution(10);
  ellipticalButtonSource->SetTextureResolution(10);
  ellipticalButtonSource->SetRadialRatio(1.05);
  ellipticalButtonSource->SetShoulderTextureCoordinate(0.0, 0.0);
  ellipticalButtonSource->SetTextureDimensions(dims[0], dims[1]);
  ellipticalButtonSource->SetTextureStyleToProportional();
  ellipticalButtonSource->TwoSidedOn();
  ellipticalButtonSource->SetWidth(aspect);
  ellipticalButtonSource->SetHeight(1.0);
  ellipticalButtonSource->SetDepth(.15);
  ellipticalButtonSource->SetCenter(2, 2, 0);

  ellipticalButtonSource->SetOutputPointsPrecision(
      vtkAlgorithm::SINGLE_PRECISION);

  vtkNew<vtkPolyDataMapper> buttonMapper;
  buttonMapper->SetInputConnection(ellipticalButtonSource->GetOutputPort());

  vtkNew<vtkActor> buttonActor;
  buttonActor->SetMapper(buttonMapper);
  buttonActor->SetTexture(texture);

  return buttonActor;
}

int main(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // Create two images for texture
  vtkNew<vtkImageData> image1;
  vtkNew<vtkImageData> image2;
  unsigned char banana[3] = {227, 207, 87};
  unsigned char tomato[3] = {255, 99, 71};
  CreateImage(image1, "Banana", "Tomato");
  CreateImage(image2, "Tomato", "Banana");

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

  // Create the button
  vtkSmartPointer<vtkTexturedButtonRepresentation2D> buttonRepresentation2d =
      vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();
  buttonRepresentation2d->SetNumberOfStates(1);
  buttonRepresentation2d->SetPlaceFactor(1);

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

  // Scale to 1, default is .5
  buttonRepresentation2d->SetPlaceFactor(1);
  buttonRepresentation2d->PlaceWidget(bds);

  vtkSmartPointer<vtkButtonWidget> buttonWidget =
      vtkSmartPointer<vtkButtonWidget>::New();

  // Create two images for texture
  //   vtkNew<vtkImageData> image1;
  //   vtkNew<vtkImageData> image2;
  //   unsigned char banana[3] = {227, 207, 87};
  //   unsigned char tomato[3] = {255, 99, 71};
  //   CreateImage(image1, "Banana", "Tomato");
  //   CreateImage(image2, "Tomato", "Banana");

  buttonRepresentation2d->SetNumberOfStates(2);
  buttonRepresentation2d->SetButtonTexture(0, image1);
  buttonRepresentation2d->SetButtonTexture(1, image2);

  buttonWidget->SetRepresentation(buttonRepresentation2d);

  //   buttonWidget->CreateDefaultRepresentation();
  buttonWidget->SetCurrentRenderer(renderer);
  buttonWidget->SetDefaultRenderer(renderer);
  buttonWidget->SetInteractor(renderWindowInteractor);

  vtkNew<vtkButtonCallback2d> callback;
  buttonWidget->AddObserver(vtkCommand::StateChangedEvent, callback);
  buttonWidget->On();

  // Begin mouse interaction
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

namespace {
void CreateImage(vtkImageData *image, std::string const &color1,
                 std::string const &color2) {
  vtkNew<vtkNamedColors> colors;

  std::array<unsigned char, 3> dc1{0, 0, 0};
  std::array<unsigned char, 3> dc2{0, 0, 0};
  auto c1 = colors->GetColor3ub(color1).GetData();
  auto c2 = colors->GetColor3ub(color2).GetData();
  for (auto i = 0; i < 3; ++i) {
    dc1[i] = c1[i];
    dc2[i] = c2[i];
  }

  // Specify the size of the image data
  image->SetDimensions(10, 10, 1);
  image->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

  int *dims = image->GetDimensions();

  // Fill the image with
  for (int y = 0; y < dims[1]; y++) {
    for (int x = 0; x < dims[0]; x++) {
      unsigned char *pixel =
          static_cast<unsigned char *>(image->GetScalarPointer(x, y, 0));
      for (int i = 0; i < 3; ++i) {
        if (x < 5) {
          pixel[i] = dc1[i];
        } else {
          pixel[i] = dc2[i];
        }
      }
    }
  }
}
} // namespace

/*
// Create the button
            vtkSmartPointer<vtkTexturedButtonRepresentation2D>
buttonRepresentation =
                vtkSmartPointer<vtkTexturedButtonRepresentation2D>::New();
            buttonRepresentation->SetNumberOfStates(1);
            buttonRepresentation->SetPlaceFactor(1);

            //            double bds[6] = {0.1, 0.1, 0.3, 0.3, 0.0, 0.0};
            //    buttonRepresentation->PlaceWidget(bounds);

            vtkNew<vtkCoordinate> upperRight;
            upperRight->SetCoordinateSystemToNormalizedDisplay();
            upperRight->SetValue(1.0, 1.0);

            double bds[6];
            double sz = 200.0;
            bds[0] = upperRight->GetComputedDisplayValue(CurrentRenderer)[0] -
sz; bds[1] = bds[0] + sz; bds[2] =
upperRight->GetComputedDisplayValue(CurrentRenderer)[1] - sz; bds[3] = bds[2] +
sz; bds[4] = bds[5] = 0.0;

            for (const auto &b : bds)
                std::cout << "............" << b << std::endl;

            // Scale to 1, default is .5
            buttonRepresentation->SetPlaceFactor(1);
            buttonRepresentation->PlaceWidget(bds);

            vtkSmartPointer<vtkButtonWidget> buttonWidget =
vtkSmartPointer<vtkButtonWidget>::New();

            // Create two images for texture
            vtkNew<vtkImageData> image1;
            vtkNew<vtkImageData> image2;
            unsigned char banana[3] = {227, 207, 87};
            unsigned char tomato[3] = {255, 99, 71};
            CreateImage(image1, "Banana", "Tomato");
            CreateImage(image2, "Tomato", "Banana");

            buttonRepresentation->SetNumberOfStates(2);
            buttonRepresentation->SetButtonTexture(0, image1);
            buttonRepresentation->SetButtonTexture(1, image2);

            buttonWidget->SetRepresentation(buttonRepresentation);
            buttonWidget->SetCurrentRenderer(CurrentRenderer);
            //                buttonWidget->CreateDefaultRepresentation();
            buttonWidget->SetDefaultRenderer(CurrentRenderer);
            buttonWidget->SetInteractor(m_iren);
            buttonWidget->On();

*/