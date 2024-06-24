#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>

class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

  virtual void OnLeftButtonDown() override {
    // std::cout << "Mouse Left Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  virtual void OnRightButtonDown() override {
    // std::cout << "Mouse Right Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
  }

  virtual void OnMouseMove() override {
    // std::cout << "Mouse Moved" << std::endl;
    vtkInteractorStyleTrackballCamera::OnMouseMove();
  }

  virtual void OnMouseWheelForward() override {
    std::cout << "OnMouseWheelForward" << std::endl;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;
      }
    }

    std::cout << "renderer->GetActors()->GetNumberOfItems():"
              << renderer->GetActors()->GetNumberOfItems() << std::endl;

    for (int i = 0; i < renderer->GetActors()->GetNumberOfItems(); i++) {

      std::cout << "renderer->GetActors()[i].GetClassName():"
                << renderer->GetActors()[i].GetClassName() << std::endl;
    }

    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
  }

  virtual void OnMouseWheelBackward() override {
    std::cout << "OnMouseWheelBackward" << std::endl;

    vtkRenderWindowInteractor *rwi = Interactor;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      //   camera->GetFrustumPlanes()
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;

        std::cout << "Distance from focal point: " << camera->GetDistance()
                  << std::endl;
      }
    }

    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
  }

  virtual void OnChar() override {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = Interactor;
    std::string key = rwi->GetKeySym();

    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    // Output the key that was pressed
    std::cout << "OnChar() " << key.c_str() << std::endl;

    // Handle an arrow key
    if (key == "up") {
      std::cout << "The up arrow was pressed." << std::endl;
    }

    if (key == "delete") {
      std::cout << "The delete key was pressed." << std::endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }

  virtual void Dolly() override {

    //   right click down + <- ->
    //   camera should rotate in its place, to do that we calculate the azimuth
    //   that rotate teh camera around We should store the old camera position,
    //   apply azimuth, then we calculate forward vector from camera to the
    //   focal point, then from previously stored camera position we go forward
    //   in the direction of forward vector and calculate the new camera focal
    //   point
    std::cout << "Dolly" << std::endl;

    vtkInteractorStyleTrackballCamera::Dolly();
  }
};

int main() {

  std::cout << "Press r to start" << std::endl;

  vtkNew<vtkPoints> originalPoints;

  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);

  double cylanderRadius = 2.0;
  for (int i = 0; i < 20000;
       i++) { // Reduced number of points for faster rendering
    double x, y, z;
    x = randomSequence->GetRangeValue(4.0, 12.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-2.0, 2.0);
    randomSequence->Next();
    z = sqrt(cylanderRadius * cylanderRadius - y * y);

    originalPoints->InsertNextPoint(x, y, -z);
  }

  vtkNew<vtkPolyData> originalPointsPolydata;
  originalPointsPolydata->SetPoints(originalPoints);

  vtkNew<vtkVertexGlyphFilter> originalPointsVertexFilter;

  originalPointsVertexFilter->SetInputData(originalPointsPolydata);
  originalPointsVertexFilter->Update();

  vtkNew<vtkPolyDataMapper> originalPointsMapper;
  originalPointsMapper->SetInputConnection(
      originalPointsVertexFilter->GetOutputPort());

  vtkNew<vtkActor> vtkActorInputPolydata;
  vtkActorInputPolydata->SetMapper(originalPointsMapper);
  vtkActorInputPolydata->GetProperty()->SetColor(0.0, 0.0, 1.0); // Blue color
  vtkActorInputPolydata->GetProperty()->SetPointSize(
      2); // Increase point size for better visibility

  // Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(vtkActorInputPolydata);

  vtkCamera *camera = renderer->GetActiveCamera();
  auto viewAngle = camera->GetViewAngle();
  std::cout << viewAngle << std::endl;

  auto clippingRange = camera->GetClippingRange();

  std::cout << clippingRange[0] << "," << clippingRange[1] << std::endl;

  camera->SetPosition(150, 0, 150);
  camera->SetFocalPoint(0, 0, 0);
  camera->Azimuth(30);
  camera->Elevation(30);
  camera->SetViewUp(0, 0, 1);
  camera->SetClippingRange(5, 15);
  camera->SetViewAngle(30);

  renderer->SetBackground(0.1, 0.2, 0.4); // Background color

  // Render window
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);

  // Render window interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<MouseInteractorStyle> mouseInteractorStyle;
  renderWindowInteractor->SetInteractorStyle(mouseInteractorStyle);

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();
}