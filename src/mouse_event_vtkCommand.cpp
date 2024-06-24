#include <cmath>
#include <iostream>
#include <vtkActor.h>
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

class MouseInteractorStyle : public vtkCommand {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

  void Execute(vtkObject *caller, unsigned long eventId,
               void *callData) override {
    vtkRenderWindowInteractor *interactor =
        dynamic_cast<vtkRenderWindowInteractor *>(caller);
    if (eventId == vtkCommand::LeftButtonPressEvent) {
      std::cout << "Mouse Left Button Pressed" << std::endl;

    } else if (eventId == vtkCommand::RightButtonPressEvent) {
      std::cout << "Mouse Right Button Pressed" << std::endl;
    } else if (eventId == vtkCommand::MouseMoveEvent) {
      std::cout << "Mouse Moved" << std::endl;
    } else if (eventId == vtkCommand::MouseWheelForwardEvent) {
      std::cout << "Mouse Wheel Forward Event" << std::endl;
    } else if (eventId == vtkCommand::MouseWheelBackwardEvent) {
      std::cout << "Mouse Wheel Backward Event" << std::endl;
    }
  }
};

int main() {

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

  renderer->SetBackground(0.1, 0.2, 0.4); // Background color

  // Render window
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);

  // Render window interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  renderWindowInteractor->SetInteractorStyle(style);

  vtkNew<MouseInteractorStyle> mouseInteractorStyle;

  style->AddObserver(vtkCommand::LeftButtonPressEvent, mouseInteractorStyle);
  style->AddObserver(vtkCommand::RightButtonPressEvent, mouseInteractorStyle);
  style->AddObserver(vtkCommand::MouseMoveEvent, mouseInteractorStyle);
  style->AddObserver(vtkCommand::MouseWheelBackwardEvent, mouseInteractorStyle);
  style->AddObserver(vtkCommand::MouseWheelForwardEvent, mouseInteractorStyle);

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();
}