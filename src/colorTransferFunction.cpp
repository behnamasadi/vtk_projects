#include <vtkColorTransferFunction.h>
#include <vtkNew.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {
  // Create a color transfer function
  vtkNew<vtkColorTransferFunction> colorTransferFunction;

  // Add colors corresponding to scalar values
  colorTransferFunction->AddRGBPoint(0.0, 1.0, 0.0,
                                     0.0); // Red at scalar value 0
  colorTransferFunction->AddRGBPoint(0.5, 0.0, 1.0,
                                     0.0); // Green at scalar value 0.5
  colorTransferFunction->AddRGBPoint(1.0, 0.0, 0.0,
                                     1.0); // Blue at scalar value 1

  // Create a renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Set background color using the color transfer function
  double r, g, b;
  double color[3];
  colorTransferFunction->GetColor(
      0.25, color); // Get the color corresponding to scalar value 0.5

  r = color[0];
  g = color[1];
  b = color[2];
  renderer->SetBackground(r, g, b);

  // Render and start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}