#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkLegendScaleActor.h>
#include <vtkLineSource.h>
#include <vtkNew.h>
#include <vtkPlaneSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVectorText.h>

int main(int, char *[]) {
  // Create a plane source with grid lines
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(
      10); // Set the number of grid lines in the X direction
  planeSource->SetYResolution(
      10); // Set the number of grid lines in the Y direction

  vtkNew<vtkTransform> transformOrigin;
  vtkNew<vtkTransform> transformPoint1;
  vtkNew<vtkTransform> transformPoint2;

  vtkNew<vtkAxesActor> axesOrigin;
  vtkNew<vtkAxesActor> axesPoint1;
  vtkNew<vtkAxesActor> axesPoint2;

  planeSource->SetOrigin(0, 0, 0);
  transformOrigin->Translate(0.0, 0.0, 0.0);
  axesOrigin->SetUserTransform(transformOrigin);
  // axesOrigin->SetAxisLabels(false);
  axesOrigin->SetXAxisLabelText("O");
  axesOrigin->SetYAxisLabelText("");
  axesOrigin->SetZAxisLabelText("");

  planeSource->SetPoint1(10, 0, 0);
  transformPoint1->Translate(10.0, 0.0, 0.0);
  axesPoint1->SetUserTransform(transformPoint1);
  axesPoint1->SetXAxisLabelText("1");
  axesPoint1->SetYAxisLabelText("");
  axesPoint1->SetZAxisLabelText("");

  planeSource->SetPoint2(0, 10, 0);
  transformPoint2->Translate(0.0, 10.0, 0.0);
  axesPoint2->SetUserTransform(transformPoint2);
  axesPoint2->SetXAxisLabelText("2");
  axesPoint2->SetYAxisLabelText("");
  axesPoint2->SetZAxisLabelText("");

  // Map the plane's data to graphics primitives
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());

  // Create an actor for the plane and set its properties to display the grid
  // lines
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()
      ->SetRepresentationToWireframe();               // Display as wireframe
  planeActor->GetProperty()->SetColor(0.5, 0.5, 0.5); // Set grid color to gray

  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);
  planeActor->PickableOff();

  // Create a renderer and a render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the plane actor to the renderer
  renderer->AddActor(planeActor);
  renderer->SetBackground(0.1, 0.1, 0.1); // Set background color to dark gray

  renderer->AddActor(axesOrigin);

  renderer->AddActor(axesPoint1);

  renderer->AddActor(axesPoint2);

  // Start the rendering loop
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
