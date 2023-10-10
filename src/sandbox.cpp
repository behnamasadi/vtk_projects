#include <vtkActor.h>
#include <vtkLookupTable.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkSphereSource.h>

int main(int, char *[]) {
  // Create a simple sphere
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  const int numStripes = 4;

  // Create a lookup table with alternating black and white stripes
  vtkNew<vtkLookupTable> lut;
  lut->SetNumberOfTableValues(numStripes);
  for (int i = 0; i < numStripes; ++i) {
    if (i % 2 == 0) {
      lut->SetTableValue(i, 1.0, 1.0, 1.0, 1.0); // white
    } else {
      lut->SetTableValue(i, 0.0, 0.0, 0.0, 1.0); // black
    }
  }
  lut->Build();

  mapper->SetLookupTable(lut);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // Create a scalar bar
  vtkNew<vtkScalarBarActor> scalarBar;
  scalarBar->SetLookupTable(mapper->GetLookupTable());
  scalarBar->SetTitle("My Scalar Bar");
  scalarBar->SetNumberOfLabels(numStripes);
  scalarBar->SetOrientationToHorizontal();

  // Position the scalar bar at the bottom of the window
  scalarBar->GetPositionCoordinate()->SetCoordinateSystemToNormalizedDisplay();
  scalarBar->GetPositionCoordinate()->SetValue(0.1, 0.05);
  scalarBar->SetWidth(0.8);
  scalarBar->SetHeight(0.1);

  // Create a renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor and scalar bar to the scene
  renderer->AddActor(actor);
  renderer->AddActor2D(scalarBar);

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
