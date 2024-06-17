#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCubeSource.h>
#include <vtkGlyph3D.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {
  vtkNew<vtkPoints> points;

  // location of  glyph in 3d space
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(10, 1, 1);
  points->InsertNextPoint(0, 1, 0);
  points->InsertNextPoint(1, 5, 0);
  points->InsertNextPoint(0, 0, 1);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);

  vtkNew<vtkCubeSource> cubeSource;

  vtkNew<vtkGlyph3D> glyph3D;
  glyph3D->SetSourceConnection(cubeSource->GetOutputPort());
  glyph3D->SetInputData(polyData);
  glyph3D->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyph3D->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  // Visualize
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("Glyphs");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderWindow->Render();

  renderer->GetActiveCamera()->Elevation(20);
  renderer->GetActiveCamera()->Azimuth(10);
  renderer->GetActiveCamera()->Zoom(0.9);

  renderWindowInteractor->Start();
}
