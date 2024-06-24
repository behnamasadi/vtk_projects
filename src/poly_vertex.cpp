#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPolyVertex.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

int main() {
  vtkNew<vtkPoints> points;

  // Insert your points here
  for (int i = 0; i < 200; ++i) {
    double x = x;
    double y = y;
    double z = z;
    points->InsertNextPoint(x, y, z);
  }

  vtkNew<vtkPolyVertex> polyVertex;
  polyVertex->GetPointIds()->SetNumberOfIds(20000);
  for (vtkIdType i = 0; i < 200; ++i) {
    polyVertex->GetPointIds()->SetId(i, i);
  }

  vtkNew<vtkCellArray> cells;
  cells->InsertNextCell(polyVertex);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetVerts(cells);

  vtkNew<vtkPolyDataMapper> pointsMapper;
  pointsMapper->SetInputData(polyData);

  vtkNew<vtkActor> pointsActor;
  pointsActor->SetMapper(pointsMapper);

  vtkNew<vtkNamedColors> colors;
  pointsActor->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(pointsActor);
  renderer->SetBackground(colors->GetColor3d("SteelBlue").GetData());

  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}
