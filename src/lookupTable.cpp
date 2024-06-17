#include <vtkActor.h>
#include <vtkFloatArray.h>
#include <vtkLookupTable.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>

int main() {
  // Create a set of points
  vtkNew<vtkPoints> points;
  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);

  // Set the id at location i
  vtkNew<vtkTriangle> triangle;
  triangle->GetPointIds()->SetId(0, 0);
  triangle->GetPointIds()->SetId(1, 1);
  triangle->GetPointIds()->SetId(2, 2);

  // Create a vertex list (cell type) to connect the points,
  vtkNew<vtkCellArray> triangles;
  triangles->InsertNextCell(triangle);

  // Create scalar data for these points
  vtkNew<vtkFloatArray> scalars;
  scalars->InsertNextValue(0.0);
  scalars->InsertNextValue(0.25);
  scalars->InsertNextValue(1.0);

  // Create a polydata object and set points and scalars
  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);
  polyData->GetPointData()->SetScalars(scalars);

  // Create a lookup table and define its properties
  vtkSmartPointer<vtkLookupTable> lookupTable =
      vtkSmartPointer<vtkLookupTable>::New();

  lookupTable->SetRange(0.0, 1.0); // Scalar range
  lookupTable->Build();            // Build the lookup table

  std::cout << "points->GetNumberOfPoints():" << points->GetNumberOfPoints()
            << std::endl;

  for (vtkIdType id = 0; id < points->GetNumberOfPoints(); id++) {
    double color[3];

    lookupTable->GetColor(id, color);
    std::cout << "id: " << id << " color: " << color[0] << "," << color[1]
              << "," << color[2] << std::endl;
  }

  // Create a mapper and set its scalar range and lookup table
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);
  mapper->SetLookupTable(lookupTable);
  mapper->SetScalarRange(0.0, 1.0);
  mapper->SetInputData(polyData);

  // Create an actor to represent the data
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}
