#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkCellArrayIterator.h>
#include <vtkNew.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTriangle.h>

int main(int, char *[]) {

  vtkNew<vtkPoints> points;

  /*

  Y----------------------▸
  |
  |
  |
  |
  |
  |
  ▾
  0                             2
  (0, 0, 0)                     (0, 1, 0)



  1                             3
  (1, 0, 0)                     (1, 0, 0)

  */

  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);
  points->InsertNextPoint(1, 1, 0);

  vtkNew<vtkTriangle> triangle1;

  triangle1->GetPointIds()->SetId(0, 0);
  triangle1->GetPointIds()->SetId(1, 1);
  triangle1->GetPointIds()->SetId(2, 3);

  vtkNew<vtkTriangle> triangle2;
  triangle2->GetPointIds()->SetId(0, 0);
  triangle2->GetPointIds()->SetId(1, 2);
  triangle2->GetPointIds()->SetId(2, 3);

  // vtkNew<vtkCellArray> triangles;

  // vtkSmartPointer<vtkCellArray> triangles =
  //     vtkSmartPointer<vtkCellArray>::New();

  vtkNew<vtkCellArray> triangles;

  triangles->InsertNextCell(triangle1);
  triangles->InsertNextCell(triangle2);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  std::cout << "There are " << polyData->GetNumberOfCells() << " cells."
            << std::endl;

  std::cout << "There are " << polyData->GetNumberOfPoints() << " points."
            << std::endl;

  auto iter = vtk::TakeSmartPointer(triangles->NewIterator());
  // auto iter = triangles->NewIterator();

  for (iter->GoToFirstCell(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    vtkIdType npts;
    const vtkIdType *pts;
    iter->GetCurrentCell(npts, pts);

    std::cout << "Connectivity: ";
    for (vtkIdType i = 0; i < npts; ++i) {
      std::cout << pts[i] << " ";
    }
    std::cout << std::endl;

    // Assuming the cell array is made of triangles, the offset is 3 * cell
    // index. For other cell types or mixed cell arrays, you would need a
    // different method to get offsets.
    vtkIdType offset = iter->GetCurrentCellId() * 3;
    std::cout << "Offset: " << offset << std::endl;
  }

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);

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

  return EXIT_SUCCESS;
}

int main1(int, char *[]) {

  vtkNew<vtkCellArray> triangles;

  // Example: Insert some triangles into the cell array
  vtkIdType pts1[3] = {0, 1, 2};
  vtkIdType pts2[3] = {2, 3, 0};
  triangles->InsertNextCell(3, pts1);
  triangles->InsertNextCell(3, pts2);

  auto iter = vtk::TakeSmartPointer(triangles->NewIterator());

  // Now iterate over the cells using the iterator
  for (iter->GoToFirstCell(); !iter->IsDoneWithTraversal();
       iter->GoToNextCell()) {
    vtkIdType npts;
    const vtkIdType *pts;
    iter->GetCurrentCell(npts, pts);

    std::cout << "Connectivity: ";
    for (vtkIdType i = 0; i < npts; ++i) {
      std::cout << pts[i] << " ";
    }
    std::cout << std::endl;

    // Assuming the cell array is made of triangles, the offset is 3 * cell
    // index. For other cell types or mixed cell arrays, you would need a
    // different method to get offsets.
    vtkIdType offset = iter->GetCurrentCellId() * 3;
    std::cout << "Offset: " << offset << std::endl;
  }

  return EXIT_SUCCESS;
}
