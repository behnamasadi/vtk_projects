#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkConeSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPyramid.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

int main(int, char *[]) {
  // Create a cone representing the robot
  vtkSmartPointer<vtkConeSource> coneSource =
      vtkSmartPointer<vtkConeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> coneMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  coneMapper->SetInputConnection(coneSource->GetOutputPort());
  vtkSmartPointer<vtkActor> coneActor = vtkSmartPointer<vtkActor>::New();
  coneActor->SetMapper(coneMapper);
  coneActor->SetPosition(0, 0, 0); // Position of the robot in the scene

  // Create a pyramid representing the camera
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->InsertNextPoint(1.0, 1.0, 1.0);   // Point 0
  points->InsertNextPoint(-1.0, 1.0, 1.0);  // Point 1
  points->InsertNextPoint(-1.0, -1.0, 1.0); // Point 2
  points->InsertNextPoint(1.0, -1.0, 1.0);  // Point 3
  points->InsertNextPoint(0.0, 0.0, 0.0);   // Point 4 (apex of the pyramid)

  vtkSmartPointer<vtkPyramid> pyramid = vtkSmartPointer<vtkPyramid>::New();
  pyramid->GetPointIds()->SetId(0, 0);
  pyramid->GetPointIds()->SetId(1, 1);
  pyramid->GetPointIds()->SetId(2, 2);
  pyramid->GetPointIds()->SetId(3, 3);
  pyramid->GetPointIds()->SetId(4, 4);

  vtkSmartPointer<vtkCellArray> cells = vtkSmartPointer<vtkCellArray>::New();
  cells->InsertNextCell(pyramid);

  vtkSmartPointer<vtkPolyData> pyramidData =
      vtkSmartPointer<vtkPolyData>::New();
  pyramidData->SetPoints(points);
  pyramidData->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

  vtkSmartPointer<vtkPolyDataMapper> pyramidMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  pyramidMapper->SetInputData(pyramidData);
  vtkSmartPointer<vtkActor> pyramidActor = vtkSmartPointer<vtkActor>::New();
  pyramidActor->SetMapper(pyramidMapper);
  pyramidActor->SetPosition(3, 0, 0); // Position of the camera in the scene

  // Create a render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(coneActor);
  renderer->AddActor(pyramidActor);
  renderer->SetBackground(0.5, 0.5, 0.5); // Set a gray background

  renderer->ResetCamera();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
