#include <vtkActor.h>
#include <vtkBoundingBox.h>
#include <vtkBox.h>
#include <vtkDataSetMapper.h>
#include <vtkExtractGeometry.h>
#include <vtkExtractPoints.h>
#include <vtkExtractSelection.h>
#include <vtkIdList.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelection.h>
#include <vtkSelectionNode.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkUnstructuredGrid.h>

int main(int, char *[]) {
  // Create a sphere source
  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  // Define a bounding box
  vtkSmartPointer<vtkBox> box = vtkSmartPointer<vtkBox>::New();
  box->SetBounds(-0.5, 0.5, -0.5, 0.5, -0.5, 0.5);

  // Extract points using vtkExtractGeometry
  vtkSmartPointer<vtkExtractGeometry> extractGeometry =
      vtkSmartPointer<vtkExtractGeometry>::New();
  extractGeometry->SetInputData(sphereSource->GetOutput());
  extractGeometry->SetImplicitFunction(box);
  extractGeometry->Update();

  // Mapper and actor for the original data
  vtkSmartPointer<vtkDataSetMapper> originalMapper =
      vtkSmartPointer<vtkDataSetMapper>::New();
  originalMapper->SetInputData(sphereSource->GetOutput());

  vtkSmartPointer<vtkActor> originalActor = vtkSmartPointer<vtkActor>::New();
  originalActor->SetMapper(originalMapper);

  // Mapper and actor for the extracted points
  vtkSmartPointer<vtkDataSetMapper> extractedMapper =
      vtkSmartPointer<vtkDataSetMapper>::New();
  extractedMapper->SetInputData(
      static_cast<vtkDataSet *>(extractGeometry->GetOutput()));

  vtkSmartPointer<vtkActor> extractedActor = vtkSmartPointer<vtkActor>::New();
  extractedActor->SetMapper(extractedMapper);
  extractedActor->GetProperty()->SetColor(1, 0,
                                          0); // Red color for extracted points

  // Renderer
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(originalActor);
  renderer->AddActor(extractedActor);
  renderer->SetBackground(0.1, 0.2, 0.3); // Background color

  // Render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // Interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
