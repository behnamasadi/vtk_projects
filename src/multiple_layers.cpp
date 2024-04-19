
#include <vtkBillboardTextActor3D.h>
#include <vtkCubeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTextProperty.h>
int main(int, char *[]) {

  // Create the default renderer and set its properties
  vtkSmartPointer<vtkRenderer> pDefaultRenderer =
      vtkSmartPointer<vtkRenderer>::New();
  pDefaultRenderer->SetBackground(0.1, 0.2, 0.3); // dark blue background
  pDefaultRenderer->SetLayer(0);

  // Create the foreground renderer and set its properties
  vtkSmartPointer<vtkRenderer> pvtkForegroundRenderer =
      vtkSmartPointer<vtkRenderer>::New();
  pvtkForegroundRenderer->SetLayer(1);

  // Set the foreground renderer to use the same camera as the default renderer
  pvtkForegroundRenderer->SetActiveCamera(pDefaultRenderer->GetActiveCamera());

  // Cube Actor for the Background Renderer
  vtkSmartPointer<vtkCubeSource> cubeSource =
      vtkSmartPointer<vtkCubeSource>::New();
  vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());
  vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);
  pDefaultRenderer->AddActor(
      cubeActor); // Add cube actor to the default renderer

  /*
    // Sphere Actor for the Foreground Renderer
    vtkSmartPointer<vtkSphereSource> sphereSource =
        vtkSmartPointer<vtkSphereSource>::New();
    sphereSource->SetRadius(0.5);
    sphereSource->Update();
    vtkSmartPointer<vtkPolyDataMapper> sphereMapper =
        vtkSmartPointer<vtkPolyDataMapper>::New();
    sphereMapper->SetInputConnection(sphereSource->GetOutputPort());
    vtkSmartPointer<vtkActor> sphereActor = vtkSmartPointer<vtkActor>::New();
    sphereActor->SetMapper(sphereMapper);
      pvtkForegroundRenderer->AddActor(
          sphereActor); // Add sphere actor to the foreground renderer
  */

  vtkNew<vtkBillboardTextActor3D> textActor;
  textActor->SetInput("Text on top, always facing camera");
  textActor->SetPosition(1, 1, 1);
  textActor->GetTextProperty()->SetFontSize(12);

  textActor->GetTextProperty()->SetJustificationToCentered();
  pvtkForegroundRenderer->AddActor(textActor);

  // Create the render window and set its properties
  vtkSmartPointer<vtkRenderWindow> pVtkRenderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  pVtkRenderWindow->SetNumberOfLayers(2);
  pVtkRenderWindow->AddRenderer(pDefaultRenderer);
  pVtkRenderWindow->AddRenderer(pvtkForegroundRenderer);
  pVtkRenderWindow->SetSize(800, 600); // window size

  // Create and initialize the render window interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(pVtkRenderWindow);

  // Start the interaction (user interface loop)
  pVtkRenderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
