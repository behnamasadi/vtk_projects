#include <vtkActor.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyle3D.h>
#include <vtkInteractorStyleImage.h>
#include <vtkInteractorStyleRubberBand3D.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleUnicam.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkParallelCoordinatesInteractorStyle.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

int main(int, char *[]) {

  vtkNew<vtkConeSource> cone;
  cone->SetRadius(1);
  cone->SetHeight(1);
  cone->Update();

  vtkNew<vtkAxesActor> axes;

  vtkNew<vtkBillboardTextActor3D> textActor;

  textActor->SetPosition(5, 1, 1);
  textActor->SetScale(5);

  vtkNew<vtkNamedColors> colors;

  // Create a sphere
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->SetCenter(1.0, 0.0, 0.0);
  sphereSource->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> sphereMapper;
  sphereMapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<vtkActor> sphereActor;
  sphereActor->SetMapper(sphereMapper);
  sphereActor->GetProperty()->SetColor(
      colors->GetColor3d("MistyRose").GetData());

  // A renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("InteractiveCamera");

  renderer->AddActor(textActor);
  renderer->AddActor(axes);

  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  renderer->AddActor(coneActor);

  // An interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(sphereActor);
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  // Render an image (lights and cameras are created automatically)
  renderWindow->Render();

  // vtkNew<vtkParallelCoordinatesInteractorStyle> style;
  // textActor->SetInput("vtkParallelCoordinatesInteractorStyle");

  // vtkNew<vtkParallelCoordinatesInteractorStyle> style;
  // textActor->SetInput("vtkParallelCoordinatesInteractorStyle");

  // vtkNew<vtkInteractorStyle3D> style;
  // textActor->SetInput("vtkInteractorStyle3D");

  // vtkNew<vtkInteractorStyleImage> style;
  // textActor->SetInput("vtkInteractorStyleImage");

  // vtkNew<vtkInteractorStyleRubberBand3D> style;
  // textActor->SetInput("vtkInteractorStyleRubberBand3D");

  // vtkNew<vtkInteractorStyleTerrain> style;
  // textActor->SetInput("vtkInteractorStyleTerrain");

  // vtkNew<vtkInteractorStyleUnicam> style;
  // textActor->SetInput("vtkInteractorStyleUnicam");

  // vtkNew<vtkInteractorStyleTrackballActor> style;
  // textActor->SetInput("vtkInteractorStyleTrackballActor");

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  textActor->SetInput("vtkInteractorStyleTrackballCamera");

  renderWindowInteractor->SetInteractorStyle(style);

  // Begin mouse interaction
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
