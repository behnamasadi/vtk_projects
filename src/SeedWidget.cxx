#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointHandleRepresentation2D.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h> // For setting the color in the handles
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSeedRepresentation.h>
#include <vtkSeedWidget.h>
#include <vtkSphereSource.h>

int main(int vtkNotUsed(argc), char *vtkNotUsed(argv)[]) {
  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->Update();

  // Create a mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());
  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("MidnightBlue").GetData());

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(actor);
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  vtkNew<vtkRenderWindow> window;
  window->AddRenderer(renderer);
  window->SetWindowName("SeedWidget");

  vtkNew<vtkRenderWindowInteractor> interactor;
  interactor->SetRenderWindow(window);

  // Create the representation for the seed widget and for its handles
  vtkNew<vtkPointHandleRepresentation2D> handleRep;
  handleRep->GetProperty()->SetColor(colors->GetColor3d("Red").GetData());
  vtkNew<vtkSeedRepresentation> widgetRep;
  widgetRep->SetHandleRepresentation(handleRep);

  // distance widget
  vtkNew<vtkDistanceWidget> distanceWidget;
  distanceWidget->SetInteractor(interactor);
  distanceWidget->CreateDefaultRepresentation();
  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->SetLabelFormat("%-#6.3g mm");

  vtkSmartPointer<vtkDistanceRepresentation3D> rep;

  vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
      vtkSmartPointer<vtkPointHandleRepresentation3D>::New();

  rep = vtkSmartPointer<vtkDistanceRepresentation3D>::New();

  rep->SetHandleRepresentation(handle);

  // Create the seed widget
  vtkNew<vtkSeedWidget> seedWidget;
  seedWidget->SetInteractor(interactor);
  seedWidget->SetRepresentation(widgetRep);
  // seedWidget->SetRepresentation(rep);

  seedWidget->On();
  window->Render();
  interactor->Start();

  return EXIT_SUCCESS;
}
