#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

int main() {
  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputData(sphereSource->GetOutput());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(actor);
  renderer->SetBackground(0, 0, 0);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkSmartPointer<vtkPointHandleRepresentation3D> handle =
      vtkSmartPointer<vtkPointHandleRepresentation3D>::New();
  vtkSmartPointer<vtkDistanceRepresentation3D> rep =
      vtkSmartPointer<vtkDistanceRepresentation3D>::New();
  rep->SetHandleRepresentation(handle);

  vtkSmartPointer<vtkDistanceWidget> widget =
      vtkSmartPointer<vtkDistanceWidget>::New();
  widget->SetInteractor(renderWindowInteractor);
  widget->SetRepresentation(rep);
  widget->SetWidgetStateToManipulate();
  widget->EnabledOn();
  widget->ProcessEventsOn();

  renderer->ResetCamera();
  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}