#include <vtkActor.h>
#include <vtkAppendPolyData.h>
#include <vtkArcSource.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

struct CallbackData {
  vtkRenderer *renderer;
  vtkArcSource *arcSource;
};

void UpdateArcPosition(vtkObject *caller, unsigned long eventId,
                       void *clientData, void *callData) {
  CallbackData *data = static_cast<CallbackData *>(clientData);
  double *focalPoint = data->renderer->GetActiveCamera()->GetFocalPoint();
  data->arcSource->SetCenter(focalPoint);
  data->renderer->Render();
}

int main(int, char *[]) {
  // Create a cone
  vtkSmartPointer<vtkConeSource> coneSource =
      vtkSmartPointer<vtkConeSource>::New();
  coneSource->Update();

  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(coneSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  // A renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();

  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  // An interactor
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Set the custom style to use for interaction.
  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  renderWindowInteractor->SetInteractorStyle(style);

  // Create arcs for visual representation of the trackball
  vtkSmartPointer<vtkArcSource> arcSource =
      vtkSmartPointer<vtkArcSource>::New();
  arcSource->SetPoint1(0, 1, 0);
  arcSource->SetPoint2(0, -1, 0);
  arcSource->SetCenter(0, 0, 0);
  arcSource->Update();

  double *focalPoint = renderer->GetActiveCamera()->GetFocalPoint();
  arcSource->SetCenter(focalPoint);

  // Create the X, Y and Z arcs using transformation
  vtkSmartPointer<vtkTransform> transformX =
      vtkSmartPointer<vtkTransform>::New();
  transformX->RotateY(90);

  vtkSmartPointer<vtkTransform> transformY =
      vtkSmartPointer<vtkTransform>::New();
  // No need for rotation on Y as it's the default orientation of arc

  vtkSmartPointer<vtkTransform> transformZ =
      vtkSmartPointer<vtkTransform>::New();
  transformZ->RotateX(90);

  // Apply the transformations
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterX =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilterX->SetTransform(transformX);
  transformFilterX->SetInputConnection(arcSource->GetOutputPort());

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilterZ =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilterZ->SetTransform(transformZ);
  transformFilterZ->SetInputConnection(arcSource->GetOutputPort());

  // Append all arcs together
  vtkSmartPointer<vtkAppendPolyData> appendFilter =
      vtkSmartPointer<vtkAppendPolyData>::New();
  appendFilter->AddInputConnection(arcSource->GetOutputPort());
  appendFilter->AddInputConnection(transformFilterX->GetOutputPort());
  appendFilter->AddInputConnection(transformFilterZ->GetOutputPort());
  appendFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> arcMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  arcMapper->SetInputConnection(appendFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> arcActor = vtkSmartPointer<vtkActor>::New();
  arcActor->SetMapper(arcMapper);
  arcActor->GetProperty()->SetColor(1.0, 0.5, 0.0); // Color the arcs

  CallbackData data;
  data.renderer = renderer.Get();
  data.arcSource = arcSource.Get();

  //   vtkSmartPointer<vtkRenderer> renderer =
  //   vtkSmartPointer<vtkRenderer>::New();
  renderer->GetActiveCamera()->AddObserver(vtkCommand::ModifiedEvent,
                                           UpdateArcPosition, &data);

  // Add actors to the render
  renderer->AddActor(actor);
  renderer->AddActor(arcActor);
  renderer->SetBackground(0.1, 0.1, 0.1);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;

  // // Add actors to the render
  // renderer->AddActor(actor);
  // renderer->SetBackground(0.1, 0.1, 0.1);

  // renderWindow->Render();
  // renderWindowInteractor->Start();

  // return EXIT_SUCCESS;
}
