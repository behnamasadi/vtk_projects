#include <QApplication>
#include <QFrame>
#include <QMainWindow>
#include <QVBoxLayout>
#include <QVTKOpenGLNativeWidget.h>
#include <vtkActor.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

int main(int argc, char **argv) {
  QApplication app(argc, argv);

  QMainWindow mainWindow;

  QFrame frame;
  QVBoxLayout layout;

  QVTKOpenGLNativeWidget vtkWidget(&frame);
  vtkSmartPointer<vtkGenericOpenGLRenderWindow> renderWindow =
      vtkSmartPointer<vtkGenericOpenGLRenderWindow>::New();
  vtkWidget.setRenderWindow(renderWindow);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetCenter(0.0, 0.0, 0.0);
  sphereSource->SetRadius(1.0);

  vtkSmartPointer<vtkPolyDataMapper> mapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  actor->SetMapper(mapper);

  renderer->AddActor(actor);
  renderer->ResetCamera();

  frame.setLayout(&layout);
  mainWindow.setCentralWidget(&frame);

  layout.addWidget(&vtkWidget);
  mainWindow.show();

  vtkRenderWindowInteractor *iren = renderWindow->GetInteractor();

  vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
      vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();
  iren->SetInteractorStyle(style);

  vtkSmartPointer<vtkDistanceWidget> distanceWidget =
      vtkSmartPointer<vtkDistanceWidget>::New();
  distanceWidget->SetInteractor(iren);
  distanceWidget->CreateDefaultRepresentation();

  iren->Initialize();
  // to avoid QVTKInteractor cannot control the event loop.
  // iren->Start();

  distanceWidget->On(); // Enable the distance widget

  return app.exec();
}
