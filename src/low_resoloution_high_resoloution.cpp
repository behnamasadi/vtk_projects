#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
// #include <vtkExtractVisiblePoints.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkSmartPointer.h>

struct CallbackData {
  vtkActor *lowResActor;
  vtkActor *highResActor;
  vtkRenderer *renderer;
};

// void CameraModifiedCallback(vtkObject *caller, long unsigned int eventId,
//                             void *clientData, void *callData) {
//   vtkCamera *camera = static_cast<vtkCamera *>(caller);
//   vtkActor *lowResActor =
//       static_cast<vtkActor *>(reinterpret_cast<vtkActor **>(clientData)[0]);
//   vtkActor *highResActor =
//       static_cast<vtkActor *>(reinterpret_cast<vtkActor **>(clientData)[1]);
//   vtkRenderer *renderer =
//       static_cast<vtkRenderer *>(reinterpret_cast<vtkActor
//       **>(clientData)[2]);

//   // Get the distance of the camera from the focal point
//   double distance = camera->GetDistance();

//   // Define your threshold distance here
//   double thresholdDistance = 50.0;

//   // Switch visibility based on the distance
//   if (distance > thresholdDistance) {
//     lowResActor->SetVisibility(1);
//     highResActor->SetVisibility(0);
//   } else {
//     lowResActor->SetVisibility(0);
//     highResActor->SetVisibility(1);
//   }

//   renderer->GetRenderWindow()->Render();
// }

void CameraModifiedCallback2(vtkObject *caller, long unsigned int eventId,
                             void *clientData, void *callData) {
  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  CallbackData *data = static_cast<CallbackData *>(clientData);

  vtkActor *lowResActor = data->lowResActor;
  vtkActor *highResActor = data->highResActor;
  vtkRenderer *renderer = data->renderer;

  // Get the distance of the camera from the focal point
  double distance = camera->GetDistance();

  // Define your threshold distance here
  double thresholdDistance = 5.0;

  // Switch visibility based on the distance
  if (distance > thresholdDistance) {
    lowResActor->SetVisibility(1);
    highResActor->SetVisibility(0);
  } else {
    lowResActor->SetVisibility(0);
    highResActor->SetVisibility(1);
  }

  renderer->GetRenderWindow()->Render();
}

void CameraModifiedCallback(vtkObject *caller, long unsigned int eventId,
                            void *clientData, void *callData) {
  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  vtkRenderer *renderer =
      static_cast<vtkRenderer *>(reinterpret_cast<void **>(clientData)[0]);
  vtkPointSource *highResPointSource =
      static_cast<vtkPointSource *>(reinterpret_cast<void **>(clientData)[1]);

  //   std::cout << "highResPointSource->GetNumberOfPoints()"
  //             << highResPointSource->GetNumberOfPoints() << std::endl;

  // Set up the vtkSelectVisiblePoints filter
  vtkSmartPointer<vtkSelectVisiblePoints> selectVisiblePoints =
      vtkSmartPointer<vtkSelectVisiblePoints>::New();
  selectVisiblePoints->SetInputConnection(highResPointSource->GetOutputPort());
  selectVisiblePoints->SetRenderer(renderer);
  selectVisiblePoints->Update();

  // Get the number of visible points
  vtkIdType numberOfVisiblePoints =
      selectVisiblePoints->GetOutput()->GetNumberOfPoints();
  std::cout << "Number of visible points: " << numberOfVisiblePoints
            << std::endl;

  renderer->GetRenderWindow()->Render();
}

int main2(int, char *[]) {
  // Create two point sources
  vtkSmartPointer<vtkPointSource> lowResPointSource =
      vtkSmartPointer<vtkPointSource>::New();
  lowResPointSource->SetNumberOfPoints(100);

  vtkSmartPointer<vtkPointSource> highResPointSource =
      vtkSmartPointer<vtkPointSource>::New();
  highResPointSource->SetNumberOfPoints(50000000);

  // Create mappers
  vtkSmartPointer<vtkPolyDataMapper> lowResMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lowResMapper->SetInputConnection(lowResPointSource->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> highResMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  highResMapper->SetInputConnection(highResPointSource->GetOutputPort());

  // Create actors
  vtkSmartPointer<vtkActor> lowResActor = vtkSmartPointer<vtkActor>::New();
  lowResActor->SetMapper(lowResMapper);

  vtkSmartPointer<vtkActor> highResActor = vtkSmartPointer<vtkActor>::New();
  highResActor->SetMapper(highResMapper);

  // Initially set low resolution actor visible and high resolution actor
  // invisible
  lowResActor->SetVisibility(1);
  highResActor->SetVisibility(0);

  // Create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(lowResActor);
  renderer->AddActor(highResActor);
  renderer->SetBackground(0, 0, 0); // Background color

  // Get the camera
  vtkCamera *camera = renderer->GetActiveCamera();

  // Create a callback command
  vtkSmartPointer<vtkCallbackCommand> callback =
      vtkSmartPointer<vtkCallbackCommand>::New();
  callback->SetCallback(CameraModifiedCallback);

  // Set up callback data
  CallbackData data;
  data.lowResActor = lowResActor;
  data.highResActor = highResActor;
  data.renderer = renderer;

  callback->SetClientData(&data);

  // Add the observer to the camera
  camera->AddObserver(vtkCommand::ModifiedEvent, callback);

  // Render and start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

int main(int, char *[]) {
  // Create two point sources
  vtkSmartPointer<vtkPointSource> lowResPointSource =
      vtkSmartPointer<vtkPointSource>::New();
  lowResPointSource->SetNumberOfPoints(100);

  vtkSmartPointer<vtkPointSource> highResPointSource =
      vtkSmartPointer<vtkPointSource>::New();
  highResPointSource->SetNumberOfPoints(500000);

  // Create mappers
  vtkSmartPointer<vtkPolyDataMapper> lowResMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lowResMapper->SetInputConnection(lowResPointSource->GetOutputPort());

  vtkSmartPointer<vtkPolyDataMapper> highResMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  highResMapper->SetInputConnection(highResPointSource->GetOutputPort());

  // Create actors
  vtkSmartPointer<vtkActor> lowResActor = vtkSmartPointer<vtkActor>::New();
  lowResActor->SetMapper(lowResMapper);

  vtkSmartPointer<vtkActor> highResActor = vtkSmartPointer<vtkActor>::New();
  highResActor->SetMapper(highResMapper);

  // Initially set low resolution actor visible and high resolution actor
  // invisible
  lowResActor->SetVisibility(1);
  highResActor->SetVisibility(0);

  // Create a renderer and render window
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene
  renderer->AddActor(lowResActor);
  renderer->AddActor(highResActor);
  renderer->SetBackground(0, 0, 0); // Background color

  // Get the camera
  vtkCamera *camera = renderer->GetActiveCamera();

  // Create a callback command
  vtkSmartPointer<vtkCallbackCommand> callback =
      vtkSmartPointer<vtkCallbackCommand>::New();
  callback->SetCallback(CameraModifiedCallback);

  // Set up client data
  void *data[2] = {renderer, highResPointSource};
  callback->SetClientData(data);

  // Add the observer to the camera
  camera->AddObserver(vtkCommand::ModifiedEvent, callback);

  // Render and start interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
