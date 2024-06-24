#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkPointData.h>
#include <vtkPointSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelectVisiblePoints.h>
#include <vtkSmartPointer.h>

struct CallbackData {
  vtkActor *highResActor;
  vtkRenderer *renderer;
};

void CameraModifiedCallback(vtkObject *caller, long unsigned int eventId,
                            void *clientData, void *callData) {
  vtkCamera *camera = static_cast<vtkCamera *>(caller);
  vtkRenderer *renderer =
      static_cast<vtkRenderer *>(reinterpret_cast<void **>(clientData)[0]);
  vtkPointSource *highResPointSource =
      static_cast<vtkPointSource *>(reinterpret_cast<void **>(clientData)[1]);

  std::cout << "highResPointSource->GetNumberOfPoints()"
            << highResPointSource->GetNumberOfPoints() << std::endl;

  // Set up the vtkSelectVisiblePoints filter
  vtkNew<vtkSelectVisiblePoints> selectVisiblePoints;
  selectVisiblePoints->SetInputConnection(highResPointSource->GetOutputPort());
  selectVisiblePoints->SetRenderer(renderer);
  selectVisiblePoints->Update();

  // Get the number of visible points
  vtkIdType numberOfVisiblePoints =
      selectVisiblePoints->GetOutput()->GetNumberOfPoints();

  std::cout << "Number of visible points: " << numberOfVisiblePoints
            << std::endl;

  /*
    // Get the output of the vtkSelectVisiblePoints filter
    vtkSmartPointer<vtkPolyData> visiblePoints =
    selectVisiblePoints->GetOutput();

    // Create an array to store the point IDs
    vtkSmartPointer<vtkIdTypeArray> visiblePointIds =
        vtkSmartPointer<vtkIdTypeArray>::New();
    visiblePointIds->SetNumberOfComponents(1);
    visiblePointIds->SetNumberOfTuples(visiblePoints->GetNumberOfPoints());

    // Extract the point IDs
    for (vtkIdType i = 0; i < visiblePoints->GetNumberOfPoints(); ++i) {
      vtkIdType pointId =
          visiblePoints->GetPointData()->GetPedigreeIds()->GetValue(i);
      visiblePointIds->SetValue(i, pointId);
    }

    // Print out the point IDs
    for (vtkIdType i = 0; i < visiblePointIds->GetNumberOfTuples(); ++i) {
      vtkIdType pointId = visiblePointIds->GetValue(i);
      std::cout << "Visible point ID: " << pointId << std::endl;
    }*/

  // // Get the output of the vtkSelectVisiblePoints filter
  // vtkSmartPointer<vtkPolyData> visiblePoints =
  // selectVisiblePoints->GetOutput();

  // // Get the IDs of the visible points

  // vtkIdTypeArray *visiblePointIds = vtkIdTypeArray::SafeDownCast(
  //     visiblePoints->GetPointData()->GetArray("vtkOriginalPointIds"));

  // if (!visiblePointIds) {
  //   std::cerr << "No vtkOriginalPointIds array found!" << std::endl;
  //   return;
  // } else {
  //   // Print out the point IDs
  //   for (vtkIdType i = 0; i < visiblePointIds->GetNumberOfTuples(); ++i) {
  //     vtkIdType pointId = visiblePointIds->GetValue(i);
  //     std::cout << "Visible point ID: " << pointId << std::endl;
  //   }
  // }

  // this one works
  /*
    // Get the output of the vtkSelectVisiblePoints filter
    vtkSmartPointer<vtkPolyData> visiblePoints =
    selectVisiblePoints->GetOutput();

    // Get the points from the original source
    vtkSmartPointer<vtkPoints> originalPoints =
        highResPointSource->GetOutput()->GetPoints();

    // Iterate through the visible points and find their IDs in the original
    point
    // set
    std::cout << "Visible point IDs: " << std::endl;
    for (vtkIdType i = 0; i < visiblePoints->GetNumberOfPoints(); ++i) {
      double visiblePoint[3];
      visiblePoints->GetPoint(i, visiblePoint);

      // Find the corresponding ID in the original points
      for (vtkIdType j = 0; j < originalPoints->GetNumberOfPoints(); ++j) {
        double originalPoint[3];
        originalPoints->GetPoint(j, originalPoint);

        if (visiblePoint[0] == originalPoint[0] &&
            visiblePoint[1] == originalPoint[1] &&
            visiblePoint[2] == originalPoint[2]) {
          std::cout << j << std::endl; // Print the original point ID
          break;
        }
      }
    }
  */
  renderer->GetRenderWindow()->Render();
}

int main(int, char *[]) {

  vtkNew<vtkPointSource> highResPointSource;
  highResPointSource->SetNumberOfPoints(5000);

  // Create mappers

  vtkNew<vtkPolyDataMapper> highResMapper;
  highResMapper->SetInputConnection(highResPointSource->GetOutputPort());

  vtkNew<vtkActor> highResActor;
  highResActor->SetMapper(highResMapper);

  // Create a renderer and render window
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actors to the scene

  renderer->AddActor(highResActor);
  renderer->SetBackground(0, 0, 0); // Background color

  // Get the camera
  vtkCamera *camera = renderer->GetActiveCamera();

  // Create a callback command
  vtkNew<vtkCallbackCommand> callback;
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
