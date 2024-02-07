// #include <vtkActor.h>
// #include <vtkCommand.h>
// #include <vtkImplicitPlaneRepresentation.h>
// #include <vtkImplicitPlaneWidget2.h>
// #include <vtkPLYReader.h>
// #include <vtkPlane.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSmartPointer.h>

// // Callback for the interaction
// class vtkIPWCallback : public vtkCommand {
// public:
//   static vtkIPWCallback *New() { return new vtkIPWCallback; }
//   virtual void Execute(vtkObject *caller, unsigned long, void *) {
//     vtkImplicitPlaneWidget2 *planeWidget =
//         reinterpret_cast<vtkImplicitPlaneWidget2 *>(caller);
//     vtkImplicitPlaneRepresentation *rep =
//         reinterpret_cast<vtkImplicitPlaneRepresentation *>(
//             planeWidget->GetRepresentation());
//     rep->GetPlane(this->Plane);
//   }
//   vtkIPWCallback() : Plane(0), Actor(0) {}
//   vtkSmartPointer<vtkPlane> Plane;
//   vtkSmartPointer<vtkActor> Actor;
// };

// int main() {
//   vtkSmartPointer<vtkPLYReader> reader =
//   vtkSmartPointer<vtkPLYReader>::New(); reader->SetFileName(
//       "/home/behnam/workspace/old.pcl_projects/3d_models/monkey.ply");

//   vtkSmartPointer<vtkPolyDataMapper> mapper =
//       vtkSmartPointer<vtkPolyDataMapper>::New();
//   mapper->SetInputConnection(reader->GetOutputPort());

//   vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
//   actor->SetMapper(mapper);

//   vtkSmartPointer<vtkRenderer> renderer =
//   vtkSmartPointer<vtkRenderer>::New(); vtkSmartPointer<vtkRenderWindow>
//   renderWindow =
//       vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//       vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   renderer->AddActor(actor);
//   renderer->SetBackground(0.1, 0.2, 0.4);

//   // Compute the bounding box of the object
//   actor->GetMapper()->Update(); // Make sure the actor's mapper is updated
//   double bounds[6];
//   actor->GetBounds(bounds);

//   // The center of the bounding box
//   double center[3] = {
//       (bounds[0] + bounds[1]) / 2.0,
//       (bounds[2] + bounds[3]) / 2.0,
//       (bounds[4] + bounds[5]) / 2.0,
//   };

//   // Create planes at the bounding box faces
//   vtkSmartPointer<vtkImplicitPlaneRepresentation> planeReps[6];

//   for (int i = 0; i < 6; ++i) {
//     vtkSmartPointer<vtkImplicitPlaneWidget2> planeWidget =
//         vtkSmartPointer<vtkImplicitPlaneWidget2>::New();
//     planeWidget->SetInteractor(renderWindowInteractor);
//     planeReps[i] = vtkSmartPointer<vtkImplicitPlaneRepresentation>::New();

//     // Set plane positions and orientations based on the bounding box
//     double position[3] = {center[0], center[1], center[2]};
//     double normal[3] = {0.0, 0.0, 0.0};
//     normal[i / 2] =
//         (i % 2 == 0) ? 1.0 : -1.0; // Alternate normals for opposite faces
//     position[i / 2] =
//         (i % 2 == 0) ? bounds[i] : bounds[i - 1]; // Position at bounds

//     planeReps[i]->SetNormal(normal);
//     planeReps[i]->SetOrigin(position);
//     planeReps[i]->PlaceWidget(bounds);
//     planeWidget->SetRepresentation(planeReps[i]);
//     planeWidget->On();

//     vtkSmartPointer<vtkIPWCallback> myCallback =
//         vtkSmartPointer<vtkIPWCallback>::New();
//     myCallback->Plane = vtkSmartPointer<vtkPlane>::New();
//     myCallback->Actor = actor;

//     planeWidget->AddObserver(vtkCommand::InteractionEvent, myCallback);
//   }

//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkClipPolyData.h>
#include <vtkCommand.h>
#include <vtkImplicitPlaneRepresentation.h>
#include <vtkImplicitPlaneWidget2.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPLYReader.h> // For loading PLY file
#include <vtkPlane.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>

namespace {
// Callback for the interaction
class vtkIPWCallback : public vtkCommand {
public:
  static vtkIPWCallback *New() { return new vtkIPWCallback; }
  virtual void Execute(vtkObject *caller, unsigned long, void *) override {
    vtkImplicitPlaneWidget2 *planeWidget =
        reinterpret_cast<vtkImplicitPlaneWidget2 *>(caller);
    vtkImplicitPlaneRepresentation *rep =
        reinterpret_cast<vtkImplicitPlaneRepresentation *>(
            planeWidget->GetRepresentation());
    rep->GetPlane(this->plane);
  }
  vtkIPWCallback() : plane(nullptr) {}
  vtkPlane *plane;
};
} // namespace

int main(int argc, char *argv[]) {
  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->SetRadius(10.0);

  vtkNew<vtkPLYReader> reader; // Use vtkPLYReader for PLY files

  vtkNew<vtkPlane> plane;
  vtkNew<vtkClipPolyData> clipper;
  clipper->SetClipFunction(plane);
  clipper->InsideOutOn();

  // Conditional logic based on input argument
  if (argc > 1) {
    reader->SetFileName(argv[1]);
    clipper->SetInputConnection(reader->GetOutputPort());
  } else {
    clipper->SetInputConnection(sphereSource->GetOutputPort());
  }

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(clipper->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkProperty> backFaces;
  backFaces->SetDiffuseColor(colors->GetColor3d("Gold").GetData());
  actor->SetBackfaceProperty(backFaces);

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("ImplicitPlaneWidget2Example");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  vtkNew<vtkIPWCallback> myCallback;
  myCallback->plane = plane;

  vtkNew<vtkImplicitPlaneRepresentation> rep;
  rep->SetPlaceFactor(1.25);
  if (argc > 1) {
    rep->PlaceWidget(reader->GetOutput()->GetBounds());
  } else {
    rep->PlaceWidget(sphereSource->GetOutput()->GetBounds());
  }
  rep->SetNormal(plane->GetNormal());

  vtkNew<vtkImplicitPlaneWidget2> planeWidget;
  planeWidget->SetInteractor(renderWindowInteractor);
  planeWidget->SetRepresentation(rep);
  planeWidget->AddObserver(vtkCommand::InteractionEvent, myCallback);

  // Setup camera view
  renderer->GetActiveCamera()->Azimuth(-60);
  renderer->GetActiveCamera()->Elevation(30);
  renderer->ResetCamera();
  renderer->GetActiveCamera()->Zoom(0.75);

  renderWindowInteractor->Initialize();
  renderWindow->Render();
  planeWidget->On();

  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
