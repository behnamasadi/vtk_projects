#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRegularPolygonSource.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

// int main() {

//   vtkSmartPointer<vtkRenderer> renderer =
//   vtkSmartPointer<vtkRenderer>::New(); vtkSmartPointer<vtkRenderWindow>
//   renderWindow =
//       vtkSmartPointer<vtkRenderWindow>::New();
//   renderWindow->AddRenderer(renderer);
//   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
//       vtkSmartPointer<vtkRenderWindowInteractor>::New();
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   // Camera focal point sphere
//   vtkSmartPointer<vtkSphereSource> focalPointSphere =
//       vtkSmartPointer<vtkSphereSource>::New();
//   focalPointSphere->SetRadius(0.05); // Small radius
//   focalPointSphere->SetCenter(
//       0.0, 0.0, 0.0); // Center at the origin or camera focal point
//   vtkSmartPointer<vtkPolyDataMapper> focalPointMapper =
//       vtkSmartPointer<vtkPolyDataMapper>::New();
//   focalPointMapper->SetInputConnection(focalPointSphere->GetOutputPort());
//   vtkSmartPointer<vtkActor> focalPointActor =
//   vtkSmartPointer<vtkActor>::New();
//   focalPointActor->SetMapper(focalPointMapper);
//   renderer->AddActor(focalPointActor);

//   // Spherical ring using a regular polygon source
//   vtkSmartPointer<vtkRegularPolygonSource> ringSource =
//       vtkSmartPointer<vtkRegularPolygonSource>::New();
//   ringSource->SetNumberOfSides(50); // Makes it look like a circle
//   ringSource->SetRadius(1.0);       // Radius of the ring
//   ringSource->SetCenter(0.0, 0.0,
//                         0.0); // Center at the origin or adjust as needed

//   // Transform the ring to encircle the object
//   vtkSmartPointer<vtkTransform> transform =
//       vtkSmartPointer<vtkTransform>::New();
//   transform->RotateX(90.0); // Rotate to make it horizontal

//   vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
//       vtkSmartPointer<vtkTransformPolyDataFilter>::New();
//   transformFilter->SetInputConnection(ringSource->GetOutputPort());
//   transformFilter->SetTransform(transform);
//   transformFilter->Update();

//   vtkSmartPointer<vtkPolyDataMapper> ringMapper =
//       vtkSmartPointer<vtkPolyDataMapper>::New();
//   ringMapper->SetInputConnection(transformFilter->GetOutputPort());

//   vtkSmartPointer<vtkActor> ringActor = vtkSmartPointer<vtkActor>::New();
//   ringActor->SetMapper(ringMapper);
//   renderer->AddActor(ringActor);

//   // Set background color and initialize the camera view
//   renderer->SetBackground(0.1, 0.2, 0.4);
//   renderer->ResetCamera();

//   // Start the interaction
//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

// #include <vtkCircleSource.h>
#include <vtkCylinderSource.h>
#include <vtkLineSource.h>
#include <vtkTubeFilter.h>

int main() {
  // Assume renderer, renderWindow, and renderWindowInteractor are already set
  // up

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Create the first part of the cross (horizontal)
  vtkSmartPointer<vtkLineSource> lineSource1 =
      vtkSmartPointer<vtkLineSource>::New();
  lineSource1->SetPoint1(-0.1, 0.0, 0.0);
  lineSource1->SetPoint2(0.1, 0.0, 0.0);

  vtkSmartPointer<vtkPolyDataMapper> lineMapper1 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lineMapper1->SetInputConnection(lineSource1->GetOutputPort());

  vtkSmartPointer<vtkActor> lineActor1 = vtkSmartPointer<vtkActor>::New();
  lineActor1->SetMapper(lineMapper1);

  // Create the second part of the cross (vertical)
  vtkSmartPointer<vtkLineSource> lineSource2 =
      vtkSmartPointer<vtkLineSource>::New();
  lineSource2->SetPoint1(0.0, -0.1, 0.0);
  lineSource2->SetPoint2(0.0, 0.1, 0.0);

  vtkSmartPointer<vtkPolyDataMapper> lineMapper2 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  lineMapper2->SetInputConnection(lineSource2->GetOutputPort());

  vtkSmartPointer<vtkActor> lineActor2 = vtkSmartPointer<vtkActor>::New();
  lineActor2->SetMapper(lineMapper2);

  // Add the cross to the renderer
  renderer->AddActor(lineActor1);
  renderer->AddActor(lineActor2);

  // First ring (horizontal)
  vtkSmartPointer<vtkRegularPolygonSource> polygonSource1 =
      vtkSmartPointer<vtkRegularPolygonSource>::New();
  polygonSource1->SetNumberOfSides(50);
  polygonSource1->SetRadius(1.0);
  polygonSource1->SetCenter(0.0, 0.0, 0.0);

  vtkSmartPointer<vtkPolyDataMapper> polygonMapper1 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  polygonMapper1->SetInputConnection(polygonSource1->GetOutputPort());

  vtkSmartPointer<vtkActor> polygonActor1 = vtkSmartPointer<vtkActor>::New();
  polygonActor1->SetMapper(polygonMapper1);

  // Second ring (vertical, perpendicular to the first)
  vtkSmartPointer<vtkTransform> transform =
      vtkSmartPointer<vtkTransform>::New();
  transform->RotateX(90.0);

  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter =
      vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetInputConnection(polygonSource1->GetOutputPort());
  transformFilter->SetTransform(transform);
  transformFilter->Update();

  vtkSmartPointer<vtkPolyDataMapper> polygonMapper2 =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  polygonMapper2->SetInputConnection(transformFilter->GetOutputPort());

  vtkSmartPointer<vtkActor> polygonActor2 = vtkSmartPointer<vtkActor>::New();
  polygonActor2->SetMapper(polygonMapper2);

  // Add the rings to the renderer
  renderer->AddActor(polygonActor1);
  renderer->AddActor(polygonActor2);

  // Set background color and initialize the camera view
  renderer->SetBackground(0.1, 0.2, 0.4);
  renderer->ResetCamera();

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
