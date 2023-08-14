#include <vtkAxes.h>
#include <vtkAxesActor.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCamera.h>
#include <vtkCameraOrientationWidget.h>
#include <vtkCellArray.h>
#include <vtkConeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkImageData.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyle3D.h>
#include <vtkInteractorStyleImage.h>
#include <vtkInteractorStyleTerrain.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkInteractorStyleUnicam.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkPlaneSource.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProgrammableSource.h>
#include <vtkProperty.h>
#include <vtkPyramid.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTetra.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
#include <vtkVertexGlyphFilter.h>

#include <algorithm>
#include <cctype>
#include <string>

//#include <vtkZSpaceInteractorStyle.h>

// https://discourse.slicer.org/t/programmatically-set-camera-position/8301/3
// http://www.codecolony.de/docs/camera2.htm
// https://stackoverflow.com/questions/43046798/how-to-rotate-a-vtk-scene-around-a-specific-point
// https://github.com/Kitware/VTK/blob/master/Interaction/Style/vtkInteractorStyleTerrain.cxx
namespace {

class CustomTerrainInteractorStyle : public vtkInteractorStyleTerrain {
public:
  static CustomTerrainInteractorStyle *New();

  virtual void Rotate() override {

    std::cout << "CustomTerrainInteractorStyle::Rotate" << std::endl;

    if (this->CurrentRenderer == nullptr) {
      return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    int dx = -(rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0]);
    int dy = -(rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1]);

    const int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

    double a = dx / static_cast<double>(size[0]) * 180.0;
    double e = dy / static_cast<double>(size[1]) * 180.0;

    if (rwi->GetShiftKey()) {
      if (abs(dx) >= abs(dy)) {
        e = 0.0;
      } else {
        a = 0.0;
      }
    }

    // Move the camera.
    // Make sure that we don't hit the north pole singularity.

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    // camera->Azimuth(a);
    camera->Azimuth(0);

    double dop[3], vup[3];

    camera->GetDirectionOfProjection(dop);
    vtkMath::Normalize(dop);
    camera->GetViewUp(vup);
    vtkMath::Normalize(vup);

    double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(dop, vup)));
    if ((angle + e) > 179.0 || (angle + e) < 1.0) {
      e = 0.0;
    }

    // camera->Elevation(0);
    camera->Elevation(e);

    if (this->AutoAdjustCameraClippingRange) {
      this->CurrentRenderer->ResetCameraClippingRange();
    }

    rwi->Render();
  }

  void Pan() {
    std::cout << "CustomTerrainInteractorStyle::Pan" << std::endl;

    if (this->CurrentRenderer == nullptr) {
      return;
    }

    vtkRenderWindowInteractor *rwi = this->Interactor;

    // Get the vector of motion

    double fp[3], focalPoint[3], pos[3], v[3], p1[4], p2[4];

    vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
    camera->GetPosition(pos);
    camera->GetFocalPoint(fp);

    this->ComputeWorldToDisplay(fp[0], fp[1], fp[2], focalPoint);

    this->ComputeDisplayToWorld(rwi->GetEventPosition()[0],
                                rwi->GetEventPosition()[1], focalPoint[2], p1);

    this->ComputeDisplayToWorld(rwi->GetLastEventPosition()[0],
                                rwi->GetLastEventPosition()[1], focalPoint[2],
                                p2);

    for (int i = 0; i < 3; i++) {
      v[i] = p2[i] - p1[i];
      pos[i] += v[i];
      fp[i] += v[i];
    }

    camera->SetPosition(pos);
    camera->SetFocalPoint(fp);

    if (rwi->GetLightFollowCamera()) {
      this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
    }

    rwi->Render();
  }

  vtkTypeMacro(CustomTerrainInteractorStyle, CustomTerrainInteractorStyle);
};

vtkStandardNewMacro(CustomTerrainInteractorStyle);

// Define interaction style
class KeyPressInteractorStyle : public vtkInteractorStyleTrackballCamera {
private:
  vtkSmartPointer<vtkActor> Actor;
  enum INTERACTION_MODE { CAMERA_ROTATION, ACTOR_MANIPULATION };
  INTERACTION_MODE interactionMode = CAMERA_ROTATION;

public:
  static KeyPressInteractorStyle *New();
  vtkTypeMacro(KeyPressInteractorStyle, vtkInteractorStyleTrackballCamera);

  virtual void OnKeyPress() override {
    // Get the keypress
    vtkRenderWindowInteractor *rwi = this->Interactor;
    std::string key = rwi->GetKeySym();

    std::transform(key.begin(), key.end(), key.begin(),
                   [](unsigned char c) { return std::tolower(c); });

    if (!key.compare("p")) {
      interactionMode = ACTOR_MANIPULATION;
    }

    // Output the key that was pressed
    std::cout << "Pressed " << key << std::endl;

    // Handle an arrow key
    if (key == "Up") {
      std::cout << "The up arrow was pressed." << std::endl;
    }

    // Handle a "normal" key
    if (key == "a") {
      std::cout << "The a key was pressed." << std::endl;
    }

    // Forward events
    vtkInteractorStyleTrackballCamera::OnKeyPress();
  }

  virtual void Spin() override {
    std::cout << "Spin." << std::endl;

    // vtkInteractorStyleTrackballCamera::Spin();
  }

  virtual void Dolly() override {
    std::cout << "Dolly." << std::endl;
    vtkInteractorStyleTrackballCamera::Dolly();
  }

  virtual void Pan() override {
    std::cout << "Pan." << std::endl;
    vtkInteractorStyleTrackballCamera::Pan();
  }

  virtual void EnvironmentRotate() override {
    std::cout << "EnvironmentRotate." << std::endl;
  }

  virtual void Rotate() override {

    // vtkRenderWindowInteractor *rwi = this->GetInteractor();

    // std::cout << "rwi->GetEventPosition():" << rwi->GetEventPosition()
    //           << std::endl;

    // rwi->GetEventPosition();
    // rwi->GetLastEventPosition();
    // // Forward events
    // vtkInteractorStyleTrackballCamera::Rotate();

    /*
        vtkRenderWindowInteractor *rwi = this->Interactor;
        vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();

        double *focalPoint = camera->GetFocalPoint();
        double *viewUp = camera->GetViewUp();
        double *position = camera->GetPosition();
        double axis[3];
        axis[0] = -camera->GetViewTransformMatrix()->GetElement(0, 0);
        axis[1] = -camera->GetViewTransformMatrix()->GetElement(0, 1);
        axis[2] = -camera->GetViewTransformMatrix()->GetElement(0, 2);

        // Build The transformatio
       /////////////////////////////////////////////////
        vtkSmartPointer<vtkTransform> transform =
            vtkSmartPointer<vtkTransform>::New();
        transform->Identity();

        transform->Translate(d->center[0], d->center[1], d->center[2]);
        transform->RotateWXYZ(rxf, viewUp); // Azimuth
        transform->RotateWXYZ(ryf, axis);   // Elevation
        transform->Translate(-d->center[0], -d->center[1], -d->center[2]);

        double newPosition[3];
        transform->TransformPoint(position, newPosition); // Transform Position
        double newFocalPoint[3];
        transform->TransformPoint(focalPoint,
                                  newFocalPoint); // Transform Focal Point

        camera->SetPosition(newPosition);
        camera->SetFocalPoint(newFocalPoint);

        // Orhthogonalize View Up
       //////////////////////////////////////////////////
        camera->OrthogonalizeViewUp();
        renderer->ResetCameraClippingRange();

        rwi->Render();

    */

    /**/
    if (this->CurrentRenderer == nullptr) {
      return;
    }

    // this->CurrentRenderer->GetActors()

    if (interactionMode == CAMERA_ROTATION) {

      vtkRenderWindowInteractor *rwi = this->Interactor;

      double *center = this->CurrentRenderer->GetCenter();
      std::cout << "center[0]: " << center[0] << " center[1]: " << center[1]
                << std::endl;

      int dx = rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0];
      int dy = rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1];

      std::cout << "dx: " << dx << " dy: " << dy << std::endl;

      const int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

      double delta_elevation = -20.0 / size[1];
      double delta_azimuth = -20.0 / size[0];

      double rxf = dx * delta_azimuth * this->MotionFactor;
      double ryf = dy * delta_elevation * this->MotionFactor;

      vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
      double *cameraPosition = camera->GetPosition();

      std::cout << "cameraPosition[0] " << cameraPosition[0]
                << " cameraPosition[1] " << cameraPosition[1]
                << " cameraPosition[2] " << cameraPosition[2] << std::endl;

      double *cameraGetFocalPoint = camera->GetFocalPoint();

      std::cout << "cameraGetFocalPoint[0] " << cameraGetFocalPoint[0]
                << " cameraGetFocalPoint[1] " << cameraGetFocalPoint[1]
                << " cameraGetFocalPoint[2] " << cameraGetFocalPoint[2]
                << std::endl;

      camera->SetViewUp(0, 1, 0);

      camera->OrthogonalizeViewUp();
      // camera->SetClippingRange();

      camera->Azimuth(rxf);
      // camera->Azimuth(0);
      camera->Elevation(ryf);

      std::cout << "rxf: " << rxf << std::endl;
      std::cout << "ryf: " << ryf << std::endl;

      camera->OrthogonalizeViewUp();

      if (this->AutoAdjustCameraClippingRange) {
        this->CurrentRenderer->ResetCameraClippingRange();
      }

      if (rwi->GetLightFollowCamera()) {
        this->CurrentRenderer->UpdateLightsGeometryToFollowCamera();
      }

      rwi->Render();

      std::cout << "Rotate." << std::endl;
    } else if (interactionMode == ACTOR_MANIPULATION) {
      std::cout << "ACTOR_MANIPULATION." << std::endl;
    }
  }

  virtual void OnLeftButtonDown() {
    std::cout << "Pressed left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
    // vtkInteractorStyleTrackballActor::OnLeftButtonUp();
  }

  virtual void OnLeftButtonUp() {
    std::cout << "Released left mouse button." << std::endl;

    vtkNew<vtkMatrix4x4> m;
    this->Actor->GetMatrix(m);
    std::cout << "Matrix: " << endl << *m << std::endl;

    // Forward events
    vtkInteractorStyleTrackballCamera::OnLeftButtonUp();
  }

  void SetActor(vtkSmartPointer<vtkActor> actor) { this->Actor = actor; }
};
vtkStandardNewMacro(KeyPressInteractorStyle);

} // namespace

/*
https://vtk.org/doc/nightly/html/classvtkCellArray.html

Internally, the connectivity table is represented as two arrays: Offsets and
Connectivity.

Offsets is an array of [numCells+1] values indicating the index in the
Connectivity array where each cell's points start. The last value is always the
length of the Connectivity array.

The Connectivity array stores the lists of point ids for each cell.

Thus, for a dataset consisting of 2 triangles, a quad, and a line, the internal
arrays will appear as follows:

Topology:
---------
Cell 0: Triangle | point ids: {0, 1, 2}
Cell 1: Triangle | point ids: {5, 7, 2}
Cell 2: Quad     | point ids: {3, 4, 6, 7}
Cell 4: Line     | point ids: {5, 8}

vtkCellArray (current):
-----------------------
Offsets:      {0, 3, 6, 10, 12}
Connectivity: {0, 1, 2, 5, 7, 2, 3, 4, 6, 7, 5, 8}
               0        3        6           10     -> size is 12
               |Cell 0  ||Cell1  ||Cell2     ||C3|
*/

static void Lorenz(void *arg) {
  double sigma = 10.0; /* The Lorenz paramaters */
  double beta = 8.0 / 3.0;
  double rho = 28.0;
  double h = .001; /* Integration step size */

  double x, y, z;
  double xx, yy, zz;
  x = 0.1;
  y = 0.1;
  z = 0.1;
  vtkNew<vtkPoints> points;
  // Get to a stable starting point
  for (int i = 0; i < 1000; ++i) {
    xx = x + h * sigma * (y - x);
    yy = y + h * (x * (rho - z) - y);
    zz = z + h * (x * y - (beta * z));
    x = xx;
    y = yy;
    z = zz;
  }
  for (int i = 0; i < 500000; ++i) {
    xx = x + h * sigma * (y - x);
    yy = y + h * (x * (rho - z) - y);
    zz = z + h * (x * y - (beta * z));
    points->InsertNextPoint(xx, yy, zz);
    x = xx;
    y = yy;
    z = zz;
  }
  vtkNew<vtkPolyData> pointsPolydata;
  pointsPolydata->SetPoints(points);
  vtkNew<vtkVertexGlyphFilter> vertexFilter;
  vertexFilter->SetInputData(pointsPolydata);
  vertexFilter->Update();
  vtkProgrammableSource *ps = static_cast<vtkProgrammableSource *>(arg);
  vtkPolyData *output = ps->GetPolyDataOutput();
  output->DeepCopy(vertexFilter->GetOutput());
}

int main() {
  vtkNew<vtkNamedColors> namedColor;

  // grid background
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(200);
  planeSource->SetYResolution(200);
  planeSource->SetOrigin(0, 0, 0);
  planeSource->SetPoint1(100, 0, 0);
  planeSource->SetPoint2(0, 0, 100);

  // Don't use this method to define the plane. Instead, use it to rotate the
  // plane around the current center point.
  // planeSource->SetCenter(0.0, 0.0, 0.0);
  // planeSource->SetNormal(0.0, 0.0, 1.0);
  // planeSource->Update();

  // planeSource->SetXResolution(10);
  // planeSource->SetYResolution(10);
  // planeSource->SetCenter(0.0, 0.0, 0.0);
  // planeSource->SetNormal(0.0, 0.0, 1.0);
  // planeSource->Update();

  // Create a mapper and actor for the plane: show it as a wireframe
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();
  planeActor->GetProperty()->SetColor(
      namedColor->GetColor3d("Silver").GetData());
  planeActor->PickableOff();

  // adding a single point
  vtkNew<vtkPoints> points;
  const float p0[3] = {1.0, 0.0, 3.0};

  vtkIdType pid[1];
  pid[0] = points->InsertNextPoint(p0);

  vtkNew<vtkCellArray> vertices;
  vertices->InsertNextCell(1, pid);

  vtkNew<vtkPolyData> polyDataPoint;
  polyDataPoint->SetPoints(points);
  polyDataPoint->SetVerts(vertices);

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyDataPoint);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(namedColor->GetColor3d("Tomato").GetData());
  actor->GetProperty()->SetPointSize(20);

  // Make a regular square pyramid.
  int numberOfVertices = 5;

  vtkSmartPointer<vtkPoints> pointsPyramid = vtkSmartPointer<vtkPoints>::New();

  float p0Pyramid[3] = {1.0, 1.0, 0.0};
  float p1Pyramid[3] = {-1.0, 1.0, 0.0};
  float p2Pyramid[3] = {-1.0, -1.0, 0.0};
  float p3Pyramid[3] = {1.0, -1.0, 0.0};
  float p4Pyramid[3] = {0.0, 0.0, 1.0};

  pointsPyramid->InsertNextPoint(p0Pyramid);
  pointsPyramid->InsertNextPoint(p1Pyramid);
  pointsPyramid->InsertNextPoint(p2Pyramid);
  pointsPyramid->InsertNextPoint(p3Pyramid);
  pointsPyramid->InsertNextPoint(p4Pyramid);

  vtkSmartPointer<vtkPyramid> pyramid = vtkSmartPointer<vtkPyramid>::New();
  for (int i = 0; i < numberOfVertices; ++i) {
    pyramid->GetPointIds()->SetId(i, i);
  }

  vtkSmartPointer<vtkUnstructuredGrid> ug =
      vtkSmartPointer<vtkUnstructuredGrid>::New();
  ug->SetPoints(pointsPyramid);
  ug->InsertNextCell(pyramid->GetCellType(), pyramid->GetPointIds());

  // Create a mapper and actor
  vtkNew<vtkDataSetMapper> mapper2;
  mapper2->SetInputData(ug);

  vtkNew<vtkActor> actor2;
  actor2->SetMapper(mapper2);
  actor2->SetForceTranslucent(true);

  actor2->GetProperty()->SetRepresentation(1);
  double r, g, b;
  r = 0.0;
  g = 0.0;
  b = 1.0;

  // actor2->GetProperty()->SetColor(r, g, b);
  // actor2->GetProperty()->SetLineWidth(1.5);

  actor2->GetProperty()->SetRepresentationToWireframe();
  actor2->GetProperty()->SetColor(namedColor->GetColor3d("Gold").GetData());

  // actor2->GetProperty()->SetColor(namedColor->GetColor3d("Yellow").GetData());

  // adding axes
  vtkNew<vtkAxesActor> axesActor;
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(axesActor);
  renderer->AddActor(actor2);
  renderer->AddActor(planeActor);

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(300, 300);

  //   // We have two different approaches.
  // #ifdef USE_SCREEN_SIZE
  //   // This allows you to resize the window and shows the window name.
  //   renderWindow->Render();
  //   renderWindow->SetSize(renderWindow->GetScreenSize());
  // #else
  //   // Set to true to get full screen mode.
  //   // This uses the whole screen for the image.
  //   renderWindow->SetFullScreen(true);
  // #endif

  renderWindow->SetWindowName("Tutorial_Step1");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<KeyPressInteractorStyle> style;
  renderWindowInteractor->SetInteractorStyle(style);

  // vtkNew<vtkInteractorStyle3D> style;
  // renderWindowInteractor->SetInteractorStyle(style);

  // vtkNew<vtkZSpaceInteractorStyle> style;
  // renderWindowInteractor->SetInteractorStyle(style);

  // vtkNew<vtkInteractorStyleUnicam> UnicamStyle;
  // renderWindowInteractor->SetInteractorStyle(UnicamStyle);

  // vtkNew<vtkInteractorStyleImage> style;
  // renderWindowInteractor->SetInteractorStyle(style);

  // vtkNew<vtkInteractorStyleTerrain> style;
  // renderWindowInteractor->SetInteractorStyle(style);

  // vtkNew<CustomTerrainInteractorStyle> style;
  // renderWindowInteractor->SetInteractorStyle(style);

  style->SetCurrentRenderer(renderer);
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetPosition(1, 1, 1);

  // renderWindowInteractor->AddObserver()
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // renderer->AddActor(actor);
  renderer->SetBackground(namedColor->GetColor3d("DarkGreen").GetData());

  vtkNew<vtkCameraOrientationWidget> camOrientManipulator;
  camOrientManipulator->SetParentRenderer(renderer);
  // Enable the widget.
  camOrientManipulator->On();

  // Create a mapper and actor
  // Create a cone
  vtkNew<vtkConeSource> coneSource;
  coneSource->Update();
  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(coneSource->GetOutputPort());
  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);
  coneActor->RotateY(45);
  coneActor->SetPosition(2, 2, 3);
  coneActor->GetProperty()->SetColor(namedColor->GetColor3d("Gold").GetData());

  renderer->AddActor(coneActor);

  // Create a renderer, render window, and interactor

  renderWindow->AddRenderer(renderer);
  renderWindow->SetWindowName("RotateActor");
  style->SetActor(coneActor);

  // Setup the text and add it to the renderer
  //📌📍🖈🚩
  vtkNew<vtkBillboardTextActor3D> textActor;
  // textActor->SetInput("📌📍🖈🚩");
  textActor->SetInput("U+1F6A9 123📌📍🖈🚩🚩456");
  textActor->SetPosition(1, 1, 1);
  textActor->GetTextProperty()->SetFontSize(12);
  textActor->GetTextProperty()->SetColor(
      namedColor->GetColor3d("Gold").GetData());
  textActor->GetTextProperty()->SetJustificationToCentered();
  renderer->AddActor(textActor);

  vtkNew<vtkImageData> image;

  vtkNew<vtkProgrammableSource> source;
  source->SetExecuteMethod(Lorenz, source);
  source->Update();

  vtkNew<vtkPolyDataMapper> mapperLorenz;
  mapperLorenz->SetInputData(source->GetPolyDataOutput());
  vtkNew<vtkActor> actorLorenz;
  actorLorenz->SetMapper(mapperLorenz);
  actorLorenz->GetProperty()->SetColor(
      namedColor->GetColor3d("RosyBrown").GetData());
  renderer->AddActor(actorLorenz);

  renderWindow->Render();

  renderWindowInteractor->Initialize();

  renderWindowInteractor->Start();
}