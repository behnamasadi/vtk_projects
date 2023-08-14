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

class CustomTerrainInteractorStyle : public vtkInteractorStyleTerrain {
public:
  static CustomTerrainInteractorStyle *New();

  // virtual void Rotate() override {

  //   std::cout << "CustomTerrainInteractorStyle::Rotate" << std::endl;

  //   if (this->CurrentRenderer == nullptr) {
  //     return;
  //   }

  //   vtkRenderWindowInteractor *rwi = this->Interactor;

  //   int dx = -(rwi->GetEventPosition()[0] - rwi->GetLastEventPosition()[0]);
  //   int dy = -(rwi->GetEventPosition()[1] - rwi->GetLastEventPosition()[1]);

  //   const int *size = this->CurrentRenderer->GetRenderWindow()->GetSize();

  //   double a = dx / static_cast<double>(size[0]) * 180.0;
  //   double e = dy / static_cast<double>(size[1]) * 180.0;

  //   if (rwi->GetShiftKey()) {
  //     if (abs(dx) >= abs(dy)) {
  //       e = 0.0;
  //     } else {
  //       a = 0.0;
  //     }
  //   }

  //   // Move the camera.
  //   // Make sure that we don't hit the north pole singularity.

  //   vtkCamera *camera = this->CurrentRenderer->GetActiveCamera();
  //   // camera->Azimuth(a);
  //   camera->Azimuth(0);

  //   double dop[3], vup[3];

  //   camera->GetDirectionOfProjection(dop);
  //   vtkMath::Normalize(dop);
  //   camera->GetViewUp(vup);
  //   vtkMath::Normalize(vup);

  //   double angle = vtkMath::DegreesFromRadians(acos(vtkMath::Dot(dop, vup)));
  //   if ((angle + e) > 179.0 || (angle + e) < 1.0) {
  //     e = 0.0;
  //   }

  //   // camera->Elevation(0);
  //   camera->Elevation(e);

  //   if (this->AutoAdjustCameraClippingRange) {
  //     this->CurrentRenderer->ResetCameraClippingRange();
  //   }

  //   rwi->Render();
  // }

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

int main(int argc, char **argv) {

  vtkNew<vtkNamedColors> namedColor;

  vtkNew<vtkConeSource> cone;
  cone->SetHeight(2);
  cone->Update();

  // mapper
  vtkNew<vtkPolyDataMapper> coneMapper;
  coneMapper->SetInputConnection(cone->GetOutputPort());

  // cone actor
  vtkNew<vtkActor> coneActor;
  coneActor->SetMapper(coneMapper);

  // adding plane

  int grid_size = 10;
  vtkNew<vtkPlaneSource> planeSource;
  planeSource->SetXResolution(20);
  planeSource->SetYResolution(20);
  planeSource->SetOrigin(0, 0, 0);
  planeSource->SetPoint1(grid_size, 0, 0);
  planeSource->SetPoint2(0, 0, grid_size);

  // Create a mapper and actor for the plane: show it as a wireframe
  vtkNew<vtkPolyDataMapper> planeMapper;
  planeMapper->SetInputConnection(planeSource->GetOutputPort());
  vtkNew<vtkActor> planeActor;
  planeActor->SetMapper(planeMapper);
  planeActor->GetProperty()->SetRepresentationToWireframe();
  planeActor->GetProperty()->SetColor(
      namedColor->GetColor3d("Silver").GetData());
  planeActor->PickableOff();
  planeActor->GetProperty()->SetAmbient(1.0);
  planeActor->GetProperty()->SetDiffuse(0.0);

  // adding axes
  vtkNew<vtkAxesActor> axesActor;

  // renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(coneActor);
  renderer->AddActor(axesActor);
  renderer->AddActor(planeActor);

  renderer->SetBackground(namedColor->GetColor3d("DarkGreen").GetData());
  renderer->GetActiveCamera()->SetViewUp(0, 1, 0);
  renderer->GetActiveCamera()->SetFocalPoint(0, 0, 0);
  renderer->GetActiveCamera()->SetPosition(1, 1, 1);

  // renderer window
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);

  // render Window Interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // InteractorStyles
  // vtkNew<vtkInteractorStyleTrackballActor> style;
  // vtkNew<CustomTerrainInteractorStyle> style;
  vtkNew<vtkInteractorStyleTerrain> style;
  // vtkNew<KeyPressInteractorStyle> style;

  renderWindowInteractor->SetInteractorStyle(style);
  // style->SetActor(coneActor);

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();
}
