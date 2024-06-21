#include <cmath>
#include <iostream>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkCellIterator.h>
#include <vtkCommand.h>
#include <vtkFrustumSource.h>
#include <vtkGlyph3D.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMath.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOctreePointLocator.h>
#include <vtkPlanes.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkProperty2D.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

/*
class MouseInteractorStyle : public vtkCommand {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

  void Execute(vtkObject *caller, unsigned long eventId,
               void *callData) override {
    vtkRenderWindowInteractor *interactor =
        dynamic_cast<vtkRenderWindowInteractor *>(caller);
    if (eventId == vtkCommand::LeftButtonPressEvent) {
      std::cout << "Mouse Left Button Pressed" << std::endl;

    } else if (eventId == vtkCommand::RightButtonPressEvent) {
      std::cout << "Mouse Right Button Pressed" << std::endl;
    } else if (eventId == vtkCommand::MouseMoveEvent) {
      std::cout << "Mouse Moved" << std::endl;
    } else if (eventId == vtkCommand::MouseWheelForwardEvent) {
      std::cout << "Mouse Wheel Forward Event" << std::endl;
    } else if (eventId == vtkCommand::MouseWheelBackwardEvent) {
      std::cout << "Mouse Wheel Backward Event" << std::endl;
    }
  }
};
*/

class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

  //   void GetNumberOfPointsInView() {
  //     vtkRenderer *renderer = this->GetDefaultRenderer();
  //     if (!renderer) {
  //       std::cerr << "No renderer found!" << std::endl;
  //       return;
  //     }

  //     vtkCamera *camera = renderer->GetActiveCamera();
  //     if (!camera) {
  //       std::cerr << "No active camera found!" << std::endl;
  //       return;
  //     }

  //     // Get the camera frustum
  //     vtkNew<vtkFrustumSource> frustumSource;
  //     vtkNew<vtkPlanes> planes;
  //     camera->GetFrustumPlanes(renderer->GetTiledAspectRatio(), planes);
  //     frustumSource->SetPlanes(planes);

  //     vtkNew<vtkPolyData> frustum;
  //     frustumSource->Update();
  //     frustum->ShallowCopy(frustumSource->GetOutput());

  //     // Generate some points (for demonstration purposes)
  //     vtkNew<vtkPointSource> pointSource;
  //     pointSource->SetNumberOfPoints(1000);
  //     pointSource->SetCenter(0, 0, 0);
  //     pointSource->SetRadius(5.0);
  //     pointSource->Update();

  //     vtkPolyData *points = pointSource->GetOutput();

  //     // Select points inside the frustum
  //     vtkNew<vtkSelectEnclosedPoints> selectEnclosedPoints;
  //     selectEnclosedPoints->SetInputData(points);
  //     selectEnclosedPoints->SetSurfaceData(frustum);
  //     selectEnclosedPoints->Update();

  //     int numPointsInView = 0;
  //     for (vtkIdType i = 0; i < points->GetNumberOfPoints(); ++i) {
  //       if (selectEnclosedPoints->IsInside(i)) {
  //         ++numPointsInView;
  //       }
  //     }

  //     std::cout << "Number of points in view: " << numPointsInView <<
  //     std::endl;
  //   }

  virtual void OnLeftButtonDown() override {
    std::cout << "Mouse Left Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  virtual void OnRightButtonDown() override {
    std::cout << "Mouse Right Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
  }

  virtual void OnMouseMove() override {
    std::cout << "Mouse Moved" << std::endl;
    vtkInteractorStyleTrackballCamera::OnMouseMove();
  }

  virtual void OnMouseWheelForward() override {
    std::cout << "OnMouseWheelForward" << std::endl;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;
      }
    }

    std::cout << "renderer->GetActors()->GetNumberOfItems():"
              << renderer->GetActors()->GetNumberOfItems() << std::endl;

    for (int i = 0; i < renderer->GetActors()->GetNumberOfItems(); i++) {

      std::cout << "renderer->GetActors()[i].GetClassName():"
                << renderer->GetActors()[i].GetClassName() << std::endl;
    }

    //     int visiblePointsCount = 0;
    // for (vtkIdType i = 0; i < transformedPoints->GetNumberOfPoints(); i++)
    // {
    //     double tp[3];
    //     transformedPoints->GetPoint(i, tp);

    //     // Frustum check: Assuming symmetric frustum
    //     double aspectRatio = camera->GetAspect();
    //     double viewAngle = camera->GetViewAngle();
    //     double nearPlane = camera->GetClippingRange()[0];
    //     double farPlane = camera->GetClippingRange()[1];

    //     double angleRad = vtkMath::RadiansFromDegrees(viewAngle / 2.0);
    //     double tanAngle = tan(angleRad);

    //     // Check if the point is within the view frustum
    //     if (tp[2] < nearPlane || tp[2] > farPlane)
    //         continue;

    //     double halfHeight = tanAngle * tp[2];
    //     double halfWidth = aspectRatio * halfHeight;

    //     if (tp[0] >= -halfWidth && tp[0] <= halfWidth && tp[1] >= -halfHeight
    //     && tp[1] <= halfHeight)
    //     {
    //         visiblePointsCount++;
    //     }
    // }

    vtkInteractorStyleTrackballCamera::OnMouseWheelForward();
  }

  virtual void OnMouseWheelBackward() override {
    std::cout << "OnMouseWheelBackward" << std::endl;

    vtkRenderWindowInteractor *rwi = Interactor;

    vtkRenderer *renderer = CurrentRenderer;

    if (renderer) {
      std::cout << "renderer" << std::endl;
      vtkCamera *camera = renderer->GetActiveCamera();
      //   camera->GetFrustumPlanes()
      if (camera) {
        std::cout << "Active Camera Position: " << camera->GetPosition()[0]
                  << ", " << camera->GetPosition()[1] << ", "
                  << camera->GetPosition()[2] << std::endl;
      }
    }

    vtkInteractorStyleTrackballCamera::OnMouseWheelBackward();
  }
};

int main() {

  vtkNew<vtkPoints> originalPoints;

  vtkNew<vtkMinimalStandardRandomSequence> randomSequence;
  randomSequence->SetSeed(8775070);

  double cylanderRadius = 2.0;
  for (int i = 0; i < 20000;
       i++) { // Reduced number of points for faster rendering
    double x, y, z;
    x = randomSequence->GetRangeValue(4.0, 12.0);
    randomSequence->Next();
    y = randomSequence->GetRangeValue(-2.0, 2.0);
    randomSequence->Next();
    z = sqrt(cylanderRadius * cylanderRadius - y * y);

    originalPoints->InsertNextPoint(x, y, -z);
  }

  vtkNew<vtkPolyData> originalPointsPolydata;
  originalPointsPolydata->SetPoints(originalPoints);

  vtkNew<vtkVertexGlyphFilter> originalPointsVertexFilter;

  originalPointsVertexFilter->SetInputData(originalPointsPolydata);
  originalPointsVertexFilter->Update();

  vtkNew<vtkPolyDataMapper> originalPointsMapper;
  originalPointsMapper->SetInputConnection(
      originalPointsVertexFilter->GetOutputPort());

  vtkNew<vtkActor> vtkActorInputPolydata;
  vtkActorInputPolydata->SetMapper(originalPointsMapper);
  vtkActorInputPolydata->GetProperty()->SetColor(0.0, 0.0, 1.0); // Blue color
  vtkActorInputPolydata->GetProperty()->SetPointSize(
      2); // Increase point size for better visibility

  // Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(vtkActorInputPolydata);

  vtkCamera *camera = renderer->GetActiveCamera();
  auto viewAngle = camera->GetViewAngle();
  std::cout << viewAngle << std::endl;

  auto clippingRange = camera->GetClippingRange();

  std::cout << clippingRange[0] << "," << clippingRange[1] << std::endl;

  // camera->SetPosition(0, 0, 100);
  // camera->SetFocalPoint(0, 0, 0);
  // camera->Azimuth(30);
  // camera->Elevation(30);
  // camera->SetViewUp(0, 0, 1);
  // camera->SetClippingRange(5, 15);
  // camera->SetViewAngle(30);

  renderer->SetBackground(0.1, 0.2, 0.4); // Background color

  // Render window
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);

  // Render window interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);
  // auto style = renderWindowInteractor->GetInteractorStyle();

  //   vtkNew<vtkInteractorStyleTrackballCamera> style;
  //   renderWindowInteractor->SetInteractorStyle(style);

  vtkNew<MouseInteractorStyle> mouseInteractorStyle;
  renderWindowInteractor->SetInteractorStyle(mouseInteractorStyle);

  //   style->AddObserver(vtkCommand::LeftButtonPressEvent,
  //   mouseInteractorStyle);
  //   style->AddObserver(vtkCommand::RightButtonPressEvent,
  //   mouseInteractorStyle); style->AddObserver(vtkCommand::MouseMoveEvent,
  //   mouseInteractorStyle);
  //   style->AddObserver(vtkCommand::MouseWheelBackwardEvent,
  //   mouseInteractorStyle);
  //   style->AddObserver(vtkCommand::MouseWheelForwardEvent,
  //   mouseInteractorStyle);

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();
}