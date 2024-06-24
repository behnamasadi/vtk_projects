// #include <iostream>
// #include <vtkCamera.h>
// #include <vtkFrustumSource.h>
// #include <vtkInteractorStyleTrackballCamera.h>
// #include <vtkNew.h>
// #include <vtkPlanes.h>
// #include <vtkPoints.h>
// #include <vtkPolyData.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>
// #include <vtkSelectEnclosedPoints.h>
// #include <vtkSmartPointer.h>
// #include <vtkVertexGlyphFilter.h>

// class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
// public:
//   static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

//   void SetPoints(vtkPoints *points) { this->Points = points; }

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

//     if (!this->Points) {
//       std::cerr << "No points data found!" << std::endl;
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

//     // Create a polydata to hold the points
//     vtkNew<vtkPolyData> pointsPolyData;
//     pointsPolyData->SetPoints(this->Points);

//     // Select points inside the frustum
//     vtkNew<vtkSelectEnclosedPoints> selectEnclosedPoints;
//     selectEnclosedPoints->SetInputData(pointsPolyData);
//     selectEnclosedPoints->SetSurfaceData(frustum);
//     selectEnclosedPoints->Update();

//     int numPointsInView = 0;
//     for (vtkIdType i = 0; i < this->Points->GetNumberOfPoints(); ++i) {
//       if (selectEnclosedPoints->IsInside(i)) {
//         ++numPointsInView;
//       }
//     }

//     std::cout << "Number of points in view: " << numPointsInView <<
//     std::endl;
//   }

//   virtual void OnLeftButtonDown() override {
//     GetNumberOfPointsInView();
//     std::cout << "Mouse Left Button Pressed" << std::endl;
//     vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
//   }

//   virtual void OnRightButtonDown() override {
//     GetNumberOfPointsInView();
//     std::cout << "Mouse Right Button Pressed" << std::endl;
//     vtkInteractorStyleTrackballCamera::OnRightButtonDown();
//   }

//   virtual void OnMouseMove() override {
//     GetNumberOfPointsInView();
//     std::cout << "Mouse Moved" << std::endl;
//     vtkInteractorStyleTrackballCamera::OnMouseMove();
//   }

// private:
//   vtkPoints *Points = nullptr;
// };

// int main(int, char *[]) {
//   vtkNew<vtkRenderWindow> renderWindow;
//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   vtkNew<vtkPoints> originalPoints;
//   vtkNew<vtkPolyData> originalPointsPolydata;
//   originalPointsPolydata->SetPoints(originalPoints);

//   vtkNew<vtkVertexGlyphFilter> originalPointsVertexFilter;
//   originalPointsVertexFilter->SetInputData(originalPointsPolydata);
//   originalPointsVertexFilter->Update();

//   vtkNew<vtkPolyDataMapper> originalPointsMapper;
//   originalPointsMapper->SetInputConnection(
//       originalPointsVertexFilter->GetOutputPort());

//   vtkNew<vtkActor> vtkActorInputPolydata;
//   vtkActorInputPolydata->SetMapper(originalPointsMapper);

//   vtkNew<vtkRenderer> renderer;
//   renderer->AddActor(vtkActorInputPolydata);

//   renderWindow->AddRenderer(renderer);

//   vtkNew<MouseInteractorStyle> style;
//   style->SetDefaultRenderer(renderer);
//   style->SetPoints(originalPoints);
//   renderWindowInteractor->SetInteractorStyle(style);

//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <iostream>
#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkVertexGlyphFilter.h>

class MouseInteractorStyle : public vtkInteractorStyleTrackballCamera {
public:
  static MouseInteractorStyle *New() { return new MouseInteractorStyle; }

  void SetPoints(vtkPoints *points) { this->Points = points; }

  void GetNumberOfPointsInView() {
    vtkRenderer *renderer = this->GetDefaultRenderer();
    if (!renderer) {
      std::cerr << "No renderer found!" << std::endl;
      return;
    }

    vtkCamera *camera = renderer->GetActiveCamera();
    if (!camera) {
      std::cerr << "No active camera found!" << std::endl;
      return;
    }

    if (!this->Points) {
      std::cerr << "No points data found!" << std::endl;
      return;
    }

    int numPointsInView = 0;

    // Get the camera's view transformation matrix
    vtkNew<vtkTransform> transform;
    transform->SetMatrix(camera->GetViewTransformMatrix());

    // Get the frustum planes
    double planes[24];
    camera->GetFrustumPlanes(renderer->GetTiledAspectRatio(), planes);

    // Loop through all points and check if they are in the view frustum
    for (vtkIdType i = 0; i < this->Points->GetNumberOfPoints(); ++i) {
      double point[3];
      this->Points->GetPoint(i, point);
      transform->TransformPoint(point, point);

      if (IsPointInViewFrustum(point, planes)) {
        ++numPointsInView;
      }
    }

    std::cout << "Number of points in view: " << numPointsInView << std::endl;
  }

  bool IsPointInViewFrustum(double point[3], double planes[24]) {
    for (int i = 0; i < 6; ++i) {
      double a = planes[i * 4 + 0];
      double b = planes[i * 4 + 1];
      double c = planes[i * 4 + 2];
      double d = planes[i * 4 + 3];
      if (a * point[0] + b * point[1] + c * point[2] + d < 0) {
        return false;
      }
    }
    return true;
  }

  virtual void OnLeftButtonDown() override {
    GetNumberOfPointsInView();
    std::cout << "Mouse Left Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnLeftButtonDown();
  }

  virtual void OnRightButtonDown() override {
    GetNumberOfPointsInView();
    std::cout << "Mouse Right Button Pressed" << std::endl;
    vtkInteractorStyleTrackballCamera::OnRightButtonDown();
  }

  virtual void OnMouseMove() override {
    GetNumberOfPointsInView();
    std::cout << "Mouse Moved" << std::endl;
    vtkInteractorStyleTrackballCamera::OnMouseMove();
  }

private:
  vtkPoints *Points = nullptr;
};

int main(int, char *[]) {
  vtkNew<vtkRenderWindow> renderWindow;
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  vtkNew<vtkPoints> originalPoints;
  originalPoints->InsertNextPoint(0, 0, 0); // Add some sample points
  originalPoints->InsertNextPoint(1, 1, 1);
  originalPoints->InsertNextPoint(-1, -1, -1);
  originalPoints->InsertNextPoint(2, 2, 2);
  originalPoints->InsertNextPoint(-2, -2, -2);

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

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(vtkActorInputPolydata);

  renderWindow->AddRenderer(renderer);

  vtkSmartPointer<MouseInteractorStyle> style =
      vtkSmartPointer<MouseInteractorStyle>::New();
  style->SetDefaultRenderer(renderer);
  style->SetPoints(originalPoints);
  renderWindowInteractor->SetInteractorStyle(style);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
