#include <QQuickVTKItem.h>
#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>

struct vtkItem : public QQuickVTKItem {
  Q_OBJECT

public:
  struct vtk_data : vtkObject {
    static vtk_data *New();
    vtkTypeMacro(vtk_data, vtkObject);

    vtkNew<vtkConeSource> cone;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
  };

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {
    vtkNew<vtk_data> vtk;
    std::cout << "this happens once" << std::endl;
    vtk->actor->SetMapper(vtk->mapper);
    vtk->mapper->SetInputConnection(vtk->cone->GetOutputPort());

    vtk->renderer->AddActor(vtk->actor);
    vtk->renderer->ResetCamera();
    vtk->renderer->SetBackground(0.0, 1.0, 1.0);
    vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
    vtk->renderer->SetGradientBackground(true);

    renderWindow->AddRenderer(vtk->renderer);
    renderWindow->SetMultiSamples(16);

    // vtkSmartPointer<CameraInteractorStyle> style =
    //     vtkSmartPointer<CameraInteractorStyle>::New();

    vtkNew<vtkRenderWindowInteractor> iRen;
    // vtk->renderer->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
    // style->SetDefaultRenderer(vtk->renderer);
    return vtk;
  }
};
