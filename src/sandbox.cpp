#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCamera.h>
#include <vtkCameraPass.h>
#include <vtkCaptionActor2D.h>
#include <vtkCell.h>
#include <vtkColorTransferFunction.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <vtkFollower.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkJPEGReader.h>
#include <vtkLODActor.h>
#include <vtkLegendScaleActor.h>
#include <vtkLineSource.h>
#include <vtkLookupTable.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkOverlayPass.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProp3DFollower.h>
#include <vtkProperty.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkSequencePass.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>

class MyLODActor : public vtkLODActor {
public:
  static MyLODActor *New();
  vtkTypeMacro(MyLODActor, vtkLODActor);

  void SetInteractionMode(bool mode) {
    interactionMode = mode;
    this->Modified();
  }

protected:
  void Render(vtkRenderer *ren) override {
    if (interactionMode) {
      // Set to low or medium resolution
      this->SetNumberOfLevels(2); // Or adjust as needed
    } else {
      // Set to high resolution
      this->SetNumberOfLevels(0);
    }
    vtkLODActor::Render(ren);
  }

private:
  bool interactionMode = false;
};

vtkStandardNewMacro(MyLODActor);

int main() {
  // Create a larger sphere
  vtkNew<vtkSphereSource> sphereSource;
  sphereSource->SetRadius(20);
  sphereSource->SetPhiResolution(100);
  sphereSource->SetThetaResolution(100);

  // Create mapper and actor
  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<MyLODActor> lodActor;
  lodActor->SetMapper(mapper);

  // Create renderer, render window, and interactor
  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the renderer
  renderer->AddActor(lodActor);

  // Add an observer to the interactor
  renderWindowInteractor->AddObserver(
      vtkCommand::LeftButtonPressEvent,
      [lodActor](vtkObject *, unsigned long, void *) {
        lodActor->SetInteractionMode(true);
      });
  renderWindowInteractor->AddObserver(
      vtkCommand::LeftButtonReleaseEvent,
      [lodActor](vtkObject *, unsigned long, void *) {
        lodActor->SetInteractionMode(false);
      });

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();

  return 0;
}
