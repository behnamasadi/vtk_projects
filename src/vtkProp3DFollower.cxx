//
#include "vtkProp3DFollower.h"
#include "vtkCellPicker.h"
#include "vtkColorTransferFunction.h"
#include "vtkCommand.h"
#include "vtkFollower.h"
#include "vtkImageActor.h"
#include "vtkImageMapper3D.h"
#include "vtkInteractorEventRecorder.h"
#include "vtkJPEGReader.h"
#include "vtkPNGReader.h"
#include "vtkPiecewiseFunction.h"
#include "vtkPlaneSource.h"
#include "vtkPolyDataMapper.h"
#include "vtkPropPicker.h"
#include "vtkProperty.h"
#include "vtkRegressionTestImage.h"
#include "vtkRenderWindow.h"
#include "vtkRenderWindowInteractor.h"
#include "vtkRenderer.h"
#include "vtkSmartPointer.h"
#include "vtkSphereSource.h"
#include "vtkStructuredPointsReader.h"
#include "vtkTestUtilities.h"
#include <vtkActor.h>
#include <vtkBoxWidget.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkTransform.h>

//------------------------------------------------------------------------------
// This does the actual work: updates the vtkPline implicit function.
// This in turn causes the pipeline to update and clip the object.
// Callback for the interaction
class vtkPickFollowerCallback : public vtkCommand {
public:
  static vtkPickFollowerCallback *New() { return new vtkPickFollowerCallback; }
  void Execute(vtkObject *caller, unsigned long, void *) override {
    //      vtkPropPicker *picker = reinterpret_cast<vtkPropPicker*>(caller);
    vtkCellPicker *picker = reinterpret_cast<vtkCellPicker *>(caller);
    if (picker->GetViewProp() != nullptr) {
      cout << "Picked\n";
    }
  }

  vtkPickFollowerCallback() = default;
};

int main(int argc, char **argv) {

  vtkNew<vtkNamedColors> colors;
  // Create some simple actors
  // vtkNew<>
  vtkNew<vtkPlaneSource> plane;

  vtkNew<vtkPolyDataMapper> mapper;

  mapper->SetInputConnection(plane->GetOutputPort());

  vtkNew<vtkFollower> follower;

  follower->SetMapper(mapper);
  follower->SetPosition(1, 2, 3);

  // Mark the origin
  vtkNew<vtkSphereSource> ss;
  vtkNew<vtkPolyDataMapper> m;
  m->SetInputConnection(ss->GetOutputPort());
  vtkNew<vtkActor> a;
  a->SetMapper(m);

  // vtkNew<vtkJPEGReader> pnmReader;
  // pnmReader->SetFileName("maps-marker.png");
  vtkNew<vtkPNGReader> pnmReader;
  pnmReader->SetFileName("../images/maps-marker.png");

  vtkNew<vtkImageActor> ia;
  ia->GetMapper()->SetInputConnection(pnmReader->GetOutputPort());
  ia->SetScale(0.01, 0.01, 0.01);

  vtkNew<vtkProp3DFollower> p3dFollower;

  p3dFollower->SetProp3D(ia);

  // Debugging code
  vtkNew<vtkPlaneSource> plane2;

  vtkNew<vtkPolyDataMapper> mapper2;
  mapper2->SetInputConnection(plane2->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper2);
  actor->AddPosition(1, 2, 3);
  actor->GetProperty()->SetRepresentationToWireframe();
  actor->GetProperty()->SetColor(0, 1, 0);

  // Picking callback
  vtkNew<vtkPickFollowerCallback> myCallback;

  //  VTK_CREATE(vtkPropPicker,picker);
  vtkNew<vtkCellPicker> picker;
  picker->AddObserver(vtkCommand::EndPickEvent, myCallback);

  // Create the rendering machinery
  //
  vtkNew<vtkRenderer> ren1;
  follower->SetCamera(ren1->GetActiveCamera());
  p3dFollower->SetCamera(ren1->GetActiveCamera());

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(ren1);
  // Turn off antialiasing so all GPUs produce the same image
  renWin->SetMultiSamples(0);

  vtkNew<vtkRenderWindowInteractor> iren;
  iren->SetRenderWindow(renWin);
  iren->SetPicker(picker);

  ren1->AddActor(p3dFollower);

  ren1->SetBackground(0.1, 0.2, 0.4);
  renWin->SetSize(300, 300);
  ren1->ResetCamera();
  iren->Initialize();
  renWin->Render();
  iren->Start();

  return 0;
}
