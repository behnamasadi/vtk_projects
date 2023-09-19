#include <vtkActor.h>
#include <vtkActorCollection.h>
#include <vtkAppendFilter.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkDataSetMapper.h>
#include <vtkInteractorStyle.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNew.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSphereSource.h>
#include <vtkTransform.h>
#include <vtkUnstructuredGrid.h>
int main() {

  vtkNew<vtkSphereSource> sphereSource1;
  sphereSource1->SetCenter(0, 0, 0);
  sphereSource1->Update();

  vtkNew<vtkSphereSource> sphereSource2;
  sphereSource2->SetCenter(2, 0, 0);
  sphereSource2->Update();

  vtkNew<vtkAppendFilter> appendFilter;
  appendFilter->AddInputData(sphereSource1->GetOutput());
  appendFilter->AddInputData(sphereSource2->GetOutput());
  appendFilter->Update();

  auto combined = appendFilter->GetOutput();
  std::cout << "There are " << combined->GetNumberOfPoints()
            << " points combined." << std::endl;

  vtkNew<vtkDataSetMapper> mapper;
  mapper->SetInputConnection(appendFilter->GetOutputPort());

  vtkNew<vtkRenderer> ren;

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  ren->AddActor(actor);

  vtkNew<vtkRenderWindow> renWin;
  renWin->AddRenderer(ren);

  vtkNew<vtkRenderWindowInteractor> i_Ren;
  i_Ren->SetRenderWindow(renWin);

  vtkNew<vtkInteractorStyleTrackballActor> style;
  i_Ren->SetInteractorStyle(style);

  i_Ren->Start();
}
