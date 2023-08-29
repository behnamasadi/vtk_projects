// VTKBackend.h

#include <QObject>
#include <QQuickVTKItem.h>
#include <QVTKInteractor.h>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMatrix4x4.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

class VTKBackend : public QQuickVTKItem {
  Q_OBJECT

public:
  struct Data : vtkObject {
    static Data *New();
    vtkTypeMacro(Data, vtkObject);

    vtkNew<vtkConeSource> cone;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
    //    vtkNew<QVTKInteractor> iRen;
    // vtkNew<vtkRenderer> iRen;
  };

  vtkNew<Data> vtk;
  // vtkNew<QVTKInteractor> iRen;
  QVTKInteractor *iRen;
  //  vtkNew<vtkRenderWindowInteractor> m_vtkRenderWindowInteractor;

  VTKBackend();

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override;

  Q_INVOKABLE void resetCamera() {
    dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
      Data *vtk = Data::SafeDownCast(userData);

      double s;
      s = 1.1;
      vtk->actor->SetScale(vtk->actor->GetScale()[0] * s);

      std::cout << "Scale: " << vtk->actor->GetScale()[0] << ","
                << vtk->actor->GetScale()[1] << "," << vtk->actor->GetScale()[2]
                << std::endl;

      vtkMatrix4x4 *m = vtk->actor->GetMatrix();

      std::cout << "actor matrix: \n";
      m->Print(std::cout);
      scheduleRender();
    });
  }
};

//
