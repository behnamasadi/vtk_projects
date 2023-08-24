// VTKBackend.h

#include <QObject>
#include <vtkCommand.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

class VTKBackend : public QQuickVTKItem {
  Q_OBJECT

public:
  VTKBackend();

  Q_INVOKABLE void handleMouseClick(int x, int y);

private:
  vtkSmartPointer<vtkRenderWindowInteractor> m_vtkRenderWindowInteractor;
};

//
