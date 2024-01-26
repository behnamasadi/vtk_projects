// CustomVTKItem.h
#include <QQuickItem>
#include <vtkGenericOpenGLRenderWindow.h>
#include <vtkNew.h>
#include <vtkRenderer.h>

class CustomVTKItem : public QQuickItem {
  Q_OBJECT
public:
  CustomVTKItem();

protected:
  void setupVTK();
  QSGNode *updatePaintNode(QSGNode *, UpdatePaintNodeData *);

private:
  vtkNew<vtkGenericOpenGLRenderWindow> renderWindow;
  vtkNew<vtkRenderer> renderer;
};
