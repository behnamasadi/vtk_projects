
In your main:

```cpp
  QObject *topLevel = engine.rootObjects().value(0);

  // Fetch the QQuick window using the standard object name set up in the constructor
  QQuickVTKRenderItem *qquickvtkItem =
      topLevel->findChild<QQuickVTKRenderItem *>("View");


  vtkNew<vtkPolyDataMapper> mapper;

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  
  qquickvtkItem->renderer()->AddActor(actor);
  qquickvtkItem->renderer()->ResetCamera();
  qquickvtkItem->renderer()->SetBackground(0.5, 0.5, 0.7);
  qquickvtkItem->update();
```


In your QML

```
import VTK 9.2

  // Instantiate the vtk render window
  VTKRenderWindow {
    id: vtkwindow
    width: 600
    height: 600
  }
 
  // add one or more vtk render items
  VTKRenderItem {
    objectName: "View"
    x: 0
    y: 0
    width: 800
    height: 800
    // Provide the handle to the render window
    renderWindow: vtkwindow
  }
```


[code](../vtk/qml_pcl.cpp)

Refs: [1](https://vtk.org/doc/nightly/html/classQQuickVTKRenderItem.html)
  
