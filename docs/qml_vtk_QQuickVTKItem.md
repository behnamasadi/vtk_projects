
# QQuickVTKItem

First define a class that inherits from `QQuickVTKItem`

```cpp
struct MyVtkItem : public QQuickVTKItem {
  Q_OBJECT
};
```

Inside this class, create a class that inherits from `vtkObject` and define your actors, mapper, etc

```cpp
public:
  struct Data : vtkObject {
    static Data *New();
    vtkTypeMacro(Data, vtkObject);

    vtkNew<vtkConeSource> cone;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
  };
```

Here In the `initializeVTK` initialize your objects:

```
vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override 
{
 vtkNew<Data> vtk;

 vtk->actor->SetMapper(vtk->mapper);
 vtk->mapper->SetInputConnection(vtk->cone->GetOutputPort());

 vtk->renderer->AddActor(vtk->actor);
 vtk->renderer->ResetCamera();
 vtk->renderer->SetBackground(0.0, 1.0, 1.0);
 vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
 vtk->renderer->SetGradientBackground(true);

 renderWindow->AddRenderer(vtk->renderer);
 renderWindow->SetMultiSamples(16);


 vtkSmartPointer<CameraInteractorStyle> style =
 vtkSmartPointer<CameraInteractorStyle>::New();

 vtkNew<vtkRenderWindowInteractor> iRen;
 vtk->renderer->GetRenderWindow()->GetInteractor()->SetInteractorStyle(
 style);
 style->SetDefaultRenderer(vtk->renderer);
 return vtk;
}
```

All VTK objects are owned by and run on the QML render thread, This means you can only touch VTK state:

1. `initializeVTK()`
2. `destroyingVTK ()`
3. `dispatch_async()`

To update your actors from GUI

```cpp
  Q_INVOKABLE void resetCamera() {
    dispatch_async([this](vtkRenderWindow *renderWindow, vtkUserData userData) {
      auto *vtk = Data::SafeDownCast(userData);

      double s;
      s = 1.1;
      std::cout << "Scale: " << vtk->actor->GetScale()[0] << ","
                << vtk->actor->GetScale()[1] << "," << vtk->actor->GetScale()[2]
                << std::endl;
      vtk->actor->SetScale(vtk->actor->GetScale()[0] * s);

      // std::cout << vtk->actor->GetUserTransform() << std::endl;
      // vtk->renderer->ResetCamera();

      scheduleRender();
    });
  }
};
```

Now in the QML side:

```
import com.vtk.example 1.0

MyVtkItem 
{
 id: vtk
 anchors.fill: parent
}

Button
{
 text: "click me"
 onClicked: 
 {

     vtk.resetCamera()
 }
}    
```

In your main:

```cpp

qmlRegisterType<MyVtkItem>("com.vtk.example", 1, 0, "MyVtkItem");

QQmlApplicationEngine engine;

engine.addImportPath("/home/behnam/usr/lib/qml");
engine.load(QUrl(QStringLiteral("qrc:/qml/main.qml")));
```

[code](../src/main.cpp)

Refs: [1](https://vtk.org/doc/nightly/html//classQQuickVTKItem.html#a9538631cf6510414e50e8e7567300e4b)


`scheduleRender()` is a method provided by `QQuickVtkItem` to request a render pass in the VTK pipeline.

You should use `scheduleRender()` in situations where:

1. **The underlying VTK data has changed:** If you've updated or changed the VTK dataset that is being visualized, you'll need to request a new render to visualize these changes in the Qt Quick item.
 
2. **Camera or view changes:** If there are changes in the viewpoint or camera settings and you wish to update the view, you'd call this method.

3. **Visual attributes change:** Any changes in visualization attributes such as color maps, contour levels, opacity, etc., will require a re-render to reflect the changes in the visualization.

4. **Interactivity:** If your application has any kind of interactivity related to the VTK visualization, such as selecting points, drawing regions, or manipulating objects in the VTK scene, you might need to call `scheduleRender()` to update the view based on user input.

5. **Animations:** If you're creating any kind of animation or time-evolving visualization using VTK in your Qt Quick application, you'd schedule renders at each frame or time step.

It's essential to be judicious about when to call `scheduleRender()` because unnecessary renders can be computationally expensive and might degrade the performance of your application. Always try to optimize and call it only when necessary.

Ensure you're familiar with the rest of the VTK pipeline and the Qt Quick rendering loop to integrate the two systems most efficiently.
