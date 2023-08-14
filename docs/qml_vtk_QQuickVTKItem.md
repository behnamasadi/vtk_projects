
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

[code](../vtk/main.cpp)

Refs: [1](https://vtk.org/doc/nightly/html//classQQuickVTKItem.html#a9538631cf6510414e50e8e7567300e4b)
