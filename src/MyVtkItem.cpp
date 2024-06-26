//#include "MyVtkItem.hpp"

struct MyVtkItem : QQuickVTKItem {

  Q_OBJECT

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {
    // Create a cone pipeline and add it to the view
    vtkNew<vtkConeSource> cone;

    vtkNew<vtkPolyDataMapper> mapper;
    mapper->SetInputConnection(cone->GetOutputPort());

    vtkNew<vtkActor> actor;
    actor->SetMapper(mapper);

    vtkNew<vtkRenderer> renderer;
    renderer->AddActor(actor);
    renderer->ResetCamera();
    renderer->SetBackground(0.0, 1.0, 1.0);
    renderer->SetBackground2(1.0, 0.0, 0.0);
    renderer->SetGradientBackground(true);

    renderWindow->AddRenderer(renderer);
    renderWindow->SetMultiSamples(16);

    vtkSmartPointer<vtkInteractorStyleTrackballCamera> style =
        vtkSmartPointer<vtkInteractorStyleTrackballCamera>::New();

    // vtkSmartPointer<vtkInteractorStyleRubberBandZoom> style =
    //     vtkSmartPointer<vtkInteractorStyleRubberBandZoom>::New();

    // vtkSmartPointer<vtkInteractorStyleTerrain> style =
    //     vtkSmartPointer<vtkInteractorStyleTerrain>::New();

    // vtkSmartPointer<vtkInteractorStyleJoystickCamera> style =
    //     vtkSmartPointer<vtkInteractorStyleJoystickCamera>::New();

    // vtkSmartPointer<CameraInteractorStyle> style =
    //     vtkSmartPointer<CameraInteractorStyle>::New();

    vtkNew<vtkRenderWindowInteractor> iRen;
    renderer->GetRenderWindow()->GetInteractor()->SetInteractorStyle(style);
    style->SetDefaultRenderer(renderer);

    this->setAcceptHoverEvents(true);

    this->forceActiveFocus();
    renderWindow->SetBorders(true);

    this->update();

    return nullptr;
  }
  void hoverEnterEvent(QHoverEvent *event) override {
    std::cout << "----" << std::endl;
  }

  void mouseDoubleClickEvent(QMouseEvent *event) override {
    std::cout << "----" << std::endl;
  }

  // Assuming you have a class named VTKBackend
  void MyVtkItem::handleMouseClick(int x, int y) {
    if (!this->vtkRenderWindowInteractor) {
      // Ensure the render window interactor is available
      std::cout << "00000000000000000000000000000000" << std::endl;
      return;
    }

    // Convert the QML mouse coordinates if needed

    // Send a mouse press event
    std::cout << "1111111111111111111111111111111111111" << std::endl;
    this->vtkRenderWindowInteractor->InvokeEvent(
        vtkCommand::LeftButtonPressEvent, nullptr);

    std::cout << "222222222222222222222222222" << std::endl;

    // ... Handle other mouse events if needed ...

    // Then, trigger a mouse release
    this->vtkRenderWindowInteractor->InvokeEvent(
        vtkCommand::LeftButtonReleaseEvent, nullptr);

    std::cout << "33333333333333333333333333" << std::endl;
  }
};
