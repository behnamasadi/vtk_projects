#include "VTKBackend.hpp"
#include <typeinfo>

VTKBackend::VTKBackend() {}

void placePoint(vtkObject *caller, unsigned long eid, void *clientData,
                void *calldata) {
  double p1[3], p2[3];

  auto callerObject = reinterpret_cast<VTKBackend *>(clientData);
  auto distanceWidget = reinterpret_cast<vtkDistanceWidget *>(caller);

  vtkSmartPointer<vtkDistanceRepresentation3D> rep;

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint1DisplayPosition(p1);

  static_cast<vtkDistanceRepresentation3D *>(
      distanceWidget->GetRepresentation())
      ->GetPoint2DisplayPosition(p2);

  std::cout << "placePoint p1: " << p1[0] << ", " << p1[1] << ", " << p1[2]
            << std::endl;
  std::cout << "placePoint p2: " << p2[0] << ", " << p2[1] << ", " << p2[2]
            << std::endl;

  // distanceWidget->GetInteractor()->GetLastEventPosition();

  vtkNew<vtkPointPicker> pointPicker;

  vtkRenderWindowInteractor *rwi = distanceWidget->GetInteractor();

  QVTKInteractor *qvtkInteractor = dynamic_cast<QVTKInteractor *>(rwi);

  std::cout << "qvtkInteractor " << typeid(qvtkInteractor).name() << "\n";
  std::cout << "qvtkInteractor " << qvtkInteractor << "\n";

  // callerObject->vtk->iRen->SetPicker(pointPicker);
  callerObject->iRen->SetPicker(pointPicker);

  // std::cout << "callerObject->vtk->iRen " << callerObject->vtk->iRen << "\n";
  std::cout << "callerObject->iRen " << callerObject->iRen << "\n";

  // callerObject->vtk->iRen->Print(std::cout);
  callerObject->iRen->Print(std::cout);

  // EventPosition: ( 420, 9 )
  //  LastEventPosition: ( 13, 11 )

  double data1[3];
  if (pointPicker->Pick(p1, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data1);
    std::cout << "point: " << data1[0] << ", " << data1[1] << ", " << data1[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint1Representation()
        ->SetWorldPosition(data1);
  }

  double data2[3];
  if (pointPicker->Pick(p2, distanceWidget->GetCurrentRenderer())) {

    pointPicker->GetPickPosition(data2);
    std::cout << "point: " << data2[0] << ", " << data2[1] << ", " << data2[2]
              << std::endl;

    static_cast<vtkDistanceRepresentation3D *>(
        distanceWidget->GetRepresentation())
        ->GetPoint2Representation()
        ->SetWorldPosition(data2);
  }

  double sumSquared = std::pow(data1[0] - data2[0], 2) +
                      std::pow(data1[1] - data2[1], 2) +
                      std::pow(data1[2] - data2[2], 2);
  double distance = std::pow(sumSquared, 0.5);
  std::cout << "distance: " << distance << std::endl;
}

VTKBackend::vtkUserData
VTKBackend::initializeVTK(vtkRenderWindow *renderWindow) {

  iRen = QVTKInteractor::New();
  vtk->actor->SetMapper(vtk->mapper);
  vtk->mapper->SetInputConnection(vtk->cone->GetOutputPort());

  vtk->renderer->AddActor(vtk->actor);
  vtk->renderer->ResetCamera();
  vtk->renderer->SetBackground(0.0, 1.0, 1.0);
  vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
  vtk->renderer->SetGradientBackground(true);

  renderWindow->AddRenderer(vtk->renderer);
  renderWindow->SetMultiSamples(16);

  vtkNew<vtkInteractorStyleTrackballCamera> style;
  style->SetCurrentRenderer(vtk->renderer);

  renderWindow->AddRenderer(vtk->renderer);
  // vtk->iRen->SetInteractorStyle(style);
  this->iRen->SetInteractorStyle(style);

  // renderWindow->SetInteractor(vtk->iRen);
  renderWindow->SetInteractor(this->iRen);

  ///////////////////////////////////////////////////

  // vtkNew<vtkDistanceWidget> distanceWidget;

  vtkDistanceWidget *distanceWidget = vtkDistanceWidget::New();

  // std::cout << "vtk->iRen typeid: " << typeid(vtk->iRen).name() << "\n";
  // std::cout << "vtk->iRen: " << vtk->iRen << "\n";

  // distanceWidget->SetInteractor(vtk->iRen);

  distanceWidget->SetInteractor(this->iRen);

  std::cout << "distanceWidget->GetInteractor(): "
            << distanceWidget->GetInteractor() << "\n";

  vtkNew<vtkCallbackCommand> placePointCallback;
  placePointCallback->SetCallback(placePoint);

  distanceWidget->AddObserver(vtkCommand::EndInteractionEvent,
                              placePointCallback);

  placePointCallback->SetClientData((void *)this);

  vtkNew<vtkPointHandleRepresentation3D> handle;

  vtkNew<vtkDistanceRepresentation3D> rep;

  rep->SetHandleRepresentation(handle);
  distanceWidget->SetRepresentation(rep);
  rep->SetMaximumNumberOfRulerTicks(2);

  this->iRen->Start();

  distanceWidget->On();

  //////////////////////////////////////////////////

  return vtk;
}
