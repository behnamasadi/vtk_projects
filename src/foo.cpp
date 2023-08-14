#include <vtkCommand.h>
#include <vtkImageActor.h>
#include <vtkInteractorStyleImage.h>
#include <vtkJPEGReader.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>

#include <vtkAngleRepresentation2D.h>
#include <vtkAngleWidget.h>
#include <vtkBiDimensionalRepresentation2D.h>
#include <vtkBiDimensionalWidget.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceWidget.h>
#include <vtkLeaderActor2D.h>
#include <vtkProperty2D.h>

#include <vtkLineRepresentation.h>
//#include <vtkLineWidget3.h>
#include <vtkLineWidget.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkWidgetCallbackMapper.h>
#include <vtkWidgetEvent.h>
#include <vtkWidgetEventTranslator.h>

#include <vtkHandleRepresentation.h>
//#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceRepresentation2D.h>
#include <vtkPointHandleRepresentation2D.h>

#include <vtkAutoInit.h>
VTK_MODULE_INIT(vtkInteractionStyle)
VTK_MODULE_INIT(vtkRenderingFreeType)
VTK_MODULE_INIT(vtkRenderingOpenGL2)
VTK_MODULE_INIT(vtkRenderingVolumeOpenGL2)

class vtkLineCallback : public vtkCommand {
public:
  static vtkLineCallback *New() { return new vtkLineCallback; }

  virtual void Execute(vtkObject *caller, unsigned long, void *) {

    vtkDistanceWidget *lineWidget =
        reinterpret_cast<vtkDistanceWidget *>(caller);

    // Get the actual box coordinates of the line
    vtkNew<vtkPolyData> polydata;
    static_cast<vtkLineRepresentation *>(lineWidget->GetRepresentation())
        ->GetPolyData(polydata);

    // Display one of the points, just so we know it's working
    double p[3];
    polydata->GetPoint(0, p);
    std::cout << "P: " << p[0] << " " << p[1] << " " << p[2] << std::endl;
  }
  vtkLineCallback() {}
};

class vtkBiDimensionalCallback : public vtkCommand {
public:
  static vtkBiDimensionalCallback *New() {
    return new vtkBiDimensionalCallback;
  }

  virtual void Execute(vtkObject *caller, unsigned long, void *) {
    vtkBiDimensionalWidget *biDimensionalWidget =
        reinterpret_cast<vtkBiDimensionalWidget *>(caller);
    vtkBiDimensionalRepresentation2D *representation =
        static_cast<vtkBiDimensionalRepresentation2D *>(
            biDimensionalWidget->GetRepresentation());
    double p1[3];
    representation->GetPoint1DisplayPosition(p1);
    double p2[3];
    representation->GetPoint1DisplayPosition(p2);
    double p3[3];
    representation->GetPoint1DisplayPosition(p3);
    double p4[3];
    representation->GetPoint1DisplayPosition(p4);
    //显示其中一个点的屏幕坐标(px)
    std::cout << "P1: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
  }
  vtkBiDimensionalCallback() {}
};

class vtkDistanceCallback : public vtkCommand {
public:
  static vtkDistanceCallback *New() { return new vtkDistanceCallback; }

  virtual void Execute(vtkObject *caller, unsigned long eventId, void *) {
    vtkDistanceWidget *distanceWidget =
        reinterpret_cast<vtkDistanceWidget *>(caller);
    vtkDistanceRepresentation2D
        *representation = //  vtkDistanceRepresentation
                          //  vtkDistanceRepresentation3D
                          //  vtkDistanceRepresentation2D
        static_cast<vtkDistanceRepresentation2D *>(
            distanceWidget->GetDistanceRepresentation());
    double p1[3];
    representation->GetPoint1WorldPosition(p1);
    double p2[3];
    representation->GetPoint2WorldPosition(p2);

    std::cout << "P1: " << p1[0] << " " << p1[1] << " " << p1[2] << std::endl;
    std::cout << "P2: " << p2[0] << " " << p2[1] << " " << p2[2] << std::endl;

    {
      // p1[0] += 100;
      // p1[1] += 100;
      // p2[0] += 100;
      // p2[1] += 100;

      _lineRepresentation->SetPoint1WorldPosition(
          p1); // GetPoint1DisplayPosition  GetPoint1WorldPosition
      _lineRepresentation->SetPoint2WorldPosition(p2);
    }

    //  representation->Print(std::cout);

    // vtkDistanceRepresentation3D* representation =
    // vtkDistanceRepresentation3D::New();
    distanceWidget->SetRepresentation(representation);
    distanceWidget->SetPriority(0.5);
    // representation->GetLineProperty()->SetOpacity(0.0);
    // representation->GetLineProperty()->SetColor(0., 0., 0);
    // representation->GetLabelProperty()->SetOpacity(0.);
    // representation->GetLabelProperty()->SetColor(0., 0., 0);
    representation->GetAxisProperty()->SetColor(0, 0, 1);
    // representation->GetAxisProperty()->SetOpacity(1.0);

    representation->SetVisibility(false); //!

    vtkPointHandleRepresentation2D *handle =
        vtkPointHandleRepresentation2D::New();
    handle->GetProperty()->SetColor(1, 1, 0);
    representation->SetHandleRepresentation(handle);

    representation->GetPoint1Representation()->SetVisibility(false);
    representation->GetPoint2Representation()->SetVisibility(false);

    // vtkProperty* pointProperty = vtkProperty::New();
    // representation->SetHandleRepresentation(pointProperty);
    representation->GetPoint1Representation();
    // representation->GetPoint1Representation()->SetVisibility(false);

    representation->SetNumberOfRulerTicks(0);
    representation->SetRulerMode(false);

    switch (eventId) {
    case vtkCommand::StartInteractionEvent:
      // this->DistanceWidget->StartDistanceInteraction(this->HandleNumber);
      break;
    case vtkCommand::InteractionEvent:
      // this->DistanceWidget->DistanceInteraction(this->HandleNumber);

      std::cout << "InteractionEvent \n";
      break;
    case vtkCommand::EndInteractionEvent:
      //  this->DistanceWidget->EndDistanceInteraction(this->HandleNumber);
      // representation->GetPoint1Representation()->SetVisibility(false);

      std::cout << "GetVisibility1 is "
                << representation->GetPoint1Representation()->GetVisibility()
                << "\n";
      representation->GetPoint2Representation()->SetVisibility(false);

      // vtkDistanceRepresentation2D * representation1 =
      // vtkDistanceRepresentation2D::New();
      // representation1->SetVisibility(false);
      // vtkPointHandleRepresentation2D *handle =
      // vtkPointHandleRepresentation2D::New();
      // handle->GetProperty()->SetColor(1, 1, 0);
      // representation1->SetHandleRepresentation(handle);

      // if (nullptr != representation1->GetPoint1Representation())
      //{
      //    representation1->GetPoint1Representation()->SetVisibility(false);
      //}
      // if (nullptr != representation1->GetPoint2Representation())
      //{
      //    representation1->GetPoint2Representation()->SetVisibility(false);
      //}
      // distanceWidget->SetRepresentation(representation1);
      std::cout << "EndInteractionEvent \n";
      break;
    }
  }

  vtkDistanceCallback() {
    _lineRepresentation = vtkSmartPointer<vtkLineRepresentation>::New();
  }

  vtkSmartPointer<vtkLineRepresentation> GetLineRepresentation() {
    return _lineRepresentation;
  }

private:
  vtkSmartPointer<vtkLineRepresentation> _lineRepresentation;
};

int main() {
  int WidgetType = 0;
  std::cout << "Please select the Measurement Distance WidgetType: "
            << std::endl;
  std::cout << "0 vtkDistanceWidget \n";
  std::cout << "1 vtkAngleWidget \n";
  std::cout << "2 vtkBiDimensionalWidget \n";
  // std::cin >> WidgetType;

  vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
  reader->SetFileName("cake_easy.jpg");
  reader->Update();

  vtkSmartPointer<vtkImageActor> imgActor =
      vtkSmartPointer<vtkImageActor>::New();
  imgActor->SetInputData(reader->GetOutput());

  vtkSmartPointer<vtkRenderer> render = vtkSmartPointer<vtkRenderer>::New();
  render->AddActor(imgActor);
  render->SetBackground(0, 0, 0);
  render->ResetCamera();

  vtkSmartPointer<vtkRenderWindow> rw = vtkSmartPointer<vtkRenderWindow>::New();
  rw->AddRenderer(render);
  rw->SetWindowName("MeasurementDistanceApp");
  rw->SetSize(800, 600);
  rw->Render();

  vtkSmartPointer<vtkRenderWindowInteractor> rwi =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  rwi->SetRenderWindow(rw);

  vtkSmartPointer<vtkInteractorStyleImage> style =
      vtkSmartPointer<vtkInteractorStyleImage>::New();
  rwi->SetInteractorStyle(style);
  /****************************************************************/

  //实例化Widget

  vtkSmartPointer<vtkDistanceWidget> distanceWidget =
      vtkSmartPointer<vtkDistanceWidget>::New();
  //指定渲染窗口交互器,来监听用户事件
  distanceWidget->SetInteractor(rwi);
  //必要时使用观察者/命令模式创建回调函数(此处没用)
  //创建几何表达实体。用SetRepresentation()把事件与Widget关联起来
  //或者使用Widget默认的几何表达实体
  distanceWidget->CreateDefaultRepresentation();
  static_cast<vtkDistanceRepresentation *>(
      distanceWidget->GetDistanceRepresentation())
      ->SetLabelFormat("%-#6.3g mm");
  // auto DistanceRep = static_cast<vtkDistanceRepresentation*>
  // (distanceWidget->GetDistanceRepresentation());
  // DistanceRep->SetDirectionalLine(true);
  // DistanceRep->SetNumberOfRulerTicks(0);
  // vtkHandleRepresentation* pRep = DistanceRep->GetPoint1Representation();

  {
    vtkDistanceRepresentation2D *representation =
        vtkDistanceRepresentation2D::New();
    distanceWidget->SetRepresentation(representation);
    distanceWidget->SetPriority(0.9); // 0.5 ==>0.9
    // representation->GetLineProperty()->SetOpacity(0.0);
    // representation->GetLineProperty()->SetColor(0.,0.,0);
    // representation->GetLabelProperty()->SetOpacity(0.);
    // representation->GetLabelProperty()->SetColor(0., 0., 0);

    representation->SetVisibility(false);

    // Create the widget and its representation
    // vtkPointHandleRepresentation2D *handle =
    // vtkPointHandleRepresentation2D::New(); handle->GetProperty()->SetColor(1,
    // 0, 0); representation->SetHandleRepresentation(handle);

    // vtkProperty* pointProperty = vtkProperty::New();
    // representation->SetHandleRepresentation(pointProperty);
    // representation->GetPoint1Representation()->SetVisibility(false);
    // representation->GetPoint2Representation()->SetVisibility(false);

    representation->SetNumberOfRulerTicks(0);
    representation->SetRulerMode(false);
  }

  vtkSmartPointer<vtkDistanceCallback> bidiCallback =
      vtkSmartPointer<vtkDistanceCallback>::New();
  distanceWidget->AddObserver(vtkCommand::InteractionEvent, bidiCallback);
  distanceWidget->AddObserver(vtkCommand::EndInteractionEvent, bidiCallback);
  distanceWidget->AddObserver(vtkCommand::StartInteractionEvent, bidiCallback);
  distanceWidget->AddObserver(vtkCommand::MouseMoveEvent, bidiCallback);

  distanceWidget->On();

  vtkNew<vtkDistanceWidget> lineWidget;
  lineWidget->SetInteractor(rwi);
  lineWidget->CreateDefaultRepresentation();

  // You could do this if you want to set properties at this point:
  // vtkNew<vtkLineRepresentation> lineRepresentation;
  auto lineRepresentation = bidiCallback->GetLineRepresentation();
  lineRepresentation->SetDistanceAnnotationVisibility(true);
  lineRepresentation->GetDistanceAnnotationProperty()->SetColor(0, 1, 0);
  lineRepresentation->GetDistanceAnnotationProperty()->SetBackfaceCulling(true);
  lineRepresentation->SetDistanceAnnotationScale(20.0, 20.0, 100.0);
  lineRepresentation->SetDistanceAnnotationFormat("%-#6.2f mm");
  // lineRepresentation->SetDistanceAnnotationFormat("%");
  lineRepresentation->SetLineColor(1, 0, 0);
  lineRepresentation->SetDirectionalLine(true);

  lineRepresentation->GetEndPointProperty()->SetColor(1, 0, 0);
  // lineRepresentation->GetEndPointProperty()->SetLineWidth(.5);
  lineRepresentation->GetEndPointProperty()->SetOpacity(0.1);

  // lineRepresentation->GetEndPoint2Property()->SetLineWidth(10.);
  lineRepresentation->GetEndPoint2Property()->SetPointSize(20.0);

  lineRepresentation->GetEndPoint2Property()->SetColor(1, 0, 0);

  // lineWidget->SetRepresentation(lineRepresentation);
  lineWidget->CreateDefaultRepresentation();
  lineWidget->SetEnabled(0);

  // vtkNew<vtkLineCallback> lineCallback;
  // lineWidget->AddObserver(vtkCommand::InteractionEvent, lineCallback);
  // lineWidget->AddObserver(vtkCommand::PlacePointEvent, lineCallback);

  double p1[3] = {-10000, -10000, 0};
  lineRepresentation->SetPoint1WorldPosition(
      p1); // GetPoint1DisplayPosition  GetPoint1WorldPosition
  lineRepresentation->SetPoint2WorldPosition(p1);

  lineRepresentation->GetPoint1WorldPosition(
      p1); // GetPoint1DisplayPosition  GetPoint1WorldPosition
  std::cout << "p1:\t" << p1[0] << "\t" << p1[1] << "\t" << p1[2] << "\n";
  lineRepresentation->GetPoint2WorldPosition(p1);
  std::cout << "p2:\t" << p1[0] << "\t" << p1[1] << "\t" << p1[2] << "\n";
  lineWidget->On();

  rw->Render();
  rwi->Initialize();
  rwi->Start();
}
