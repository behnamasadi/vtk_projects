/*
#include <QtQml/QQmlApplicationEngine>

#include <QtQuick/QQuickWindow>

#include <QtGui/QGuiApplication>
#include <QtGui/QSurfaceFormat>

#include <QQuickVTKItem.h>
#include <QVTKRenderWindowAdapter.h>

#include <vtkActor.h>
#include <vtkConeSource.h>
#include <vtkPolyDataMapper.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>

#include <vtkInteractorStyleJoystickCamera.h>
#include <vtkInteractorStyleRubberBandZoom.h>
#include <vtkInteractorStyleTerrain.h>

#include <QVTKInteractor.h>
#include <algorithm>
#include <pcl/io/vtk_lib_io.h>
#include <typeinfo>
#include <vtkAngleWidget.h>
#include <vtkCallbackCommand.h>
#include <vtkCamera.h>
#include <vtkCommand.h>
#include <vtkDistanceRepresentation.h>
#include <vtkDistanceRepresentation3D.h>
#include <vtkDistanceWidget.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkObjectFactory.h>
#include <vtkPointHandleRepresentation3D.h>
#include <vtkPointPicker.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkRendererCollection.h>
#include <vtkTextProperty.h>
#include <vtkTimeStamp.h>
#include <vtkTransform.h>
#include <vtkWidgetEvent.h>

#include "CameraInteractorStyle.hpp"
#include "InteractorStyleSwitch.hpp"

struct MyVtkItem : public QQuickVTKItem {

  Q_OBJECT
public:
  struct Data : vtkObject {

    void static func(vtkObject *caller, unsigned long eid, void *clientdata,
                     void *calldata) {

      std::cout << "--------------------" << std::endl;
      double p1[3], p2[3], center[3];

      auto angleWidget = reinterpret_cast<vtkAngleWidget *>(caller);

      static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
          ->GetPoint1DisplayPosition(p1);

      static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
          ->GetPoint2DisplayPosition(p2);

      static_cast<vtkAngleRepresentation3D *>(angleWidget->GetRepresentation())
          ->GetCenterDisplayPosition(center);

      std::cout << "p1: " << p1[0] << ", " << p1[1] << ", " << p1[2]
                << std::endl;
      std::cout << "p2: " << p2[0] << ", " << p2[1] << ", " << p2[2]
                << std::endl;
      std::cout << "center: " << center[0] << ", " << center[1] << ", "
                << center[2] << std::endl;

      vtkNew<vtkPointPicker> pointPicker;
      angleWidget->GetInteractor()->SetPicker(pointPicker);

      if (pointPicker->Pick(p1, angleWidget->GetCurrentRenderer())) {
        double data[3];
        pointPicker->GetPickPosition(data);
        std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
                  << std::endl;

        static_cast<vtkAngleRepresentation3D *>(
            angleWidget->GetRepresentation())
            ->GetPoint1Representation()
            ->SetWorldPosition(data);
      }

      if (pointPicker->Pick(p2, angleWidget->GetCurrentRenderer())) {
        double data[3];
        pointPicker->GetPickPosition(data);
        std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
                  << std::endl;

        static_cast<vtkAngleRepresentation3D *>(
            angleWidget->GetRepresentation())
            ->GetPoint2Representation()
            ->SetWorldPosition(data);
      }

      if (pointPicker->Pick(center, angleWidget->GetCurrentRenderer())) {
        double data[3];
        pointPicker->GetPickPosition(data);
        std::cout << "point: " << data[0] << ", " << data[1] << ", " << data[2]
                  << std::endl;

        static_cast<vtkAngleRepresentation3D *>(
            angleWidget->GetRepresentation())
            ->GetCenterRepresentation()
            ->SetWorldPosition(data);
      }
    }

    static Data *New();
    vtkTypeMacro(Data, vtkObject);

    vtkNew<vtkConeSource> cone;
    vtkNew<vtkPolyDataMapper> mapper;
    vtkNew<vtkActor> actor;
    vtkNew<vtkRenderer> renderer;
    vtkNew<QVTKInteractor> iRen;

    // AngleWidget
    vtkNew<vtkAngleWidget> angleWidget;
  };

  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override {

    // Adding a cube
    vtkNew<vtkPolyData> polydata;

    // pcl::PointCloud<pcl::PointXYZ> cloud;

    // std::vector<pcl::PointXYZ> points = {{0, 0, 0}, {0, 0, 1}, {0, 1, 0},
    //                                      {1, 1, 0}, {1, 0, 0}, {1, 1, 1}};
    // for (auto const &point : points) {
    //   cloud.push_back(point);
    // }

    pcl::PointCloud<pcl::PointXYZ> cloud;
    pcl::PointXYZ point;

    for (int i = 0; i < 10000000; i++) {

      point.x = rand() * 10 - 5;
      point.y = rand() * 10 - 5;
      point.z = rand() * 10 - 5;
      cloud.points.push_back(point);
    }

    vtkNew<Data> vtk;
    vtkNew<vtkInteractorStyleTrackballCamera> style;

    pcl::io::pointCloudTovtkPolyData(cloud, polydata);

    vtkNew<vtkPolyDataMapper> pointcloudMapper;
    pointcloudMapper->SetInputData(polydata);

    vtkNew<vtkActor> cloudActor;
    cloudActor->SetMapper(pointcloudMapper);

    style->SetCurrentRenderer(vtk->renderer);
    style->SetInteractor(vtk->iRen);

    vtk->actor->SetMapper(vtk->mapper);
    vtk->mapper->SetInputConnection(vtk->cone->GetOutputPort());

    //////////////////////////////////////////
    // renderer
    vtk->renderer->AddActor(cloudActor);
    cloudActor->GetProperty()->SetPointSize(12);

    // DistanceWidget
    vtkNew<vtkPointHandleRepresentation3D> handled;
    vtkNew<vtkDistanceRepresentation3D> repd;
    repd->SetHandleRepresentation(handled);
    vtkNew<vtkDistanceWidget> widget;
    widget->SetInteractor(vtk->iRen);
    widget->SetRepresentation(repd);
    widget->SetWidgetStateToManipulate();
    widget->EnabledOn();
    widget->ProcessEventsOn();

    vtkNew<vtkCallbackCommand> callbackd;
    callbackd->SetCallback(Data::func);
    widget->AddObserver(vtkCommand::EndInteractionEvent, callbackd);

    widget->On();

    /////////////////////////////////////////////

    vtk->renderer->AddActor(vtk->actor);
    vtk->renderer->ResetCamera();
    vtk->renderer->SetBackground(0.0, 1.0, 0.0);
    vtk->renderer->SetBackground2(1.0, 0.0, 0.0);
    vtk->renderer->SetGradientBackground(true);

    // vtkNew<InteractorStyleSwitch> style;

    vtk->angleWidget->SetInteractor(vtk->iRen);
    vtk->angleWidget->CreateDefaultRepresentation();

    vtk->angleWidget->On();

    renderWindow->AddRenderer(vtk->renderer);
    renderWindow->SetMultiSamples(16);

    vtk->iRen->SetInteractorStyle(style);

    // Callback
    vtkNew<vtkCallbackCommand> placeAnglePointCallback;
    placeAnglePointCallback->SetCallback(Data::func);
    vtk->angleWidget->AddObserver(vtkCommand::EndInteractionEvent,
                                  placeAnglePointCallback);

    vtk->angleWidget->SetWidgetStateToManipulate();
    vtk->angleWidget->EnabledOn();
    vtk->angleWidget->ProcessEventsOn();

    // Representation3D
    vtkNew<vtkPointHandleRepresentation3D> handle;
    vtkNew<vtkAngleRepresentation3D> rep;

    rep->SetHandleRepresentation(handle);
    vtk->angleWidget->SetRepresentation(rep);

    renderWindow->SetInteractor(vtk->iRen);

    return vtk;
  }
};
*/

#ifndef MYVTKITEM_HPP
#define MYVTKITEM_HPP

#include "QQuickVtkItem.h"
#include <QLoggingCategory>
#include <QQuickWindow>
#include <QVTKRenderWindowAdapter.h>
#include <QtGui/QGuiApplication>
#include <QtGui/QSurfaceFormat>
#include <QtQml/QQmlApplicationEngine>
#include <QtQuick/QQuickWindow>
#include <pdal/Options.hpp>
#include <pdal/PointTable.hpp>
#include <pdal/PointView.hpp>
#include <pdal/StageFactory.hpp>
#include <pdal/io/LasReader.hpp>
#include <vtkActor.h>
#include <vtkCallbackCommand.h>
#include <vtkCommand.h>
#include <vtkConeSource.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkMinimalStandardRandomSequence.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkPropPicker.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSliderRepresentation2D.h>
#include <vtkSliderWidget.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkVertexGlyphFilter.h>

class MyVtkItem : public QQuickVtkItem {
public:
  vtkUserData initializeVTK(vtkRenderWindow *renderWindow) override;
};

#endif // MYVTKITEM_HPP
