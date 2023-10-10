cmake_minimum_required(VERSION 3.15 FATAL_ERROR)
project(vtk_projects)

if(NOT "${CMAKE_GENERATOR}" STREQUAL "Ninja Multi-Config")
	# message(WARNING "It's recommended to use -G \"Ninja Multi-Config\" for this project.")
	# OR, if you want to force-stop the configuration:
	message(FATAL_ERROR "Please use cmake -S . -G \"Ninja Multi-Config\" -B build for this project.")
endif()

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")

find_package(Boost COMPONENTS filesystem system REQUIRED)

set(VTK_DIR "$ENV{HOME}/usr/lib/cmake/vtk-9.2/")
message("VTK_DIR:" ${VTK_DIR})

find_package(VTK REQUIRED)
message("VTK_VERSION:" ${VTK_VERSION})
message("VTK_LIBRARIES:" ${VTK_LIBRARIES})

set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

find_package(QT NAMES Qt6 Qt5 REQUIRED COMPONENTS Qml Quick Widgets Gui QuickControls2 Xml Sql PrintSupport Svg Network Concurrent)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Qml Quick Widgets Gui QuickControls2 Xml Sql PrintSupport Svg Network Concurrent)

message(STATUS "QT version: " ${QT_VERSION_MAJOR}.${QT_VERSION_MINOR})
message(STATUS "QT dir" ${Qt${QT_VERSION_MAJOR}_DIR})

set(PCL_DIR "$ENV{HOME}/usr/share/pcl-1.13/")
message("PCL_DIR:" ${PCL_DIR})

if(EXISTS "${PCL_DIR}")
	find_package(PCL 1.13 REQUIRED COMPONENTS
		common
		octree
		io
		kdtree
		search
		sample_consensus
		filters
		2d
		features
		registration
		geometry
		visualization
		outofcore
		surface
		keypoints
		ml
		segmentation
		recognition
		people
		tracking
		stereo)
endif()

if(${PCL_FOUND})
	include_directories(${PCL_INCLUDE_DIRS})
	link_directories(${PCL_LIBRARY_DIRS})
	add_definitions(${PCL_DEFINITIONS})

	add_executable(qml_pcl src/qml_pcl.cpp qml.qrc)
	target_link_libraries(qml_pcl PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

	add_executable(DownsamplePointCloud src/DownsamplePointCloud.cpp)
	target_link_libraries(DownsamplePointCloud PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

	add_executable(pointcloud src/pointcloud.cpp qml.qrc)
	target_link_libraries(pointcloud PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

	add_executable(polygon_mesh src/polygon_mesh.cpp qml.qrc)
	target_link_libraries(polygon_mesh PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

	add_executable(sphere src/sphere.cpp qml.qrc)
	target_link_libraries(sphere PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})

	add_executable(AreaPicking src/AreaPicking.cxx)
	target_link_libraries(AreaPicking PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
	vtk_module_autoinit(TARGETS AreaPicking MODULES ${VTK_LIBRARIES})
endif()

add_executable(vtk_demo src/vtk_demo.cpp)
target_link_libraries(vtk_demo PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS vtk_demo MODULES ${VTK_LIBRARIES})

add_executable(widget src/widget.cpp)
target_link_libraries(widget PRIVATE ${VTK_LIBRARIES})

add_executable(BillboardTextActor3D src/BillboardTextActor3D.cxx)
target_link_libraries(BillboardTextActor3D PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS BillboardTextActor3D MODULES ${VTK_LIBRARIES})

add_executable(Camera MACOSX_BUNDLE src/Camera.cxx)
target_link_libraries(Camera PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Camera MODULES ${VTK_LIBRARIES})

add_executable(ShareCamera src/ShareCamera.cxx)
target_link_libraries(ShareCamera PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ShareCamera MODULES ${VTK_LIBRARIES})

add_executable(ResetCameraOrientation src/ResetCameraOrientation.cxx)
target_link_libraries(ResetCameraOrientation PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ResetCameraOrientation MODULES ${VTK_LIBRARIES})

add_executable(KeypressEvents src/KeypressEvents.cxx)
target_link_libraries(KeypressEvents PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS KeypressEvents MODULES ${VTK_LIBRARIES})

add_executable(InteractorStyleTerrain src/InteractorStyleTerrain.cxx)
target_link_libraries(InteractorStyleTerrain PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS InteractorStyleTerrain MODULES ${VTK_LIBRARIES})

add_executable(CameraOrientationWidgetGizmo src/CameraOrientationWidgetGizmo.cxx)
target_link_libraries(CameraOrientationWidgetGizmo PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CameraOrientationWidgetGizmo MODULES ${VTK_LIBRARIES})

add_executable(AffineWidget src/AffineWidget.cxx)
target_link_libraries(AffineWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AffineWidget MODULES ${VTK_LIBRARIES})

add_executable(AngleWidget src/AngleWidget.cxx)
target_link_libraries(AngleWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AngleWidget MODULES ${VTK_LIBRARIES})

add_executable(AngleWidget2D src/AngleWidget2D.cxx)
target_link_libraries(AngleWidget2D PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AngleWidget2D MODULES ${VTK_LIBRARIES})

add_executable(CaptionWidget src/CaptionWidget.cxx)
target_link_libraries(CaptionWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CaptionWidget MODULES ${VTK_LIBRARIES})

add_executable(CompassWidget src/CompassWidget.cxx)
target_link_libraries(CompassWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CompassWidget MODULES ${VTK_LIBRARIES})

add_executable(ContourWidget src/ContourWidget.cxx)
target_link_libraries(ContourWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ContourWidget MODULES ${VTK_LIBRARIES})

add_executable(DistanceWidget src/DistanceWidget.cxx)
target_link_libraries(DistanceWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS DistanceWidget MODULES ${VTK_LIBRARIES})

add_executable(ImageTracerWidgetNonPlanar src/ImageTracerWidgetNonPlanar.cxx)
target_link_libraries(ImageTracerWidgetNonPlanar PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ImageTracerWidgetNonPlanar MODULES ${VTK_LIBRARIES})

# ./ImplicitPlaneWidget2 cow.vtp
add_executable(ImplicitPlaneWidget2 src/ImplicitPlaneWidget2.cxx)
target_link_libraries(ImplicitPlaneWidget2 PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ImplicitPlaneWidget2 MODULES ${VTK_LIBRARIES})

add_executable(LineWidget2 src/LineWidget2.cxx)
target_link_libraries(LineWidget2 PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS LineWidget2 MODULES ${VTK_LIBRARIES})

add_executable(ScalarBarWidget src/ScalarBarWidget.cxx)
target_link_libraries(ScalarBarWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ScalarBarWidget MODULES ${VTK_LIBRARIES})

add_executable(Slider src/Slider.cxx)
target_link_libraries(Slider PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Slider MODULES ${VTK_LIBRARIES})

add_executable(TextWidget src/TextWidget.cxx)
target_link_libraries(TextWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TextWidget MODULES ${VTK_LIBRARIES})

add_executable(ViewportBorders src/ViewportBorders.cxx)
target_link_libraries(ViewportBorders PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ViewportBorders MODULES ${VTK_LIBRARIES})

add_executable(WorldPointPicker src/WorldPointPicker.cxx)
target_link_libraries(WorldPointPicker PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS WorldPointPicker MODULES ${VTK_LIBRARIES})

add_executable(TrackballCamera src/TrackballCamera.cxx)
target_link_libraries(TrackballCamera PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TrackballCamera MODULES ${VTK_LIBRARIES})

add_executable(TrackballActor src/TrackballActor.cxx)
target_link_libraries(TrackballActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TrackballActor MODULES ${VTK_LIBRARIES})

add_executable(StyleSwitch src/StyleSwitch.cxx)
target_link_libraries(StyleSwitch PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS StyleSwitch MODULES ${VTK_LIBRARIES})

add_executable(SelectAnActor src/SelectAnActor.cxx)
target_link_libraries(SelectAnActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SelectAnActor MODULES ${VTK_LIBRARIES})

add_executable(RubberBandZoom src/RubberBandZoom.cxx)
target_link_libraries(RubberBandZoom PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS RubberBandZoom MODULES ${VTK_LIBRARIES})

add_executable(LandmarkTransform src/LandmarkTransform.cxx)
target_link_libraries(LandmarkTransform PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS LandmarkTransform MODULES ${VTK_LIBRARIES})

add_executable(DisplayCoordinateAxes src/DisplayCoordinateAxes.cxx)
target_link_libraries(DisplayCoordinateAxes PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS DisplayCoordinateAxes MODULES ${VTK_LIBRARIES})

# Press the e key
add_executable(SaveSceneToFile src/SaveSceneToFile.cxx)
target_link_libraries(SaveSceneToFile PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SaveSceneToFile MODULES ${VTK_LIBRARIES})

add_executable(ProgrammableSource src/ProgrammableSource.cxx)
target_link_libraries(ProgrammableSource PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ProgrammableSource MODULES ${VTK_LIBRARIES})

# ./StreamLines combxyz.bin combq.bin
add_executable(StreamLines src/StreamLines.cxx)
target_link_libraries(StreamLines PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS StreamLines MODULES ${VTK_LIBRARIES})

add_executable(TransformOrderDemo src/TransformOrderDemo.cxx)
target_link_libraries(TransformOrderDemo PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TransformOrderDemo MODULES ${VTK_LIBRARIES})

add_executable(Axes src/Axes.cxx)
target_link_libraries(Axes PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Axes MODULES ${VTK_LIBRARIES})

add_executable(CubeAxesActor src/CubeAxesActor.cxx)
target_link_libraries(CubeAxesActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CubeAxesActor MODULES ${VTK_LIBRARIES})

add_executable(MovableAxes src/MovableAxes.cxx)
target_link_libraries(MovableAxes PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS MovableAxes MODULES ${VTK_LIBRARIES})

add_executable(AlignTwoPolyDatas src/AlignTwoPolyDatas.cxx)
target_link_libraries(AlignTwoPolyDatas PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AlignTwoPolyDatas MODULES ${VTK_LIBRARIES})

add_executable(CameraActor src/CameraActor.cxx)
target_link_libraries(CameraActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CameraActor MODULES ${VTK_LIBRARIES})

add_executable(LinearCellDemo src/LinearCellDemo.cxx)
target_link_libraries(LinearCellDemo PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS LinearCellDemo MODULES ${VTK_LIBRARIES})

add_executable(vtk_sandbox src/vtk_sandbox.cxx)
target_link_libraries(vtk_sandbox PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS vtk_sandbox MODULES ${VTK_LIBRARIES})

add_executable(Tetrahedron src/Tetrahedron.cxx)
target_link_libraries(Tetrahedron PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Tetrahedron MODULES ${VTK_LIBRARIES})

add_executable(Cell3DDemonstration src/Cell3DDemonstration.cxx)
target_link_libraries(Cell3DDemonstration PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Cell3DDemonstration MODULES ${VTK_LIBRARIES})

add_executable(MultipleViewports src/MultipleViewports.cxx)
target_link_libraries(MultipleViewports PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS MultipleViewports MODULES ${VTK_LIBRARIES})

add_executable(SmartVolumeMapper src/SmartVolumeMapper.cxx)
target_link_libraries(SmartVolumeMapper PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SmartVolumeMapper MODULES ${VTK_LIBRARIES})

# ./ProjectedTexture fran_cut.vtk fran_cut.png
add_executable(ProjectedTexture src/ProjectedTexture.cxx)
target_link_libraries(ProjectedTexture PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ProjectedTexture MODULES ${VTK_LIBRARIES})

add_executable(SCurveSpline src/SCurveSpline.cxx)
target_link_libraries(SCurveSpline PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SCurveSpline MODULES ${VTK_LIBRARIES})

add_executable(AddCell src/AddCell.cxx)
target_link_libraries(AddCell PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AddCell MODULES ${VTK_LIBRARIES})

add_executable(Pyramid src/Pyramid.cxx)
target_link_libraries(Pyramid PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Pyramid MODULES ${VTK_LIBRARIES})

add_executable(TransparentBackground src/TransparentBackground.cxx)
target_link_libraries(TransparentBackground PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TransparentBackground MODULES ${VTK_LIBRARIES})

add_executable(Follower src/Follower.cxx)
target_link_libraries(Follower PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Follower MODULES ${VTK_LIBRARIES})

add_executable(vtkProp3DFollower src/vtkProp3DFollower.cxx)
target_link_libraries(vtkProp3DFollower PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS vtkProp3DFollower MODULES ${VTK_LIBRARIES})

add_executable(RotateActor src/RotateActor.cxx)
target_link_libraries(RotateActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS RotateActor MODULES ${VTK_LIBRARIES})

add_executable(vtkInteractorStyle src/vtkInteractorStyle.cpp)
target_link_libraries(vtkInteractorStyle PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS vtkInteractorStyle MODULES ${VTK_LIBRARIES})

add_executable(PickableOff src/PickableOff.cxx)
target_link_libraries(PickableOff PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS PickableOff MODULES ${VTK_LIBRARIES})

add_executable(HighlightBadCells src/HighlightBadCells.cxx)
target_link_libraries(HighlightBadCells PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS HighlightBadCells MODULES ${VTK_LIBRARIES})

add_executable(HighlightPickedActor src/HighlightPickedActor.cxx)
target_link_libraries(HighlightPickedActor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS HighlightPickedActor MODULES ${VTK_LIBRARIES})

add_executable(HighlightSelectedPoints src/HighlightSelectedPoints.cxx)
target_link_libraries(HighlightSelectedPoints PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS HighlightSelectedPoints MODULES ${VTK_LIBRARIES})

add_executable(HighlightSelection src/HighlightSelection.cxx)
target_link_libraries(HighlightSelection PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS HighlightSelection MODULES ${VTK_LIBRARIES})

add_executable(HighlightWithSilhouette src/HighlightWithSilhouette.cxx)
target_link_libraries(HighlightWithSilhouette PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS HighlightWithSilhouette MODULES ${VTK_LIBRARIES})

add_executable(multi_interactor src/multi_interactor.cpp src/InteractorStyleSwitch.hpp src/InteractorStyleSwitch.cpp src/ActorManipulationInteractorStyle.hpp src/ActorManipulationInteractorStyle.cpp src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp)
target_link_libraries(multi_interactor PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS multi_interactor MODULES ${VTK_LIBRARIES})



add_executable(DistanceWidget3D src/DistanceWidget3D.cpp)
target_link_libraries(DistanceWidget3D PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS DistanceWidget3D MODULES ${VTK_LIBRARIES})

add_executable(Callback src/Callback.cpp)
target_link_libraries(Callback PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Callback MODULES ${VTK_LIBRARIES})

add_executable(PolygonalSurfaceContourLineInterpolator src/PolygonalSurfaceContourLineInterpolator.cxx)
target_link_libraries(PolygonalSurfaceContourLineInterpolator PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS PolygonalSurfaceContourLineInterpolator MODULES ${VTK_LIBRARIES})

add_executable(PointPicker src/PointPicker.cxx)
target_link_libraries(PointPicker PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS PointPicker MODULES ${VTK_LIBRARIES})

add_executable(SeedWidget src/SeedWidget.cxx)
target_link_libraries(SeedWidget PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SeedWidget MODULES ${VTK_LIBRARIES})

add_executable(SeedWidgetWithCustomCallback src/SeedWidgetWithCustomCallback.cxx)
target_link_libraries(SeedWidgetWithCustomCallback PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS SeedWidgetWithCustomCallback MODULES ${VTK_LIBRARIES})

add_executable(ActorProperties src/ActorProperties.cpp)
target_link_libraries(ActorProperties PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS ActorProperties MODULES ${VTK_LIBRARIES})


add_executable(PlaneSource src/PlaneSource.cpp)
target_link_libraries(PlaneSource PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS PlaneSource MODULES ${VTK_LIBRARIES})


add_executable(QtQuickCone src/QtQuickCone.cpp src/MyVtkItem.hpp qml.qrc src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp src/InteractorStyleSwitch.hpp src/InteractorStyleSwitch.cpp src/ActorManipulationInteractorStyle.hpp src/ActorManipulationInteractorStyle.cpp src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp)
target_link_libraries(QtQuickCone PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})

 add_executable(main src/main.cpp src/VTKBackend.hpp src/VTKBackend.cpp qml.qrc)
 target_link_libraries(main PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${PCL_LIBRARIES} ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS QtQuickCone MODULES ${VTK_LIBRARIES})

add_executable(dynamic_split_view qml.qrc src/dynamic_split_view.cpp)
target_link_libraries(dynamic_split_view PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS dynamic_split_view MODULES ${VTK_LIBRARIES})



add_executable(CommandObserverClass src/CommandObserverClass.cpp)
target_link_libraries(CommandObserverClass PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CommandObserverClass MODULES ${VTK_LIBRARIES})

add_executable(multiple_renderers_within_a_render_window src/multiple_renderers_within_a_render_window.cpp)
target_link_libraries(multiple_renderers_within_a_render_window PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS multiple_renderers_within_a_render_window MODULES ${VTK_LIBRARIES})

add_executable(setting_interaction_style src/setting_interaction_style.cpp)
target_link_libraries(setting_interaction_style PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS setting_interaction_style MODULES ${VTK_LIBRARIES})

add_executable(InteractorStyleSwitchExample src/InteractorStyleSwitchExample.cpp src/InteractorStyleSwitch.hpp src/InteractorStyleSwitch.cpp src/ActorManipulationInteractorStyle.hpp src/ActorManipulationInteractorStyle.cpp src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp)
target_link_libraries(InteractorStyleSwitchExample PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS InteractorStyleSwitchExample MODULES ${VTK_LIBRARIES})

add_executable(boxWidget src/boxWidget.cpp)
target_link_libraries(boxWidget PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS boxWidget MODULES ${VTK_LIBRARIES})

add_executable(CameraPosition src/CameraPosition.cpp)
target_link_libraries(CameraPosition PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CameraPosition MODULES ${VTK_LIBRARIES})

add_executable(QQuickVTKRender src/QQuickVTKRender.cpp qml.qrc src/ActorManipulationInteractorStyle.hpp src/ActorManipulationInteractorStyle.cpp src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp)
target_link_libraries(QQuickVTKRender PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})

add_executable(CylinderExample src/CylinderExample.cpp)
target_link_libraries(CylinderExample PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CylinderExample MODULES ${VTK_LIBRARIES})

add_executable(TransformActorCollection src/TransformActorCollection.cxx)
target_link_libraries(TransformActorCollection PRIVATE ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS TransformActorCollection MODULES ${VTK_LIBRARIES})

add_executable(sandbox src/sandbox.cpp src/InteractorStyleSwitch.hpp src/InteractorStyleSwitch.cpp src/ActorManipulationInteractorStyle.hpp src/ActorManipulationInteractorStyle.cpp src/CameraInteractorStyle.cpp src/CameraInteractorStyle.hpp )
target_link_libraries(sandbox PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS sandbox MODULES ${VTK_LIBRARIES})


 




add_executable(BoundingBox src/BoundingBox.cpp)
target_link_libraries(BoundingBox PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS BoundingBox MODULES ${VTK_LIBRARIES})

add_executable(OutlineFilter src/OutlineFilter.cpp)
target_link_libraries(OutlineFilter PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS OutlineFilter MODULES ${VTK_LIBRARIES})

add_executable(Frustum src/Frustum.cxx)
target_link_libraries(Frustum PRIVATE Qt${QT_VERSION_MAJOR}::Core Qt${QT_VERSION_MAJOR}::Quick Qt5::Widgets ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Frustum MODULES ${VTK_LIBRARIES})

add_executable(MyVTKQtApp src/MyVTKQtApp.cpp)
target_link_libraries(MyVTKQtApp PRIVATE ${VTK_LIBRARIES} Qt5::Widgets)
vtk_module_autoinit(TARGETS MyVTKQtApp MODULES ${VTK_LIBRARIES})


add_executable(SyncTransformations src/SyncTransformations.cpp)
target_link_libraries(SyncTransformations PRIVATE ${VTK_LIBRARIES} )
vtk_module_autoinit(TARGETS SyncTransformations MODULES ${VTK_LIBRARIES})


add_executable(CommandObserverFunction src/CommandObserverFunction.cpp)
target_link_libraries(CommandObserverFunction PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS CommandObserverFunction MODULES ${VTK_LIBRARIES})


add_executable(AppendFilter src/AppendFilter.cpp)
target_link_libraries(AppendFilter PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS AppendFilter MODULES ${VTK_LIBRARIES})

add_executable(Assembly src/Assembly.cpp)
target_link_libraries(Assembly PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS Assembly MODULES ${VTK_LIBRARIES})


add_executable(LegendScale src/LegendScale.cpp)
target_link_libraries(LegendScale PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS LegendScale MODULES ${VTK_LIBRARIES})


add_executable(LegendScaleActor src/LegendScaleActor.cxx)
target_link_libraries(LegendScaleActor PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS LegendScaleActor MODULES ${VTK_LIBRARIES})

add_executable(BackfaceCulling src/BackfaceCulling.cxx)
target_link_libraries(BackfaceCulling PRIVATE  ${VTK_LIBRARIES})
vtk_module_autoinit(TARGETS BackfaceCulling MODULES ${VTK_LIBRARIES})

# add_executable(DisplayTrackball src/DisplayTrackball.cpp)
# target_link_libraries(DisplayTrackball PRIVATE  ${VTK_LIBRARIES})
# vtk_module_autoinit(TARGETS DisplayTrackball MODULES ${VTK_LIBRARIES})



# add_executable(vtk_basic src/vtk_basic.cpp src/vtk_basic.hpp src/vtk_basic_main.cpp)
# target_link_libraries(vtk_basic PRIVATE ${VTK_LIBRARIES})
# vtk_module_autoinit(TARGETS vtk_basic MODULES ${VTK_LIBRARIES})