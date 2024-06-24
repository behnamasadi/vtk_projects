# Building VTK

```
git clone https://github.com/Kitware/VTK.git
```

configure it:
```
cmake -G "Ninja Multi-Config"  -S . -B build -DQT_QMAKE_EXECUTABLE=/usr/bin/qmake -DCMAKE_PREFIX_PATH=/usr/lib/x86_64-linux-gnu/cmake/Qt5  -DCMAKE_INSTALL_PREFIX=~/usr -DVTK_REPORT_OPENGL_ERRORS=OFF -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES     -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick=YES -DVTK_WRAP_JAVA=OFF -DVTK_WRAP_PYTHON=OFF  -DVTK_ENABLE_WRAPPING=OFF  -DVTK_BUILD_TESTING=OFF -DVTK_GROUP_ENABLE_Rendering=DONT_WANT -DVTK_MODULE_ENABLE_VTK_RenderingQt=YES -DVTK_MODULE_ENABLE_VTK_hdf5=YES -DVTK_MODULE_ENABLE_VTK_IOHDF=YES -DVTK_MODULE_ENABLE_VTK_InteractionImage=YES    -DVTK_MODULE_ENABLE_VTK_RenderingLOD=YES -DVTK_MODULE_ENABLE_VTK_ViewsCore=YES -DVTK_MODULE_ENABLE_VTK_ViewsContext2D=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_GROUP_ENABLE_Qt=YES -DVTK_MODULE_ENABLE_VTK_ViewsQt=YES -DVTK_GROUP_ENABLE_Views=YES -DVTK_MODULE_ENABLE_VTK_RenderingUI=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolume=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolumeOpenGL2=YES -DVTK_QT_VERSION=5 -DVTK_MODULE_ENABLE_VTK_opengl=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_MODULE_ENABLE_VTK_RenderingSceneGraph=YES -DVTK_MODULE_ENABLE_VTK_InteractionWidgets=YES -DVTK_MODULE_ENABLE_VTK_IOPDAL=YES -DVTK_REPORT_OPENGL_ERRORS=OFF 
```

build and install:

```
cmake --build  build  --config Debug  --target install
```
or 
```
cmake --build build --config Release --target install
```

```
cd build
ninja install
```

to check if you have installed debug or release:

```
objdump -h libvtkImagingStencil-9.3.so.9.3 | grep 'debug_info'
```


[Supported Data Formats](https://docs.vtk.org/en/latest/supported_data_formats.html)  

[List of VTK Modules](https://docs.vtk.org/en/latest/modules/index.html)  

[VTK - numpy integration](https://docs.vtk.org/en/latest/learning.html)  


# How to build Project

configure it:

```
cmake -G "Ninja Multi-Config"  -S . -B build -DVTK_DIR="/home/$USER/usr/lib/cmake/vtk-9.3/" -DPCL_DIR="/home/$USER/usr/share/pcl-1.14/"
```

build it:

```
cmake --build build
```

or be more specific:

```
cmake --build build --target all --config Release
```

If you prefer `preset` use:

```
cmake --preset ninja-multi
```
and 

```
cmake --build --preset ninja-multi-debug
```
or 
```
cmake --build --preset ninja-multi-release
```


Prevent a "command line is too long" failure in Windows:

```
set(CMAKE_NINJA_FORCE_RESPONSE_FILE "ON" CACHE BOOL "Force Ninja to use response files.")
add_executable(VisualDebugging MACOSX_BUNDLE src/vt/VisualDebugging.cxx)
target_link_libraries(VisualDebugging PRIVATE ${VTK_LIBRARIES} )
```

## Basics
[vtkFloatArray, vtkPoints, vtkPointData, vtkPolyData, vtkPointSource, vtkPolyVertex](docs/basic_data_types.md)  
[Glyphs, vtkGlyph3D, vtkVertexGlyphFilter](docs/glyphs.md)  
[SetInputData vtkDataObject*, SetInputConnection vtkAlgorithmOutput*](docs/SetInputData_SetInputConnection.md)


[vtkCell, vtkCellArray, vtkTriangle](docs/cell.md)  
[colorTransferFunction](docs/colorTransferFunction.md)  
[LookupTable](docs/lookupTable.md)  
[Height based color map](src/height_based_color_map.cpp)  
[The Basic Setup](docs/the_basic_setup.md)  
[Command/Observer and CallBacks for Events](docs/command_observer_for_events_callback.md)  
[Mouse Event vtkCommand](../src/mouse_event_vtkCommand.cpp)  
[Actor Properties](docs/actor_properties.md)  
[Plane Source/ Grid Background](docs/plane_source_grid_background.md)  
[BoundingBox](docs/boundingbox.md)  
[Prop3D](docs/prop3d.md)  
[Grouping Actors, Actor Collection, Transforming Multiple Actors, Assembly](docs/grouping_actors_actor_collection_assembly.md)  
[Remove Actor](docs/remove_actor.md)  
[Remove Observer](docs/remove_observer.md)  
[Actor SetPosition, SetUserMatrix, SetUserTransform](docs/actor_transform.md)  
[AppendFilter](docs/append_filter.md)  
[ConnectivityFilter](docs/connectivity_filter.md)  
[RestoreSceneFromFile](https://kitware.github.io-examples/site/Cxx/Snippets/RestoreSceneFromFile/)  
[ProjectedTexture](src/ProjectedTexture.cxx)  
[LegendScale](src/LegendScaleActor.cpp)  
[Anti-Aliasing](docs/anti-aliasing.md)  
[Multiple Layers](docs/multiple_layers.md)  
[CaptionActor2D](docs/captionActor2D.md)  
[Octree](docs/octree.md)  

## Viewport

[Multiple Renderers within a Render Window, Viewport](docs/multiple_renderers_within_a_render_window.md)  
[ShareCamera, Viewport](docs/shareCamera_viewport.md)  

## Widget

[BoxWidget](docs/boxWidget.md)  
[AffineWidget](docs/affineWidget.md)  
[DistanceWidget](docs/distanceWidget.md)  
[AngleWidget](docs/angleWidget.md)  
[TextWidget](docs/textWidget.md)  
[Gizmo, CameraOrientationWidget](docs/gizmo_camera_orientation_widget.md)  
[Plane Widget](src/ImplicitPlaneWidget2.cxx)  
[Add/ remove multiple Widget](docs/add_remove_multiple_widget.md)  

## Axes

[AxesActor](docs/axesActor.md)  

## Point Picker

[World Point Picker](docs/world_point_picker.md)  
[Area Picker](docs/area_picker.md)  
[Prop /Actor Picker](docs/prop_actor_picker.md)  
[Pickable Off](docs/pickable_off.md)  

## Following Camera

[Follower](docs/follower.md)  
[Prop3DFollower, Image Actor](docs/prop3DFollower_image_actor.md)  
[BillboardTextActor3D](docs/billboard_text_actor3D.md)  

## Window Interaction

[Setting Interaction Style](docs/setting_interaction_style.md)  
[vtkInteractorStyleTrackballCamera](docs/setting_interaction_style.md#vtkInteractorStyleTrackballCamera)  
[vtkInteractorStyleTrackballActor](docs/setting_interaction_style.md#vtkInteractorStyleTrackballActor)  
[InteractorStyleTrackballActor Rotate/ Transform Actor](docs/rotate_actor.md)  
[Key Press Interactor](docs/key_press_interactor.md)  
[InteractorStyleSwitch](docs/interactor_style_switch.md)  

## Camera

[Modify Renderer Camera, Set Position, FocalPoint, ViewUp, Azimuth, Elevation, ViewAngle, clipping Range](docs/modify_renderer_camera.md)  
[Renderer Camera Position Call back, OnLeftButtonDown, OnChar, Pan, Dolly, Get Position, ViewAngle](docs/camera_position.md)  
[Frustum Source, Camera Frustum Planes](docs/frustum.md)  
[Select Visible Points](docs/select_visible_points.md)
[High resolution, Low resolution Actor](docs/high_resolution_low_resolution_actor.md)  
[Definition of Pan, Tilt and Spin](docs/images/Definition-of-pan-tilt-and-spin.png)  
[Culling](docs/culling.md)  
  

## PCL

[PCL Pointcloud](docs/pcl_pointcloud.md)  

## QML, QT

[QQuickVTKItem](docs/qml_vtk_QQuickVTKItem.md)  
[QQuickVTKRenderItem](docs/QQuickVTKRenderItem.md)  








