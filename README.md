# Building VTK

```
git checkout v9.2.6
```

configure it:
```
cmake -G "Ninja Multi-Config"  -S . -B build   -DCMAKE_INSTALL_PREFIX=~/usr -DVTK_REPORT_OPENGL_ERRORS=OFF -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES     -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick=YES -DVTK_WRAP_JAVA=OFF -DVTK_WRAP_PYTHON=OFF  -DVTK_ENABLE_WRAPPING=OFF  -DVTK_BUILD_TESTING=OFF -DVTK_GROUP_ENABLE_Rendering=DONT_WANT -DVTK_MODULE_ENABLE_VTK_RenderingQt=YES -DVTK_MODULE_ENABLE_VTK_hdf5=YES -DVTK_MODULE_ENABLE_VTK_IOHDF=YES -DVTK_MODULE_ENABLE_VTK_InteractionImage=YES    -DVTK_MODULE_ENABLE_VTK_RenderingLOD=YES -DVTK_MODULE_ENABLE_VTK_ViewsCore=YES -DVTK_MODULE_ENABLE_VTK_ViewsContext2D=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_GROUP_ENABLE_Qt=YES -DVTK_MODULE_ENABLE_VTK_ViewsQt=YES -DVTK_GROUP_ENABLE_Views=YES -DVTK_MODULE_ENABLE_VTK_RenderingUI=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolume=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolumeOpenGL2=YES -DVTK_QT_VERSION=5 -DVTK_MODULE_ENABLE_VTK_opengl=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_MODULE_ENABLE_VTK_RenderingSceneGraph=YES -DVTK_MODULE_ENABLE_VTK_InteractionWidgets=YES

```



cmake -G "Ninja Multi-Config"  -S . -B build   -DCMAKE_INSTALL_PREFIX=~/usr -DVTK_REPORT_OPENGL_ERRORS=OFF  -DVTK_BUILD_ALL_MODULES=TRUE -DVTK_MODULE_ENABLE_VTK_vtkm=NO -DVTKOSPRAY_ENABLE_DENOISER=OFF -DVTK_ENABLE_OSPRAY=OFF -DVTK_MODULE_ENABLE_VTK_RenderingOpenXR=NO -DVTK_MODULE_ENABLE_VTK_RenderingOpenVR=NO -DVTK_MODULE_ENABLE_VTK_IOOpenVDB=NO -DVTK_MODULE_ENABLE_VTK_IOMySQL=NO -DVTK_MODULE_ENABLE_VTK_IOLAS=NO -DVTK_MODULE_ENABLE_VTK_IOADIOS2=NO -DVTK_MODULE_ENABLE_VTK_FiltersOpenTURNS=NO -DOPENSLIDE_LIBRARY-ADVANCED=0 -DVTK_MODULE_ENABLE_VTK_DomainsMicroscopy=NO -DVTK_MODULE_ENABLE_VTK_CommonArchive=NO



cmake -S . -B "$vtkBuildDir"  -DCMAKE_TOOLCHAIN_FILE="${CMakeToolChainFile}"  -DCMAKE_PREFIX_PATH="$CMakePrefixPath" -DCMAKE_INSTALL_PREFIX="$vtkInstallationDir"  -DCMAKE_BUILD_TYPE="$buildType" -DVTK_REPORT_OPENGL_ERRORS=OFF -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES     -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick=YES -DVTK_WRAP_JAVA=OFF -DVTK_WRAP_PYTHON=OFF  -DVTK_ENABLE_WRAPPING=OFF  -DVTK_BUILD_TESTING=OFF -DVTK_GROUP_ENABLE_Rendering=DONT_WANT -DVTK_MODULE_ENABLE_VTK_RenderingQt=YES -DVTK_MODULE_ENABLE_VTK_hdf=YES -DVTK_MODULE_ENABLE_VTK_IOHDF=YES -DVTK_MODULE_ENABLE_VTK_InteractionImage=YES    -DVTK_MODULE_ENABLE_VTK_RenderingLOD=YES -DVTK_MODULE_ENABLE_VTK_ViewsCore=YES -DVTK_MODULE_ENABLE_VTK_ViewsContext2D=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES





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

# How to build

configure it:

```
cmake -G "Ninja Multi-Config"  -S . -B build
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

[The Basic Setup](docs/the_basic_setup.md)  
[Command/Observer and CallBacks for Events](docs/command_observer_for_events_callback.md)  
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
[Vertex Glyph Filter](docs/vertex_glyph_filter.md)  


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

[Modify Renderer Camera, SetPosition, SetFocalPoint, SetViewUp, Azimuth, Elevation, SetViewAngle](docs/modify_renderer_camera.md)  
[Renderer Camera Position Call back](docs/camera_position.md)  
[Camera Frustum](docs/frustum.md)  
[Definition of Pan, Tilt and Spin](docs/images/Definition-of-pan-tilt-and-spin.png)  

[Culling](docs/culling.md)  
  

## PCL

[PCL Pointcloud](docs/pcl_pointcloud.md)  

## QML, QT

[QQuickVTKItem](docs/qml_vtk_QQuickVTKItem.md)  
[QQuickVTKRenderItem](docs/QQuickVTKRenderItem.md)  








