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

## Viewport

[Multiple Renderers within a Render Window, Viewport](docs/multiple_renderers_within_a_render_window.md)  
[ShareCamera, Viewport](docs/shareCamera_viewport.md)  

## Widget

[BoxWidget](docs/boxWidget.md)  
[AffineWidget](docs/affineWidget.md)  
[DistanceWidget](docs/distanceWidget.md)  
[TextWidget](docs/textWidget.md)

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
[Rotate/ Transform Actor](docs/rotate_actor.md)  
[Key Press Interactor](docs/key_press_interactor.md)  
[InteractorStyleSwitch](docs/interactor_style_switch.md)  

## Camera

[Modify Renderer Camera](docs/modify_renderer_camera.md)  
[Renderer Camera Position Call back](docs/camera_position.md)  

## PCL

[PCL Pointcloud](docs/pcl_pointcloud.md)  

## QML, QT

[QQuickVTKItem](docs/qml_vtk_QQuickVTKItem.md)  
[QQuickVTKRenderItem](docs/QQuickVTKRenderItem.md)
