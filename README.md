# Building VTK

Build and install PDAL **before** configuring VTK. VTK discovers PDAL only at
VTK configure time; installing PDAL later will not enable `VTK::IOPDAL` in an
existing VTK build.

## Build and install PDAL

Install PDAL's PROJ and GDAL development dependencies first:

```
sudo apt install libproj-dev
sudo apt install libgdal-dev
```

Then build PDAL into the same prefix that will be used for VTK:

```
git clone https://github.com/PDAL/PDAL.git
cmake -S PDAL -B PDAL/build -G Ninja -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_INSTALL_PREFIX="$HOME/usr"
cmake --build PDAL/build --target install
```

## Build and install VTK

```
git clone https://github.com/Kitware/VTK.git
```

Configure VTK with the PDAL installation prefix so `VTK::IOPDAL` can be enabled and found:

```
cmake -G "Ninja Multi-Config" -S . -B build -DQT_QMAKE_EXECUTABLE=/usr/bin/qmake \
  -DCMAKE_PREFIX_PATH="$HOME/usr;/usr/lib/x86_64-linux-gnu/cmake/Qt5" \
  -DPDAL_DIR="$HOME/usr/lib/cmake/PDAL" -DCMAKE_INSTALL_PREFIX="$HOME/usr" \
  -DVTK_REPORT_OPENGL_ERRORS=OFF -DVTK_MODULE_ENABLE_VTK_GUISupportQt=YES     -DVTK_MODULE_ENABLE_VTK_GUISupportQtQuick=YES -DVTK_WRAP_JAVA=OFF -DVTK_WRAP_PYTHON=OFF  -DVTK_ENABLE_WRAPPING=OFF  -DVTK_BUILD_TESTING=OFF -DVTK_GROUP_ENABLE_Rendering=DONT_WANT -DVTK_MODULE_ENABLE_VTK_RenderingQt=YES -DVTK_MODULE_ENABLE_VTK_hdf5=YES -DVTK_MODULE_ENABLE_VTK_IOHDF=YES -DVTK_MODULE_ENABLE_VTK_InteractionImage=YES    -DVTK_MODULE_ENABLE_VTK_RenderingLOD=YES -DVTK_MODULE_ENABLE_VTK_ViewsCore=YES -DVTK_MODULE_ENABLE_VTK_ViewsContext2D=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_GROUP_ENABLE_Qt=YES -DVTK_MODULE_ENABLE_VTK_ViewsQt=YES -DVTK_GROUP_ENABLE_Views=YES -DVTK_MODULE_ENABLE_VTK_RenderingUI=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolume=YES -DVTK_MODULE_ENABLE_VTK_RenderingVolumeOpenGL2=YES -DVTK_QT_VERSION=5 -DVTK_MODULE_ENABLE_VTK_opengl=YES -DVTK_MODULE_ENABLE_VTK_RenderingContextOpenGL2=YES -DVTK_MODULE_ENABLE_VTK_RenderingSceneGraph=YES -DVTK_MODULE_ENABLE_VTK_InteractionWidgets=YES -DVTK_MODULE_ENABLE_VTK_IOPDAL=YES -DVTK_REPORT_OPENGL_ERRORS=OFF
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

## Supported Data Formats
[Supported Data Formats](https://docs.vtk.org/en/latest/supported_data_formats.html)  
[LAS, LAZ and COPC — the file format](docs/copc_format.md)  
[COPC, LAZ, LOD, PDAL and the camera — tutorial index](docs/copc_laz_lod_tutorial.md)  

## List of VTK Modules
[List of VTK Modules](https://docs.vtk.org/en/latest/modules/index.html)  

## VTK - numpy integration
[VTK - numpy integration](https://docs.vtk.org/en/latest/learning.html)  


# How to build Project

## Dependencies

The project needs VTK (built from source into `~/usr`) and the Qt5 development
packages (the runtime `libqt5*-5` packages are **not** enough — CMake needs the
`-dev` packages that ship the `Qt5*Config.cmake` files):

```
sudo apt install qtdeclarative5-dev qtquickcontrols2-5-dev libqt5svg5-dev \
                 qtbase5-dev libqt5sql5 libboost-filesystem-dev libboost-system-dev
```

configure it (VTK-only, no PCL/PDAL):

```
cmake -G "Ninja Multi-Config"  -S . -B build -DVTK_DIR="/home/$USER/usr/lib/cmake/vtk-9.3/"
```

### PCL and PDAL (optional)

Both **PCL** and **PDAL** are **optional and disabled by default** — the project
configures and builds without them. Their dependent targets are only built when
you opt in:

| Option | Enables targets |
|--------|-----------------|
| `-DUSE_PCL=ON`  | `qml_pcl`, `DownsamplePointCloud`, `pointcloud`, `polygon_mesh`, `sphere`, `AreaPicking`, `toolbox_qml` |
| `-DUSE_PDAL=ON` | `pdal_las_vtk`, `las_time_based_filter`, `pdal_intensity`, `height_based_color_map`, `vtk_basic`, `create_copc_in_memory`, `copc_lod_queries`, `copc_partial_load_vtk`, `copc_camera_streaming` |

PDAL must be built and installed into `~/usr` before VTK, as described above.
PCL may also need to be built from source. Enable one or both:

```
cmake -G "Ninja Multi-Config"  -S . -B build -DVTK_DIR="/home/$USER/usr/lib/cmake/vtk-9.3/" \
      -DUSE_PCL=ON  -DPCL_DIR="/home/$USER/usr/share/pcl-1.14/" \
      -DUSE_PDAL=ON -DPDAL_DIR="/home/$USER/usr/lib/cmake/PDAL/"
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
[Clean PolyData](https://vtk.org/doc/nightly/html/classvtkCleanPolyData.html)  
[Downsample PointCloud](https://examples.vtk.org/site/Cxx/PolyData/DownsamplePointCloud/)  
[ExtractPoints with ImplicitFunction](docs/extract_points.md)  
[Glyphs, vtkGlyph3D, vtkVertexGlyphFilter](docs/glyphs.md)  
[SetInputData vtkDataObject*, SetInputConnection vtkAlgorithmOutput*](docs/SetInputData_SetInputConnection.md)  
[vtkCell, vtkCellArray, vtkTriangle](docs/cell.md)  
[colorTransferFunction](docs/colorTransferFunction.md)  
[LookupTable](docs/lookupTable.md)  
[Height based color map](src/height_based_color_map.cpp)  
[The Basic Setup](docs/the_basic_setup.md)  
[Command/Observer and CallBacks for Events](docs/command_observer_for_events_callback.md)  
[Mouse Event vtkCommand](src/mouse_event_vtkCommand.cpp)  
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
[LegendScale](src/LegendScaleActor.cxx)  
[Anti-Aliasing](docs/anti-aliasing.md)  
[Multiple Layers](docs/multiple_layers.md)  
[CaptionActor2D](docs/captionActor2D.md)  
[Octree  <span style="color:red">*needs update*</span>](docs/octree.md) 
[COPC, LAZ, LOD, PDAL, Potree, VTK octrees, and camera pivot](docs/copc_laz_lod_tutorial.md)  
[PDAL — reading and writing COPC, `bounds` and `resolution`](docs/copc_pdal.md)  

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
[Definition of Pan, Tilt and Spin](docs/images/Definition-of-pan-tilt-and-spin.png)  
[Camera-driven LOD: frustum → `bounds`, screen-space error → `resolution`](docs/copc_vtk_camera.md)  

## Filtering, Culling, Decimation, Multi Level of Details, Visible Points in Camera

[Actor Multiple Levels of Detail vtkLODActor  <span style="color:red">*needs update*</span>](docs/actor_multiple_levels_of_detail.md)  
[High resolution, Low resolution Actor](docs/high_resolution_low_resolution_actor.md)  
[Culling <span style="color:red">*needs update*</span>](docs/culling.md)  
[MaskPoints](docs/mask_points.md)  
[MaskPointsFilter](docs/mask_points_filter.md)  
[OutlineFilter](docs/outline_filter.md)  
[DecimatePro (reducing the number of triangles in a mesh)](docs/decimate_pro.md)  
[ThresholdPoints](docs/threshold_points.md)  
[Extract Point Based on Implicit Function ExtractGeometry<span style="color:red">*needs update*</span>](docs/extract_point_based_implicit_function_extract_geometry.md)  
[HierarchicalBinningFilter](docs/hierarchical_binning_filter.md)  
[How COPC builds levels of detail — spacing, thinning, node chunks, hierarchy VLR](docs/copc_lod_mechanisms.md)  
[Out-of-core LOD worked examples — inspect the octree, partial load, camera streaming](docs/copc_worked_examples.md)  
[Select Visible Points In Camera ( z-buffer) <span style="color:red">*needs update*</span>](docs/select_visible_points_in_camera.md)  
[Point Visibility In Camera Frustum <span style="color:red">*needs update*</span>](docs/point_visibility_in_camera_frustum.md)  
[HyperTreeGrid <span style="color:red">*needs update*</span>](https://www.kitware.com/hypertreegrid-in-vtk-an-introduction/)  
[PointCloudFilter  <span style="color:red">*needs update*</span>](docs/point_cloud_filter.md)  
  - [vtkRadiusOutlierRemoval](docs/point_cloud_filter.md#vtkRadiusOutlierRemoval)  
  - [vtkStatisticalOutlierRemoval](docs/point_cloud_filter.md#vtkStatisticalOutlierRemoval)  
  - [vtkExtractHierarchicalBins](docs/point_cloud_filter.md#vtkExtractHierarchicalBins)  
  - [vtkExtractPoints](docs/point_cloud_filter.md#vtkExtractPoints)  
  - [vtkFitImplicitFunction](docs/point_cloud_filter.md#vtkFitImplicitFunction)    
  

## Volumetric Entity 
[vtkVolume](https://vtk.org/doc/nightly/html/classvtkVolume.html#details)  
[vtkMarchingCubes](https://vtk.org/doc/nightly/html/classvtkMarchingCubes.html)  

## PCL

[PCL Pointcloud](docs/pcl_pointcloud.md)  

## QML, QT

[QQuickVTKItem](docs/qml_vtk_QQuickVTKItem.md)  
[QQuickVTKRenderItem](docs/QQuickVTKRenderItem.md)  

## 
[Surface Properties](https://examples.vtk.org/site/VTKBook/03Chapter3/#34-surface-properties)
specular coefficients
