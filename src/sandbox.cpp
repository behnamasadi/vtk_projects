#include <vtkActor.h>
#include <vtkAssembly.h>
#include <vtkBillboardTextActor3D.h>
#include <vtkCamera.h>
#include <vtkCameraPass.h>
#include <vtkCaptionActor2D.h>
#include <vtkCell.h>
#include <vtkColorTransferFunction.h>
#include <vtkConeSource.h>
#include <vtkCubeSource.h>
#include <vtkFloatArray.h>
#include <vtkFollower.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkInteractorStyleTrackballActor.h>
#include <vtkJPEGReader.h>
#include <vtkLegendScaleActor.h>
#include <vtkLineSource.h>
#include <vtkLookupTable.h>
#include <vtkMatrix4x4.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkOverlayPass.h>
#include <vtkPNGReader.h>
#include <vtkPlaneSource.h>
#include <vtkPointData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProp3DFollower.h>
#include <vtkProperty.h>
#include <vtkRenderPassCollection.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkScalarBarActor.h>
#include <vtkSequencePass.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTextProperty.h>
#include <vtkTexture.h>
#include <vtkTransform.h>
#include <vtkTriangle.h>
#include <vtkVertexGlyphFilter.h>

int main(int, char *[]) {

  //   vtkNew<vtkPoints> points;
  //   points->InsertPoint(0, 0, 0, 0);
  //   points->InsertPoint(1, 1, 0, 0);
  //   points->InsertPoint(2, 1, 1, 0);
  //   points->InsertPoint(3, 0, 1, 0);
  //   points->InsertNextPoint(0, 0, 1);

  //   vtkNew<vtkPolyData> polyData;
  //   polyData->SetPoints(points);

  //   //polyData->SetPolys()

  //   vtkNew<vtkVertexGlyphFilter> glyphFilter;
  //   glyphFilter->SetInputData(polyData);
  //   glyphFilter->Update();

  //   vtkNew<vtkPolyDataMapper> mapper;
  //   mapper->SetInputConnection(glyphFilter->GetOutputPort());

  //   vtkNew<vtkActor> actor;
  //   actor->SetMapper(mapper);

  //   vtkNew<vtkRenderer> renderer;
  //   renderer->AddActor(actor);
  //   renderer->SetBackground(0.1, 0.2, 0.4); // Dark blue background

  //   vtkNew<vtkRenderWindow> renderWindow;
  //   renderWindow->AddRenderer(renderer);
  //   renderWindow->Render();

  //   vtkNew<vtkRenderWindowInteractor> iren;
  //   renderWindow->SetInteractor(iren);
  //   iren->Initialize();
  //   iren->Start();

  //   /*
  //     vtkNew<vtkPointData> pointData;
  //     vtkNew<vtkCellArray> cellArray;
  //     //   vtkNew<vtkCell> cell;
  //     vtkNew<vtkPolyData> polyData;
  //     vtkNew<vtkTriangle> triangle;
  //     vtkNew<vtkFloatArray> scalars;
  //   */

  //   // Create points
  //   vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  //   points->InsertNextPoint(0.0, 0.0, 0.0);
  //   points->InsertNextPoint(1.0, 0.0, 0.0);
  //   points->InsertNextPoint(0.5, 1.0, 0.0);

  //   // Create a polydata object
  //   vtkSmartPointer<vtkPolyData> polyData =
  //   vtkSmartPointer<vtkPolyData>::New(); polyData->SetPoints(points);

  //   // Create scalars for each point
  //   vtkSmartPointer<vtkFloatArray> scalars =
  //       vtkSmartPointer<vtkFloatArray>::New();
  //   scalars->SetNumberOfValues(3);
  //   scalars->SetValue(0, 1.0); // Scalar value at point 0
  //   scalars->SetValue(1, 2.0); // Scalar value at point 1
  //   scalars->SetValue(2, 3.0); // Scalar value at point 2

  //   // Set the scalars to the point data
  //   polyData->GetPointData()->SetScalars(scalars);

  //   // Create a lookup table to map scalar values to colors
  //   vtkSmartPointer<vtkLookupTable> lut =
  //   vtkSmartPointer<vtkLookupTable>::New(); lut->SetNumberOfTableValues(3);
  //   lut->Build();
  //   lut->SetTableValue(0, 1.0, 0.0, 0.0, 1.0); // Red
  //   lut->SetTableValue(1, 0.0, 1.0, 0.0, 1.0); // Green
  //   lut->SetTableValue(2, 0.0, 0.0, 1.0, 1.0); // Blue

  //   // Mapper and actor
  //   vtkSmartPointer<vtkPolyDataMapper> mapper =
  //       vtkSmartPointer<vtkPolyDataMapper>::New();
  //   mapper->SetInputData(polyData);
  //   mapper->SetLookupTable(lut);
  //   mapper->SetScalarRange(1, 3);

  //   vtkSmartPointer<vtkActor> actor = vtkSmartPointer<vtkActor>::New();
  //   actor->SetMapper(mapper);

  //   // Renderer and render window
  //   vtkSmartPointer<vtkRenderer> renderer =
  //   vtkSmartPointer<vtkRenderer>::New(); vtkSmartPointer<vtkRenderWindow>
  //   renderWindow =
  //       vtkSmartPointer<vtkRenderWindow>::New();
  //   renderWindow->AddRenderer(renderer);
  //   vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
  //       vtkSmartPointer<vtkRenderWindowInteractor>::New();
  //   renderWindowInteractor->SetRenderWindow(renderWindow);

  //   // Add the actor to the renderer
  //   renderer->AddActor(actor);

  //   // Scalar bar to show color map
  //   vtkSmartPointer<vtkScalarBarActor> scalarBar =
  //       vtkSmartPointer<vtkScalarBarActor>::New();
  //   scalarBar->SetLookupTable(mapper->GetLookupTable());
  //   renderer->AddActor2D(scalarBar);

  //   // Background color and rendering
  //   renderer->SetBackground(0.1, 0.2, 0.3); // Dark blue background
  //   renderWindow->Render();
  //   renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

// int main3(int, char *[]) {
//   vtkNew<vtkConeSource> coneSource;
//   coneSource->Update();

//   vtkNew<vtkLineSource> lineSource;
//   lineSource->SetPoint1(coneSource->GetCenter());
//   lineSource->SetPoint2(coneSource->GetCenter()[0],
//                         coneSource->GetCenter()[1] - 1.0,
//                         coneSource->GetCenter()[2]);
//   lineSource->Update();

//   vtkNew<vtkPolyDataMapper> coneMapper;
//   coneMapper->SetInputConnection(coneSource->GetOutputPort());

//   vtkNew<vtkPolyDataMapper> lineMapper;
//   lineMapper->SetInputConnection(lineSource->GetOutputPort());

//   // Set offset parameters to avoid z-fighting
//   // ActiveMapper settings
//   coneMapper->SetRelativeCoincidentTopologyLineOffsetParameters(0, -66000);
//   coneMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(0,
//   -66000);
//   coneMapper->SetRelativeCoincidentTopologyPointOffsetParameter(-66000);

//   // LinesMapper settings
//   lineMapper->SetRelativeCoincidentTopologyLineOffsetParameters(-1, -1);
//   lineMapper->SetRelativeCoincidentTopologyPolygonOffsetParameters(-1, -1);
//   lineMapper->SetRelativeCoincidentTopologyPointOffsetParameter(-1);

//   vtkNew<vtkActor> coneActor;
//   coneActor->SetMapper(coneMapper);

//   vtkNew<vtkActor> lineActor;
//   lineActor->SetMapper(lineMapper);
//   lineActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // Red color for
//   visibility

//   vtkNew<vtkRenderer> renderer;
//   renderer->AddActor(coneActor);
//   renderer->AddActor(lineActor);
//   renderer->SetBackground(0.1, 0.2, 0.4); // Dark blue background

//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetSize(300, 300);
//   renderWindow->Render();

//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   vtkNew<vtkCamera> camera;
//   camera->SetPosition(0, -1, 0);
//   camera->SetFocalPoint(0, 0, 0);
//   renderer->SetActiveCamera(camera);
//   renderer->ResetCamera();

//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

int mainTranslucent(int, char *[]) {
  // Create a cube
  vtkNew<vtkCubeSource> cubeSource;
  cubeSource->SetXLength(5.0);
  cubeSource->SetYLength(5.0);
  cubeSource->SetZLength(5.0);

  vtkNew<vtkPolyDataMapper> cubeMapper;
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());

  vtkNew<vtkActor> cubeActor;
  cubeActor->SetForceTranslucent(true);
  cubeActor->SetMapper(cubeMapper);

  // Create a billboard text actor
  vtkNew<vtkBillboardTextActor3D> textActor;
  textActor->SetInput("Hello VTK!");
  textActor->SetPosition(0, 0, 3); // Positioning the text above the cube
  textActor->GetTextProperty()->SetFontSize(24);
  textActor->GetTextProperty()->SetColor(1.0, 0.0, 0.0); // Red color

  // Renderer
  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(cubeActor);
  renderer->AddActor(textActor);
  renderer->SetBackground(0.1, 0.2, 0.3); // Dark blue background

  // Render Window
  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);
  renderWindow->SetWindowName("Cube with Translucent Text");

  // Interactor
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Initialize the interactor and start the rendering loop
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

int notworkinmain(int, char *[]) {
  vtkNew<vtkNamedColors> colors;

  // Create the main sphere
  vtkNew<vtkSphereSource> sphereSource;

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(sphereSource->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);
  actor->GetProperty()->SetColor(colors->GetColor3d("Banana").GetData());

  // Setup renderer
  vtkNew<vtkRenderer> renderer;
  renderer->SetBackground(colors->GetColor3d("SlateGray").GetData());

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->SetSize(600, 600);
  renderWindow->AddRenderer(renderer);

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);

  // Create overlay sphere
  vtkNew<vtkSphereSource> overlaySphereSource;
  overlaySphereSource->SetCenter(0, 0, 1);
  overlaySphereSource->SetRadius(0.5);

  vtkNew<vtkPolyDataMapper> overlayMapper;
  overlayMapper->SetInputConnection(overlaySphereSource->GetOutputPort());

  vtkNew<vtkActor> overlayActor;
  overlayActor->SetMapper(overlayMapper);
  overlayActor->GetProperty()->SetColor(colors->GetColor3d("Tomato").GetData());

  renderer->AddActor(overlayActor);

  // Setup the sequence of passes
  vtkNew<vtkRenderPassCollection> passes;
  vtkNew<vtkOverlayPass> overlayPass;
  passes->AddItem(renderer->GetPass());
  passes->AddItem(overlayPass);

  vtkNew<vtkSequencePass> sequencePass;
  sequencePass->SetPasses(passes);

  vtkNew<vtkCameraPass> cameraPass;
  cameraPass->SetDelegatePass(sequencePass);

  // Applying the camera pass to the renderer
  renderer->SetPass(cameraPass);

  // Start the rendering loop
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}

int mainfoo(int argc, char *argv[]) {

  vtkSmartPointer<vtkJPEGReader> reader = vtkSmartPointer<vtkJPEGReader>::New();
  if (argc > 1) {
    reader->SetFileName(argv[1]); // Command line argument for image path
  } else {
    reader->SetFileName(
        "/home/behnam/workspace/vtk_projects/images/HQ.jpg"); // Default image
                                                              // path
  }
  reader->Update();

  vtkSmartPointer<vtkTexture> texture = vtkSmartPointer<vtkTexture>::New();
  texture->SetInputConnection(reader->GetOutputPort());

  vtkSmartPointer<vtkPlaneSource> plane =
      vtkSmartPointer<vtkPlaneSource>::New();
  plane->SetCenter(0.0, 0.0, 0.0);
  plane->SetNormal(0.0, 0.0, 1.0);

  vtkSmartPointer<vtkPolyDataMapper> planeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  planeMapper->SetInputConnection(plane->GetOutputPort());

  vtkSmartPointer<vtkFollower> planeActor = vtkSmartPointer<vtkFollower>::New();
  planeActor->SetMapper(planeMapper);
  planeActor->SetTexture(texture);

  vtkSmartPointer<vtkCubeSource> cubeSource =
      vtkSmartPointer<vtkCubeSource>::New();
  cubeSource->SetXLength(1.0);
  cubeSource->SetYLength(1.0);
  cubeSource->SetZLength(1.0);

  vtkSmartPointer<vtkPolyDataMapper> cubeMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  cubeMapper->SetInputConnection(cubeSource->GetOutputPort());

  vtkSmartPointer<vtkActor> cubeActor = vtkSmartPointer<vtkActor>::New();
  cubeActor->SetMapper(cubeMapper);
  cubeActor->SetPosition(1.5, 0.0, -5.0);

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(planeActor);
  renderer->AddActor(cubeActor);
  renderer->SetBackground(0.1, 0.2, 0.4);

  planeActor->SetCamera(renderer->GetActiveCamera());

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}