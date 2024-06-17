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

int main1(int, char *[]) {

  // Create a vtkPointData object
  vtkPointData *pointData = vtkPointData::New();

  // Create a vtkDoubleArray to store temperature data at each point
  vtkDoubleArray *temperatureArray = vtkDoubleArray::New();
  temperatureArray->SetName("Temperature");
  temperatureArray->SetNumberOfComponents(1); // One component for temperature

  // Add some temperature values to the array
  temperatureArray->InsertNextValue(20.0); // Temperature at first point
  temperatureArray->InsertNextValue(25.0); // Temperature at second point
  temperatureArray->InsertNextValue(18.0); // Temperature at third point

  // Associate the temperature array with the point data
  pointData->AddArray(temperatureArray);

  // Clean up (not strictly necessary in this simple example)
  temperatureArray->Delete();
  pointData->Delete();

  return EXIT_SUCCESS;
}

int main(int, char *[]) {

  vtkNew<vtkPoints> points;

  /*

  Y----------------------▸
  |
  |
  |
  |
  |
  |
  ▾
  0                             2
  (0, 0, 0)                     (0, 1, 0)



  1                             3
  (1, 0, 0)                     (1, 0, 0)

  */

  points->InsertNextPoint(0, 0, 0);
  points->InsertNextPoint(1, 0, 0);
  points->InsertNextPoint(0, 1, 0);
  points->InsertNextPoint(1, 1, 0);

  vtkNew<vtkTriangle> triangle1;

  triangle1->GetPointIds()->SetId(0, 0);
  triangle1->GetPointIds()->SetId(1, 1);
  triangle1->GetPointIds()->SetId(2, 3);

  vtkNew<vtkTriangle> triangle2;
  triangle2->GetPointIds()->SetId(0, 0);
  triangle2->GetPointIds()->SetId(1, 2);
  triangle2->GetPointIds()->SetId(2, 3);

  vtkNew<vtkCellArray> triangles;
  triangles->InsertNextCell(triangle1);
  triangles->InsertNextCell(triangle2);

  vtkNew<vtkPolyData> polyData;
  polyData->SetPoints(points);
  polyData->SetPolys(triangles);

  std::cout << "There are " << polyData->GetNumberOfCells() << " cells."
            << std::endl;

  std::cout << "There are " << polyData->GetNumberOfPoints() << " points."
            << std::endl;

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputData(polyData);

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkRenderer> renderer;
  vtkNew<vtkRenderWindow> renderWindow;

  renderWindow->AddRenderer(renderer);
  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;

  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderer->AddActor(actor);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
