#include <cmath>
#include <vtkActor.h>
#include <vtkCellArray.h>
#include <vtkInteractorStyleTrackballCamera.h>
#include <vtkLODActor.h>
#include <vtkMapperCollection.h>
#include <vtkMaskPoints.h>
#include <vtkNamedColors.h>
#include <vtkOutlineFilter.h>
#include <vtkPointSource.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkSphereSource.h>
#include <vtkTriangle.h>

int main(int argc, char *argv[]) {
  // Create a sphere source as the normal data
  vtkSmartPointer<vtkSphereSource> sphereSource =
      vtkSmartPointer<vtkSphereSource>::New();
  sphereSource->SetThetaResolution(800);
  sphereSource->SetPhiResolution(800);
  sphereSource->Update();

  vtkNew<vtkNamedColors> colors;

  //   vtkNew<vtkPointSource> highResPointSource;
  //   highResPointSource->SetNumberOfPoints(50000000);

  // Create a mapper for the normal data
  vtkSmartPointer<vtkPolyDataMapper> normalMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  normalMapper->SetInputConnection(sphereSource->GetOutputPort());
  //   normalMapper->SetInputConnection(highResPointSource->GetOutputPort());

  // Create the outline filter for the lowest level of detail
  vtkSmartPointer<vtkOutlineFilter> outlineFilter =
      vtkSmartPointer<vtkOutlineFilter>::New();
  outlineFilter->SetInputConnection(sphereSource->GetOutputPort());

  //     vtkNew<vtkPolyDataMapper> lowResMapper;
  //   lowResMapper->SetInputConnection(lowResPointSource->GetOutputPort());

  // Create a mapper for the outline
  vtkSmartPointer<vtkPolyDataMapper> outlineMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  outlineMapper->SetInputConnection(outlineFilter->GetOutputPort());

  // Create the mask points filter for the middle level of detail
  vtkSmartPointer<vtkMaskPoints> maskPoints =
      vtkSmartPointer<vtkMaskPoints>::New();
  maskPoints->SetInputConnection(sphereSource->GetOutputPort());
  int n = 10;
  maskPoints->SetOnRatio(n); // every n'th point
  maskPoints->RandomModeOn();

  // Create a mapper for the point cloud
  vtkSmartPointer<vtkPolyDataMapper> pointCloudMapper =
      vtkSmartPointer<vtkPolyDataMapper>::New();
  pointCloudMapper->SetInputConnection(maskPoints->GetOutputPort());

  // Create the LOD actor and add the mappers
  vtkSmartPointer<vtkLODActor> lodActor = vtkSmartPointer<vtkLODActor>::New();
  lodActor->SetMapper(normalMapper);        // High resolution
  lodActor->AddLODMapper(pointCloudMapper); // Medium resolution
  lodActor->AddLODMapper(outlineMapper);    // Low resolution

  lodActor->GetProperty()->SetRepresentationToSurface();
  lodActor->GetProperty()->SetInterpolationToGouraud();
  lodActor->GetProperty()->SetAmbient(0.1);
  lodActor->GetProperty()->SetDiffuse(0.7);
  lodActor->GetProperty()->SetSpecular(0.5);
  lodActor->GetProperty()->SetSpecularPower(80);
  lodActor->GetProperty()->SetSpecularColor(
      colors->GetColor3d("White").GetData());

  // Set the colors for the different levels of detail
  lodActor->GetProperty()->SetColor(1.0, 0.0, 0.0); // High resolution: Red

  // ostream &os, vtkIndent indent
  lodActor->GetLODMappers()->DebugOn();
  std::cout << "lodActor->GetLODMappers()->GetNumberOfItems(): "
            << lodActor->GetLODMappers()->GetNumberOfItems() << std::endl;

  lodActor->GetLODMappers()->PrintSelf(std::cout, vtkIndent());

  //   for (std::size_t i = 0; i <
  //   lodActor->GetLODMappers()->GetNumberOfItems();
  //        i++) {
  //     lodActor->GetLODMappers().   .get[i].PrintSelf(std::cout, vtkIndent());
  //   }

  vtkObjectBase *obj = nullptr;

  vtkMapperCollection *mappers = lodActor->GetLODMappers();

  for (mappers->InitTraversal(); (obj = mappers->GetNextItem());) {
    vtkMapper *mapper = vtkMapper::SafeDownCast(obj);
    if (mapper) {
      // Do something with the mapper
      // std::cout <<  << std::endl; // Example
      mapper->PrintSelf(std::cout, vtkIndent());
      std::cout << "----------------------" << std::endl; // Example
    }
  }

  //   lodActor->GetLODProperty(0)->SetColor(1.0, 0.0, 0.0); // High resolution:
  //   Red lodActor->GetLODProperty(1)->SetColor(0.0, 1.0,
  //                                         0.0); // Medium resolution: Green
  //   lodActor->GetLODProperty(2)->SetColor(0.0, 0.0, 1.0); // Low resolution:
  //   Blue

  // Set up the renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the LOD actor to the scene
  renderer->AddActor(lodActor);
  renderer->SetBackground(0.1, 0.2, 0.4); // Background color

  // Control the frame rate
  renderWindowInteractor->SetDesiredUpdateRate(15.0);
  renderWindowInteractor->SetStillUpdateRate(0.05);

  // Start the interaction
  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
