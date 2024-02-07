// #include <vtkActor.h>
// #include <vtkCamera.h>
// #include <vtkNamedColors.h>
// #include <vtkNew.h>
// #include <vtkPDALReader.h>
// #include <vtkPointGaussianMapper.h>
// #include <vtkPolyDataMapper.h>
// #include <vtkProperty.h>
// #include <vtkRenderWindow.h>
// #include <vtkRenderWindowInteractor.h>
// #include <vtkRenderer.h>

// int main(int argc, char *argv[]) {
//   if (argc < 2) {
//     std::cerr << "Usage: " << argv[0] << " <Path to LAS file>" << std::endl;
//     return EXIT_FAILURE;
//   }

//   std::string inputFilename = argv[1];

//   vtkNew<vtkPDALReader> reader;
//   reader->SetFileName(inputFilename.c_str());
//   reader->Update();

//   vtkNew<vtkPointGaussianMapper> mapper;
//   mapper->SetInputConnection(reader->GetOutputPort());

//   vtkNew<vtkActor> actor;
//   actor->SetMapper(mapper);

//   vtkNew<vtkNamedColors> colors;

//   vtkNew<vtkRenderer> renderer;
//   renderer->AddActor(actor);
//   renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

//   vtkNew<vtkRenderWindow> renderWindow;
//   renderWindow->AddRenderer(renderer);
//   renderWindow->SetSize(800, 600);
//   renderWindow->SetWindowName("VTK PDAL Reader Example");

//   vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
//   renderWindowInteractor->SetRenderWindow(renderWindow);

//   renderWindow->Render();
//   renderWindowInteractor->Start();

//   return EXIT_SUCCESS;
// }

#include <vtkActor.h>
#include <vtkCamera.h>
#include <vtkNamedColors.h>
#include <vtkNew.h>
#include <vtkPDALReader.h>
#include <vtkPolyDataMapper.h>
#include <vtkProperty.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkVertexGlyphFilter.h>

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "Usage: " << argv[0] << " <Path to LAS file>" << std::endl;
    return EXIT_FAILURE;
  }

  std::string inputFilename = argv[1];

  vtkNew<vtkPDALReader> reader;
  reader->SetFileName(inputFilename.c_str());
  reader->Update();

  vtkNew<vtkVertexGlyphFilter> glyphFilter;
  glyphFilter->SetInputConnection(reader->GetOutputPort());
  glyphFilter->Update();

  vtkNew<vtkPolyDataMapper> mapper;
  mapper->SetInputConnection(glyphFilter->GetOutputPort());

  vtkNew<vtkActor> actor;
  actor->SetMapper(mapper);

  vtkNew<vtkNamedColors> colors;

  vtkNew<vtkRenderer> renderer;
  renderer->AddActor(actor);
  renderer->SetBackground(colors->GetColor3d("MidnightBlue").GetData());

  vtkNew<vtkRenderWindow> renderWindow;
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);
  renderWindow->SetWindowName("VTK PDAL Reader - Display Points");

  vtkNew<vtkRenderWindowInteractor> renderWindowInteractor;
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
