#include <vtkActor.h>
#include <vtkCaptionActor2D.h>
#include <vtkImageActor.h>
#include <vtkImageMapper3D.h>
#include <vtkJPEGReader.h>
#include <vtkProp3DFollower.h>
#include <vtkRenderWindow.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkRenderer.h>
#include <vtkSmartPointer.h>
#include <vtkTextProperty.h>

int main(int argc, char *argv[]) {
  vtkSmartPointer<vtkJPEGReader> imageReader =
      vtkSmartPointer<vtkJPEGReader>::New();
  if (argc > 1) {
    imageReader->SetFileName(argv[1]);
  } else {
    imageReader->SetFileName("/home/behnam/workspace/vtk_projects/images/"
                             "HQ.jpg");
  }
  imageReader->Update();

  vtkSmartPointer<vtkImageActor> imageActor =
      vtkSmartPointer<vtkImageActor>::New();
  imageActor->GetMapper()->SetInputConnection(imageReader->GetOutputPort());

  vtkSmartPointer<vtkCaptionActor2D> captionActor =
      vtkSmartPointer<vtkCaptionActor2D>::New();
  captionActor->SetCaption("Your Caption Here");
  //   captionActor->SetAttachmentPoint(150, 150, 0);
  captionActor->GetCaptionTextProperty()->SetColor(1.0, 1.0, 1.0);
  captionActor->GetCaptionTextProperty()->SetFontSize(12);
  captionActor->GetCaptionTextProperty()->BoldOn();
  captionActor->GetCaptionTextProperty()->ItalicOn();
  captionActor->GetCaptionTextProperty()->ShadowOn();
  captionActor->GetCaptionTextProperty()->SetFontFamilyToArial();

  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  renderer->AddActor(imageActor);
  renderer->AddActor(captionActor);

  vtkSmartPointer<vtkRenderWindow> renderWindow =
      vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  renderWindow->SetSize(800, 600);

  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  renderWindow->Render();
  renderWindowInteractor->Initialize();
  renderWindowInteractor->Start();

  return EXIT_SUCCESS;
}
