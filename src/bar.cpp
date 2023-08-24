#include <vector>
#include <vtkAngleWidget.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkSmartPointer.h>

int main() {
  vtkSmartPointer<vtkRenderWindowInteractor> interactor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();

  // Create multiple angle widgets
  std::vector<vtkSmartPointer<vtkAngleWidget>> angleWidgets;
  for (int i = 0; i < 3; ++i) { // example: create 3 angle widgets
    vtkSmartPointer<vtkAngleWidget> angleWidget =
        vtkSmartPointer<vtkAngleWidget>::New();
    angleWidget->SetInteractor(interactor);
    angleWidgets.push_back(angleWidget);
  }

  // Now, suppose you want to remove them one by one
  for (auto &widget : angleWidgets) {
    widget->Off(); // disable the widget
  }

  // If you want to completely clear them from memory (assuming you're using
  // vtkSmartPointer)
  angleWidgets.clear();

  return 0;
}
