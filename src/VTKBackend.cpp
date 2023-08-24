#include "VTKBackend.hpp"

VTKBackend::VTKBackend() {
  // Assuming you initialize your VTK components somewhere here
  this->m_vtkRenderWindowInteractor =
      vtkSmartPointer<vtkRenderWindowInteractor>::New();
  // ... Other VTK initialization...
}

void VTKBackend::handleMouseClick(int x, int y) {
  if (!this->m_vtkRenderWindowInteractor) {
    return;
  }
  this->m_vtkRenderWindowInteractor->InvokeEvent(
      vtkCommand::LeftButtonPressEvent, nullptr);
  this->m_vtkRenderWindowInteractor->InvokeEvent(
      vtkCommand::LeftButtonReleaseEvent, nullptr);
}
