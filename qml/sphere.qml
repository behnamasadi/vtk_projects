// import related modules
import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Window 2.12
 
// import the VTK module
import VTK 9.2
 
// window containing the application
ApplicationWindow {
  // title of the application
  title: qsTr("VTK QtQuick App")
  width: 600
  height: 600
  color: palette.window
 
  SystemPalette {
    id: palette
    colorGroup: SystemPalette.Active
  }


 
  // Instantiate the vtk render window
  VTKRenderWindow {
    id: vtkwindow
    width: 600
    height: 600
  }
 
  // add one or more vtk render items
  VTKRenderItem {
    objectName: "View"
    x: 0
    y: 0
    width: 600
    height: 600
    // Provide the handle to the render window
    renderWindow: vtkwindow
  }


}
