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
  width: 400
  height: 400
  color: palette.window
 
  SystemPalette {
    id: palette
    colorGroup: SystemPalette.Active
  }

  Column{

    Button
    {
        text: "click me"
    }
 
  // Instantiate the vtk render window
  VTKRenderWindow {
    id: vtkwindow
    width: 400
    height: 400
  }
 }
  // add one or more vtk render items
  VTKRenderItem {
    objectName: "ConeView"
    id: coneView
    x: 200
    y: 200
    width: 200
    height: 200
    // Provide the handle to the render window
    focus: true
    renderWindow: vtkwindow
    //onEntered:{console.log("entered: ", coneView.focus)   }
 


        // MouseArea {
        //     anchors.fill: parent
        //     propagateComposedEvents: true
        //     // onClicked: (mouse)=> {
        //     //     console.log("clicked blue")
        //     //     mouse.accepted = false
        //     // }
        // }

    MouseArea {
        enabled : true
        hoverEnabled: true
        anchors.fill: parent
        propagateComposedEvents: true
        onEntered:
        {
          parent.focus= true;
          console.log("Entered: ", coneView.focus); 
        }

        onPressAndHold:
        {
          console.log("onPressAndHold: "); 
        }

        onClicked: (mouse)=>
        {
          parent.focus= true;
          console.log("Clicked: ", coneView.focus);
          if (mouse.button == Qt.RightButton)
                
                console.log("RightButton");
            else
                
                console.log("not RightButton ");
        }
    }
  }


}
