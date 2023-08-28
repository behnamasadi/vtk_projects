import QtQuick 2.12
import QtQuick.Controls 2.12
import QtQuick.Window 2.2
import VTKBackend 1.0

Window {
  id: win
  visible: true
  width: 640
  height: 640
  title: qsTr("Hello World")
   Rectangle {
    anchors.fill: parent
    color: "darkseagreen"
  }

  Rectangle {
    id: r
    border { width: 5; color: "steelblue" }
    radius: 5
    color: "hotpink"
    anchors.fill: parent
    anchors.margins: 100
    VTKBackend {
      id: vtk
      anchors.fill: parent
      focus: hoverHandler.hovered ? true : false
      HoverHandler {
          id: hoverHandler
      }
    }
  }
  Button
  {
      text: "click me"
      onClicked: {

            console.log("foo")
            vtk.resetCamera()
        }
  }
 }


