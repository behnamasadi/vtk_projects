import QtQuick 2.12
import QtQuick.Controls 2.12
import VTKBackend 1.0

ApplicationWindow {
    visible: true
    width: 800
    height: 600

    VTKBackend {
        id: vtkBackend
    }

    QQuickVTKItem {
        id: vtkItem
        anchors.fill: parent

        MouseArea {
            anchors.fill: parent
            onClicked: {
                vtkBackend.handleMouseClick(mouse.x, mouse.y)
            }
        }
    }
}

