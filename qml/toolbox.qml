import QtQuick 2.12
import QtQuick.Controls 2.12

ApplicationWindow {
    visible: true
    width: 640
    height: 480
    title: "QML Toolbox"

    Rectangle {
        id: panel
        width: 200
        height: parent.height
        color: "#333333"

        Column {
            spacing: 10
            anchors.fill: parent
            anchors.margins: 20

            Button {
                text: "Button 1"
                // Add your action for the button here
                onClicked: {
                    console.log("Button 1 clicked")
                }
            }

            Button {
                text: "Button 2"
                // Add your action for the button here
                onClicked: {
                    console.log("Button 2 clicked")
                }
            }

            Button {
                text: "Button 3"
                // Add your action for the button here
                onClicked: {
                    console.log("Button 3 clicked")
                }
            }
        }
    }
}
