import QtQuick 2.12
import QtQuick.Window 2.2
import com.vtk.example 1.0
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.15


Window {
    id: win
    visible: true
    width: 640
    height: 640
    title: qsTr("Hello World")

    ColumnLayout {
        anchors.fill: parent

        RowLayout {
            Layout.fillWidth: true

            Button {
                text: "decimate"
                onClicked: {
                    console.log("foo")
                    // vtk.resetCamera()
                }
            }

            Slider {
                id: slider
                from: 1
                to: 100
                value: 20
                stepSize: 1
                Layout.fillWidth: true
                onValueChanged: label.text = String(value) // Update label on value change
            }

            Label {
                id: label
                text: String(slider.value) // Initial value
                Layout.alignment: Qt.AlignHCenter
                Layout.preferredWidth: 100 // Set the preferred width for layout
                Layout.preferredHeight: 50 // Set the preferred height for layout
                width: 100 // Fixed width
                height: 50 // Fixed height
                wrapMode: Label.Wrap // Optional, enables text wrapping
            }
        }


        Rectangle {
            id: r
            border { width: 5; color: "steelblue" }
            radius: 5
            color: "hotpink"
            Layout.fillWidth: true
            Layout.fillHeight: true
            Layout.margins: 100 // Use Layout.margins instead of anchors.margins
            MyVtkItem {
                id: vtk
                anchors.fill: parent
                focus: hoverHandler.hovered ? true : false
                HoverHandler {
                    id: hoverHandler
                }
            }
        }


    }
}