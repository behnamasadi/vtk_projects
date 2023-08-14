import QtQuick 2.12
import QtQuick.Window 2.12
import QtQuick.Controls 2.12
import QtQuick.Layouts 1.12


/*
Window {
    width: 640
    height: 480
    visible: true
    title: qsTr("Hello World")


    Rectangle
    {
        id:rect1;
        width: 100;
        height:50;
        color: "red";
        anchors.centerIn: parent;
    }

    Rectangle
    {
        id:rect2;
        width: 100;
        height:100;
        color: "blue";
        anchors.top: rect1.bottom ;
        anchors.left: rect1.right ;
        radius: 5
    }



    onWidthChanged:
    {
           // Will be executed after window.width value changes.
        back_qml.changeValue(1);
        btn1.scale=btn1.scale/ 1.3
    }

    onHeightChanged:
    {
           // Will be executed after window.height value changes.
           back_qml.changeValue(2);

           btn1.text  = "xx"
           btn1.scale=btn1.scale* 1.3
    }

    Label
    {
        id:label
        x:22
        y:88
        width:355
        height:62
        text: qsTr("lable")

        Connections
        {
            target: back_qml
            onValueChanged: label.text=s;
        }
    }


    Button {

        id: btn1
        x: 47
        y: 229
        text: "btn1"
        scale: 1.3
        onClicked: back_qml.changeValue(1);
    }
    Button {
        id: btn2
        x: 163
        y: 229
        text: "btn2"
        scale: 1.3
        onClicked: back_qml.changeValue(2);
    }

    Button {
        id: btn3
        x: 163
        y: 329
        text: "btn3"
        width: btn1.width+btn2.width
        onClicked: back_qml.changeValue(2);
    }

}
*/


/*


ApplicationWindow is a Window that adds convenience for positioning items, such as MenuBar, ToolBar, and StatusBar
 in a platform independent manner.
That is, it is an item that inherits from Window but has certain default attributes, it is similar to
QMainWindow with respect to QWidget.
*/
ApplicationWindow {
    width: 800
    height: 600
    visible: true
    title: qsTr("foo")

    //signal clicked

    //color: tapHandler.pressed ? "goldenrod" : hoverHandler.hovered ? "wheat" : "beige"
    Text {
        id: txt1
        text: qsTr("txt1")
        x: 400
        y: 400

        MouseArea {
            anchors.fill: parent // the defaul size is 0x0 so we have to set fill parnet
            onPressed: parent.color = "red"
            onReleased: parent.color = "green"
            onClicked: txt1.font.bold = !txt1.font.bold
            acceptedButtons: Qt.RightButton
        }
    }

    Text {
        anchors.top: txt1.bottom
        anchors.right: txt1.right
        id: txt2
        text: qsTr("txt2")
        //property binidng, if txt2MouseArea.pressed is rtue
        color: txt2MouseArea.pressed ? "green" : "red"

        //color: txt2MouseArea.clicked ? "green": "red"
        MouseArea {
            id: txt2MouseArea
            anchors.fill: parent
        }

        width: 100
        height: 2 * width
    }

    menuBar: MenuBar {
        Menu {
            title: qsTr("File")
            MenuItem {
                text: qsTr("Open")
                onTriggered: console.log("foo foo")
            }
            MenuItem {
                text: qsTr("close")
                onTriggered: Qt.quit()
            }
        }

        Menu {
            title: qsTr("Edit")
        }
    }

    Button {
        id: btn1
        text: qsTr("click me")
        anchors.horizontalCenter: parent.horizontalCenter
        anchors.verticalCenter: parent.verticalCenter
        width: txtInput1.width
        //height: txtInput1.height;
    }

    ColumnLayout {
        spacing: 0
        Rectangle {
            id: rect1
            color: "red"
            width: 100
            height: 100
        }
        Rectangle {
            id: rect2
            color: "blue"
            width: 100
            height: 100
        }

        TapHandler {
            onTapped: switch (point.modifiers) {
                      case Qt.ControlModifier | Qt.AltModifier:
                          console.log("CTRL+ALT")
                          break
                      case Qt.ControlModifier | Qt.AltModifier | Qt.MetaModifier:
                          console.log("CTRL+META+ALT")
                          break
                      default:
                          console.log("other modifiers", point.modifiers)
                          break
                      }
        }
    }

    Item {

        anchors.top: btn1.bottom
        anchors.right: btn1.right
        TextInput {
            id: txtInput1

            text: "complete me"
        }
    }
}
