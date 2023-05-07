/*
 * Copyright (c) 2020-2023 Alex Spataru <https://github.com/alex-spataru>
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */
import QtQuick
import QtQuick.Window
import QtQuick.Layouts
import QtQuick.Controls
import "../Widgets" as Widgets

Control {
    id: root

    // Dummy string to increase width of buttons
    readonly property string _btSpacer: "  "
    property alias consoleChecked: consoleBt.checked
    property alias dashboardChecked: dashboardBt.checked
    property alias ros2WsChecked: ros2Bt.checked
    // Aliases to button check status
    property alias setupChecked: setupBt.checked
    // Reference to parent window to be able to drag it with the toolbar
    property Window window

    signal consoleClicked
    signal dashboardClicked
    signal projectEditorClicked
    signal ros2WsClicked
    // Custom signals
    signal setupClicked

    // Connections with mac touchbar
    Connections {
        function onConsoleClicked() {
            consoleBt.clicked();
            Cpp_Misc_MacExtras.setConsoleChecked(consoleBt.checked);
        }
        function onDashboardClicked() {
            dashboardBt.clicked();
            Cpp_Misc_MacExtras.setDashboardChecked(dashboardBt.checked);
        }
        function onSetupClicked() {
            setupBt.clicked();
            Cpp_Misc_MacExtras.setSetupChecked(setupBt.checked);
        }

        target: Cpp_Misc_MacExtras
    }
    // Toolbar shadow
    Widgets.Shadow {
        anchors.fill: bg
    }
    // Background gradient + border
    Rectangle {
        id: bg
        anchors.fill: parent

        gradient: Gradient {
            GradientStop {
                color: Cpp_ThemeManager.toolbarGradient1
                position: 0
            }
            GradientStop {
                color: Cpp_ThemeManager.toolbarGradient2
                position: 1
            }
        }

        Rectangle {
            anchors.fill: parent
            border.color: Qt.darker(Cpp_ThemeManager.toolbarGradient2, 1.5)
            border.width: 1
            color: "transparent"
            visible: Cpp_ThemeManager.titlebarSeparator
        }
        Rectangle {
            color: Qt.darker(Cpp_ThemeManager.toolbarGradient1, 1.5)
            height: 1
            visible: Cpp_ThemeManager.titlebarSeparator

            anchors {
                bottom: parent.bottom
                left: parent.left
                right: parent.right
            }
        }
    }
    // Toolbar icons
    RowLayout {
        anchors.fill: parent
        anchors.margins: app.spacing
        spacing: app.spacing

        Button {
            id: setupBt
            Layout.fillHeight: true
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/settings.svg"
            icon.width: 24
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("Setup") + _btSpacer

            background: Rectangle {
                border.color: "#040600"
                border.width: 1
                color: "transparent"
                opacity: parent.checked ? 0.2 : 0.0
                radius: 3

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    border.color: "#c2c2c2"
                    border.width: 1
                    color: "#524545"
                    radius: parent.radius - 1
                }
            }

            onCheckedChanged: Cpp_Misc_MacExtras.setSetupChecked(checked)
            onClicked: root.setupClicked()
        }

        Button {
            id: consoleBt
            Layout.fillHeight: true
            // enabled: dashboardBt.enabled
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/code.svg"
            icon.width: 24
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("Console") + _btSpacer

            background: Rectangle {
                border.color: "#040600"
                border.width: 1
                color: "transparent"
                opacity: parent.checked ? 0.2 : 0.0
                radius: 3

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    border.color: "#c2c2c2"
                    border.width: 1
                    color: "#626262"
                    radius: parent.radius - 1
                }
            }

            onCheckedChanged: Cpp_Misc_MacExtras.setConsoleChecked(checked)
            onClicked: root.consoleClicked()
        }

        Button {
            id: ros2Bt
            Layout.fillHeight: true
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/group.svg"
            icon.width: 24
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("ROS2") + _btSpacer

            background: Rectangle {
                border.color: "#63968d"
                border.width: 1
                color: "transparent"
                opacity: parent.checked ? 0.2 : 0.0
                radius: 3

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    border.color: "#c2c2c2"
                    border.width: 1
                    color: "#524545"
                    radius: parent.radius - 1
                }
            }

            // onCheckedChanged: Cpp_Misc_MacExtras.setSetupChecked(checked)
            onClicked: root.ros2WsClicked()
        }

        Button {
            id: dashboardBt
            Layout.fillHeight: true
            // enabled: Cpp_UI_Dashboard.available
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/dashboard.svg"
            icon.width: 24
            opacity: enabled ? 1 : 0.5
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("Dashboard") + _btSpacer

            background: Rectangle {
                border.color: "#040600"
                border.width: 1
                color: "transparent"
                opacity: parent.checked ? 0.2 : 0.0
                radius: 3

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    border.color: "#c2c2c2"
                    border.width: 1
                    color: "#626262"
                    radius: parent.radius - 1
                }
            }

            onCheckedChanged: Cpp_Misc_MacExtras.setDashboardChecked(checked)
            onClicked: root.dashboardClicked()
            onEnabledChanged: Cpp_Misc_MacExtras.setDashboardEnabled(enabled)
        }
        // Window drag handler
        Item {
            Layout.fillWidth: true
            height: parent.height

            MouseArea {
                anchors.fill: parent

                onPressedChanged: {
                    if (pressed)
                        window.startSystemMove();
                }
            }
        }
        Button {
            Layout.fillHeight: true
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/json.svg"
            icon.width: 24
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("Project Editor") + _btSpacer

            background: Item {
            }

            onClicked: root.projectEditorClicked()
        }
        Button {
            Layout.fillHeight: true
            enabled: !Cpp_CSV_Player.isOpen
            flat: true
            icon.color: Cpp_ThemeManager.menubarText
            icon.height: 24
            icon.source: "qrc:/icons/open.svg"
            icon.width: 24
            opacity: enabled ? 1 : 0.5
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: Cpp_ThemeManager.menubarText
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: qsTr("Open CSV") + _btSpacer

            background: Item {
            }

            onClicked: {
                if (Cpp_CSV_Export.isOpen)
                    Cpp_CSV_Export.openCurrentCsv();
                else
                    Cpp_CSV_Player.openFile();
            }
        }

        // connect button
        Button {
            id: connectBt
            Layout.fillHeight: true
            // Connection-dependent
            checked: Cpp_IO_Manager.connected
            enabled: Cpp_IO_Manager.configurationOk
            // Button properties
            flat: true
            font.bold: true
            icon.color: checked ? Cpp_ThemeManager.connectButtonChecked : Cpp_ThemeManager.connectButtonUnchecked
            icon.height: 24
            icon.source: checked ? "qrc:/icons/disconnect.svg" : "qrc:/icons/connect.svg"
            icon.width: 24
            // Only enable button if it can be clicked
            opacity: enabled ? 1 : 0.5
            palette.button: Cpp_ThemeManager.toolbarGradient1
            palette.buttonText: checked ? Cpp_ThemeManager.connectButtonChecked : Cpp_ThemeManager.connectButtonUnchecked
            palette.window: Cpp_ThemeManager.toolbarGradient1
            text: (checked ? qsTr("Disconnect") : qsTr("Connect")) + _btSpacer

            // Custom button background
            background: Rectangle {
                border.color: "#040600"
                border.width: 1
                color: "transparent"
                opacity: parent.checked ? 0.2 : 0.0
                radius: 3

                Rectangle {
                    anchors.fill: parent
                    anchors.margins: parent.border.width
                    border.color: "#c2c2c2"
                    border.width: 1
                    color: "#626262"
                    radius: parent.radius - 1
                }
            }

            // Connect/disconnect device when button is clicked
            onClicked: Cpp_IO_Manager.toggleConnection()
        }
    }
}
