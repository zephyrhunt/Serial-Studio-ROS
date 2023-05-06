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
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings
import "../Widgets" as Widgets
import "SetupPanes" as SetupPanes

Item {
    id: root

    property int displayedWidth: 380 + app.spacing * 1.5
    readonly property int maxItemWidth: column.width - 2 * spacing
    // Custom properties
    property int setupMargin: 0

    // Hides the setup panel
    function hide() {
        setupMargin = -1 * displayedWidth;
    }
    // Displays the setup panel
    function show() {
        setupMargin = 0;
    }

    // Animations
    visible: setupMargin > -1 * displayedWidth

    Behavior on setupMargin  {
        NumberAnimation {
        }
    }

    // Save settings
    Settings {
        // Misc settings
        property alias auto: commAuto.checked
        property alias csvExport: csvLogging.checked


        // //
        // // App settings
        // //
        // property alias language: settings.language
        // property alias tcpPlugins: settings.tcpPlugins
        // property alias windowShadows: settings.windowShadows
        property alias manual: commManual.checked
        property alias tabIndex: tab.currentIndex
    }
    // Update manual/auto checkboxes
    Connections {
        function onOperationModeChanged() {
            commAuto.checked = (Cpp_JSON_Generator.operationMode === 1);
            commManual.checked = (Cpp_JSON_Generator.operationMode === 0);
        }

        target: Cpp_JSON_Generator
    }
    // Window
    Widgets.Window {
        anchors.fill: parent
        anchors.leftMargin: 0
        anchors.margins: (app.spacing * 1.5) - 5
        backgroundColor: Cpp_ThemeManager.paneWindowBackground
        gradient: true
        headerDoubleClickEnabled: false
        icon.source: "qrc:/icons/settings.svg"
        title: qsTr("Setup")

        // Control arrangement
        ColumnLayout {
            id: column
            anchors.fill: parent
            anchors.margins: app.spacing * 1.5
            spacing: app.spacing / 2

            // Comm mode selector
            Label {
                font.bold: true
                text: qsTr("Communication Mode") + ":"
            }
            RadioButton {
                id: commAuto
                Layout.maximumWidth: root.maxItemWidth
                checked: true
                text: qsTr("No parsing (device sends JSON data)")

                onCheckedChanged: {
                    if (checked)
                        Cpp_JSON_Generator.setOperationMode(1);
                    else
                        Cpp_JSON_Generator.setOperationMode(0);
                }
            }
            RadioButton {
                id: commManual
                Layout.maximumWidth: root.maxItemWidth
                checked: false
                text: qsTr("Parse via JSON project file")

                onCheckedChanged: {
                    if (checked)
                        Cpp_JSON_Generator.setOperationMode(0);
                    else
                        Cpp_JSON_Generator.setOperationMode(1);
                }
            }
            // Map file selector button
            Button {
                Layout.fillWidth: true
                Layout.maximumWidth: root.maxItemWidth
                enabled: commManual.checked
                opacity: enabled ? 1 : 0.5
                text: (Cpp_JSON_Generator.jsonMapFilename.length ? qsTr("Change project file (%1)").arg(Cpp_JSON_Generator.jsonMapFilename) : qsTr("Select project file") + "...")

                onClicked: Cpp_JSON_Generator.loadJsonMap()
            }
            // Spacer
            Item {
                height: app.spacing / 2
            }
            // Enable/disable CSV logging
            RowLayout {
                Layout.fillWidth: true

                Switch {
                    id: csvLogging
                    Layout.alignment: Qt.AlignVCenter
                    Layout.maximumWidth: root.maxItemWidth
                    checked: Cpp_CSV_Export.exportEnabled
                    palette.highlight: Cpp_ThemeManager.csvCheckbox
                    text: qsTr("Create CSV file")

                    onCheckedChanged: {
                        if (Cpp_CSV_Export.exportEnabled !== checked)
                            Cpp_CSV_Export.exportEnabled = checked;
                    }
                }
                Item {
                    Layout.fillWidth: true
                }
                RoundButton {
                    Layout.alignment: Qt.AlignVCenter
                    icon.color: Cpp_ThemeManager.text
                    icon.height: 24
                    icon.source: "qrc:/icons/help.svg"
                    icon.width: 24

                    onClicked: Qt.openUrlExternally("https://github.com/Serial-Studio/Serial-Studio/wiki")
                }
            }
            // Spacer
            Item {
                height: app.spacing / 2
            }
            // Tab bar
            TabBar {
                id: tab
                Layout.fillWidth: true
                Layout.maximumWidth: root.maxItemWidth
                height: 24

                TabButton {
                    height: tab.height + 3
                    text: qsTr("Device")
                    width: implicitWidth + 2 * app.spacing
                }
                TabButton {
                    height: tab.height + 3
                    text: qsTr("Ros2")
                    width: implicitWidth + 2 * app.spacing
                }
            }
            // Tab bar contents
            StackLayout {
                id: stack
                Layout.fillHeight: true
                Layout.fillWidth: true
                Layout.topMargin: -parent.spacing - 1
                clip: true
                currentIndex: tab.currentIndex

                SetupPanes.Hardware {
                    id: hardware
                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    background: TextField {
                        enabled: false
                        palette.base: Cpp_ThemeManager.setupPanelBackground
                    }
                }

                SetupPanes.Ros2 {
                    id: ros2
                    Layout.fillHeight: true
                    Layout.fillWidth: true

                    background: TextField {
                        enabled: false
                        palette.base: Cpp_ThemeManager.setupPanelBackground
                    }
                }
                // remove mqtt and setting
            }
        }
    }
}
