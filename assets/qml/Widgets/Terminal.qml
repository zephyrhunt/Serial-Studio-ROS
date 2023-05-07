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
import SerialStudio as SerialStudio

Item {
    id: root

    property bool isExternalWindow: false
    property alias vt100emulation: textEdit.vt100emulation
    property alias widgetEnabled: textEdit.widgetEnabled

    // Clears console output
    function clear() {
        Cpp_IO_Console.clear();
        textEdit.clear();
    }

    // Copy function
    function copy() {
        textEdit.copy();
    }

    // Select all text
    function selectAll() {
        textEdit.selectAll();
    }

    // Function to send through serial port data
    function sendData() {
        Cpp_IO_Console.send(send.text);
        send.clear();
    }

    // Save settings
    Settings {
        property alias autoWrap: autoWrapCheck.checked
        property alias autoscroll: autoscrollCheck.checked
        property alias displayMode: displayModeCombo.currentIndex
        property alias echo: echoCheckbox.checked
        property alias hex: hexCheckbox.checked
        property alias lineEnding: lineEndingCombo.currentIndex
        property alias timestamp: timestampCheck.checked
        property alias vt100Enabled: textEdit.vt100emulation
    }

    // Right-click context menu
    Menu {
        id: contextMenu

        MenuItem {
            id: copyMenu

            enabled: textEdit.copyAvailable
            opacity: enabled ? 1 : 0.5
            text: qsTr("Copy")
            onClicked: textEdit.copy()
        }

        MenuItem {
            enabled: !textEdit.empty
            opacity: enabled ? 1 : 0.5
            text: qsTr("Select all")
            onTriggered: textEdit.selectAll()
        }

        MenuItem {
            enabled: Cpp_IO_Console.saveAvailable
            opacity: enabled ? 1 : 0.5
            text: qsTr("Clear")
            onTriggered: root.clear()
        }

        MenuSeparator {
        }

        MenuItem {
            enabled: Cpp_IO_Console.saveAvailable
            opacity: enabled ? 1 : 0.5
            text: qsTr("Print")
            onTriggered: Cpp_IO_Console.print(app.monoFont)
        }

        MenuItem {
            enabled: Cpp_IO_Console.saveAvailable
            opacity: enabled ? 1 : 0.5
            text: qsTr("Save as") + "..."
            onTriggered: Cpp_IO_Console.save()
        }

    }

    // Controls
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: app.spacing * 1.5
        spacing: app.spacing

        // Console display
        SerialStudio.Terminal {
            id: textEdit

            Layout.fillHeight: true
            Layout.fillWidth: true
            autoWrap: Cpp_IO_Console.autoWrap
            autoscroll: Cpp_IO_Console.autoscroll
            centerOnScroll: false
            focus: true
            font.family: app.monoFont
            font.pixelSize: 14
            maximumBlockCount: 12000
            placeholderText: qsTr("Nichijou Ros. No data received so far") + "..." + qsTr("Please connect device ...")
            readOnly: true
            renderTarget: PaintedItem.FramebufferObject
            undoRedoEnabled: false
            vt100emulation: true //模拟终端
            wordWrapMode: Text.WrapAtWordBoundaryOrAnywhere

            MouseArea {
                id: mouseArea

                acceptedButtons: Qt.RightButton
                anchors.fill: parent
                anchors.rightMargin: textEdit.scrollbarWidth
                cursorShape: Qt.IBeamCursor
                propagateComposedEvents: true
                onClicked: {
                    if (mouse.button === Qt.RightButton) {
                        contextMenu.popup();
                        mouse.accepted = true;
                    }
                }
            }

        }

        // Data-write controls
        RowLayout {
            Layout.fillWidth: true

            TextField {
                id: send

                Layout.fillWidth: true
                enabled: Cpp_IO_Manager.readWrite
                font: textEdit.font
                height: 24
                palette.base: Cpp_ThemeManager.consoleBase
                palette.text: Cpp_ThemeManager.consoleText
                placeholderText: qsTr("Send data to device") + "..."
                Component.onCompleted: {
                    if (Cpp_Qt6)
                        placeholderTextColor = Cpp_ThemeManager.consolePlaceholderText;

                }
                // Navigate command history downwards with <down>
                Keys.onDownPressed: {
                    Cpp_IO_Console.historyDown();
                    send.text = Cpp_IO_Console.currentHistoryString;
                }
                // Validate hex strings
                //validator: RegExpValidator {
                //    regExp: hexCheckbox.checked ? /^(?:([a-f0-9]{2})\s*)+$/i : /[\s\S]*/
                //}
                // Send data on <enter>
                Keys.onReturnPressed: root.sendData()
                // Navigate command history upwards with <up>
                Keys.onUpPressed: {
                    Cpp_IO_Console.historyUp();
                    send.text = Cpp_IO_Console.currentHistoryString;
                }
                // Add space automatically in hex view
                onTextChanged: {
                    if (hexCheckbox.checked)
                        send.text = Cpp_IO_Console.formatUserHex(send.text);

                }
            }

            CheckBox {
                id: hexCheckbox

                enabled: Cpp_IO_Manager.readWrite
                opacity: enabled ? 1 : 0.5
                text: "HEX"
                // checked: Cpp_IO_Console.dataMode === 1
                onCheckedChanged: Cpp_IO_Console.dataMode = checked ? 1 : 0
            }

            CheckBox {
                id: echoCheckbox

                checked: Cpp_IO_Console.echo
                enabled: Cpp_IO_Manager.readWrite
                opacity: enabled ? 1 : 0.5
                text: qsTr("Echo")
                visible: false
                onCheckedChanged: {
                    if (Cpp_IO_Console.echo !== checked)
                        Cpp_IO_Console.echo = checked;

                }
            }

        }

        // Terminal output options
        RowLayout {
            id: layout_row

            Layout.fillWidth: true

            // columns:2
            GridLayout {
                columns: 2

                CheckBox {
                    id: autoscrollCheck

                    Layout.alignment: Qt.AlignVCenter
                    checked: Cpp_IO_Console.autoscroll
                    text: qsTr("Autoscroll")
                    onCheckedChanged: {
                        if (Cpp_IO_Console.autoscroll !== checked)
                            Cpp_IO_Console.autoscroll = checked;

                    }
                }

                CheckBox {
                    id: timestampCheck

                    Layout.alignment: Qt.AlignVCenter
                    checked: Cpp_IO_Console.showTimestamp
                    text: qsTr("Show timestamp")
                    onCheckedChanged: {
                        if (Cpp_IO_Console.showTimestamp !== checked)
                            Cpp_IO_Console.showTimestamp = checked;

                    }
                }

                CheckBox {
                    id: autoWrapCheck

                    Layout.alignment: Qt.AlignVCenter
                    checked: Cpp_IO_Console.autoWrap
                    text: qsTr("AutoWrap")
                    onCheckedChanged: {
                        if (Cpp_IO_Console.autoWrap !== checked)
                            Cpp_IO_Console.autoWrap = checked;

                    }
                }

                RowLayout {
                    CheckBox {
                        id: txData

                        Layout.alignment: Qt.AlignVCenter
                        checked: Cpp_IO_Console.echo
                        text: qsTr("TX")
                        onCheckedChanged: {
                            if (Cpp_IO_Console.echo !== checked)
                                Cpp_IO_Console.echo = checked;
                        }
                    }

                    CheckBox {
                        id: rxData
                        Layout.alignment: Qt.AlignVCenter
                        checked: Cpp_IO_Console.echoRx
                        text: qsTr("RX")
                        onCheckedChanged: {
                            if (Cpp_IO_Console.echoRx !== checked)
                                Cpp_IO_Console.echoRx = checked;
                        }
                    }
                    // Label {
                    //     opacity: enabled ? 1 : 0.5
                    //     text: qsTr("In time(ms)") + ":"
                    // }
                    //
                    // ComboBox {
                    //     id: _timeCombo
                    //
                    //     // Layout.fillWidth: true
                    //     currentIndex: 1
                    //     editable: true
                    //     model: Cpp_IO_Console.inTimeList() //function and property
                    //     palette.base: Cpp_ThemeManager.setupPanelBackground
                    //     implicitWidth: 60
                    //     onCurrentTextChanged: {
                    //         var value = currentText;
                    //         Cpp_IO_Console.inTime = value;
                    //     }
                    //
                    //     validator: IntValidator {
                    //         bottom: 0
                    //     }
                    // }

                }

            }

            Item {
                Layout.fillWidth: true
            }

            ComboBox {
                id: lineEndingCombo

                Layout.alignment: Qt.AlignVCenter
                currentIndex: Cpp_IO_Console.lineEnding
                model: Cpp_IO_Console.lineEndings()
                onCurrentIndexChanged: {
                    if (currentIndex !== Cpp_IO_Console.lineEnding)
                        Cpp_IO_Console.lineEnding = currentIndex;

                }
            }

            ComboBox {
                id: displayModeCombo

                Layout.alignment: Qt.AlignVCenter
                currentIndex: Cpp_IO_Console.displayMode
                model: Cpp_IO_Console.displayModes()
                onCurrentIndexChanged: {
                    if (currentIndex !== Cpp_IO_Console.displayMode)
                        Cpp_IO_Console.displayMode = currentIndex;

                }
            }

            Button {
                Layout.maximumWidth: 32
                enabled: Cpp_IO_Console.saveAvailable
                height: 24
                icon.color: palette.text
                icon.source: "qrc:/icons/save.svg"
                opacity: enabled ? 1 : 0.5
                onClicked: Cpp_IO_Console.save()
            }

            Button {
                Layout.maximumWidth: 32
                enabled: Cpp_IO_Console.saveAvailable
                height: 24
                icon.color: palette.text
                icon.source: "qrc:/icons/delete.svg"
                opacity: enabled ? 1 : 0.5
                onClicked: root.clear()
            }

        }

    }

}
