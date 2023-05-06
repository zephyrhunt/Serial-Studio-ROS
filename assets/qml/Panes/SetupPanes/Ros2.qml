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

Control {
    id: root
    //
    // Layout
    //
    ColumnLayout {
        id: layout
        anchors.fill: parent
        anchors.margins: app.spacing

        //
        // Controls
        //
        GridLayout {
            columns: 2
            Layout.fillWidth: true
            rowSpacing: app.spacing
            columnSpacing: app.spacing

            Label {
                text: qsTr("ROS Topic") + ":"
            } ComboBox {
                id: _topicCombo
                Layout.fillWidth: true
                currentIndex: Cpp_IO_RosNode.topicIndex
                model: Cpp_IO_RosNode.topicList
                // enabled: 1
                onCurrentIndexChanged: {
                    // if (currentIndex !== Cpp_IO_RosNode.topicIndex)
                    //     Cpp_IO_Serial.setTopicIndex(currentIndex);
                }
            }

            Label {
                text: qsTr("Pub Test") + ": "
            } Switch {
                id: _tcpPub
                Layout.leftMargin: -app.spacing
                Layout.alignment: Qt.AlignLeft
                // checked: Cpp_Plugins_Bridge.enabled
                onCheckedChanged: {

                }
            }
            //
            // Window shadows
            //
            Label {
                text: qsTr("Custom window decorations") + ": "
            } Switch {
                id: _windowShadows
                Layout.leftMargin: -app.spacing
                Layout.alignment: Qt.AlignLeft
                checked: Cpp_ThemeManager.customWindowDecorations
                onCheckedChanged: {
                    if (checked != Cpp_ThemeManager.customWindowDecorations)
                        Cpp_ThemeManager.customWindowDecorations = checked
                }
            }
        }

        //
        // Plugins label
        //
        Label {
            opacity: 0.8
            font.pixelSize: 12
            Layout.fillWidth: true
            wrapMode: Label.WrapAtWordBoundaryOrAnywhere
            color: Cpp_ThemeManager.highlightedTextAlternative
            text: qsTr("Applications/plugins can interact with %1 by " +
                       "establishing a TCP connection on port 7777.").arg(Cpp_AppName)
        }

        //
        // Vertical spacer
        //
        Item {
            Layout.fillHeight: true
        }
    }
}
