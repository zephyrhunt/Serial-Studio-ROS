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

    property alias service: _serviceCombo.currentIndex
    property alias subEnable: _subCheck.checked
    property alias pubEnable: _pubCheck.checked
    // Access to properties
    property alias topic: _topicCombo.currentIndex

    // Update listbox models when translation is changed
    Connections {
        function onLanguageChanged() {
            var oldParityIndex = _parityCombo.currentIndex;
            var oldFlowControlIndex = _flowCombo.currentIndex;
            _parityCombo.model = Cpp_IO_Serial.parityList;
            _flowCombo.model = Cpp_IO_Serial.flowControlList;
            _parityCombo.currentIndex = oldParityIndex;
            _flowCombo.currentIndex = oldFlowControlIndex;
        }

        target: Cpp_Misc_Translator
    }
    // Control layout
    ColumnLayout {
        anchors.fill: parent
        anchors.margins: app.spacing

        // Controls
        GridLayout {
            id: layout
            Layout.fillWidth: true
            columnSpacing: app.spacing
            columns: 2
            rowSpacing: app.spacing

            // topic port selector
            Label {
                text: qsTr("Topic") + ":"
            }
            ComboBox {
                id: _topicCombo
                Layout.fillWidth: true
                currentIndex: Cpp_IO_RosNode.topicIndex
                model: Cpp_IO_RosNode.topicList
                palette.base: Cpp_ThemeManager.setupPanelBackground

                onCurrentIndexChanged: {
                    if (currentIndex !== Cpp_IO_RosNode.portIndex)
                        Cpp_IO_RosNode.topicIndex = currentIndex;
                }
            }

            // service port selector
            Label {
                text: qsTr("Service") + ":"
            }
            ComboBox {
                id: _serviceCombo
                Layout.fillWidth: true
                currentIndex: Cpp_IO_RosNode.serviceIndex
                model: Cpp_IO_RosNode.serviceList
                palette.base: Cpp_ThemeManager.setupPanelBackground

                onCurrentIndexChanged: {
                    if (currentIndex !== Cpp_IO_RosNode.portIndex)
                        Cpp_IO_RosNode.serviceIndex = currentIndex;
                }
            }

            RowLayout {
                spacing: 10
                // enable topic pub
                Label {
                    text: qsTr("publish enable") + ":"
                }
                CheckBox {
                    id: _pubCheck
                    Layout.alignment: Qt.AlignLeft
                    Layout.leftMargin: -app.spacing
                    checked: Cpp_IO_RosNode.pubEnable
                    palette.base: Cpp_ThemeManager.setupPanelBackground

                    onCheckedChanged: {
                        if (Cpp_IO_RosNode.pubEnable !== checked)
                            Cpp_IO_RosNode.pubEnable = checked;
                    }
                }
            }

            RowLayout {
                spacing: 10
                // enable topic sub
                Label {
                    text: qsTr("subscribe enable") + ":"
                }
                CheckBox {
                    id: _subCheck
                    Layout.alignment: Qt.AlignLeft
                    Layout.leftMargin: -app.spacing
                    checked: Cpp_IO_RosNode.subEnable
                    palette.base: Cpp_ThemeManager.setupPanelBackground

                    onCheckedChanged: {
                        if (Cpp_IO_RosNode.subEnable !== checked)
                            Cpp_IO_RosNode.subEnable = checked;
                    }
                }
            }
        }
    }
}

