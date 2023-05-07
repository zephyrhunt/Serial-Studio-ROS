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
    // }

    id: root

    // Save settings
    Widgets.Window {
        anchors.fill: parent
        anchors.leftMargin: 0
        anchors.margins: (app.spacing * 1.5) - 5
        backgroundColor: Cpp_ThemeManager.paneWindowBackground
        gradient: true
        headerDoubleClickEnabled: false
        icon.source: "qrc:/icons/bug.svg"
        title: qsTr("Debug")

        // Comm mode selector
        Label {
            anchors.topMargin: 20
            anchors.top: parent.top
            anchors.horizontalCenter: parent.horizontalCenter // 将标签水平居中于父级控件
            height: 30
            font.bold: true
            text: qsTr("PID Regular, Drag to control parameter")
        }

        GridLayout {
            columns: 3
            anchors.fill: parent
            anchors.margins: app.spacing * 2.5
            anchors.topMargin: 60
            // spacing: app.spacing / 2
            Label {
                text: "Kp:"
            }

            Slider {
                id: kpSld

                stepSize: 0.2
                from: 0
                to: 10
                value: 5
            }

            Label {
                text: kpSld.value.toFixed(2)
            }

            Label {
                text: "Ki:"
            }

            Slider {
                id: kiSld

                stepSize: 0.2
                from: 0
                to: 10
                value: 5
            }

            Label {
                text: kiSld.value.toFixed(2)
            }

            Label {
                text: "Kd:"
            }

            Slider {
                id: kdSld

                stepSize: 0.2
                from: 0
                to: 10
                value: 5
            }

            Label {
                text: kdSld.value.toFixed(2)
            }

        }

    }

}
