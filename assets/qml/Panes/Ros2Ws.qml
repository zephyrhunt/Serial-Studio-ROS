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

import "../Widgets" as Widgets

Item {
    id: root
    //
    // Load welcome guide
    //
    function showWelcomeGuide() {
        clear()
        Cpp_IO_Console.append(Cpp_Misc_Translator.welcomeConsoleText() + "\n")
    }

    //
    // Re-load welcome text when the language is changed
    //
    Connections {
        target: Cpp_Misc_Translator
        function onLanguageChanged() {
            root.showWelcomeGuide()
        }
    }

    //
    // Console window
    //
    Widgets.Window {
        id: window
        gradient: true
        anchors.fill: parent
        title: qsTr("Ros2")
        headerDoubleClickEnabled: false
        icon.source: "qrc:/icons/add.svg"
        anchors.margins: (app.spacing * 1.5) - 5
    }
}
