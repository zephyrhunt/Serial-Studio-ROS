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
import QtQuick.Controls

MenuBar {
    id: root

    //
    // Set background color
    //
    background: Rectangle {
        color: "transparent"
    }

    //
    // Palette
    //
    palette.text: Cpp_ThemeManager.menubarText
    palette.base: Cpp_ThemeManager.toolbarGradient1
    palette.window: Cpp_ThemeManager.toolbarGradient2
    palette.highlightedText: Cpp_ThemeManager.highlightedText

    //
    // File menu
    //
    Menu {
        title: qsTr("File")

        DecentMenuItem {
            sequence: "ctrl+j"
            text: qsTr("Select JSON file") + "..."
            onTriggered: Cpp_JSON_Generator.loadJsonMap()
        }

        MenuSeparator {}

        Menu {
            title: qsTr("CSV export")

            DecentMenuItem {
                checkable: true
                text: qsTr("Enable CSV export")
                checked: Cpp_CSV_Export.exportEnabled
                onTriggered: Cpp_CSV_Export.exportEnabled = checked
            }

            DecentMenuItem {
                sequence: "ctrl+shift+o"
                enabled: Cpp_CSV_Export.isOpen
                text: qsTr("Show CSV in explorer")
                onTriggered: Cpp_CSV_Export.openCurrentCsv()
            }
        }

        DecentMenuItem {
            sequence: "ctrl+o"
            text: qsTr("Replay CSV") + "..."
            onTriggered: Cpp_CSV_Player.openFile()
            enabled: Cpp_JSON_Generator.operationMode === 0
        }

        MenuSeparator {}

        DecentMenuItem {
            sequence: "ctrl+p"
            text: qsTr("Print") + "..."
            enabled: Cpp_IO_Console.saveAvailable
            onTriggered: Cpp_IO_Console.print(app.monoFont)
        }

        DecentMenuItem {
            sequence: "ctrl+s"
            onClicked: Cpp_IO_Console.save()
            enabled: Cpp_IO_Console.saveAvailable
            text: qsTr("Export console output") + "..."
        }

        MenuSeparator {}

        DecentMenuItem {
            text: qsTr("Quit")
            onTriggered: Qt.quit()
            sequence: "ctrl+q"
        }
    }

    //
    // Edit menu
    //
    Menu {
        title: qsTr("Edit")

        DecentMenuItem {
            text: qsTr("Copy")
            sequence: "ctrl+c"
            onTriggered: mainWindow.consoleCopy()
        }

        DecentMenuItem {
            sequence: "ctrl+a"
            text: qsTr("Select all") + "..."
            onTriggered: mainWindow.consoleSelectAll()
        }

        DecentMenuItem {
            sequence: "ctrl+d"
            onTriggered: mainWindow.consoleClear()
            text: qsTr("Clear console output")
        }

        MenuSeparator{}

        Menu {
            title: qsTr("Communication mode")

            DecentMenuItem {
                checkable: true
                text: qsTr("Device sends JSON")
                checked: Cpp_JSON_Generator.operationMode === 1
                onTriggered: Cpp_JSON_Generator.operationMode = checked ? 1 : 0
            }

            DecentMenuItem {
                checkable: true
                text: qsTr("Load JSON from computer")
                checked: Cpp_JSON_Generator.operationMode === 0
                onTriggered: Cpp_JSON_Generator.operationMode = checked ? 0 : 1
            }
        }
    }

    //
    // View menu
    //
    Menu {
        title: qsTr("View")

        DecentMenuItem {
            checkable: true
            sequence: "ctrl+t"
            text: qsTr("Console")
            checked: mainWindow.consoleVisible
            onTriggered: mainWindow.showConsole()
            onCheckedChanged: {
                if (mainWindow.consoleVisible !== checked)
                    checked = mainWindow.consoleVisible
            }
        }

        DecentMenuItem {
            checkable: true
            sequence: "ctrl+d"
            text: qsTr("Dashboard")
            checked: mainWindow.dashboardVisible
            enabled: Cpp_UI_Dashboard.available
            onTriggered: mainWindow.showDashboard()
            onCheckedChanged: {
                if (mainWindow.dashboardVisible !== checked)
                    checked = mainWindow.dashboardVisible
            }
        }

        MenuSeparator {}

        DecentMenuItem {
            checkable: true
            sequence: "ctrl+,"
            checked: mainWindow.setupVisible
            text: qsTr("Show setup pane")
            onTriggered: mainWindow.showSetup()
        }

        MenuSeparator {}

        DecentMenuItem {
            sequence: "f11"
            onTriggered: mainWindow.toggleFullscreen()
            text: mainWindow.isFullscreen ? qsTr("Exit full screen") :
                                            qsTr("Enter full screen")
        }
    }

    //
    // Console format
    //
    Menu {
        title: qsTr("Console")

        DecentMenuItem {
            checkable: true
            text: qsTr("Autoscroll")
            checked: Cpp_IO_Console.autoscroll
            onTriggered: Cpp_IO_Console.autoscroll = checked
        }

        DecentMenuItem {
            checkable: true
            text: qsTr("Show timestamp")
            checked: Cpp_IO_Console.showTimestamp
            onTriggered: Cpp_IO_Console.showTimestamp = checked
        }

        DecentMenuItem {
            checkable: true
            checked: mainWindow.vt100emulation
            text: qsTr("VT-100 emulation")
            onTriggered: mainWindow.vt100emulation = checked
        }

        DecentMenuItem {
            checkable: true
            text: qsTr("Echo user commands")
            checked: Cpp_IO_Console.echo
            onTriggered: Cpp_IO_Console.echo = checked
        }

        MenuSeparator{}

        Menu {
            title: qsTr("Display mode")

            DecentMenuItem {
                checkable: true
                text: qsTr("Normal (plain text)")
                checked: Cpp_IO_Console.displayMode === 0
                onTriggered: Cpp_IO_Console.displayMode = checked ? 0 : 1
            }

            DecentMenuItem {
                checkable: true
                text: qsTr("Binary (hexadecimal)")
                checked: Cpp_IO_Console.displayMode === 1
                onTriggered: Cpp_IO_Console.displayMode = checked ? 1 : 0
            }
        }

        Menu {
            title: qsTr("Line ending character")

            DecentMenuItem {
                checkable: true
                text: Cpp_IO_Console.lineEndings()[0]
                checked: Cpp_IO_Console.lineEnding === 0
                onTriggered: Cpp_IO_Console.lineEnding = 0
            }

            DecentMenuItem {
                checkable: true
                text: Cpp_IO_Console.lineEndings()[1]
                checked: Cpp_IO_Console.lineEnding === 1
                onTriggered: Cpp_IO_Console.lineEnding = 1
            }

            DecentMenuItem {
                checkable: true
                text: Cpp_IO_Console.lineEndings()[2]
                checked: Cpp_IO_Console.lineEnding === 2
                onTriggered: Cpp_IO_Console.lineEnding = 2
            }

            DecentMenuItem {
                checkable: true
                text: Cpp_IO_Console.lineEndings()[3]
                checked: Cpp_IO_Console.lineEnding === 3
                onTriggered: Cpp_IO_Console.lineEnding = 3
            }
        }
    }
    //
    // ROS2 menu
    //
    Menu {
        title: qsTr("ROS2")

        DecentMenuItem {
            checkable: true
            checked: Cpp_IO_Console.autoscroll
            text: qsTr("ros console")

            onTriggered: Cpp_IO_Console.autoscroll = checked
        }
        DecentMenuItem {
            checkable: true
            checked: Cpp_IO_Console.showTimestamp
            text: qsTr("image")

            onTriggered: Cpp_IO_Console.showTimestamp = checked
        }
        MenuSeparator {}
    }

    // Help menu
    //
    Menu {
        title: qsTr("Help")

        DecentMenuItem {
            onTriggered: app.aboutDialog.show()
            text: qsTr("About %1").arg(Cpp_AppName)
        }

        DecentMenuItem {
            text: qsTr("About %1").arg("Qt")
            onTriggered: Cpp_Misc_Utilities.aboutQt()
        }

        MenuSeparator {
            visible: Cpp_UpdaterEnabled
            enabled: Cpp_UpdaterEnabled
        }

        DecentMenuItem {
            text: qsTr("Project website") + "..."
            onTriggered: Qt.openUrlExternally("https://www.alex-spataru.com/serial-studio")
        }

        DecentMenuItem {
            sequence: "f1"
            text: qsTr("Documentation/wiki") + "..."
            onTriggered: Qt.openUrlExternally("https://github.com/Serial-Studio/Serial-Studio/wiki")
        }
    }
}
