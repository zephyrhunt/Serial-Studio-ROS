import QtQuick
import QtQuick.Window 2.1
import QtQuick.Layouts
import QtQuick.Controls
import Qt.labs.settings

import "../Panes"
import "../Windows"
import "../Widgets"
import "../ProjectEditor"
import "../FramelessWindow" as FramelessWindow
import "../PlatformDependent" as PlatformDependent


FramelessWindow.CustomWindow {
    id: root

    //
    // Quit application when this window is closed
    //
    onClosed: Qt.quit()

    //
    // Customize window border
    //
    showIcon: true
    borderWidth: 2
    borderColor: Qt.darker(Cpp_ThemeManager.toolbarGradient2, 1.5)

    //
    // Global properties
    //
    readonly property bool setupVisible: setup.visible
    readonly property bool consoleVisible: terminal.visible
    readonly property bool dashboardVisible: dashboard.visible

    //
    // Custom properties
    //
    property int appLaunchCount: 0
    property bool firstValidFrame: false
    property bool automaticUpdates: false
    property alias vt100emulation: terminal.vt100emulation

    //
    // Toolbar functions aliases
    //
    function showSetup()     { toolbar.setupClicked()     }
    function showConsole()   { toolbar.consoleClicked()   }
    function showDashboard() { dbTimer.start() }

    //
    // Wait a little before showing the dashboard to avoid UI glitches and/or overloading
    // the rendering engine
    //
    Timer {
        id: dbTimer
        interval: 500
        onTriggered: toolbar.dashboardClicked()
    }

    //
    // Console-related functions
    //
    function consoleCopy()      { terminal.copy()      }
    function consoleClear()     { terminal.clear()     }
    function consoleSelectAll() { terminal.selectAll() }

    //
    // Window geometry
    //
    visible: false
    title: Cpp_AppName
    width: minimumWidth
    height: minimumHeight
    minimumWidth: 1090 + 2 * root.shadowMargin
    backgroundColor: Cpp_ThemeManager.windowBackground
    minimumHeight: 650 + 2 * root.shadowMargin + root.titlebar.height

    // ------------------------------
    // Startup code
    // ------------------------------
    Component.onCompleted: {
        // Load welcome text
        // terminal.showWelcomeGuide()

        // Increment app launch count
        ++appLaunchCount

        // Show app window
        if (root.isFullscreen)
            root.showFullScreen()
        else if (root.isMaximized)
            root.showMaximized()
        else {
            // Fix maximize not working on first try on macOS & Windows
            root.opacity = 0
            var x = root.x
            var y = root.y
            var w = root.width
            var h = root.height
            root.showMaximized()
            root.showNormal()
            root.setGeometry(x, y, w,h)
            root.opacity = 1
        }

        // Force active focus
        root.requestActivate()
        root.requestUpdate()

        // remove donation and update
    }

    //
    // Hide console & device manager when we receive first valid frame
    //
    Connections {
        target: Cpp_UI_Dashboard

        function onUpdated()  {
            if (root.firstValidFrame)
                return

            if ((Cpp_IO_Manager.connected || Cpp_CSV_Player.isOpen) && Cpp_UI_Dashboard.frameValid()) {
                setup.hide()
                root.showDashboard()
                root.firstValidFrame = true
            }

            else {
                setup.show()
                root.showConsole()
                root.firstValidFrame = false
            }
        }
    }

    //
    // Show console tab on serial disconnect
    //
    Connections {
        target: Cpp_UI_Dashboard
        function onDataReset() {
            setup.show()
            root.showConsole()
            root.firstValidFrame = false
        }
    }

    //
    // Save window size & position
    //
    Settings {
        property alias wx: root.x
        property alias wy: root.y
        property alias ww: root.width
        property alias wh: root.height
        property alias wm: root.isMaximized
        property alias wf: root.isFullscreen
        property alias appStatus: root.appLaunchCount
        property alias autoUpdater: root.automaticUpdates
    }

    //
    // macOS menubar loader
    //
    Loader {
        active: Cpp_IsMac
        asynchronous: false
        sourceComponent: PlatformDependent.MenubarMacOS {}
    }

    //
    // Rectangle for the menubar (only used if custom window flags are disabled)
    //
    Rectangle {
        color: root.titlebarColor
        anchors.fill: menubarLayout
        visible: !Cpp_ThemeManager.customWindowDecorations
    }

    //
    // Menubar, shown by default on Windows & Linux and when the app is fullscreen
    //
    RowLayout {
        id: menubarLayout
        z: titlebar.z + 1
        spacing: app.spacing
        height: !showMenubar ? titlebar.height : 38

        readonly property bool showMenubar: !root.showMacControls || isFullscreen

        anchors {
            top: parent.top
            left: parent.left
            right: parent.right
            leftMargin: root.leftTitlebarMargin + root.shadowMargin
            rightMargin: root.rightTitlebarMargin + root.shadowMargin
            topMargin: root.shadowMargin + (!root.showMacControls ? 1 : 0)
        }

        //
        // Menubar
        //
        Loader {
            opacity: 0.8
            asynchronous: false
            Layout.alignment: Qt.AlignVCenter
            sourceComponent: PlatformDependent.Menubar {
                enabled: !root.showMacControls || isFullscreen
                visible: !root.showMacControls || isFullscreen
            }
        }

        //
        // Spacer
        //
        Item {
            Layout.fillWidth: true
        }
    }

    //
    // Main layout
    //
    Page {
        clip: true
        anchors.fill: parent
        anchors.margins: root.shadowMargin
        palette.text: Cpp_ThemeManager.text
        palette.buttonText: Cpp_ThemeManager.text
        palette.windowText: Cpp_ThemeManager.text
        anchors.topMargin: menubarLayout.height + root.shadowMargin

        background: Rectangle {
            radius: root.radius
            color: Cpp_ThemeManager.windowBackground
        }

        ColumnLayout {
            spacing: 3
            anchors.fill: parent

            //
            // Application toolbar
            //
            Toolbar {
                id: toolbar
                window: root
                z: titlebar.z
                Layout.fillWidth: true
                Layout.minimumHeight: 48
                Layout.maximumHeight: 48
                setupChecked: root.setupVisible
                consoleChecked: root.consoleVisible
                dashboardChecked: root.dashboardVisible

                onProjectEditorClicked: app.projectEditorWindow.show()
                onSetupClicked: setup.visible ? setup.hide() : setup.show()

                onDashboardClicked: {
                    if (Cpp_UI_Dashboard.available) {
                        consoleChecked = 0
                        dashboardChecked = 1
                        if (stack.currentItem != dashboard)
                            stack.push(dashboard)
                    }
                    else
                        root.showConsole()
                }

                onConsoleClicked: {
                    consoleChecked = 1
                    dashboardChecked = 0
                    stack.pop()
                }
            }

            //
            // Console, dashboard & setup panel & ros2 panel
            //
            RowLayout {
                spacing: 0
                Layout.fillWidth: true
                Layout.fillHeight: true

                StackView {
                    id: stack
                    clip: true
                    initialItem: terminal
                    Layout.fillWidth: true
                    Layout.fillHeight: true
                    data: [
                        Console {
                            id: terminal
                            visible: false
                            width: parent.width
                            height: parent.height
                        },

                        Dashboard {
                            id: dashboard
                            visible: false
                            width: parent.width
                            height: parent.height
                        }
                    ]
                }

                Setup {
                    id: setup
                    Layout.fillHeight: true
                    Layout.rightMargin: setupMargin
                    Layout.minimumWidth: displayedWidth
                    Layout.maximumWidth: displayedWidth
                }
            }
        }
    }

    //
    // JSON project drop area
    //
    JSONDropArea {
        anchors.fill: parent
        enabled: !Cpp_IO_Manager.connected
    }

    //
    // Resize handler
    //
    FramelessWindow.ResizeHandles {
        window: root
        anchors.fill: parent
        handleSize: root.handleSize
    }
}
