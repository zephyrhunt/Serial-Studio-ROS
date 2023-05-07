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

    // Custom properties
    property int appLaunchCount: 0
    property bool automaticUpdates: false
    readonly property bool consoleVisible: terminal.visible
    readonly property bool dashboardVisible: dashboard.visible
    property bool firstValidFrame: false
    property bool ros2wsVisible: true
    // Global properties
    readonly property bool setupVisible: setup.visible
    property alias vt100emulation: terminal.vt100emulation

    function consoleClear() {
        terminal.clear();
    }
    // Console-related functions
    function consoleCopy() {
        terminal.copy();
    }
    function consoleSelectAll() {
        terminal.selectAll();
    }
    function showConsole() {
        toolbar.consoleClicked();
    }
    function showDashboard() {
        dbTimer.start();
    }
    function showRos2Ws() {
        toolbar.ros2WsClicked();
    }
    // Toolbar functions aliases
    function showSetup() {
        toolbar.setupClicked();
    }

    backgroundColor: Cpp_ThemeManager.windowBackground
    borderColor: Qt.darker(Cpp_ThemeManager.toolbarGradient2, 1.5)
    borderWidth: 2
    height: minimumHeight
    minimumHeight: 650 + 2 * root.shadowMargin + root.titlebar.height
    minimumWidth: 1090 + 2 * root.shadowMargin
    // Customize window border
    showIcon: true
    title: Cpp_AppName
    // Window geometry
    visible: false
    width: minimumWidth

    // ------------------------------
    // Startup code
    // ------------------------------
    Component.onCompleted: {
        // Load welcome text
        // terminal.showWelcomeGuide()

        // Increment app launch count
        ++appLaunchCount;

        // Show app window
        if (root.isFullscreen)
            root.showFullScreen();
        else if (root.isMaximized)
            root.showMaximized();
        else {
            // Fix maximize not working on first try on macOS & Windows
            root.opacity = 0;
            var x = root.x;
            var y = root.y;
            var w = root.width;
            var h = root.height;
            root.showMaximized();
            root.showNormal();
            root.setGeometry(x, y, w, h);
            root.opacity = 1;
        }

        // Force active focus
        root.requestActivate();
        root.requestUpdate();

        // remove donation and update
    }
    // Quit application when this window is closed
    onClosed: Qt.quit()

    // Wait a little before showing the dashboard to avoid UI glitches and/or overloading
    // the rendering engine
    Timer {
        id: dbTimer
        interval: 500

        onTriggered: toolbar.dashboardClicked()
    }
    // Hide console & device manager when we receive first valid frame
    Connections {
        function onUpdated() {
            if (root.firstValidFrame)
                return;
            if ((Cpp_IO_Manager.connected || Cpp_CSV_Player.isOpen) && Cpp_UI_Dashboard.frameValid()) {
                setup.hide();
                root.showDashboard();
                root.firstValidFrame = true;
            } else {
                setup.show();
                root.showConsole();
                root.firstValidFrame = false;
            }
        }

        target: Cpp_UI_Dashboard
    }
    // Show console tab on serial disconnect
    Connections {
        function onDataReset() {
            setup.show();
            root.showConsole();
            root.firstValidFrame = false;
        }

        target: Cpp_UI_Dashboard
    }
    // Save window size & position
    Settings {
        property alias appStatus: root.appLaunchCount
        property alias autoUpdater: root.automaticUpdates
        property alias wf: root.isFullscreen
        property alias wh: root.height
        property alias wm: root.isMaximized
        property alias ww: root.width
        property alias wx: root.x
        property alias wy: root.y
    }
    // macOS menubar loader
    Loader {
        active: Cpp_IsMac
        asynchronous: false

        sourceComponent: PlatformDependent.MenubarMacOS {
        }
    }
    // Rectangle for the menubar (only used if custom window flags are disabled)
    Rectangle {
        anchors.fill: menubarLayout
        color: root.titlebarColor
        visible: !Cpp_ThemeManager.customWindowDecorations
    }
    // Menubar, shown by default on Windows & Linux and when the app is fullscreen
    RowLayout {
        id: menubarLayout

        readonly property bool showMenubar: !root.showMacControls || isFullscreen

        height: !showMenubar ? titlebar.height : 38
        spacing: app.spacing
        z: titlebar.z + 1

        anchors {
            left: parent.left
            leftMargin: root.leftTitlebarMargin + root.shadowMargin
            right: parent.right
            rightMargin: root.rightTitlebarMargin + root.shadowMargin
            top: parent.top
            topMargin: root.shadowMargin + (!root.showMacControls ? 1 : 0)
        }
        // Menubar
        Loader {
            Layout.alignment: Qt.AlignVCenter
            asynchronous: false
            opacity: 0.8

            sourceComponent: PlatformDependent.Menubar {
                enabled: !root.showMacControls || isFullscreen
                visible: !root.showMacControls || isFullscreen
            }
        }
        // Spacer
        Item {
            Layout.fillWidth: true
        }
    }
    // Main layout
    Page {
        anchors.fill: parent
        anchors.margins: root.shadowMargin
        anchors.topMargin: menubarLayout.height + root.shadowMargin
        clip: true
        palette.buttonText: Cpp_ThemeManager.text
        palette.text: Cpp_ThemeManager.text
        palette.windowText: Cpp_ThemeManager.text

        background: Rectangle {
            color: Cpp_ThemeManager.windowBackground
            radius: root.radius
        }

        ColumnLayout {
            anchors.fill: parent
            spacing: 3

            // Application toolbar
            Toolbar {
                id: toolbar
                Layout.fillWidth: true
                Layout.maximumHeight: 48
                Layout.minimumHeight: 48
                consoleChecked: root.consoleVisible
                dashboardChecked: root.dashboardVisible  // control button if visible
                ros2WsChecked: root.ros2WsVisible
                setupChecked: root.setupVisible
                window: root
                z: titlebar.z

                onConsoleClicked: {
                    consoleChecked = 1;
                    ros2WsChecked = 0;
                    dashboardChecked = 0;
                    if (stack.currentItem !== terminal)
                        stack.replace(terminal, StackView.Push);
                }
                onDashboardClicked: {
                    consoleChecked = 0;
                    ros2WsChecked = 0;
                    dashboardChecked = 1;
                    if (stack.currentItem !== dashboard)
                        stack.replace(dashboard, StackView.Pop);
                }
                onProjectEditorClicked: app.projectEditorWindow.show()
                onRos2WsClicked: {
                    consoleChecked = 0;
                    dashboardChecked = 0;
                    ros2WsChecked = 1;
                    if (stack.currentItem !== ros2ws)
                        stack.replace(ros2ws, StackView.SlideDown);
                }
                onSetupClicked: setup.visible ? setup.hide() : setup.show()
            }

            // Console, dashboard & setup panel & ros2 panel
            RowLayout {
                Layout.fillHeight: true
                Layout.fillWidth: true
                spacing: 0

                StackView {
                    id: stack
                    Layout.fillHeight: true
                    Layout.fillWidth: true
                    clip: true
                    initialItem: terminal

                    data: [
                        Console {
                            id: terminal
                            height: parent.height
                            visible: false
                            width: parent.width
                        },
                        Dashboard {
                            id: dashboard
                            height: parent.height
                            visible: false
                            width: parent.width
                        },
                        Ros2Ws {
                            id: ros2ws
                            height: parent.height
                            visible: false
                            width: parent.width
                        }
                    ]
                }
                Setup {
                    id: setup
                    Layout.fillHeight: true
                    Layout.maximumWidth: displayedWidth
                    Layout.minimumWidth: displayedWidth
                    Layout.rightMargin: setupMargin
                }
            }
        }
    }
    // JSON project drop area
    JSONDropArea {
        anchors.fill: parent
        enabled: !Cpp_IO_Manager.connected
    }
    // Resize handler
    FramelessWindow.ResizeHandles {
        anchors.fill: parent
        handleSize: root.handleSize
        window: root
    }
}
