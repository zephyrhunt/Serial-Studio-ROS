cmake_minimum_required(VERSION 3.16)
project(serial-studio VERSION 6.3 LANGUAGES C CXX)

set(CMAKE_PREFIX_PATH "/home/nichijou/Qt/6.3.2/gcc_64")
set(CMAKE_INCLUDE_CURRENT_DIR ON)
set(CMAKE_AUTOUIC ON)

#set(QT_VERSION_MAJOR 6)
#set(CMAKE_PREFIX_PATH )
set(OPENSSL_ROOT_DIR /usr/bin/openssl)
#set(OPENSSL_INCLUDE_DIR "/usr/bin/openssl/include")
#set(OPENSSL_LIBRARIES "/usr/bin/openssl/lib/libssl.so;/usr/bin/openssl/lib/libcrypto.so")
find_package(OpenSSL REQUIRED)
find_package(QT NAMES Qt5 Qt6 REQUIRED COMPONENTS Core)
find_package(Qt${QT_VERSION_MAJOR} REQUIRED COMPONENTS Bluetooth Gui Network PrintSupport Qml Quick QuickControls2 SerialPort Svg Widgets Xml)
find_package(Qt${QT_VERSION_MAJOR} OPTIONAL_COMPONENTS Concurrent Core5Compat OpenGL OpenGLWidgets)

qt_standard_project_setup()

qt_add_resources(RESOURCE assets/Resources.qrc)
qt_add_executable(serial-studio WIN32 MACOSX_BUNDLE
    libs/QMapControl/src/arrowpoint.cpp libs/QMapControl/src/arrowpoint.h
    libs/QMapControl/src/bingapimapadapter.cpp libs/QMapControl/src/bingapimapadapter.h
    libs/QMapControl/src/circlepoint.cpp libs/QMapControl/src/circlepoint.h
    libs/QMapControl/src/curve.cpp libs/QMapControl/src/curve.h
    libs/QMapControl/src/emptymapadapter.cpp libs/QMapControl/src/emptymapadapter.h
    libs/QMapControl/src/fixedimageoverlay.cpp libs/QMapControl/src/fixedimageoverlay.h
    libs/QMapControl/src/geometry.cpp libs/QMapControl/src/geometry.h
    libs/QMapControl/src/geometrylayer.cpp libs/QMapControl/src/geometrylayer.h
    libs/QMapControl/src/googleapimapadapter.cpp libs/QMapControl/src/googleapimapadapter.h
    libs/QMapControl/src/googlemapadapter.cpp libs/QMapControl/src/googlemapadapter.h
    libs/QMapControl/src/gps_position.cpp libs/QMapControl/src/gps_position.h
    libs/QMapControl/src/imagemanager.cpp libs/QMapControl/src/imagemanager.h
    libs/QMapControl/src/imagepoint.cpp libs/QMapControl/src/imagepoint.h
    libs/QMapControl/src/invisiblepoint.cpp libs/QMapControl/src/invisiblepoint.h
    libs/QMapControl/src/layer.cpp libs/QMapControl/src/layer.h
    libs/QMapControl/src/layermanager.cpp libs/QMapControl/src/layermanager.h
    libs/QMapControl/src/linestring.cpp libs/QMapControl/src/linestring.h
    libs/QMapControl/src/mapadapter.cpp libs/QMapControl/src/mapadapter.h
    libs/QMapControl/src/mapcontrol.cpp libs/QMapControl/src/mapcontrol.h
    libs/QMapControl/src/maplayer.cpp libs/QMapControl/src/maplayer.h
    libs/QMapControl/src/mapnetwork.cpp libs/QMapControl/src/mapnetwork.h
    libs/QMapControl/src/openaerialmapadapter.cpp libs/QMapControl/src/openaerialmapadapter.h
    libs/QMapControl/src/osmmapadapter.cpp libs/QMapControl/src/osmmapadapter.h
    libs/QMapControl/src/point.cpp libs/QMapControl/src/point.h
    libs/QMapControl/src/qmapcontrol_global.h
    libs/QMapControl/src/tilemapadapter.cpp libs/QMapControl/src/tilemapadapter.h
    libs/QMapControl/src/wmsmapadapter.cpp libs/QMapControl/src/wmsmapadapter.h
    libs/QRealFourier/fftreal/Array.h libs/QRealFourier/fftreal/Array.hpp
    libs/QRealFourier/fftreal/DynArray.h libs/QRealFourier/fftreal/DynArray.hpp
    libs/QRealFourier/fftreal/FFTReal.h libs/QRealFourier/fftreal/FFTReal.hpp
    libs/QRealFourier/fftreal/FFTRealFixLen.h libs/QRealFourier/fftreal/FFTRealFixLen.hpp
    libs/QRealFourier/fftreal/FFTRealFixLenParam.h
    libs/QRealFourier/fftreal/FFTRealPassDirect.h libs/QRealFourier/fftreal/FFTRealPassDirect.hpp
    libs/QRealFourier/fftreal/FFTRealPassInverse.h libs/QRealFourier/fftreal/FFTRealPassInverse.hpp
    libs/QRealFourier/fftreal/FFTRealSelect.h libs/QRealFourier/fftreal/FFTRealSelect.hpp
    libs/QRealFourier/fftreal/FFTRealUseTrigo.h libs/QRealFourier/fftreal/FFTRealUseTrigo.hpp
    libs/QRealFourier/fftreal/OscSinCos.h libs/QRealFourier/fftreal/OscSinCos.hpp
    libs/QRealFourier/fftreal/def.h
    libs/QRealFourier/headers/qcomplexnumber.h
    libs/QRealFourier/headers/qfouriercalculator.h
    libs/QRealFourier/headers/qfourierfixedcalculator.h
    libs/QRealFourier/headers/qfouriertransformer.h
    libs/QRealFourier/headers/qfouriervariablecalculator.h
    libs/QRealFourier/headers/qwindowfunction.h
    libs/QRealFourier/sources/qcomplexnumber.cpp
    libs/QRealFourier/sources/qfouriercalculator.cpp
    libs/QRealFourier/sources/qfourierfixedcalculator.cpp
    libs/QRealFourier/sources/qfouriertransformer.cpp
    libs/QRealFourier/sources/qfouriervariablecalculator.cpp
    libs/QRealFourier/sources/qwindowfunction.cpp
    libs/QSimpleUpdater/include/QSimpleUpdater.h
    libs/QSimpleUpdater/src/Downloader.cpp libs/QSimpleUpdater/src/Downloader.h libs/QSimpleUpdater/src/Downloader.ui
    libs/QSimpleUpdater/src/QSimpleUpdater.cpp
    libs/QSimpleUpdater/src/Updater.cpp libs/QSimpleUpdater/src/Updater.h
    libs/QSourceHighlite/languagedata.cpp libs/QSourceHighlite/languagedata.h
    libs/QSourceHighlite/qsourcehighliter.cpp libs/QSourceHighlite/qsourcehighliter.h
    libs/QSourceHighlite/qsourcehighliterthemes.cpp libs/QSourceHighlite/qsourcehighliterthemes.h
    libs/qmqtt/src/mqtt/qmqtt.h
    libs/qmqtt/src/mqtt/qmqtt_client.cpp libs/qmqtt/src/mqtt/qmqtt_client.h libs/qmqtt/src/mqtt/qmqtt_client_p.cpp libs/qmqtt/src/mqtt/qmqtt_client_p.h
    libs/qmqtt/src/mqtt/qmqtt_frame.cpp libs/qmqtt/src/mqtt/qmqtt_frame.h
    libs/qmqtt/src/mqtt/qmqtt_global.h
    libs/qmqtt/src/mqtt/qmqtt_message.cpp libs/qmqtt/src/mqtt/qmqtt_message.h libs/qmqtt/src/mqtt/qmqtt_message_p.h
    libs/qmqtt/src/mqtt/qmqtt_network.cpp libs/qmqtt/src/mqtt/qmqtt_network_p.h
    libs/qmqtt/src/mqtt/qmqtt_networkinterface.h
    libs/qmqtt/src/mqtt/qmqtt_routedmessage.h
    libs/qmqtt/src/mqtt/qmqtt_router.cpp libs/qmqtt/src/mqtt/qmqtt_router.h
    libs/qmqtt/src/mqtt/qmqtt_routesubscription.cpp libs/qmqtt/src/mqtt/qmqtt_routesubscription.h
    libs/qmqtt/src/mqtt/qmqtt_socket.cpp libs/qmqtt/src/mqtt/qmqtt_socket_p.h
    libs/qmqtt/src/mqtt/qmqtt_socketinterface.h
    libs/qmqtt/src/mqtt/qmqtt_ssl_socket.cpp libs/qmqtt/src/mqtt/qmqtt_ssl_socket_p.h
    libs/qmqtt/src/mqtt/qmqtt_timer.cpp libs/qmqtt/src/mqtt/qmqtt_timer_p.h
    libs/qmqtt/src/mqtt/qmqtt_timerinterface.h
    libs/qtcsv/include/qtcsv/abstractdata.h
    libs/qtcsv/include/qtcsv/qtcsv_global.h
    libs/qtcsv/include/qtcsv/reader.h
    libs/qtcsv/include/qtcsv/stringdata.h
    libs/qtcsv/include/qtcsv/variantdata.h
    libs/qtcsv/include/qtcsv/writer.h
    libs/qtcsv/sources/contentiterator.cpp libs/qtcsv/sources/contentiterator.h
    libs/qtcsv/sources/filechecker.h
    libs/qtcsv/sources/reader.cpp
    libs/qtcsv/sources/stringdata.cpp
    libs/qtcsv/sources/symbols.h
    libs/qtcsv/sources/variantdata.cpp
    libs/qtcsv/sources/writer.cpp
    libs/qwt/src/qwt.cpp libs/qwt/src/qwt.h
    libs/qwt/src/qwt_abstract_scale_draw.cpp libs/qwt/src/qwt_abstract_scale_draw.h
    libs/qwt/src/qwt_bezier.cpp libs/qwt/src/qwt_bezier.h
    libs/qwt/src/qwt_clipper.cpp libs/qwt/src/qwt_clipper.h
    libs/qwt/src/qwt_color_map.cpp libs/qwt/src/qwt_color_map.h
    libs/qwt/src/qwt_column_symbol.cpp libs/qwt/src/qwt_column_symbol.h
    libs/qwt/src/qwt_date.cpp libs/qwt/src/qwt_date.h
    libs/qwt/src/qwt_date_scale_draw.cpp libs/qwt/src/qwt_date_scale_draw.h
    libs/qwt/src/qwt_date_scale_engine.cpp libs/qwt/src/qwt_date_scale_engine.h
    libs/qwt/src/qwt_dyngrid_layout.cpp libs/qwt/src/qwt_dyngrid_layout.h
    libs/qwt/src/qwt_event_pattern.cpp
    libs/qwt/src/qwt_global.h
    libs/qwt/src/qwt_graphic.cpp libs/qwt/src/qwt_graphic.h
    libs/qwt/src/qwt_interval.cpp libs/qwt/src/qwt_interval.h
    libs/qwt/src/qwt_interval_symbol.cpp libs/qwt/src/qwt_interval_symbol.h
    libs/qwt/src/qwt_magnifier.cpp libs/qwt/src/qwt_magnifier.h
    libs/qwt/src/qwt_math.cpp libs/qwt/src/qwt_math.h
    libs/qwt/src/qwt_null_paintdevice.cpp libs/qwt/src/qwt_null_paintdevice.h
    libs/qwt/src/qwt_painter.cpp libs/qwt/src/qwt_painter.h
    libs/qwt/src/qwt_painter_command.cpp libs/qwt/src/qwt_painter_command.h
    libs/qwt/src/qwt_panner.cpp libs/qwt/src/qwt_panner.h
    libs/qwt/src/qwt_picker.cpp libs/qwt/src/qwt_picker.h
    libs/qwt/src/qwt_picker_machine.cpp libs/qwt/src/qwt_picker_machine.h
    libs/qwt/src/qwt_pixel_matrix.cpp libs/qwt/src/qwt_pixel_matrix.h
    libs/qwt/src/qwt_point_3d.cpp libs/qwt/src/qwt_point_3d.h
    libs/qwt/src/qwt_point_polar.cpp libs/qwt/src/qwt_point_polar.h
    libs/qwt/src/qwt_round_scale_draw.cpp libs/qwt/src/qwt_round_scale_draw.h
    libs/qwt/src/qwt_scale_div.cpp libs/qwt/src/qwt_scale_div.h
    libs/qwt/src/qwt_scale_draw.cpp libs/qwt/src/qwt_scale_draw.h
    libs/qwt/src/qwt_scale_engine.cpp libs/qwt/src/qwt_scale_engine.h
    libs/qwt/src/qwt_scale_map.cpp libs/qwt/src/qwt_scale_map.h
    libs/qwt/src/qwt_spline.cpp libs/qwt/src/qwt_spline.h
    libs/qwt/src/qwt_spline_basis.cpp libs/qwt/src/qwt_spline_basis.h
    libs/qwt/src/qwt_spline_cubic.cpp libs/qwt/src/qwt_spline_cubic.h
    libs/qwt/src/qwt_spline_local.cpp libs/qwt/src/qwt_spline_local.h
    libs/qwt/src/qwt_spline_parametrization.cpp libs/qwt/src/qwt_spline_parametrization.h
    libs/qwt/src/qwt_spline_pleasing.cpp libs/qwt/src/qwt_spline_pleasing.h
    libs/qwt/src/qwt_spline_polynomial.cpp libs/qwt/src/qwt_spline_polynomial.h
    libs/qwt/src/qwt_symbol.cpp libs/qwt/src/qwt_symbol.h
    libs/qwt/src/qwt_system_clock.cpp libs/qwt/src/qwt_system_clock.h
    libs/qwt/src/qwt_text.cpp libs/qwt/src/qwt_text.h
    libs/qwt/src/qwt_text_engine.cpp libs/qwt/src/qwt_text_engine.h
    libs/qwt/src/qwt_text_label.cpp libs/qwt/src/qwt_text_label.h
    libs/qwt/src/qwt_transform.cpp libs/qwt/src/qwt_transform.h
    libs/qwt/src/qwt_widget_overlay.cpp libs/qwt/src/qwt_widget_overlay.h
    src/AppInfo.h
    src/CSV/Export.cpp src/CSV/Export.h
    src/CSV/Player.cpp src/CSV/Player.h
    src/DataTypes.h
    src/IO/Checksum.cpp src/IO/Checksum.h
    src/IO/Console.cpp src/IO/Console.h
    src/IO/Drivers/BluetoothLE.cpp src/IO/Drivers/BluetoothLE.h
    src/IO/Drivers/Network.cpp src/IO/Drivers/Network.h
    src/IO/Drivers/Serial.cpp src/IO/Drivers/Serial.h
    src/IO/HAL_Driver.h
    src/IO/Manager.cpp src/IO/Manager.h
    src/JSON/Dataset.cpp src/JSON/Dataset.h
    src/JSON/Frame.cpp src/JSON/Frame.h
    src/JSON/Generator.cpp src/JSON/Generator.h
    src/JSON/Group.cpp src/JSON/Group.h
    src/MQTT/Client.cpp src/MQTT/Client.h
    src/Misc/MacExtras.cpp src/Misc/MacExtras.h
    src/Misc/ModuleManager.cpp src/Misc/ModuleManager.h
    src/Misc/ThemeManager.cpp src/Misc/ThemeManager.h
    src/Misc/TimerEvents.cpp src/Misc/TimerEvents.h
    src/Misc/Translator.cpp src/Misc/Translator.h
    src/Misc/Utilities.cpp src/Misc/Utilities.h
    src/Plugins/Server.cpp src/Plugins/Server.h
    src/Project/CodeEditor.cpp src/Project/CodeEditor.h
    src/Project/Model.cpp src/Project/Model.h
    src/UI/Dashboard.cpp src/UI/Dashboard.h
    src/UI/DashboardWidget.cpp src/UI/DashboardWidget.h
    src/UI/DeclarativeWidget.cpp src/UI/DeclarativeWidget.h
    src/UI/Widgets/Accelerometer.cpp src/UI/Widgets/Accelerometer.h
    src/UI/Widgets/Bar.cpp src/UI/Widgets/Bar.h
    src/UI/Widgets/Common/AnalogGauge.cpp src/UI/Widgets/Common/AnalogGauge.h
    src/UI/Widgets/Common/AttitudeIndicator.cpp src/UI/Widgets/Common/AttitudeIndicator.h
    src/UI/Widgets/Common/BaseWidget.cpp src/UI/Widgets/Common/BaseWidget.h
    src/UI/Widgets/Common/ElidedLabel.cpp src/UI/Widgets/Common/ElidedLabel.h
    src/UI/Widgets/Common/KLed.cpp src/UI/Widgets/Common/KLed.h
    src/UI/Widgets/Compass.cpp src/UI/Widgets/Compass.h
    src/UI/Widgets/DataGroup.cpp src/UI/Widgets/DataGroup.h
    src/UI/Widgets/FFTPlot.cpp src/UI/Widgets/FFTPlot.h
    src/UI/Widgets/GPS.cpp src/UI/Widgets/GPS.h
    src/UI/Widgets/Gauge.cpp src/UI/Widgets/Gauge.h
    src/UI/Widgets/Gyroscope.cpp src/UI/Widgets/Gyroscope.h
    src/UI/Widgets/LEDPanel.cpp src/UI/Widgets/LEDPanel.h
    src/UI/Widgets/MultiPlot.cpp src/UI/Widgets/MultiPlot.h
    src/UI/Widgets/Plot.cpp src/UI/Widgets/Plot.h
    src/UI/Widgets/Terminal.cpp src/UI/Widgets/Terminal.h
    src/main.cpp

        ${RESOURCE}
)
qt_add_qml_module(serial-studio
    URI serial_studio
    VERSION ${PROJECT_VERSION}
    QML_FILES
        assets/qml/Dashboard/DashboardTitle.qml
        assets/qml/Dashboard/ViewOptions.qml
        assets/qml/Dashboard/ViewOptionsDelegate.qml
        assets/qml/Dashboard/WidgetDelegate.qml
        assets/qml/Dashboard/WidgetGrid.qml
        assets/qml/Dashboard/WidgetModel.qml
        assets/qml/FramelessWindow/CustomWindow.qml
        assets/qml/FramelessWindow/ResizeHandles.qml
        assets/qml/FramelessWindow/Titlebar.qml
        assets/qml/FramelessWindow/WindowButton.qml
        assets/qml/FramelessWindow/WindowButtonMacOS.qml
        assets/qml/Panes/SetupPanes/Devices/BluetoothLE.qml
        assets/qml/Panes/SetupPanes/Devices/Network.qml
        assets/qml/Panes/SetupPanes/Devices/Serial.qml
        assets/qml/Panes/SetupPanes/Hardware.qml
        assets/qml/Panes/SetupPanes/MQTT.qml
        assets/qml/Panes/SetupPanes/Settings.qml
        assets/qml/Panes/Console.qml
        assets/qml/Panes/Dashboard.qml
        assets/qml/Panes/Setup.qml
        assets/qml/Panes/Toolbar.qml
        assets/qml/PlatformDependent/DecentMenuItem.qml
        assets/qml/PlatformDependent/Menubar.qml
        assets/qml/PlatformDependent/MenubarMacOS.qml
        assets/qml/ProjectEditor/Footer.qml
        assets/qml/ProjectEditor/GroupEditor.qml
        assets/qml/ProjectEditor/Header.qml
        assets/qml/ProjectEditor/JsonDatasetDelegate.qml
        assets/qml/ProjectEditor/JsonGroupDelegate.qml
        assets/qml/ProjectEditor/TreeView.qml
        assets/qml/Widgets/Icon.qml
        assets/qml/Widgets/JSONDropArea.qml
        assets/qml/Widgets/Shadow.qml
        assets/qml/Widgets/Terminal.qml
        assets/qml/Widgets/Window.qml
        assets/qml/Windows/About.qml
        assets/qml/Windows/Acknowledgements.qml
        assets/qml/Windows/CsvPlayer.qml
        assets/qml/Windows/Donate.qml
        assets/qml/Windows/MainWindow.qml
        assets/qml/Windows/MQTTConfiguration.qml
        assets/qml/Windows/ProjectEditor.qml
        assets/qml/main.qml
        assets/scripts/frame-parser.js
#    RESOURCES
#        assets/fonts/Roboto-Bold.ttf
#        assets/fonts/Roboto-Regular.ttf
#        assets/fonts/RobotoMono-Bold.ttf
#        assets/fonts/RobotoMono-Regular.ttf
#        assets/icons/ac-power.svg
#        assets/icons/accelerometer.svg
#        assets/icons/add.svg
#        assets/icons/apply.svg
#        assets/icons/arrow-right.svg
#        assets/icons/backspace.svg
#        assets/icons/bar.svg
#        assets/icons/battery.svg
#        assets/icons/bug.svg
#        assets/icons/close.svg
#        assets/icons/code.svg
#        assets/icons/compass.svg
#        assets/icons/connect.svg
#        assets/icons/copy.svg
#        assets/icons/cut.svg
#        assets/icons/dashboard.svg
#        assets/icons/dataset.svg
#        assets/icons/delete-item.svg
#        assets/icons/delete.svg
#        assets/icons/developer-board.svg
#        assets/icons/device-hub.svg
#        assets/icons/disconnect.svg
#        assets/icons/down.svg
#        assets/icons/drag-drop.svg
#        assets/icons/end-sequence.svg
#        assets/icons/error.svg
#        assets/icons/ethernet.svg
#        assets/icons/expand.svg
#        assets/icons/fft.svg
#        assets/icons/fullscreen-exit.svg
#        assets/icons/fullscreen.svg
#        assets/icons/gauge.svg
#        assets/icons/gps.svg
#        assets/icons/graphs.svg
#        assets/icons/group.svg
#        assets/icons/gyro.svg
#        assets/icons/heart-broken.svg
#        assets/icons/help.svg
#        assets/icons/hide-all.svg
#        assets/icons/info.svg
#        assets/icons/json.svg
#        assets/icons/led.svg
#        assets/icons/link.svg
#        assets/icons/location-off.svg
#        assets/icons/location-on.svg
#        assets/icons/map.svg
#        assets/icons/media-next.svg
#        assets/icons/media-pause.svg
#        assets/icons/media-play.svg
#        assets/icons/media-prev.svg
#        assets/icons/multiplot.svg
#        assets/icons/new.svg
#        assets/icons/open.svg
#        assets/icons/plot.svg
#        assets/icons/points.svg
#        assets/icons/power.svg
#        assets/icons/ram.svg
#        assets/icons/redo.svg
#        assets/icons/refresh.svg
#        assets/icons/registration.svg
#        assets/icons/restore.svg
#        assets/icons/right.svg
#        assets/icons/save.svg
#        assets/icons/schedule.svg
#        assets/icons/scroll-bottom.svg
#        assets/icons/scroll-down.svg
#        assets/icons/scroll-top.svg
#        assets/icons/scroll-up.svg
#        assets/icons/search.svg
#        assets/icons/send.svg
#        assets/icons/separator.svg
#        assets/icons/settings.svg
#        assets/icons/show-all.svg
#        assets/icons/start-sequence.svg
#        assets/icons/template.svg
#        assets/icons/thermometer.svg
#        assets/icons/undo.svg
#        assets/icons/up.svg
#        assets/icons/update.svg
#        assets/icons/usb.svg
#        assets/icons/visibility.svg
#        assets/icons/warning.svg
#        assets/icons/widget.svg
#        assets/images/donate-qr.svg
#        assets/images/icon-small@1x.png
#        assets/images/icon-small@2x.png
#        assets/images/icon-window.svg
#        assets/images/icon@1x.png
#        assets/images/icon@2x.png
#        assets/images/shadow.png
#        assets/images/window-shadow.png
#        assets/messages/Acknowledgements.txt
#        assets/messages/Welcome_DE.txt
#        assets/messages/Welcome_EN.txt
#        assets/messages/Welcome_ES.txt
#        assets/messages/Welcome_RU.txt
#        assets/messages/Welcome_ZH.txt
#        assets/themes/1_Flat.json
#        assets/themes/2_Dark.json
#        assets/themes/3_Classic.json
#        assets/themes/4_Midnight.json
#        assets/themes/5_Noir.json
#        assets/themes/6_Mathworks.json
#        assets/touchbar/console.png
#        assets/touchbar/dashboard.png
#        assets/touchbar/setup.png
#        assets/translations/de.qm
#        assets/translations/de.ts
#        assets/translations/en.qm
#        assets/translations/en.ts
#        assets/translations/es.qm
#        assets/translations/es.ts
#        assets/translations/ru.qm
#        assets/translations/ru.ts
#        assets/translations/zh.qm
#        assets/translations/zh.ts
#        assets/window-border/macOS/close-active.svg
#        assets/window-border/macOS/close-hover.svg
#        assets/window-border/macOS/close-normal.svg
#        assets/window-border/macOS/maximize-active.svg
#        assets/window-border/macOS/maximize-hover.svg
#        assets/window-border/macOS/maximize-normal.svg
#        assets/window-border/macOS/minimize-active.svg
#        assets/window-border/macOS/minimize-hover.svg
#        assets/window-border/macOS/minimize-normal.svg
#        assets/window-border/close.svg
#        assets/window-border/fullscreen.svg
#        assets/window-border/maximize.svg
#        assets/window-border/minimize.svg
#        assets/window-border/restore.svg
#        assets/window-border/unmaximize.svg
#        assets/icons/paste.svg
#        ${RESOURCE}
    NO_RESOURCE_TARGET_PATH
)

target_include_directories(serial-studio PRIVATE
    libs
    libs/QMapControl/src
    libs/QRealFourier/fftreal
    libs/QRealFourier/headers
    libs/QSimpleUpdater/include
    libs/QSourceHighlite
    libs/qmqtt/src/mqtt
    libs/qtcsv
    libs/qtcsv/include
    libs/qwt/classincludes
    libs/qwt/src
    src
)

target_compile_definitions(serial-studio PRIVATE
    MQTT_INCLUDE_MOC=1
    MQTT_PROJECT_INCLUDE_SRC
    QMAPCONTROL_PROJECT_INCLUDE_SRC
    QSU_INCLUDE_MOC=1
    QTCSV_STATIC_LIB
    QT_DISABLE_DEPRECATED_BEFORE=0x050F00
    QWT_MOC_INCLUDE=1
    SERIAL_STUDIO_INCLUDE_MOC
    _USE_MATH_DEFINES
)

target_link_libraries(serial-studio PRIVATE
    Qt::Bluetooth
    Qt::Core
    Qt::Gui
    Qt::Network
    Qt::PrintSupport
    Qt::Qml
    Qt::Quick
    Qt::QuickControls2
    Qt::SerialPort
    Qt::Svg
    Qt::Widgets
    Qt::Xml
)


# Resources:
set_source_files_properties("libs/QSimpleUpdater/etc/resources/update.png"
    PROPERTIES QT_RESOURCE_ALIAS "update.png"
)
set(qsimpleupdater_resource_files
    "libs/QSimpleUpdater/etc/resources/update.png"
)

qt_add_resources(serial-studio "qsimpleupdater"
    PREFIX
        "/icons"
    BASE
        "libs/QSimpleUpdater/etc/resources"
    FILES
        ${qsimpleupdater_resource_files}
)

if((QT_VERSION_MAJOR EQUAL 6))
    target_link_libraries(serial-studio PRIVATE
        Qt::Core5Compat
    )
endif()

if(CMAKE_BUILD_TYPE STREQUAL Release AND _x_msvc_x_)
    target_include_directories(serial-studio PRIVATE
        ${CMAKE_CURRENT_BINARY_DIR}
        ${CMAKE_CURRENT_BINARY_DIR}/debug
        ${CMAKE_CURRENT_BINARY_DIR}/release
    )

    target_compile_options(serial-studio
        -MP
    )
endif()

if(win32_x_)
    target_include_directories(serial-studio PRIVATE
        libs/OpenSSL/include
    )

    target_link_libraries(serial-studio PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/dll/Windows/libcrypto.lib
        ${CMAKE_CURRENT_SOURCE_DIR}/dll/Windows/libssl.lib
        advapi32
        crypt32
        gdi32
        user32
        ws2_32
    )
endif()

if(macx_x_)
    target_sources(serial-studio PUBLIC
        libs/KDMacTouchBar/src/kdmactouchbar.h libs/KDMacTouchBar/src/kdmactouchbar.mm
        libs/KDMacTouchBar/src/kdmactouchbar_global.h
    )

    target_include_directories(serial-studio PRIVATE
        libs/KDMacTouchBar/src
        libs/OpenSSL/include
    )

    target_compile_definitions(serial-studio PRIVATE
        KDMACTOUCHBAR_BUILD_KDMACTOUCHBAR_SRC
    )

    target_link_libraries(serial-studio PRIVATE
        "-framework Cocoa"
        ${CMAKE_CURRENT_SOURCE_DIR}/dll/macOS/libcrypto.a
        ${CMAKE_CURRENT_SOURCE_DIR}/dll/macOS/libssl.a
    )
endif()

if(LINUX)
    target_link_libraries(serial-studio PRIVATE
        crypto
        ssl
    )
endif()

if(MINGW)
    target_compile_definitions(serial-studio PRIVATE
        DISABLE_QSU
    )
endif()

#if(QWT_CONFIG___contains___QwtPlot)
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_abstract_legend.cpp libs/qwt/src/qwt_abstract_legend.h
        libs/qwt/src/qwt_axis.h
        libs/qwt/src/qwt_axis_id.h
        libs/qwt/src/qwt_curve_fitter.cpp libs/qwt/src/qwt_curve_fitter.h
        libs/qwt/src/qwt_event_pattern.h
        libs/qwt/src/qwt_legend.cpp libs/qwt/src/qwt_legend.h
        libs/qwt/src/qwt_legend_data.cpp libs/qwt/src/qwt_legend_data.h
        libs/qwt/src/qwt_legend_label.cpp libs/qwt/src/qwt_legend_label.h
        libs/qwt/src/qwt_matrix_raster_data.cpp libs/qwt/src/qwt_matrix_raster_data.h
        libs/qwt/src/qwt_plot.cpp libs/qwt/src/qwt_plot.h
        libs/qwt/src/qwt_plot_abstract_barchart.cpp libs/qwt/src/qwt_plot_abstract_barchart.h
        libs/qwt/src/qwt_plot_abstract_canvas.cpp libs/qwt/src/qwt_plot_abstract_canvas.h
        libs/qwt/src/qwt_plot_axis.cpp
        libs/qwt/src/qwt_plot_barchart.cpp libs/qwt/src/qwt_plot_barchart.h
        libs/qwt/src/qwt_plot_canvas.cpp libs/qwt/src/qwt_plot_canvas.h
        libs/qwt/src/qwt_plot_curve.cpp libs/qwt/src/qwt_plot_curve.h
        libs/qwt/src/qwt_plot_dict.cpp libs/qwt/src/qwt_plot_dict.h
        libs/qwt/src/qwt_plot_directpainter.cpp libs/qwt/src/qwt_plot_directpainter.h
        libs/qwt/src/qwt_plot_graphicitem.cpp libs/qwt/src/qwt_plot_graphicitem.h
        libs/qwt/src/qwt_plot_grid.cpp libs/qwt/src/qwt_plot_grid.h
        libs/qwt/src/qwt_plot_histogram.cpp libs/qwt/src/qwt_plot_histogram.h
        libs/qwt/src/qwt_plot_intervalcurve.cpp libs/qwt/src/qwt_plot_intervalcurve.h
        libs/qwt/src/qwt_plot_item.cpp libs/qwt/src/qwt_plot_item.h
        libs/qwt/src/qwt_plot_layout.cpp libs/qwt/src/qwt_plot_layout.h
        libs/qwt/src/qwt_plot_legenditem.cpp libs/qwt/src/qwt_plot_legenditem.h
        libs/qwt/src/qwt_plot_magnifier.cpp libs/qwt/src/qwt_plot_magnifier.h
        libs/qwt/src/qwt_plot_marker.cpp libs/qwt/src/qwt_plot_marker.h
        libs/qwt/src/qwt_plot_multi_barchart.cpp libs/qwt/src/qwt_plot_multi_barchart.h
        libs/qwt/src/qwt_plot_panner.cpp libs/qwt/src/qwt_plot_panner.h
        libs/qwt/src/qwt_plot_picker.cpp libs/qwt/src/qwt_plot_picker.h
        libs/qwt/src/qwt_plot_rasteritem.cpp libs/qwt/src/qwt_plot_rasteritem.h
        libs/qwt/src/qwt_plot_renderer.cpp libs/qwt/src/qwt_plot_renderer.h
        libs/qwt/src/qwt_plot_rescaler.cpp libs/qwt/src/qwt_plot_rescaler.h
        libs/qwt/src/qwt_plot_scaleitem.cpp libs/qwt/src/qwt_plot_scaleitem.h
        libs/qwt/src/qwt_plot_seriesitem.cpp libs/qwt/src/qwt_plot_seriesitem.h
        libs/qwt/src/qwt_plot_shapeitem.cpp libs/qwt/src/qwt_plot_shapeitem.h
        libs/qwt/src/qwt_plot_spectrocurve.cpp libs/qwt/src/qwt_plot_spectrocurve.h
        libs/qwt/src/qwt_plot_spectrogram.cpp libs/qwt/src/qwt_plot_spectrogram.h
        libs/qwt/src/qwt_plot_textlabel.cpp libs/qwt/src/qwt_plot_textlabel.h
        libs/qwt/src/qwt_plot_tradingcurve.cpp libs/qwt/src/qwt_plot_tradingcurve.h
        libs/qwt/src/qwt_plot_vectorfield.cpp libs/qwt/src/qwt_plot_vectorfield.h
        libs/qwt/src/qwt_plot_zoneitem.cpp libs/qwt/src/qwt_plot_zoneitem.h
        libs/qwt/src/qwt_plot_zoomer.cpp libs/qwt/src/qwt_plot_zoomer.h
        libs/qwt/src/qwt_point_data.cpp libs/qwt/src/qwt_point_data.h
        libs/qwt/src/qwt_point_mapper.cpp libs/qwt/src/qwt_point_mapper.h
        libs/qwt/src/qwt_raster_data.cpp libs/qwt/src/qwt_raster_data.h
        libs/qwt/src/qwt_samples.h
        libs/qwt/src/qwt_sampling_thread.cpp libs/qwt/src/qwt_sampling_thread.h
        libs/qwt/src/qwt_scale_widget.cpp libs/qwt/src/qwt_scale_widget.h
        libs/qwt/src/qwt_series_data.cpp libs/qwt/src/qwt_series_data.h
        libs/qwt/src/qwt_series_store.h
        libs/qwt/src/qwt_spline_curve_fitter.cpp libs/qwt/src/qwt_spline_curve_fitter.h
        libs/qwt/src/qwt_vectorfield_symbol.cpp libs/qwt/src/qwt_vectorfield_symbol.h
        libs/qwt/src/qwt_weeding_curve_fitter.cpp libs/qwt/src/qwt_weeding_curve_fitter.h
    )
#endif()

if(((QWT_CONFIG___contains___QwtPlot) AND (QWT_CONFIG___contains___QwtOpenGL)) AND ((QT_VERSION_MAJOR LESS 6)))
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_plot_glcanvas.cpp libs/qwt/src/qwt_plot_glcanvas.h
    )
endif()

if(((((QWT_CONFIG___contains___QwtPlot) AND (QWT_CONFIG___contains___QwtOpenGL)) AND ((QT_VERSION_MAJOR GREATER 4))) AND (( QT_MAJOR_VERSION LESS 6))) AND ((QT_VERSION_MINOR GREATER 3)))
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_plot_opengl_canvas.cpp libs/qwt/src/qwt_plot_opengl_canvas.h
    )
endif()

if((((QWT_CONFIG___contains___QwtPlot) AND (QWT_CONFIG___contains___QwtOpenGL)) AND ((QT_VERSION_MAJOR GREATER 4))) AND (NOT (( QT_MAJOR_VERSION LESS 6))))
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_plot_opengl_canvas.cpp libs/qwt/src/qwt_plot_opengl_canvas.h
    )

    target_link_libraries(serial-studio PRIVATE
        Qt::OpenGLWidgets
    )
endif()

if(QWT_CONFIG___contains___QwtPlot AND QWT_CONFIG___contains___QwtSvg)
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_plot_svgitem.cpp libs/qwt/src/qwt_plot_svgitem.h
    )
endif()

if(QWT_CONFIG___contains___QwtPlot AND QWT_CONFIG___contains___QwtPolar)
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_polar.h
        libs/qwt/src/qwt_polar_canvas.cpp libs/qwt/src/qwt_polar_canvas.h
        libs/qwt/src/qwt_polar_curve.cpp libs/qwt/src/qwt_polar_curve.h
        libs/qwt/src/qwt_polar_fitter.cpp libs/qwt/src/qwt_polar_fitter.h
        libs/qwt/src/qwt_polar_grid.cpp libs/qwt/src/qwt_polar_grid.h
        libs/qwt/src/qwt_polar_item.cpp libs/qwt/src/qwt_polar_item.h
        libs/qwt/src/qwt_polar_itemdict.cpp libs/qwt/src/qwt_polar_itemdict.h
        libs/qwt/src/qwt_polar_layout.cpp libs/qwt/src/qwt_polar_layout.h
        libs/qwt/src/qwt_polar_magnifier.cpp libs/qwt/src/qwt_polar_magnifier.h
        libs/qwt/src/qwt_polar_marker.cpp libs/qwt/src/qwt_polar_marker.h
        libs/qwt/src/qwt_polar_panner.cpp libs/qwt/src/qwt_polar_panner.h
        libs/qwt/src/qwt_polar_picker.cpp libs/qwt/src/qwt_polar_picker.h
        libs/qwt/src/qwt_polar_plot.cpp libs/qwt/src/qwt_polar_plot.h
        libs/qwt/src/qwt_polar_renderer.cpp libs/qwt/src/qwt_polar_renderer.h
        libs/qwt/src/qwt_polar_spectrogram.cpp libs/qwt/src/qwt_polar_spectrogram.h
    )
endif()

if((QT_VERSION_MAJOR GREATER 4))
    target_link_libraries(serial-studio PRIVATE
        Qt::Concurrent
        Qt::PrintSupport
    )
endif()

if(((QWT_CONFIG___contains___QwtSvg) AND ((QT_VERSION_MAJOR GREATER 4))) AND (TARGET Qt::Svg))
    target_link_libraries(serial-studio PRIVATE
        Qt::Svg
    )
endif()

if((QWT_CONFIG___contains___QwtSvg) AND (NOT ((QT_VERSION_MAJOR GREATER 4))))
    target_link_libraries(serial-studio PRIVATE
        Qt::Svg
    )
endif()

if(NOT QWT_CONFIG___contains___QwtSvg)
    target_compile_definitions(serial-studio PRIVATE
        QWT_NO_SVG
    )
endif()

if(QWT_CONFIG___contains___QwtOpenGL)
    target_link_libraries(serial-studio PRIVATE
        Qt::OpenGL
    )
endif()

if(((QWT_CONFIG___contains___QwtOpenGL) AND ((QT_VERSION_MAJOR GREATER 4))) AND (TARGET Qt::OpenGL))
    target_link_libraries(serial-studio PRIVATE
        Qt::OpenGL
    )
endif()

if((QWT_CONFIG___contains___QwtOpenGL) AND (NOT ((QT_VERSION_MAJOR GREATER 4))))
    target_link_libraries(serial-studio PRIVATE
        Qt::OpenGL
    )
endif()

if(NOT QWT_CONFIG___contains___QwtOpenGL)
    target_compile_definitions(serial-studio PRIVATE
        QWT_NO_OPENGL
    )
endif()

#if(QWT_CONFIG___contains___QwtWidgets)
    target_sources(serial-studio PUBLIC
        libs/qwt/src/qwt_abstract_scale.cpp libs/qwt/src/qwt_abstract_scale.h
        libs/qwt/src/qwt_abstract_slider.cpp libs/qwt/src/qwt_abstract_slider.h
        libs/qwt/src/qwt_analog_clock.cpp libs/qwt/src/qwt_analog_clock.h
        libs/qwt/src/qwt_arrow_button.cpp libs/qwt/src/qwt_arrow_button.h
        libs/qwt/src/qwt_compass.cpp libs/qwt/src/qwt_compass.h
        libs/qwt/src/qwt_compass_rose.cpp libs/qwt/src/qwt_compass_rose.h
        libs/qwt/src/qwt_counter.cpp libs/qwt/src/qwt_counter.h
        libs/qwt/src/qwt_dial.cpp libs/qwt/src/qwt_dial.h
        libs/qwt/src/qwt_dial_needle.cpp libs/qwt/src/qwt_dial_needle.h
        libs/qwt/src/qwt_knob.cpp libs/qwt/src/qwt_knob.h
        libs/qwt/src/qwt_slider.cpp libs/qwt/src/qwt_slider.h
        libs/qwt/src/qwt_thermo.cpp libs/qwt/src/qwt_thermo.h
        libs/qwt/src/qwt_wheel.cpp libs/qwt/src/qwt_wheel.h
    )
#endif()

if(NOT DEFINES___contains___QTCSV_LIBRARY)
    target_compile_definitions(serial-studio PRIVATE
        QTCSV_MAKE_LIB
    )
endif()

if(QMQTT_WEBSOCKETS)
    target_sources(serial-studio PUBLIC
        libs/qmqtt/src/mqtt/qmqtt_websocket.cpp
        libs/qmqtt/src/mqtt/qmqtt_websocketiodevice.cpp
    )
endif()

install(TARGETS serial-studio
    BUNDLE DESTINATION .
    RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
)

qt_generate_deploy_qml_app_script(
    TARGET serial-studio
    FILENAME_VARIABLE deploy_script
    NO_UNSUPPORTED_PLATFORM_ERROR
    DEPLOY_USER_QML_MODULES_ON_UNSUPPORTED_PLATFORM
    MACOS_BUNDLE_POST_BUILD
)
install(SCRIPT ${deploy_script})
