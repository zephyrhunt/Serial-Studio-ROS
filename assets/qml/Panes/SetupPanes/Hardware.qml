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
import Qt.labs.settings as QtSettings
import "Devices" as Devices
import "../../Windows" as Windows

Control {
    id: root
    // Save settings
    QtSettings.Settings {
        property alias address: network.address
        property alias autoReconnect: serial.autoReconnect
        property alias baudRate: serial.baudRate
        property alias dataBits: serial.dataBits
        property alias driver: _driverCombo.currentIndex
        property alias flowControl: serial.flowControl
        property alias parity: serial.parity
        property alias socketType: network.socketType
        property alias stopBits: serial.stopBits
        property alias tcpPort: network.tcpPort
        property alias udpLocalPort: network.udpLocalPort
        property alias udpMulticastEnabled: network.udpMulticastEnabled
        property alias udpProcessDatagramsDirectly: network.udpProcessDatagramsDirectly
        property alias udpRemotePort: network.udpRemotePort
    }
    ColumnLayout {
        id: layout
        anchors.fill: parent
        anchors.margins: app.spacing

        // Device type selector
        RowLayout {
            Layout.fillWidth: true
            spacing: app.spacing

            Label {
                Layout.alignment: Qt.AlignVCenter
                text: qsTr("Data source") + ":"
            }
            ComboBox {
                id: _driverCombo
                Layout.alignment: Qt.AlignVCenter
                Layout.fillWidth: true
                model: Cpp_IO_Manager.availableDrivers()

                onCurrentIndexChanged: Cpp_IO_Manager.selectedDriver = currentIndex
            }
        }
        // Device configuration
        StackLayout {
            id: stack
            Layout.fillHeight: true
            Layout.fillWidth: true
            clip: true
            currentIndex: Cpp_IO_Manager.selectedDriver

            Devices.Serial {
                id: serial
                Layout.fillHeight: true
                Layout.fillWidth: true

                background: TextField {
                    enabled: false
                }
            }
            Devices.Network {
                id: network
                Layout.fillHeight: true
                Layout.fillWidth: true

                background: TextField {
                    enabled: false
                }
            }
            Devices.BluetoothLE {
                id: bluetoothLE
                Layout.fillHeight: true
                Layout.fillWidth: true

                background: TextField {
                    enabled: false
                }
            }
            Devices.Ros {
                id: ros
                Layout.fillHeight: true
                Layout.fillWidth: true

                background: TextField {
                    enabled: false
                }
            }
        }
    }
}
