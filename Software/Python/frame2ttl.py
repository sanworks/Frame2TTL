"""
----------------------------------------------------------------------------

This file is part of the Sanworks Frame2TTL repository
Copyright (C) 2023 Sanworks LLC, Rochester, New York, USA

----------------------------------------------------------------------------

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, version 3.

This program is distributed  WITHOUT ANY WARRANTY and without even the
implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
See the GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.

----------------------------------------------------------------------------

Usage:
Initialize:
F = Frame2TTL('COM3') # Where COM3 is the Frame2TTL device's usb serial port
F.dark_threshold = -75 # Set the threshold for detecting a light -> dark sync patch transition.
                         Units are bits / ms, a 1ms sliding window average of sample-wise change in luminance
F.set_dark_threshold_auto()  # Automatically set the dark threshold, while the sync patch is white at max intensity
sensorValue = F.read_sensor()  # Return the light sensor's current luminance measurement. Units are bits in range 0, 2^16
sensorValues = F.read_sensor(1000)  # Return 1000 consecutive luminance measurements from the sensor
F.stream_ui() # Launches a GUI window to view the raw sensor data

Disconnect:
del F

"""

from PyQt5 import QtWidgets, QtCore
from pyqtgraph import AxisItem
import pyqtgraph as pg
import numpy as np
import sys
import time
import math
from arcom import ArCom  # ArCOM wraps PySerial to simplify transactions of numpy data types

# Constants
MIN_SUPPORTED_HARDWARE = 2
MIN_SUPPORTED_FIRMWARE = 2
HANDSHAKE_BYTE = 218


class Frame2TTL:
    """Python class to control and read from the Frame2TTL device."""

    def __init__(self, port_name):

        # Create GUI vars
        self.qt_app = None  # for GUI
        self.qt_main = None  # for GUI

        # Connect to hardware
        self.port = ArCom(port_name, 12000000)
        self.port.write(ord('C'), 'uint8')  # Handshake
        reply = self.port.read(1, 'uint8')
        if reply != HANDSHAKE_BYTE:
            raise Frame2TTLError('Error: An incorrect handshake byte was received.')

        # Verify firmware version
        self.port.write(ord('F'), 'uint8')  # Request Firmware version (this op did not exist in firmware v1)
        time.sleep(0.25)  # Wait for a reply. Firmware v1 will return nothing
        old_firmware_found = 0
        if self.port.bytes_available() == 0:
            old_firmware_found = 1
        self._firmware_version = self.port.read(1, 'uint8')
        if old_firmware_found or self._firmware_version < MIN_SUPPORTED_FIRMWARE:
            raise Frame2TTLError('Error: Old Frame2TTL firmware detected. Update to firmware v' +
                                 str(MIN_SUPPORTED_FIRMWARE) + ' or newer.')

        # Verify hardware version
        self.port.write(ord('#'), 'uint8')  # Request HW version.
        self._hardware_version = self.port.read(1, 'uint8')
        if self._hardware_version == 3:  # For HW 3, reinitialize to full baud rate
            self.port = []
            time.sleep(0.25)
            self.port = ArCom(port_name, 480000000)

        # Set default thresholds
        if self._hardware_version == 2:
            self._light_threshold = 100
            self._dark_threshold = -150
        elif self._hardware_version == 3:
            self._light_threshold = 75
            self._dark_threshold = -75

    @property
    def light_threshold(self):
        return self._light_threshold

    @light_threshold.setter
    def light_threshold(self, value):
        if value <= 0 or not isinstance(value, int):
            raise Frame2TTLError('Error: light_threshold must be a positive integer.')
        self.port.write(ord('T'), 'uint8', [value, self._dark_threshold], 'int16')
        self._light_threshold = value

    @property
    def dark_threshold(self):
        return self._dark_threshold

    @dark_threshold.setter
    def dark_threshold(self, value):
        if value >= 0 or not isinstance(value, int):
            raise Frame2TTLError('Error: dark_threshold must be a negative integer.')
        self.port.write(ord('T'), 'uint8', [self._light_threshold, value], 'int16')
        self._dark_threshold = value

    def set_dark_threshold_auto(self):
        """Auto-set threshold for detecting light --> dark transitions. Run with the sync patch set to WHITE."""
        self.port.write(ord('D'), 'uint8')
        time.sleep(3)  # The instrument measures for ~2.5 seconds and calculates the new threshold
        self._dark_threshold = self.port.read(1, 'int16')

    def set_light_threshold_auto(self):
        """Auto-set threshold for detecting dark --> light transitions. Run with the sync patch set to BLACK."""
        self.port.write(ord('L'), 'uint8')
        time.sleep(3)  # The instrument measures for ~2.5 seconds and calculates the new threshold
        self._light_threshold = self.port.read(1, 'int16')

    def read_sensor(self, n_samples=1):
        """Returns raw light measurements from the sensor."""
        if n_samples <= 0 or not isinstance(n_samples, int):
            raise Frame2TTLError('Error: n_samples must be a positive integer.')
        self.port.write(ord('V'), 'uint8', n_samples, 'uint32')
        measurements = self.port.read(n_samples, 'uint16')
        return measurements

    def stream_ui(self):
        """Launches a plot to display a live stream of light sensor measurements."""
        if not QtWidgets.QApplication.instance():
            self.qt_app = QtWidgets.QApplication(sys.argv)
        else:
            self.qt_app = QtWidgets.QApplication.instance()

        self.qt_main = MainWindow(self.port)
        self.qt_main.show()
        self.qt_app.exec_()

    # Self-description when the object is entered into the Python console with no properties or methods specified
    def __repr__(self):
        return ('\nFrame2TTL with user properties:' + '\n\n'
                'Port: ArCOMObject(' + self.port.serialObject.port + ')' + '\n'
                '_hardware_version: ' + str(self._hardware_version) + '\n'
                '_firmware_version: ' + str(self._firmware_version) + '\n'
                'light_threshold: ' + str(self._light_threshold) + '\n'
                'dark_threshold: ' + str(self._dark_threshold) + '\n'
                )

    def __del__(self):
        self.port = []


class Frame2TTLError(Exception):
    pass


class MainWindow(QtWidgets.QMainWindow):
    """Class to create and update the live sensor plot."""
    def __init__(self, serial_port, *args, **kwargs):
        super().__init__()

        self.setWindowTitle("Frame2TTL")
        self.resize(1280, 960)
        self.port = serial_port

        # Define custom axis items
        left_axis = AxisItem(orientation='left')
        left_axis.setTickFont(pg.Qt.QtGui.QFont("Arial", 12))
        bottom_axis = AxisItem(orientation='bottom')
        bottom_axis.setTickFont(pg.Qt.QtGui.QFont("Arial", 12))

        # Initialize PlotWidget with custom axis items
        self.plotWidget = pg.PlotWidget(axisItems={'left': left_axis, 'bottom': bottom_axis})
        self.setStyleSheet("background-color:white;")
        central_widget = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout()
        layout.setContentsMargins(30, 30, 30, 30)  # left, top, right, bottom
        layout.addWidget(self.plotWidget)
        central_widget.setLayout(layout)
        self.setCentralWidget(central_widget)
        self.x = np.array(list(range(2000))) / 1000  # 2000 samples
        self.y = np.empty(2000)  # 2000 data points
        self.y[:] = np.nan
        self.ypos = 0  # y position
        self.plotWidget.setBackground((255, 255, 255))
        my_pen = pg.mkPen(color=(0, 0, 0), width=3)
        self.data_line = self.plotWidget.plot(self.x, self.y, pen=my_pen)
        self.plotWidget.setTitle("<span style=\"color:black;font-size:15pt\">Sensor Data</span>")
        styles = {'color': 'r', 'font-size': '10rem'}
        self.plotWidget.setLabel('left', "<span style=\"font-size:14pt\">Sensor Value (Bits)</span>", **styles)
        self.plotWidget.setLabel('bottom', "<span style=\"font-size:14pt\">Time (s)</span>", **styles)
        self.plotWidget.setYRange(0, 65535)

        #  Set up and start the plot update timer
        self.timer = QtCore.QTimer()
        self.timer.setInterval(25)
        self.timer.timeout.connect(self.update_plot_data)
        self.timer.start()

        # Instruct the device to begin returning sensor data via USB serial
        self.port.write((ord('S'), 1), 'uint8')

    def update_plot_data(self):
        bytes_available = self.port.bytes_available()
        if bytes_available > 1:  # Samples are 16-bit, so a single sample available to read is 2 bytes long
            n_samples = math.floor(bytes_available/2)
            new_samples = self.port.read(n_samples, 'uint16')
            if self.ypos + n_samples >= 2000:
                self.y[:] = np.nan
                self.ypos = 0
            else:
                self.y[self.ypos:self.ypos + n_samples] = new_samples  # np.random.rand(1, n_samples)*65535
                self.ypos += n_samples
            self.data_line.setData(self.x, self.y)  # Update the data.

    def closeEvent(self, event):
        self.port.write((ord('S'), 0), 'uint8')  # Instruct the device to stop returning sensor data via USB serial
        event.accept()
