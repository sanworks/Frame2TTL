"""
----------------------------------------------------------------------------

This file is part of the Sanworks Frame2TTL repository
Copyright (C) Sanworks LLC, Rochester, New York, USA

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
F = Frame2TTL('COM3')  # Where COM3 is the Frame2TTL device's usb serial port
F.detect_mode = 1      # Determines how light signal is processed to detect sync patch transitions.
                       # In mode 0, light intensity is compared with a fixed light intensity threshold.
                       # In mode 1, a 1ms sliding window average of sample-wise changes in luminance
                       # is compared with a threshold.
F.dark_threshold = -75 # Set the threshold for detecting a light -> dark sync patch transition.
                       # If detect_mode = 0, units are bits of light intensity
                       # If detect_mode = 1 (default), units are average signed sample-wise change in bits
                       # in a 1ms sliding window. Typical threshold values are in range [-150, 150]
F.set_dark_threshold_auto()  # Automatically set the dark threshold, while the sync patch is white at max intensity
sensorValue = F.read_sensor()  # Return the light sensor's current luminance measurement.
                               # Units are bits in range [0, 65535]
sensorValues = F.read_sensor(1000)  # Return 1000 consecutive luminance measurements from the sensor
F.stream_ui() # Launches a live streaming plot of the raw sensor data for diagnostics and to assist with
              # threshold setting in detect_mode 0

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
CURRENT_FIRMWARE = 4
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
        if old_firmware_found or self._firmware_version < CURRENT_FIRMWARE:
            raise Frame2TTLError('Error: Old Frame2TTL firmware detected. Update to firmware v' +
                                 str(CURRENT_FIRMWARE) + ' or newer.')
        if self._firmware_version > CURRENT_FIRMWARE:
            raise Frame2TTLError('Error: Future Frame2TTL firmware detected. Downgrade to firmware v' +
                                 str(CURRENT_FIRMWARE) + ' or update Frame2TTL software.')

        # Verify hardware version
        self.port.write(ord('#'), 'uint8')  # Request HW version.
        self._hardware_version = self.port.read(1, 'uint8')
        if self._hardware_version == 3:  # For HW 3, reinitialize to full baud rate
            self.port = []
            time.sleep(0.25)
            self.port = ArCom(port_name, 480000000)

        # Set default detection parameters
        self._light_threshold = 75
        self._dark_threshold = -75
        self._activation_margin = 1000
        self._detect_mode = 1

        # Initialize detect mode and thresholds
        self.detect_mode = 1

        # Program the default threshold activation margin (units = bits, used only in detect_mode 0)
        self.port.write(ord('G'), 'uint8', self._activation_margin, 'uint32')

    @property
    def light_threshold(self):
        return self._light_threshold

    @light_threshold.setter
    def light_threshold(self, value):
        if value <= 0 or not isinstance(value, int):
            raise Frame2TTLError('Error: light_threshold must be a positive integer.')
        if self.detect_mode == 0:
            maxThresh = 65535 - self._activation_margin
            if not ((value >= self._activation_margin) and (value <= 65535)):
                raise Frame2TTLError('Error: in detect_mode 0, light_threshold must be in range ['
                                     + str(self._activation_margin) + ', ' + str(maxThresh) + '].')
            if value >= self.dark_threshold:
                raise Frame2TTLError('Error: in detect_mode 0, light_threshold must be lower than dark_threshold.')
        self.port.write(ord('T'), 'uint8', value, 'int32')
        self._light_threshold = value

    @property
    def dark_threshold(self):
        return self._dark_threshold

    @dark_threshold.setter
    def dark_threshold(self, value):
        if (self.detect_mode == 1 and value >= 0) or not isinstance(value, int):
            raise Frame2TTLError('Error: dark_threshold must be a negative integer.')
        if self.detect_mode == 0:
            maxThresh = 65535-self._activation_margin
            if not ((value >= 0) and (value <= maxThresh)):
                raise Frame2TTLError('Error: in detect_mode 0, dark_threshold must be in range [' +
                                     str(self._activation_margin) + ', ' + str(maxThresh) + '].')
            if value <= self.light_threshold:
                raise Frame2TTLError('Error: in detect_mode 0, dark_threshold must be higher than light_threshold.')
        self.port.write(ord('K'), 'uint8', value, 'int32')
        self._dark_threshold = value

    @property
    def detect_mode(self):
        return self._detect_mode

    @detect_mode.setter
    def detect_mode(self, value):
        original_mode = self._detect_mode;
        if value not in [0, 1]:
            raise Frame2TTLError('Error: threshold_mode must be either 0 or 1.')
        self.port.write((ord('M'), value), 'uint8')
        self._detect_mode = value
        if value != original_mode:
            self._set_threshold_defaults(value)

    def set_dark_threshold_auto(self):
        """Auto-set threshold for detecting light --> dark transitions. Run with the sync patch set to WHITE."""
        if self.detect_mode == 0:
            raise Frame2TTLError('Error: Automatic threshold detection is only available for detect_mode 1.')
        self.port.write(ord('D'), 'uint8')
        time.sleep(3)  # The instrument measures for ~2.5 seconds and calculates the new threshold
        self._dark_threshold = self.port.read(1, 'int32')

    def set_light_threshold_auto(self):
        """Auto-set threshold for detecting dark --> light transitions. Run with the sync patch set to BLACK."""
        if self.detect_mode == 0:
            raise Frame2TTLError('Error: Automatic threshold detection is only available for detect_mode 1.')
        self.port.write(ord('L'), 'uint8')
        time.sleep(3)  # The instrument measures for ~2.5 seconds and calculates the new threshold
        self._light_threshold = self.port.read(1, 'int32')

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

        self.qt_main = MainWindow(self.port, (self._light_threshold, self._dark_threshold), self.detect_mode)
        self.qt_main.show()
        self.qt_main.activateWindow()
        self.qt_app.exec_()

    def _set_threshold_defaults(self, detect_mode):
        if detect_mode == 0:
            self.dark_threshold = 30000
            self.light_threshold = 20000
        elif detect_mode == 1:
            if self._hardware_version == 2:
                self.light_threshold = 100
                self.dark_threshold = -150
            elif self._hardware_version == 3:
                self.light_threshold = 75
                self.dark_threshold = -75

    # Self-description when the object is entered into the Python console with no properties or methods specified
    def __repr__(self):
        return ('\nFrame2TTL with user properties:' + '\n\n'
                'Port: ArCOMObject(' + self.port.serialObject.port + ')' + '\n'
                '_hardware_version: ' + str(self._hardware_version) + '\n'
                '_firmware_version: ' + str(self._firmware_version) + '\n'
                'detect_mode: ' + str(self._detect_mode) + '\n'
                'light_threshold: ' + str(self._light_threshold) + '\n'
                'dark_threshold: ' + str(self._dark_threshold) + '\n'
                )

    def __del__(self):
        self.port = []


class Frame2TTLError(Exception):
    pass


class MainWindow(QtWidgets.QMainWindow):
    """Class to create and update the live sensor plot."""
    def __init__(self, serial_port, thresholds, detect_mode, *args, **kwargs):
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

        if detect_mode == 0:
            # Add threshold lines
            x_points = [0, 2]
            y_points = [thresholds[0], thresholds[0]]
            line_pen = pg.mkPen(color=(255, 204, 0), width=2, style=pg.QtCore.Qt.DashLine)
            self.plotWidget.plot(x_points, y_points, pen=line_pen)
            y_points = [thresholds[1], thresholds[1]]
            line_pen = pg.mkPen(color=(128, 128, 128), width=2, style=pg.QtCore.Qt.DashLine)
            self.plotWidget.plot(x_points, y_points, pen=line_pen)

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
