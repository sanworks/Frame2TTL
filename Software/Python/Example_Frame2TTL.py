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
"""

# Example script demonstrating basic usage of frame2ttl.py to control the Frame2TTL module via its USB port
# IMPORTANT: Before running the demo, set usb_port_name below!

from frame2ttl import Frame2TTL

# ----- Set the line below to match the valve driver's port on your system
usb_port_name = 'COM3'
# -----

F = Frame2TTL(usb_port_name)  # Create an instance of the Frame2TTL device
F.light_threshold = 75  # Set the threshold for sync patch dark --> light detection (units = change in bits/ms)
F.dark_threshold = -75  # Set the threshold for sync patch light --> dark detection (units = change in bits/ms)
current_sensor_value = F.read_sensor()  # Read the current value of the sensor
print('Frame2TTL Connected. Current sensor value = ' + str(current_sensor_value))
print('Close the live plot to exit the demo.')
F.stream_ui()  # Run a live plot to assist with setting thresholds and for diagnostics.

del F
print('Frame2TTL Disconnected.')
