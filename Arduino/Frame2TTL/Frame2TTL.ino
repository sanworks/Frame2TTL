/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Frame2TTL repository
  Copyright (C) 2017 Sanworks LLC, Sound Beach, New York, USA

  ----------------------------------------------------------------------------

  This program is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, version 3.

  This program is distributed  WITHOUT ANY WARRANTY and without even the
  implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
  See the GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

*/

#include "ArCOM.h"
ArCOM USBCOM(SerialUSB); // Creates an ArCOM object called USBCOM, wrapping SerialUSB connection to MATLAB/Python

// Lines
const byte sensorLine = 10;
const byte TTLoutput = 12;

// Params
uint16_t lightThresh = 40;
uint16_t darkThresh = 80;
unsigned long refractoryDuration = 2000; // Minimum time before output line can change states again, units = us

// Program Variables
byte sensorState = 0; // Current state of the sensor logic line (high or low)
byte lastSensorState = 0; // Last known state of the sensor logic line (high or low)
byte outputState = 0; // Current state of the TTL output line
unsigned long currentTime = 0; // Current time (us)
unsigned long lastTransitionTime = 0; // Last time the output line changed states
unsigned long refractoryStartTime = 0; // Last time the output line changed states
unsigned long lastStreamingSampleTime = 0; // Last time sensor value was sent while streaming
unsigned long StreamSampleInterval = 10000; // Interval between samples while streaming (Hard coded to 10k us = 100Hz)
unsigned long sensorValue = 0; // microseconds between sensorState level changes
boolean inRefractory = false; // True if currently within refractoryDuration microseconds of last output level change
boolean inPulse = false; // True if the output state is high
boolean isStreaming = false; // True if streaming output to MATLAB

// Communication variables
byte op = 0; // Operation code from MATLAB

void setup() {
  pinMode(sensorLine, INPUT);
  pinMode(TTLoutput, OUTPUT);
  currentTime = micros();
  lastTransitionTime = currentTime;
}

void loop() {
  if (USBCOM.available()) {
    op = USBCOM.readByte();
    switch(op) {
      case 'C': // Confirm USB connection
        USBCOM.writeByte(218);
      break;
      case 'T': // Set Threshold
        lightThresh = USBCOM.readUint16();
        darkThresh = USBCOM.readUint16();
      break;
      case 'S': // Stream sensor value
        isStreaming = USBCOM.readByte();
        lastStreamingSampleTime = currentTime;
      break;
      case 'V': // Return last sensor value
        USBCOM.writeUint32(sensorValue);
      break;
    }
  }
  lastSensorState = sensorState;
  sensorState = digitalRead(sensorLine);
  currentTime = micros();
  if (sensorState != lastSensorState) {
    if (currentTime < lastTransitionTime) { // There was a 32-bit micros() clock roll-over
      sensorValue = currentTime + (4294967296-lastTransitionTime);
    } else {
      sensorValue = currentTime - lastTransitionTime;
    }
    lastTransitionTime = currentTime;
    if (isStreaming) {
      if ((currentTime - lastStreamingSampleTime) > StreamSampleInterval) {
        USBCOM.writeUint32(sensorValue);
        lastStreamingSampleTime = currentTime;
        inRefractory = true;
        refractoryStartTime = currentTime;
      }
    }
  }
  if (!inRefractory) {
    if (inPulse == false) {
      if (sensorValue < lightThresh) {
        digitalWrite(TTLoutput, HIGH);
        inRefractory = true;
        inPulse = true;
        refractoryStartTime = currentTime;
      }
    } else {
      if (sensorValue > darkThresh) {
        digitalWrite(TTLoutput, LOW);
        inRefractory = true;
        inPulse = false;
        refractoryStartTime = currentTime;
      }
    }
  }
  if ((currentTime - refractoryStartTime) > refractoryDuration) {
    inRefractory = false;
  }
}
