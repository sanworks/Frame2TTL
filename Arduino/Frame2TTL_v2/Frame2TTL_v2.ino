/*
  ----------------------------------------------------------------------------

  This file is part of the Sanworks Frame2TTL repository
  Copyright (C) 2021 Sanworks LLC, Rochester, New York, USA

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
#define HARDWARE_VERSION 2 // v1 was the previous versions of Frame2TTL

ArCOM USBCOM(Serial);
const byte TTLoutputLine = 12;

IntervalTimer hwTimer;

// Params
const uint32_t samplingRate = 20000; // Analog sampling rate (Hz)
const uint16_t windowSize = 20; // Number of analog samples to consider for edge detection
unsigned long refractoryDuration = 2000; // Minimum time before output line can change states again, units = us

// Program Variables
uint32_t currentTime = 0; // Current time (us)
uint32_t lastStreamingSampleTime = 0; // Last time sensor value was sent while streaming
uint32_t streamSampleInterval = 1000; // Interval between samples while streaming, units = us (Hard coded to 1k us = 1000Hz)
volatile uint16_t sensorValue = 0; // Analog value read from sensor
uint32_t analogOutputValue = 0; // Analog output voltage proprotional to light signal
const uint32_t streamingBufferNsamples = 10;
uint16_t sampleBuffer[windowSize] = {0}; // Buffer of light samples used to calculate sliding window average of luminance changes
uint16_t streamingBuffer[streamingBufferNsamples] = {0}; // Buffer of light samples to stream to the PC
uint32_t nSamplesToRead = 0; // For use when relaying a fixed number of contiguous samples to PC
uint32_t nSamplesAdded = 0; // For USB Streaming
int32_t total = 0; // For computing sliding window average
volatile int16_t avgLightDiff = 0; // Sliding window average of sample-wise changes in light intensity
int16_t lightThresh = 150; // Average change in luminance necessary to transition to "inPulse" state
int16_t darkThresh = -150; // Average change in luminance necessary to transition from "inPulse" state

// State Variables
boolean inPulse = false; // True if the output sync line is high
boolean isStreaming = false; // True if streaming output to MATLAB
volatile boolean sampleReadyFlag = false; // True if a sample has been read

// Communication variables
byte op = 0; // Operation code from PC

void setup() {
  analogReadResolution(16); // Set analog input resolution to 16-bit (Maximum supported; only 13-bits usable due to noise floor)
  analogWriteResolution(12); // Set analog output resolution to 12-bit (Maximum supported)
  pinMode(TTLoutputLine, OUTPUT); // Configure TTL sync line as a digital output. This output is galvanically isolated from USB ground by ADUM6200 chip.
  pinMode(13, OUTPUT); // Onboard LED (initialized so it can be used for troubleshooting)
  digitalWrite(13, LOW); // Set board LED off
  hwTimer.begin(readNewSample, (1/(double)samplingRate)*1000000);  // readNewSample to run every sample
}

void readNewSample() {
  currentTime = micros();
  sensorValue = analogRead(A0); // Read the sensor voltage (16-bit, range = 0-65535)
  analogWrite(A14, sensorValue/16); // Map sensorValue from 16-bit to 12-bit range (0-4096) and write to analog output
  
// Compute avgLightDiff, the sliding window average of sample-wise change in light level
  memcpy(sampleBuffer, &sampleBuffer[1], sizeof(sampleBuffer)-2);
  sampleBuffer[windowSize-1] = sensorValue;
  total = 0;
  for (int i = 0; i < windowSize-1; i++) {
    total += (sampleBuffer[i+1] - sampleBuffer[i]);
  }
  avgLightDiff = total/windowSize;
  
  // Detect light patch transition events and set sync state
  if (inPulse == false) {
    if (avgLightDiff > lightThresh) {
      inPulse = true;
      digitalWrite(TTLoutputLine, HIGH);
    }
  } else {
    if (avgLightDiff < darkThresh) {
      inPulse = false;
      digitalWrite(TTLoutputLine, LOW);
    }
  }
  sampleReadyFlag = true;
}

void loop() {
  if (USBCOM.available()) {
    op = USBCOM.readByte();
    switch(op) {
      case 'C': // Confirm USB connection
        USBCOM.writeByte(218);
      break;
      case 'T': // Set Light + Dark Thresholds (Manual)
        lightThresh = USBCOM.readInt16();
        darkThresh = USBCOM.readInt16();
      break;
      case 'D': // Set Dark Threshold (Automatic; measured with sync patch on to ensure that on -> off transition is below noise)
        darkThresh = autoSetThreshold(0);
        USBCOM.writeInt16(darkThresh);
      break;
      case 'L': // Set Light Threshold (Automatic; measured with sync patch off to ensure that off -> on transition is above noise)
        lightThresh = autoSetThreshold(1);
        USBCOM.writeInt16(lightThresh);
      break;
      case 'S': // Stream sensor value (subsampling sensor datastream at 1kHz)
        isStreaming = USBCOM.readByte();
        lastStreamingSampleTime = currentTime;
      break;
      case 'V': // Return contiguous raw sensor values
        nSamplesToRead = USBCOM.readUint32();
        sampleReadyFlag = false;
        for (int i = 0; i < nSamplesToRead; i++) {
          while (sampleReadyFlag == false) {} // Wait for hardware timer to call readNewSample() and update sensorValue
          sampleReadyFlag = false;
          USBCOM.writeUint16(sensorValue);
        }
      break;
      case '#': // Return hardware version
        USBCOM.writeByte(HARDWARE_VERSION);
      break;
      case 'Q': // Return sampling rate
        USBCOM.writeUint32(samplingRate);
      break;
    }
  }
  
  if (isStreaming) {
    if ((currentTime - lastStreamingSampleTime) > streamSampleInterval) {
      streamingBuffer[nSamplesAdded] = sensorValue;
      nSamplesAdded++; 
      if (nSamplesAdded == streamingBufferNsamples) {
        USBCOM.writeUint16Array(streamingBuffer, streamingBufferNsamples);
        nSamplesAdded = 0;
      }
      lastStreamingSampleTime = currentTime;
    }
  }
}

uint16_t autoSetThreshold(byte thresholdIndex) { // Measure 1 second of light during a fixed screen state and set a threshold safely outside the noise
  // thresholdIndex = 0 (Light -> Dark threshold) or 1 (Dark -> Light threshold)
  int16_t newThreshold = 0;
  int16_t minValue = 65535;
  int16_t maxValue = -65535;
  uint32_t nSamplesToMeasure = 30000; // 3 seconds at 20kHz
  sampleReadyFlag = false;
  for (int i = 0; i < nSamplesToMeasure; i++) {
    while (sampleReadyFlag == false) {} // Wait for hardware timer to call readNewSample() and update avgLightDiff
    sampleReadyFlag = false; // Reset flag
    if (avgLightDiff > maxValue) {
      maxValue = avgLightDiff;
    }
    if (avgLightDiff < minValue) {
      minValue = avgLightDiff;
    }
  }
  switch (thresholdIndex) {
    case 0: // Dark threshold
      newThreshold = minValue*2; // 2x of largest measured negative sliding window average change in light intensity
    break;
    case 1: // Light threshold
      newThreshold = maxValue*2; // 2x of largest measured positive sliding window average change in light intensity
    break;
  }
  return newThreshold;
}
