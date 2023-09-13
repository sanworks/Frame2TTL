/*
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

*/

#include "ArCOM.h"
#include <SPI.h>

// SETUP MACROS TO COMPILE FOR TARGET DEVICE:
#define HARDWARE_VERSION 0 // IMPORTANT! Set this to 2 or 3 to match your hardware. DO NOT set to 1. Use the 'Legacy' firmware file for hardware v1.
// ------------------------------------------

#define FIRMWARE_VERSION 3

ArCOM USBCOM(Serial); // Wrap Serial interface with ArCOM (for easy transmission of different data types)

IntervalTimer hwTimer; // A hardware timer peripheral is used to achieve even sampling

// I/O pins
#if HARDWARE_VERSION == 2
  #define TTL_OUTPUT_LINE 12
#elif HARDWARE_VERSION == 3
  #define DAC_CS_PIN 9
#else
  #error Error! HARDWARE_VERSION must be either 2 or 3. HARDWARE_VERSION 1 requires the legacy firmware.
#endif

// Params
const uint32_t samplingRate = 20000; // Analog sampling rate (Hz)
const uint16_t windowSize = 20; // Number of analog samples to consider for edge detection

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
int16_t lightThresh = 75; // Average change in luminance necessary to transition to "inPulse" state
int16_t darkThresh = -75; // Average change in luminance necessary to transition from "inPulse" state

// State Variables
boolean inPulse = false; // True if the output sync line is high
boolean isStreaming = false; // True if streaming output to MATLAB
volatile boolean sampleReadyFlag = false; // True if a sample has been read

// Communication variables
byte op = 0; // Operation code from PC

// DAC variables (v3 only)
SPISettings DACSettings(30000000, MSBFIRST, SPI_MODE0); // Settings for DAC
byte dacBuffer[3] = {0}; // Holds bytes to be written to the DAC
union {
  byte byteArray[4];
  uint16_t uint16[2];
} dacValue; // This structure allows for very efficient type casting


void setup() {
  SPI.begin(); // Initialize SPI bus (for communication with DAC)
  #if HARDWARE_VERSION == 2
    analogReadResolution(16); // Set analog reads to 16-bit
    analogWriteResolution(12); // Set analog writes to 12-bit
    pinMode(TTL_OUTPUT_LINE, OUTPUT);
  #elif HARDWARE_VERSION == 3
    analogReadResolution(12); // Set analog reads to 12-bit
    pinMode(A4, INPUT_DISABLE); // Configure analog input pin to high impedance
    pinMode(DAC_CS_PIN, OUTPUT); // Configure SPI Chip Select pin for DAC
    digitalWrite(DAC_CS_PIN, HIGH);
    SPI.beginTransaction(DACSettings);
    powerUpDAC();
  #endif
  hwTimer.begin(readNewSample, (1/(double)samplingRate)*1000000);  // set readNewSample() to run once per sample
}

void readNewSample() {
  currentTime = micros();

  // Read sensor and update analog output
  
  #if HARDWARE_VERSION == 2
    sensorValue = analogRead(A0);
    analogWrite(A14, sensorValue/16); // Map sensorValue from 16-bit to 12-bit range (0-4096) and write to analog output
  #else
    sensorValue = analogRead(A4)*16;
    dacValue.uint16[1] = sensorValue;
  #endif
  
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
      #if HARDWARE_VERSION == 2
        digitalWrite(TTL_OUTPUT_LINE, HIGH);
      #endif
      dacValue.uint16[0] = 54067;
    }
  } else {
    if (avgLightDiff < darkThresh) {
      inPulse = false;
      #if HARDWARE_VERSION == 2
        digitalWrite(TTL_OUTPUT_LINE, LOW);
      #endif
      dacValue.uint16[0] = 0;
    }
  }
  #if HARDWARE_VERSION == 3
    dacWrite();
  #endif
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
      case 'F': // Return firmware version
        USBCOM.writeByte(FIRMWARE_VERSION);
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
  uint32_t nSamplesToMeasure = 40000; // 2 seconds at 20kHz
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

#if HARDWARE_VERSION == 3
  void dacWrite() {
    digitalWriteFast(DAC_CS_PIN,LOW);
    dacBuffer[0] = B00110000; // Channel
    dacBuffer[1] = dacValue.byteArray[1];
    dacBuffer[2] = dacValue.byteArray[0];
    SPI.transfer(dacBuffer,3);
    digitalWriteFast(DAC_CS_PIN,HIGH);
    digitalWriteFast(DAC_CS_PIN,LOW);
    dacBuffer[0] = B00110001; // Channel
    dacBuffer[1] = dacValue.byteArray[3];
    dacBuffer[2] = dacValue.byteArray[2];
    SPI.transfer(dacBuffer,3);
    digitalWriteFast(DAC_CS_PIN,HIGH);
  }

  void powerUpDAC() {
    digitalWriteFast(DAC_CS_PIN,LOW);
    dacBuffer[0] = B01100000; // Enable internal reference
    dacBuffer[1] = 0;
    dacBuffer[2] = 0;
    SPI.transfer(dacBuffer,3);
    digitalWriteFast(DAC_CS_PIN,HIGH);
  }
#endif
