/*
  Audio Direction Detection with OLED Display using FFT and Multiple Microphones
  
  Description:
  This program captures sound from four microphones and analyzes the frequency spectrum 
  using Fast Fourier Transform (FFT). The target frequency is set to 3000 Hz by default.
  Based on the detected sound direction (calculated from the strongest magnitude of the 
  target frequency), it displays corresponding "eyes" on an OLED screen to indicate the 
  source of the sound. If no significant sound is detected, the system enters a "sleep mode" 
  by showing sleepy eyes.

  Features:
  - Multi-microphone setup to capture sound from multiple directions.
  - Fast Fourier Transform (FFT) analysis to extract frequency data.
  - Visual representation of sound direction via "eyes" on an OLED display.
  - Sleep mode: Displays "sleepy eyes" when no significant sound is detected.
  - Angle calculation based on the loudest sound source in 90-degree intervals.

  Specifications:
  - Sampling Frequency: 10,000 Hz
  - FFT Samples: 64 samples per microphone
  - OLED Resolution: 128x64 pixels
  - Target Frequency: 3000 Hz (default, can be modified)
  - Angle Output: 0°, 90°, 180°, 270° (corresponding to sound direction)
  
  Hardware Requirements:
  - 4 analog microphones (connected to A0, A1, A2, A3)
  - SSD1306 OLED display (128x64)
  - Arduino-compatible board
  
  Libraries Required:
  - arduinoFFT (for FFT computation)
  - Adafruit GFX (for OLED graphics)
  - Adafruit SSD1306 (for controlling the OLED display)
  
  Author: RK_ME, Circuit Digest
*/


#include "arduinoFFT.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// OLED display settings
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Analog inputs for microphones, when using Arduino UNO. in case of other Compactable microcontroller, you can use it analog input pins. Tested in ESP32
#define MIC1 A0
#define MIC2 A1
#define MIC3 A2
#define MIC4 A3

// Constants
#define SAMPLES 64                                                // Number of samples for FFT, must be a power of 2 - Adjust as per your Requirement, but limited to the specific hardware
#define SAMPLING_FREQ 10000                                       // Hz, limited by ADC conversion time - Adjust as per your Requirement, but limited to the specific hardware
int SAMPLING_PERIOD_US = round(1000000 * (1.0 / SAMPLING_FREQ));  // Sampling period in microseconds

// FFT object and data arrays
double Real1[SAMPLES], Imag1[SAMPLES];
double Real2[SAMPLES], Imag2[SAMPLES];
double Real3[SAMPLES], Imag3[SAMPLES];
double Real4[SAMPLES], Imag4[SAMPLES];
ArduinoFFT<double> FFT1 = ArduinoFFT<double>(Real1, Imag1, SAMPLES, SAMPLING_FREQ, true);
ArduinoFFT<double> FFT2 = ArduinoFFT<double>(Real2, Imag2, SAMPLES, SAMPLING_FREQ, true);
ArduinoFFT<double> FFT3 = ArduinoFFT<double>(Real3, Imag3, SAMPLES, SAMPLING_FREQ, true);
ArduinoFFT<double> FFT4 = ArduinoFFT<double>(Real4, Imag4, SAMPLES, SAMPLING_FREQ, true);

// Variables
float targetFreq = 3000.0;        // Default target frequency (Hz) - Adjust as per your Requirement
float selectedFreqMag[4];         // Magnitudes for target frequency
float angle = 0;                  // Initial angle
float magnitudeThreshold = 73.0;  // Threshold to determine if the frequency is detected - Adjust as per your Requirement

// Variables to track the angle and a counter for detecting stable angles
float lastAngle = -1;     // Stores the previous angle (-1 indicates uninitialized)
int angleCounter = 0;     // Counter for how many times the same angle is detected
int angleThreshold = 50;  // Threshold for how many times the same angle must be detected before showing sleepy eyes - Adjust as per your Requirement

// Function to map frequency to FFT bin
int frequencyToBin(float freq, float samplingFreq, int numSamples) {
  return round((freq * numSamples) / samplingFreq);
}

// Function to find the index of the maximum value in an array
int findMaxIndex(float arr[], int arrSize) {
  int index = 0;
  for (int i = 1; i < arrSize; i++) {
    if (arr[i] > arr[index]) index = i;
  }
  return index;  // Return the index of the maximum value
}

// Function to calculate the angle based on the microphone with the highest magnitude
float calculateAngle(float arr[], int arrSize) {
  return findMaxIndex(arr, arrSize) * 90;  // Map index to angle, assuming 90-degree intervals
}

// Function to draw sleepy eyes on the OLED display
void drawSleepyEyes() {
  display.drawRect(64, 24, 16, 3, 1);  // Right eye
  display.drawRect(42, 24, 15, 3, 1);  // Left eye
  display.drawRect(52, 40, 19, 2, 1);  // Mouth
}

// Function to draw eyes based on the detected angle
void drawEyes() {
  // Check if the current angle is the same as the previous angle
  if (angle == lastAngle) {
    angleCounter++;  // Increment the counter if the angle is the same
  } else {
    angleCounter = 0;  // Reset counter if the angle changes
  }
  
  // Update the last angle
  lastAngle = angle;

  // If the angle remains the same for a certain threshold, display sleepy eyes
  if (angleCounter >= angleThreshold) {
    drawSleepyEyes();
  } else {
    // Draw normal eyes based on the angle
    // Adjust eye positions based on the angle
    if (angle == 90) {  // Downward
      display.drawCircle(74, 52, 7, 1);
      display.fillCircle(74, 54, 3, 1);
      display.drawCircle(48, 52, 7, 1);
      display.fillCircle(48, 54, 3, 1);
      display.drawCircle(60, 62, 31, 1);
    }
    else if (angle == 0) {  // Rightward
      display.drawCircle(119, 35, 7, 1);
      display.fillCircle(121, 35, 3, 1);
      display.drawCircle(98, 37, 7, 1);
      display.fillCircle(100, 37, 3, 1);
      display.drawCircle(111, 52, 31, 1);
      display.drawCircle(96, 58, 3, 1);
    }
    else if (angle == 180) {  // Leftward
       display.drawCircle(28, 33, 7, 1);
      display.fillCircle(26, 33, 3, 1);
      display.drawCircle(9, 26, 7, 1);
      display.fillCircle(7, 26, 3, 1);
      display.drawCircle(14, 42, 31, 1);
      display.drawCircle(20, 54, 3, 1);
    }
    else if (angle == 270) {  // Upward
      display.drawCircle(72, 9, 8, 1);
      display.drawCircle(48, 11, 9, 1);
      display.fillCircle(72, 6, 3, 1);
      display.fillCircle(48, 7, 3, 1);
      display.drawCircle(61, 24, 31, 1);
      display.drawCircle(65, 34, 4, 1);
    }
  }
}

// Function to take samples from microphones and perform FFT
void determineAngle() {
  long microseconds = micros();  // Start timing for sampling

  // Take samples for each microphone
  for (int i = 0; i < SAMPLES; i++) {
    Real1[i] = analogRead(MIC1);  // Sample from MIC1
    Real2[i] = analogRead(MIC2);  // Sample from MIC2
    Real3[i] = analogRead(MIC3);  // Sample from MIC3
    Real4[i] = analogRead(MIC4);  // Sample from MIC4
    Imag1[i] = Imag2[i] = Imag3[i] = Imag4[i] = 0;  // Initialize imaginary components to 0
    
    // Wait for the sampling period to finish
    while ((micros() - microseconds) < SAMPLING_PERIOD_US);
    microseconds += SAMPLING_PERIOD_US;
  }

  // Perform FFT on each microphone's data
  FFT1.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT1.compute(FFTDirection::Forward);
  FFT1.complexToMagnitude();
  
  FFT2.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT2.compute(FFTDirection::Forward);
  FFT2.complexToMagnitude();
  
  FFT3.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT3.compute(FFTDirection::Forward);
  FFT3.complexToMagnitude();
  
  FFT4.windowing(FFTWindow::Hamming, FFTDirection::Forward);
  FFT4.compute(FFTDirection::Forward);
  FFT4.complexToMagnitude();

  // Get magnitudes for the target frequency for each microphone
  int targetBin = frequencyToBin(targetFreq, SAMPLING_FREQ, SAMPLES);
  selectedFreqMag[0] = Real1[targetBin];
  selectedFreqMag[1] = Real2[targetBin];
  selectedFreqMag[2] = Real3[targetBin];
  selectedFreqMag[3] = Real4[targetBin];

  // Determine if the maximum magnitude exceeds the threshold
  float maxMagnitude = selectedFreqMag[0];
  for (int i = 1; i < 4; i++) {
    if (selectedFreqMag[i] > maxMagnitude) {
      maxMagnitude = selectedFreqMag[i];
    }
  }

  // If no significant frequency is detected, display sleepy eyes
  if (maxMagnitude < magnitudeThreshold) {
    drawSleepyEyes();
  } else {
    // Calculate the angle based on the highest magnitude
    angle = calculateAngle(selectedFreqMag, 4);
  }
   Serial.println("Angle: " + String(angle));
}

// Setup function
void setup() {
  Serial.begin(115200);  // Start serial communication
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("SSD1306 allocation failed"));
    for (;;);
  }
  display.clearDisplay();  // Clear the display
  display.display();  // Initialize the display
}
// Main loop
void loop() {
  determineAngle();

  display.clearDisplay();
  drawEyes();
  display.display();

  delay(1);
}
