/*
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
  ESP32 WEMOS LOLIN32 Lite based guitar tuner 
  by Mikołaj Kawa, 2024
  This code samples guitar signal of plucked string and estimates pitch.
  Estimated value is compared to 12TET frequencies (A = 440Hz) and 
  deviations are displayed via LEDs and 7seg display (look at guitarTunerDisplay.h file)
+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++
*/

#include "guitarTunerDisplay.h"
#include <arduinoFFT.h>

#define DEBBUG_LED LED_BUILTIN
#define SAMPLING_FREQUENCY 4840
#define ANALOG_PIN 36
#define BUFFER_LENGTH 512

#define NOISE_THRESHOLD_CONTROL 0
#define FREQUENCY_PEAK_THRESHOLD 900
#define FREQUENCY_SEARCH_LOWER_BOUNDARY 8 //in freq bins
#define FREQUENCY_SEARCH_UPPER_BOUNDARY 70 //in freq bins

hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool ISRFlag = false;
volatile int signalBuffer[BUFFER_LENGTH];
volatile int sampleIndex = 0; 

float realValues[BUFFER_LENGTH];
float imaginaryValues[BUFFER_LENGTH];

const float frequencies[43] = {
    65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00, 
    116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 
    196.00, 207.65, 220.00, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 
    329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88, 523.25, 
    554.37, 587.33, 622.25, 659.25};  

const char* noteNames[43] = {
    "C", "C#", "D", "D#", "E", "F", "F#", "G", "G#", "A", 
    "A#", "B", "C", "C#", "D", "D#", "E", "F", "F#", "G", 
    "G#", "A", "A#", "B", "C", "C#", "D", "D#", "E", "F", 
    "F#", "G", "G#", "A", "A#", "B", "C", "C#", "D", "D#", 
    "E"};

ArduinoFFT<float> FFT(realValues, imaginaryValues, BUFFER_LENGTH, SAMPLING_FREQUENCY);


void IRAM_ATTR onTimer()
{
  portENTER_CRITICAL_ISR(&timerMux);
  if (sampleIndex < BUFFER_LENGTH)
  {
    ISRFlag = true;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}


void setup() 
{
  cli();
  /*
  //serial communication for debugging only
  Serial.begin(1000000);
  */
  pinMode(DEBBUG_LED, OUTPUT);
  pinMode(ANALOG_PIN, INPUT);
  timer = timerBegin(0, 80, true);
  timerAttachInterrupt(timer, &onTimer, true);
  timerAlarmWrite(timer, 1000000 / SAMPLING_FREQUENCY, true);
  timerAlarmEnable(timer); 
  sei();

  for (int i = 0; i < BUFFER_LENGTH; i++) {
    realValues[i] = 0.0;
    imaginaryValues[i] = 0.0;
  }
  setupLedDisplay();
  delay(200);
}


void loop() 
{ 
 
  if (ISRFlag)
  {
    portENTER_CRITICAL_ISR(&timerMux);
    ISRFlag = false;
    signalBuffer[sampleIndex] = analogRead(ANALOG_PIN);
    sampleIndex++;
    portEXIT_CRITICAL_ISR(&timerMux);
  }
  
  if (sampleIndex >= BUFFER_LENGTH) 
  { 
    sampleIndex = 0;
    digitalWrite(DEBBUG_LED, !digitalRead(DEBBUG_LED));
    portENTER_CRITICAL_ISR(&timerMux);
    float signalAbsMaxValue = 0.0;
    for (int i = 0; i < BUFFER_LENGTH; i++) 
    {
      realValues[i] = (float)signalBuffer[i];
      imaginaryValues[i] = 0;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    FFT.dcRemoval();
    FFT.windowing(FFTWindow::Blackman_Harris, FFTDirection::Forward);
    FFT.compute(FFT_FORWARD);
    calculateMagnitude();
   
    /*
    // Print spectrum in search range DO NOT PRINT WHILE RESULTS ARE ALSO PRINTING
    for (int i = FREQUENCY_SEARCH_LOWER_BOUNDARY; i < FREQUENCY_SEARCH_UPPER_BOUNDARY; i++) {Serial.println(realValues[i]);}
    */

    int peakIndex = findFirstMajorPeak(realValues);
    float interpolatedFreqBin = calculateParabolicInterpolation(peakIndex, realValues);
    float estimatedFrequency = interpolatedFreqBin * SAMPLING_FREQUENCY / BUFFER_LENGTH;
    const float baseTETFrequency = getClosestTETFrequency(estimatedFrequency);
    displayCentsDeviation(baseTETFrequency, estimatedFrequency);

    /*
    //Value prints for debbuging DO NOT PRINT WHILE SPECTRA ARE ALSO PRINTED
    float dev = calculateDeviationInCents(estimatedFrequency, baseTETFrequency);
    Serial.print(peakIndex); 
    Serial.print(" "); 
    Serial.print(baseTETFrequency,2);
    Serial.print(" "); 
    Serial.print(estimatedFrequency,4);
    Serial.print(" ");
    Serial.println(dev);
    */
  }
}

void calculateMagnitude()
{
  for (int i = 0; i < BUFFER_LENGTH; i++) 
  {
    realValues[i] = sqrt(realValues[i] * realValues[i] + imaginaryValues[i] * imaginaryValues[i]);
  }
}

int findFirstMajorPeak(float magnitude[])
{
  int indexAboveThreshold = findIndexAboveThreshold(magnitude);
  int indexBelowThreshold = findIndexBelowThreshold(magnitude,indexAboveThreshold);
  int peakIndex = findMaxIndexInRange(magnitude, indexAboveThreshold, indexBelowThreshold);
  return peakIndex;
}

int findIndexAboveThreshold(float arr[]) 
{
  for (int i = FREQUENCY_SEARCH_LOWER_BOUNDARY; i < FREQUENCY_SEARCH_UPPER_BOUNDARY; i++) {if (arr[i] > FREQUENCY_PEAK_THRESHOLD) { return i; }}
  return -1;  
}

int findIndexBelowThreshold(float arr[], int firstFoundIndex) 
{
  for (int i = firstFoundIndex; i < FREQUENCY_SEARCH_UPPER_BOUNDARY; i++) {if (arr[i] < FREQUENCY_PEAK_THRESHOLD) {return i;}}
  return -1; 
}

int findMaxIndexInRange(float arr[], int start, int end) {
  int maxIndex = start;
  for (int i = start; i <= end; i++) {
    if (arr[i] > arr[maxIndex]) {
      maxIndex = i; 
    }
  }
  return maxIndex;
}




const float getClosestTETFrequency(float measuredFrequency)
{ 
  float closestFrequency = frequencies[0];
  float minDifference = abs(measuredFrequency - frequencies[0]);

  if (isnan(measuredFrequency) || measuredFrequency < frequencies[0]) 
  {
    clearNoteName();
    clearIndicatorDiode();
    return 0.0;
  }

  for (int i = 1; i < 43; i++) 
  {
    float diff = abs(measuredFrequency - frequencies[i]);
    if (diff < minDifference) 
    {
      minDifference = diff;
      closestFrequency = frequencies[i];
      displayNoteName(noteNames[i]);
    }
  }
  return closestFrequency;
}

void displayCentsDeviation(float baseFrequency, float measuredFrequency)
{
  if (isnan(measuredFrequency) || isnan(baseFrequency) || measuredFrequency < 0 || baseFrequency <= 0) 
  {
    return;
  }

  float lower50 = centsToFreq(baseFrequency, -50);
  float lower40 = centsToFreq(baseFrequency, -32);
  float lower30 = centsToFreq(baseFrequency, -16);
  float lower20 = centsToFreq(baseFrequency, -8);
  float lower10 = centsToFreq(baseFrequency, -3);
  float upper10 = centsToFreq(baseFrequency, 3);
  float upper20 = centsToFreq(baseFrequency, 8);
  float upper30 = centsToFreq(baseFrequency, 16);
  float upper40 = centsToFreq(baseFrequency, 32);
  float upper50 = centsToFreq(baseFrequency, 50);

  if (measuredFrequency >= lower50 && measuredFrequency < lower40) {displayIndicatorDiode(0);}
  if (measuredFrequency >= lower40 && measuredFrequency < lower30) {displayIndicatorDiode(1);}
  if (measuredFrequency >= lower30 && measuredFrequency < lower20) {displayIndicatorDiode(2);}
  if (measuredFrequency >= lower20 && measuredFrequency < lower10) {displayIndicatorDiode(3);}
  if (measuredFrequency >= lower10 && measuredFrequency <= upper10){displayIndicatorDiode(4);}
  if (measuredFrequency > upper10 && measuredFrequency <= upper20) {displayIndicatorDiode(5);}
  if (measuredFrequency > upper20 && measuredFrequency <= upper30) {displayIndicatorDiode(6);}
  if (measuredFrequency > upper30 && measuredFrequency <= upper40) {displayIndicatorDiode(7);}
  if (measuredFrequency > upper40 && measuredFrequency <= upper50) {displayIndicatorDiode(8);} 
}

float centsToFreq(float baseFrequency, float cents) {return baseFrequency * pow(2, cents / 1200.0);}

float calculateDeviationInCents(float measuredFrequency, float targetFrequency) {
    //funtion for serial print debugging
    float deviationInCents = 1200 * log2(measuredFrequency / targetFrequency);
    deviationInCents = (int)deviationInCents;
    return deviationInCents;
}

float calculateParabolicInterpolation(int peakIndex, float frequencyMagnitude[])
{
  float offset;
  float interpolatedIndex; 

  if (peakIndex > 0 && peakIndex < BUFFER_LENGTH - 1) 
  {
    offset = 0.5 * (frequencyMagnitude[peakIndex - 1] - frequencyMagnitude[peakIndex + 1])
        / (frequencyMagnitude[peakIndex - 1] - 2 * frequencyMagnitude[peakIndex] + frequencyMagnitude[peakIndex + 1]);
    interpolatedIndex = peakIndex + offset;
    return interpolatedIndex;
  }
  else {return 0.0;}
}