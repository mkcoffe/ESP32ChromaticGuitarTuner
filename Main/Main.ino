#include "guitarTunerDisplay.h"
#include <arduinoFFT.h>


#define DEBBUG_LED LED_BUILTIN
#define SAMPLING_FREQUENCY 4840
#define ANALOG_PIN 36
#define BUFFER_LENGTH 512

#define NOISE_THRESHOLD_CONTROL 0
#define FREQUENCY_PEAK_THRESHOLD 1000
#define FREQUENCY_SEARCH_LOWER_BOUNDARY 8 //in freq bins ///
#define FREQUENCY_SEARCH_UPPER_BOUNDARY 60 //in freq bins


hw_timer_t *timer = NULL;
portMUX_TYPE timerMux = portMUX_INITIALIZER_UNLOCKED;

volatile bool ISRFlag = false;
volatile int signalBuffer[BUFFER_LENGTH];
volatile int sampleIndex = 0; 

float realValues[BUFFER_LENGTH];
float imaginaryValues[BUFFER_LENGTH];
float previousFrequencyReading = 0.0;
int previousFrequencyIndex = 0;

 /*
const float frequencies[43] = {
    65.41, 69.30, 73.42, 77.78, 82.41, 87.31, 92.50, 98.00, 103.83, 110.00, 
    116.54, 123.47, 130.81, 138.59, 146.83, 155.56, 164.81, 174.61, 185.00, 
    196.00, 207.65, 220.00, 233.08, 246.94, 261.63, 277.18, 293.66, 311.13, 
    329.63, 349.23, 369.99, 392.00, 415.30, 440.00, 466.16, 493.88, 523.25, 
    554.37, 587.33, 622.25, 659.25};  

    const float frequencies[43] = {
    65.4, 69.3, 73.4, 77.8, 82.4, 87.3, 92.5, 98.0, 103.8, 110.0, 
    116.5, 123.5, 130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 
    196.0, 207.7, 220.0, 233.1, 246.9, 261.6, 277.2, 293.7, 311.1, 
    329.6, 349.2, 370.0, 392.0, 415.3, 440.0, 466.2, 493.9, 523.6, 
    554.4, 587.3, 622.3, 659.3};

  const float frequencies[43] = {
    65, 69, 73, 78, 82, 87, 93, 98, 104, 110, 
    117, 124, 131, 139, 147, 156, 165, 175, 185, 
    196, 208, 220, 233, 247, 261, 277, 293, 311, 
    330, 349, 370, 392, 415, 440, 466, 494, 524, 
    554, 587, 622, 659}; 
*/


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
  Serial.begin(1000000);
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

    float estimatedFrequency = 0.0;
    portENTER_CRITICAL_ISR(&timerMux);
    sampleIndex = 0;
    digitalWrite(DEBBUG_LED, !digitalRead(DEBBUG_LED));
    
    float signalAbsMaxValue = 0.0;
    for (int i = 0; i < BUFFER_LENGTH; i++) 
    {
      realValues[i] = (float)signalBuffer[i];
      imaginaryValues[i] = 0;
    }
    portEXIT_CRITICAL_ISR(&timerMux);
    FFT.dcRemoval();
    
    signalAbsMaxValue = calculateMaxAbs(realValues);
    if (signalAbsMaxValue <= NOISE_THRESHOLD_CONTROL) 
    {
      sampleIndex = 0;
      return;
    }

    FFT.windowing(FFTWindow::Blackman, FFTDirection::Forward);
    FFT.compute(FFT_FORWARD);
    calculateMagnitude();
    //normalizeSignal();

    //calculateMovingAverage();
    //for (int i = 0; i< BUFFER_LENGTH; i++) {realValues[i] = log10(realValues[i]); imaginaryValues[i] = 0;}
    //for (int i = 0; i< BUFFER_LENGTH; i++) 
    //FFT.compute(realValues, imaginaryValues, BUFFER_LENGTH, FFT_REVERSE);

    float spectrumMaxAbsValue = calculateMaxAbs(realValues);
    
    //for (int i = 0; i < BUFFER_LENGTH; i++) {if (realValues[i] <= 0) {realValues[i] = 1e-6;}}

    //for (int i = FREQUENCY_SEARCH_LOWER_BOUNDARY; i < FREQUENCY_SEARCH_UPPER_BOUNDARY; i++) {Serial.println(realValues[i]);}
    //for (int i = 25; i < BUFFER_LENGTH/2; i++) {Serial.println(realValues[i]);}
    //for (int i = 0; i < BUFFER_LENGTH; i++) {realValues[i] = log10(realValues[i]);}
    int peakIndex = findFirstMajorPeak(realValues);

    //Serial.println(peakIndex);
  
    float offset = 0.0;
    float interpolatedIndex = 0.0;
    if (peakIndex > 0 && peakIndex < BUFFER_LENGTH - 1) 
    {
      interpolatedIndex = calculateParabolicInterpolation(peakIndex, realValues);
    }

    estimatedFrequency = interpolatedIndex * SAMPLING_FREQUENCY / BUFFER_LENGTH;
    
  
    estimatedFrequency = decimalRound(estimatedFrequency,2);
 

  
    //Serial.println(estimatedFrequency);

    const float baseFrequency = getClosestTETFrequency(estimatedFrequency);
    float dev = calculateDeviationInCents(estimatedFrequency, baseFrequency);
  
    
    Serial.print(peakIndex);
    Serial.print(" "); 
    Serial.print(baseFrequency,2);
    Serial.print(" "); 
    Serial.print(estimatedFrequency,4);
    Serial.print(" ");
    Serial.println(dev);
    
    
    
    displayCentsDeviation(baseFrequency, estimatedFrequency);
    
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


float calculateMaxAbs(float arr[]) 
{
  float maxVal = abs(arr[0]);
  for (int i = 1; i < BUFFER_LENGTH; i++) {if (abs(arr[i]) > maxVal) {maxVal = abs(arr[i]);}}
  return maxVal;
}

const float getClosestTETFrequency(float measuredFrequency)
{
  
  
  /*
  const float frequencies[43] = {
    65.4, 69.3, 73.4, 77.8, 82.4, 87.3, 92.5, 98.0, 103.8, 110.0, 
    116.5, 123.5, 130.8, 138.6, 146.8, 155.6, 164.8, 174.6, 185.0, 
    196.0, 207.7, 220.0, 233.1, 246.9, 261.6, 277.2, 293.7, 311.1, 
    329.6, 349.2, 370.0, 392.0, 415.3, 440.0, 466.2, 493.9, 523.6, 
    554.4, 587.3, 622.3, 659.3};  
  */

  
    
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
    //clearNoteName();
    //clearIndicatorDiode();
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
    float deviationInCents = 1200 * log2(measuredFrequency / targetFrequency);
    deviationInCents = (int)deviationInCents;
    return deviationInCents;
}

float calculateParabolicInterpolation(int peakIndex, float frequencyMagnitude[])
{
  float offset;
  float interpolatedIndex; 

  offset = 0.5 * (frequencyMagnitude[peakIndex - 1] - frequencyMagnitude[peakIndex + 1])
      / (frequencyMagnitude[peakIndex - 1] - 2 * frequencyMagnitude[peakIndex] + frequencyMagnitude[peakIndex + 1]);
  interpolatedIndex = peakIndex + offset;
  return interpolatedIndex;
}


float decimalRound(float input, int decimals)
{
  float scale=pow(10,decimals);
  return round(input*scale)/scale;
}





//KOD WSTYDU (A NÓŻ WIDELEC SIĘ MOŻE PRZYDA)
 //for (int i = 0; i< BUFFER_LENGTH; i++) {Serial.println(frequencyMagnitude[i]);}

    /* 
    for (int i = 0; i< BUFFER_LENGTH; i++) {frequencyMagnitude[i] = log10(frequencyMagnitude[i]);}
    for (int i = 0; i< BUFFER_LENGTH; i++) 
    {
      realValues[i] = frequencyMagnitude[i];
      imaginaryValues[i] = 0;
    }
    FFT.compute(realValues, imaginaryValues, BUFFER_LENGTH, FFT_REVERSE);
    */
    //for (int i = CEPSTRUM_LOWER_BOUNDARY; i < CEPSTRUM_UPPER_BOUNDARY ; i++){Serial.println(realValues[i]);}
   
    //int peakIndex = findPeakIndex(realValues,CEPSTRUM_LOWER_BOUNDARY,CEPSTRUM_UPPER_BOUNDARY);
    //if (peakIndex == 0) { return; }
    //estimatedFundamentalFrequency = float(SAMPLING_FREQUENCY) / peakIndex;
/*
int findPeakIndex(float arr[], int minIdx, int maxIdx)
{
    float maxMagnitude; 
    int peakIndex = minIdx;

    for (int sampleIndex = minIdx; sampleIndex <= maxIdx; sampleIndex++) 
    {
        if (arr[sampleIndex] >= maxMagnitude) 
        {
            maxMagnitude = arr[sampleIndex];
            peakIndex = sampleIndex;
        }
    }
    return peakIndex;
}


void normalizeSignal()
{
  float minVal = realValues[0];  
  float maxVal = realValues[0];  

  for (int i = 1; i < BUFFER_LENGTH; i++) 
  {
    if (realValues[i] < minVal) minVal = realValues[i];
    if (realValues[i] > maxVal) maxVal = realValues[i];
  }

  if (maxVal - minVal < 1e-6) { return; }
  for (int i = 0; i < BUFFER_LENGTH; i++) 
  {
    float normalizedValue = 2.0 * (realValues[i] - minVal) / (maxVal - minVal) - 1.0;
    realValues[i] = normalizedValue; 
  }
}

float calculateMean(float arr[]) 
{
  long sum = 0; 
  for (int i = 0; i < BUFFER_LENGTH; i++) {sum += arr[i];}
  return (float)sum / BUFFER_LENGTH; 
}

float calculateDeviationInCents(float measuredFrequency, float targetFrequency) {
    // Calculate the deviation in cents from the target frequency
    float deviationInCents = 1200 * log2(measuredFrequency / targetFrequency);
    return deviationInCents;
}

void normalizeSignal()
{
  float minVal = realValues[0];  
  float maxVal = realValues[0];  

  for (int i = 1; i < BUFFER_LENGTH; i++) 
  {
    if (realValues[i] < minVal) minVal = realValues[i];
    if (realValues[i] > maxVal) maxVal = realValues[i];
  }

  if (maxVal - minVal < 1e-6) { return; }
  for (int i = 0; i < BUFFER_LENGTH; i++) 
  {
    float normalizedValue = 2.0 * (realValues[i] - minVal) / (maxVal - minVal) - 1.0;
    realValues[i] = normalizedValue; 
  }
}

void normalizeSignal()
{
  float minVal = realValues[0];  
  float maxVal = realValues[0];  

  for (int i = 1; i < BUFFER_LENGTH; i++) 
  {
    if (realValues[i] < minVal) minVal = realValues[i];
    if (realValues[i] > maxVal) maxVal = realValues[i];
  }

  if (maxVal - minVal < 1e-6) { return; }
  for (int i = 0; i < BUFFER_LENGTH; i++) 
  {
    float normalizedValue = 2.0 * (realValues[i] - minVal) / (maxVal - minVal) - 1.0;
    realValues[i] = normalizedValue; 
  }
}



void calculateMovingAverage() 
{
  int N = 2; // Size of the moving average window
  float smoothedValues[BUFFER_LENGTH]; // Temporary array for smoothed values

  for (int i = 0; i < BUFFER_LENGTH; i++) {
    float sum = 0.0;
    int count = 0;

    // Sum over the window range
    for (int j = -N / 2; j <= N / 2; j++) {
      int index = i + j;
      if (index >= 0 && index < BUFFER_LENGTH) { // Ensure index is within bounds
        sum += realValues[index];
        count++;
      }
    }

    // Store the smoothed value in the temporary array
    smoothedValues[i] = sum / count;
  }

  // Copy the smoothed values back to the original array if needed
  for (int i = 0; i < BUFFER_LENGTH; i++) {
    realValues[i] = smoothedValues[i];
  }
}

void normalizeSignal()
{
  float minVal = realValues[0];  
  float maxVal = realValues[0];  

  for (int i = 1; i < BUFFER_LENGTH; i++) 
  {
    if (realValues[i] < minVal) minVal = realValues[i];
    if (realValues[i] > maxVal) maxVal = realValues[i];
  }

  if (maxVal - minVal < 1e-6) { return; }
  for (int i = 0; i < BUFFER_LENGTH; i++) 
  {
    float normalizedValue = (realValues[i] - minVal) / (maxVal - minVal);
    realValues[i] = normalizedValue; 
  }
}


*/