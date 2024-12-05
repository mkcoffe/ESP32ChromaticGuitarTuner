#include "esp32-hal-gpio.h"
#include "guitarTunerDisplay.h"
#include <Arduino.h>

char ledLineArray[LED_LINE_ARRAY_LENGTH] = {LED_0, LED_1, LED_2, LED_3, LED_4, LED_5, LED_6, LED_7, LED_8};
char ledCathodesArray[LED_CATHODES_ARRAY_LENGTH] = {LED_g,LED_f,LED_d,LED_e,LED_b,LED_a,LED_c,LED_dp};

void displayIndicatorDiode(char diodeNumber) {
  for (int i = 0; i < LED_LINE_ARRAY_LENGTH; i++) {digitalWrite(ledLineArray[i], LOW);}
  digitalWrite(ledLineArray[diodeNumber], HIGH);
}

void clearIndicatorDiode() {for (int i = 0; i < LED_LINE_ARRAY_LENGTH; i++) { digitalWrite(ledLineArray[i], LOW); } }

void clearNoteName() { for (int i = 0; i < LED_CATHODES_ARRAY_LENGTH; i++) { digitalWrite(ledCathodesArray[i],HIGH); } }

void displayNoteName(const char* noteName) 
{
  int noteIdx = 0;
  bool ledLettersArray[12][LED_CATHODES_ARRAY_LENGTH] = {
    {HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH}, //C
    {HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH, LOW}, //C#
    {LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, HIGH}, //D
    {LOW, HIGH, LOW, LOW, LOW, HIGH, LOW, LOW}, //D#
    {LOW, LOW, LOW, LOW, HIGH, LOW, HIGH, HIGH}, //E
    {LOW, LOW, HIGH, LOW, HIGH, LOW, HIGH, HIGH}, //F
    {LOW, LOW, HIGH, LOW, HIGH, LOW, HIGH, LOW}, //F#
    {HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, HIGH}, //G
    {HIGH, LOW, LOW, LOW, HIGH, LOW, LOW, LOW}, //G#
    {LOW, LOW, HIGH, LOW, LOW, LOW, LOW, HIGH}, //A
    {LOW, LOW, HIGH, LOW, LOW, LOW, LOW, LOW}, //A#
    {LOW, LOW, LOW, LOW, HIGH, HIGH, LOW, HIGH} //B
  };

  if (strcmp(noteName, "c") == 0 || strcmp(noteName, "C") == 0) {
    noteIdx = 0;
} else if (strcmp(noteName, "c#") == 0 || strcmp(noteName, "C#") == 0) {
    noteIdx = 1;
} else if (strcmp(noteName, "d") == 0 || strcmp(noteName, "D") == 0) {
    noteIdx = 2;
} else if (strcmp(noteName, "d#") == 0 || strcmp(noteName, "D#") == 0) {
    noteIdx = 3;
} else if (strcmp(noteName, "e") == 0 || strcmp(noteName, "E") == 0) {
    noteIdx = 4;
} else if (strcmp(noteName, "f") == 0 || strcmp(noteName, "F") == 0) {
    noteIdx = 5;
} else if (strcmp(noteName, "f#") == 0 || strcmp(noteName, "F#") == 0) {
    noteIdx = 6;
} else if (strcmp(noteName, "g") == 0 || strcmp(noteName, "G") == 0) {
    noteIdx = 7;
} else if (strcmp(noteName, "g#") == 0 || strcmp(noteName, "G#") == 0) {
    noteIdx = 8;
} else if (strcmp(noteName, "a") == 0 || strcmp(noteName, "A") == 0) {
    noteIdx = 9;
} else if (strcmp(noteName, "a#") == 0 || strcmp(noteName, "A#") == 0) {
    noteIdx = 10;
} else if (strcmp(noteName, "b") == 0 || strcmp(noteName, "B") == 0) {
    noteIdx = 11;
} else {
    // Handle the case when the note is invalid or not recognized
}
  clearNoteName();
  for (int i = 0; i < (LED_CATHODES_ARRAY_LENGTH); i++) { digitalWrite(ledCathodesArray[i],ledLettersArray[noteIdx][i]); }
}

void setupLedDisplay() {

  for (int i = 0; i < LED_LINE_ARRAY_LENGTH; i++) { pinMode(ledLineArray[i], OUTPUT); } 
  for (int i = 0; i < LED_CATHODES_ARRAY_LENGTH; i++) { pinMode(ledCathodesArray[i],OUTPUT); } 

  delay(100);
  for(int i = 0; i < LED_LINE_ARRAY_LENGTH; i++) 
  {
    delay(20);
    displayIndicatorDiode(i);
    delay(20);
  }
  for(int i = (LED_LINE_ARRAY_LENGTH-1); i >= 0; i--)
   { 
    delay(20);
    displayIndicatorDiode(i);
    delay(20);
  }
  clearIndicatorDiode();
  clearNoteName();
}




