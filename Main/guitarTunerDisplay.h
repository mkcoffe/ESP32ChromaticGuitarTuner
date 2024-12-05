#ifndef GUITARTUNERDISPLAY_H
#define GUITARTUNERDISPLAY_H


#define LED_0 16
#define LED_1 17
#define LED_2 5
#define LED_3 18
#define LED_4 23 // indicates string in tune 
#define LED_5 19
#define LED_6 32
#define LED_7 33
#define LED_8 25
#define LED_LINE_ARRAY_LENGTH 9
#define LED_g 4
#define LED_f 2
#define LED_d 15
#define LED_e 13
#define LED_b 26 
#define LED_a 27
#define LED_c 14
#define LED_dp 12 
#define LED_CATHODES_ARRAY_LENGTH 8

extern char ledCathodesArray[];
extern char ledLineArray[];

void displayIndicatorDiode(char diodeNumber);
void clearIndicatorDiode();
void clearNoteName();
void displayNoteName(const char* noteName);
void setupLedDisplay();


#endif // GUITARTUNERDISPLAY_H