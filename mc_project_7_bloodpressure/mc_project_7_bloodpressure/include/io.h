#ifndef IO_H
#define IO_H

#include "Adafruit_SPIFlash.h"
#include "SdFat.h"

#define PUMP 9
#define VALVE 10
#define startButton 7
#define interruptButton 12
#define LED_BTN_RED 11
#define LED_BTN_GREEN 5

// Waveshare 17344 Display
#define TFT_CS 17
#define TFT_DC 16
#define TFT_RST 15
#define TFT_BL 14

void setupIO();

#endif