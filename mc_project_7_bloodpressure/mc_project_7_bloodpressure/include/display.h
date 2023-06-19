#ifndef DISPLAY_H
#define DISPLAY_H

#include <Arduino.h>
#include <Adafruit_ST7789.h>

void setupDisplay(Adafruit_ST7789 tft);
void debugPrint(Adafruit_ST7789 tft, String text, int line);
void bigPrint(Adafruit_ST7789 tft, String text);
void clearDisplay(Adafruit_ST7789 tft);

#endif