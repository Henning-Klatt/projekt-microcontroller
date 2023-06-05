#include <Arduino.h>
#include <Adafruit_ST7789.h>

void setupDisplay(Adafruit_ST7789 tft);
void debugPrint(Adafruit_ST7789 tft, String text);
void printTimeElapsed(Adafruit_ST7789 tft, String text);