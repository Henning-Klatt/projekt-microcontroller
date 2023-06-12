#include <Arduino.h>
#include <Adafruit_ST7789.h>
#include <Adafruit_GFX.h> // Core graphics library

void setupDisplay(Adafruit_ST7789 tft)
{
    // tft.init(240, 320);
    tft.setRotation(3);
    // tft.setTextSize(3);
    tft.fillScreen(ST77XX_BLACK);
}

void debugPrint(Adafruit_ST7789 tft, String text, int line)
{
    // clear display:
    tft.fillRect(0, 20 * line, 240, 20, ST77XX_BLACK);
    tft.setTextSize(2);
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(0, 20 * line);
    tft.print(text + "\n");
}

void printTimeElapsed(Adafruit_ST7789 tft, String text)
{
    tft.setTextColor(ST77XX_GREEN, ST77XX_BLACK);
    tft.setCursor(0, 50);
    tft.print(text);
}