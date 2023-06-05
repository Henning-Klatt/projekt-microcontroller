#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <SdFat.h> // SD card & FAT filesystem library
#include <Adafruit_TinyUSB.h>
#include "Adafruit_MPRLS.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include "Adafruit_SPIFlash.h"

#ifndef FLASH_CONFIG_H_
#define FLASH_CONFIG_H_
#endif

#include "main.h"
#include "io.h"
#include "display.h"
#include "music.h"

Adafruit_MPRLS pressureSensor = Adafruit_MPRLS();
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

volatile bool measurementRunning = false;
volatile bool measurementJustStarted = false;
float pressure_hPa = 0;
unsigned long startTimer = 0;
unsigned long endTimer = 0;
unsigned long sensorSpeed = 0;
unsigned long measPeriod = 10; // set the measurement interval to 10ms
unsigned long realPeriod = 0;
unsigned long previousMillis = 0;
unsigned long startTime = 0;

void greenButton()
{
  int state = !digitalRead(BUT_G);
  if (state)
  {
    measurementRunning = true;
    measurementJustStarted = true;
  }
  digitalWrite(11, state);
  // Serial.println("Green Button: " + String(state));
}

void redButton()
{
  int state = !digitalRead(7);
  if (state)
  {
    measurementRunning = false;
    digitalWrite(VENT, LOW); // open solanoid -> release air
    digitalWrite(PUMP, LOW); // disable Pump
  }
  digitalWrite(5, state);
  // Serial.println("Red Button: " + String(state));
}

void setup()
{

  setupIO();
  attachInterrupt(digitalPinToInterrupt(BUT_G), greenButton, CHANGE); // green Button
  attachInterrupt(digitalPinToInterrupt(BUT_R), redButton, CHANGE);   // red Button
  Serial.begin(115200);
  Serial.println("MPRLS Simple Test");
  if (!pressureSensor.begin())
  {
    Serial.println("Failed to communicate with MPRLS sensor, check wiring?");
    while (1)
    {
      delay(10);
    }
  }
  Serial.println("Found MPRLS sensor");

  tft.init(240, 320);
  setupDisplay(tft);
  tft.setTextSize(3);
  // Init external flash
  flash.begin();
  writeToFile(flash);
  readFromFile(flash);
  playNokia();
}

void loop()
{

  if (!measurementRunning)
  {
    delay(100);
    debugPrint(tft, "<- Push   ");
    return;
  }

  if (measurementJustStarted)
  {
    startTime = millis();
    measurementJustStarted = false;
  }

  startTimer = millis();
  digitalWrite(VENT, HIGH); // Close Solanoid
  digitalWrite(PUMP, HIGH); // activate Pump

  if (startTimer - previousMillis >= measPeriod)
  {
    previousMillis = startTimer;
    pressure_hPa = pressureSensor.readPressure();
    Serial.print("Pressure (hPa): "); // Print the measured pressure
    Serial.println(pressure_hPa);
    debugPrint(tft, String(pressure_hPa) + " hPa");
    sensorSpeed = millis() - startTimer;
    Serial.println("Read-out speed (ms): " + String(sensorSpeed)); // Print the read-out time of one measurement
    realPeriod = startTimer - endTimer;
    Serial.println("Time since last measurement: " + String(realPeriod)); // Print your set measurement interval
    // delay(1000);
    endTimer = millis();
    float elapsed = ((millis() - startTime) / 1000.0);
    Serial.println("Time elapsed: " + String(elapsed));
    printTimeElapsed(tft, String(elapsed) + " s");
  }
}