#include <Arduino.h>
#include <Adafruit_TinyUSB.h> // for Serial
#include <Wire.h>
#include "Adafruit_MPRLS.h"
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <math.h>
#include "SdFat.h"
#include "Adafruit_SPIFlash.h"

#include "io.h"
#include "display.h"
#include "filter.h"
#include "measure.h"

// #include "evaluate.h"

// Library object for the flash configuration
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs; // file system
File32 myFile;   // file format for the SD memory

// Library for the pressure sensor
Adafruit_MPRLS pressureSensor = Adafruit_MPRLS();

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

void interruptFunction()
{
  int state = !digitalRead(interruptButton);
  if (state)
  {
    measurementRunning = false;
    measurementJustStarted = false;
    flagInterrupt = true;
    digitalWrite(VALVE, LOW); // open solanoid -> release air
    digitalWrite(PUMP, LOW);  // disable Pump
  }
  digitalWrite(LED_BTN_RED, state);
}

void greenButton()
{
  int state = !digitalRead(startButton);
  if (state)
  {
    measurementRunning = true;
    measurementJustStarted = true;
    flagInterrupt = false;
  }
  digitalWrite(LED_BTN_GREEN, state);
  // Serial.println("Green Button: " + String(state));
}

void setup()
{
  // start serial communication and set baud rate
  Serial.begin(9600);
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
  Serial.println("Initializing Filesystem on external flash...");
  // Init external flash
  flash.begin();
  // Open file system on the flash
  if (!fatfs.begin(&flash))
  {
    Serial.println("Error: filesystem is not existed. Please try SdFat_format example to make one.");
    while (1)
    {
      delay(10);
    }
  }
  tft.init(240, 320);
  setupDisplay(tft);
  tft.setTextSize(3);
  Serial.println("initialization done.");
  setupIO();
  attachInterrupt(digitalPinToInterrupt(interruptButton), interruptFunction, CHANGE);
  attachInterrupt(digitalPinToInterrupt(startButton), greenButton, CHANGE); // green Button
  calibrationPressure = pressureSensor.readPressure();
  // refresh memory of sample-buffer
  memset(currentPressure, 0, sizeof(currentPressure));
  memset(measSample, 0, sizeof(measSample));
  memset(HPbuffer_5Hz, 0, sizeof(HPbuffer_5Hz));
  memset(HPbuffer_0_5Hz, 0, sizeof(HPbuffer_0_5Hz));
  memset(HPmeasSample, 0, sizeof(HPmeasSample));
}

void loop()
{
  writeCount = 0;

  if (!measurementRunning)
  {
    delay(100);
    clearDisplay(tft);
    debugPrint(tft, "<- Push to start", 1);
    Serial.println("Press the green button to start!");
    return;
  }

  // Pump to relative 180mmHg (i.e. 240hPa)
  while (measurementJustStarted && !flagInterrupt)
  {
    relativePressure = pressureSensor.readPressure() - calibrationPressure;
    if (relativePressure < pressureIncrease)
    {
      digitalWrite(PUMP, HIGH);  // activate Pump
      digitalWrite(VALVE, HIGH); // Close Solanoid -> build up pressure
      Serial.println("relative Pressure: " + String(relativePressure));
      bigPrint(tft, String(relativePressure, 0) + " hPa / " + String(pressureIncrease) + " hPa");
    }
    else
    {
      digitalWrite(PUMP, LOW); // deactivate Pump
      measurementJustStarted = false;
    }
  }

  currentMillis = millis();

  if (currentMillis - previousMillis >= measPeriod && !flagInterrupt && measurementRunning)
  {
    previousMillis = currentMillis;
    Serial.println("=====================================");
    pressure_hPa = pressureSensor.readPressure();
    measSample[SampleCount] = (uint16_t)pressure_hPa;

    relativePressure = pressure_hPa - calibrationPressure;
    Serial.println("abs. Pressure: " + String(measSample[SampleCount])); // Print the measured pressure
    sensorSpeed = millis() - currentMillis;
    Serial.println("Read-out speed (ms): " + String(sensorSpeed)); // Print the read-out time of one measurement
    realPeriod = currentMillis - endTimer;
    Serial.println("Time since last measurement: " + String(realPeriod)); // Print your set measurement interval

    // highpass filter
    HPbuffer_5Hz[0] = pressure_hPa;
    HPbuffer_5Hz[1] = HPbuffer_5Hz[0]; // [0] ist der aktuellere Wert

    // save every 10th sample
    if (SampleCount % 10 == 0)
    {
      HPbuffer_0_5Hz[0] = pressure_hPa;
      HPbuffer_0_5Hz[1] = HPbuffer_0_5Hz[0];
      Serial.println("Sample " + String(SampleCount) + " saved!");
    }

    HPfilter(&pressure_hPa, &HPbuffer_5Hz[0], &HPbuffer_0_5Hz[0]);

    HPmeasSample[SampleCount] = (uint16_t)pressure_hPa;

    SampleCount++;

    Serial.println("rel. Pressure: " + String(relativePressure, 0));
    Serial.println("SampleCount: " + String(SampleCount) + " / 12000");
    endTimer = millis();
    float elapsed = ((millis() - currentMillis));
    Serial.println("==== Time elapsed: " + String(elapsed) + " ====");
    /*
    Display too slow
    printTimeElapsed(tft, String(elapsed) + " s");
    debugPrint(tft, "abs. Pressure: " + String(pressure_hPa, 0), 1);
    debugPrint(tft, "rel. Pressure: " + String(relativePressure, 0), 2);
    debugPrint(tft, "Samples: " + String(SampleCount) + " / " + String(sizeof(measSample) / 2), 3);
    */
  }

  // write measurement array to a .txt file
  if ((SampleCount >= 12000 || (relativePressure < pressureThreshold)) && !flagInterrupt)
  {
    measurementRunning = false; // stop measurement -> save to file
    digitalWrite(VALVE, LOW);   // open solanoid -> release air
    bigPrint(tft, "Saving...");
    fatfs.remove("pressureMeasurement.txt");
    myFile = fatfs.open("pressureMeasurement.txt", FILE_WRITE);
    if (myFile)
    {
      Serial.print("Writing to pressureMeasurement.txt...");
      for (int i = 0; i < sizeof(measSample) / 2; i++)
      {
        myFile.print(measSample[i]);
        myFile.print(",");
      }
      myFile.close();
      Serial.println("done!");
    }
    else
    {
      Serial.println("error opening pressureTest.txt");
    }
    fatfs.remove("HPpressureMeasurement.txt");
    myFile = fatfs.open("HPpressureMeasurement.txt", FILE_WRITE);
    if (myFile)
    {
      Serial.print("Writing to HPpressureMeasurement.txt...");
      for (int i = 0; i < sizeof(measSample) / 2; i++)
      {
        myFile.print(HPmeasSample[i]);
        myFile.print(",");
      }
      myFile.close();
      Serial.println("done!");
      bigPrint(tft, "Done         ");
    }
    else
    {
      Serial.println("error opening pressureTest.txt");
    }
    // readFlash(fatfs);
    // Serial.println(findHeartRate());
  }
}