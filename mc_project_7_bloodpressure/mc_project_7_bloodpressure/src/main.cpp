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

void findHeartbeat()
{
  bool flagHeart = false;
  float absMax = 0;
  int16_t indexAbsMax = 0;
  int16_t peakMax = 0;
  int16_t peakMin = 100;
  float heartRate = 0;

  int indexSweep = 0;
  int i = 0;

  int16_t peakMeas[360];     // Measured peaks, for meas time of 2min and heart-rate of 180 -> max. 360 (also enough overhead for false noise-peaks)
  uint16_t tPeakMeas[360];   // time-stamp of the detected peak
  int16_t peakMeasTh[360];   // higher threshold of peakdetection, depending on the maxima from peakMeas array
  uint16_t tPeakMeasTh[360]; // corresponding time stamp

  // reset buffer arrays
  memset(peakMeas, 0, sizeof(peakMeas));
  memset(tPeakMeas, 0, sizeof(tPeakMeas));
  memset(peakMeasTh, 0, sizeof(peakMeas));
  memset(tPeakMeasTh, 0, sizeof(tPeakMeas));

  while (i < 12000 && indexSweep < 360)
  {

    while (HPmeasSample[i] > 100)
    {
      if (peakMax < HPmeasSample[i])
      {
        peakMax = HPmeasSample[i];
        tPeakMeas[indexSweep] = i;
      }
      i++;
    }
    while (HPmeasSample[i] <= 100 && i < 12000)
    {
      if (peakMin > HPmeasSample[i])
      {
        peakMin = HPmeasSample[i];
      }
      i++;
    }
    peakMeas[indexSweep] = peakMax - peakMin;
    peakMax = 0;
    peakMin = 100;

    Serial.println(indexSweep);
    Serial.println(peakMeas[indexSweep]);
    indexSweep++;
  }

  i = 0;
  while (i < 360)
  {
    if (absMax < peakMeas[i])
    {
      absMax = peakMeas[i];
      Serial.println(absMax);
    }
    i++;
  }

  i = 0;
  for (int a = 0; a < 360; a++)
  {
    if (peakMeas[a] > 0.4 * absMax)
    {
      peakMeasTh[i] = peakMeas[a];
      tPeakMeasTh[i] = tPeakMeas[a];
      Serial.println(peakMeasTh[i]);
      i++;
    }
  }

  Serial.println("done!");
  Serial.print("Number of Peaks: ");
  Serial.println(indexSweep);
  Serial.print("Absolut maxima (hPa): ");
  Serial.println(((float)absMax) / 1000.0);
  Serial.print("Index absolut maxima: ");
  Serial.println(indexAbsMax);
  Serial.print("Heart-rate /1/min): ");
  Serial.println(round(heartRate));
  Serial.println("Finished printing!");
  Serial.println("\n ################### \n");
  delay(1000);
}

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
    // clearDisplay(tft);
    // textPrint(tft, "Cancelled!");
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
    // clearDisplay(tft);
    debugPrint(tft, "<- Push to start", 1);
    debugPrint(tft, "<- Push to cancel", 11);
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
      textPrint(tft, String(relativePressure, 0) + " / " + String(pressureIncrease) + " hPa");
    }
    else
    {
      digitalWrite(PUMP, LOW); // deactivate Pump
      measurementJustStarted = false;
      textPrint(tft, "                     ");
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

    // HPmeasSample[SampleCount] = (uint16_t)pressure_hPa;
    // HPmeasSample[SampleCount] = (uint16_t)HPbuffer_5Hz[0];
    //*(&measSample[0] + writeCount) = (uint16_t)(round(100 * (pressure_hPa - startPressure))); // write samples to the measurement array
    //*(&HPmeasSample[0] + writeCount) = (int16_t)(round(1000 * HPbuffer_0_5Hz[0]));                  // write samples to the measurement array

    HPmeasSample[SampleCount] = (int16_t)(round(1000 * HPbuffer_0_5Hz[0]));
    measSample[SampleCount] = (uint16_t)(round(100 * (currentPressure[0] - calibrationPressure))); // write samples to the measurement array

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
    bigPrint(tft, "Saving...    ");
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
    // Serial.println(getHeartRate());

    Serial.println("Searching for the peaks in the measurement data ... ");

    int16_t peakMeas[360]; // buffer for the detected peak, maximum 2 minutes with heart rate of 180/min

    int peakMax = 0; // intermediate maximum value of the peak
    int peakMin = 0; // intermediate minimum value of the peak

    int absMax = 0;      // absolut maximum of the peaks
    int indexAbsMax = 0; // index of the maximum peak

    // look at the data
    for (int i = 0; i < sizeof(measSample) / 2; i++)
    {
      // and find the peaks, where a certain threshold is crossed. As threshold value use +0.1hPa.
      if (HPmeasSample[i] > 100) // 0.1 hPa = 100 weil davor *1000 gemacht wurde
      {
        // Then when a peak is detected
        // find the maximum and minimum value, until the next positive peak is detected.
        if (HPmeasSample[i] > peakMax)
        {
          peakMax = HPmeasSample[i];
          absMax = peakMax;
          indexAbsMax = i;
        }
        if (HPmeasSample[i] < peakMin)
        {
          peakMin = HPmeasSample[i];
        }
      }
    }
    // By subtracting the found maxima and minima the peak is found and the envelope can be calculated.
    peakMax - peakMin;
    findHeartbeat();
  }
}