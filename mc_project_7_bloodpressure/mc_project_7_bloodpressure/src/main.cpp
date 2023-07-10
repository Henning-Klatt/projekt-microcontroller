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

// #include <Adafruit_nRFCrypto.h>
// #include <bluefruit.h>

#include "io.h"
#include "display.h"
#include "filter.h"
#include "measure.h"

#define DEBUG

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

void startAdv(void)
{
  // Advertising packet
  // Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
  // Bluefruit.Advertising.addTxPower();

  // Include bleuart 128-bit uuid
  // Bluefruit.Advertising.addService(bleuart);

  // Secondary Scan Response packet (optional)
  // Since there is no room for 'Name' in Advertising packet
  // Bluefruit.ScanResponse.addName();
  // Bluefruit.setName("Hennings Blutdruckmessgerät");
  // Bluefruit.Advertising.restartOnDisconnect(true);
  // Bluefruit.Advertising.setInterval(32, 244); // in unit of 0.625 ms
  // Bluefruit.Advertising.setFastTimeout(30);   // number of seconds in fast mode
  // Bluefruit.Advertising.start(0);             // 0 = Don't stop advertising after n seconds
}

void findHeartbeat()
{

  int16_t tPeakMeas[NOP];   // timestamp of the detected peak
  int16_t tPeakMeasTh[NOP]; // timestamp of the buffer
  int16_t peakMeas[NOP];    // buffer for the detected peak, maximum 2 minutes with heart rate of 180/min
  int16_t peakMeasTh[NOP];  // buffer for the detected peak, after threshold detection

  int peakMax = 0;          // intermediate maximum value of the peak
  int peakMin = 100;        // intermediate minimum value of the peak
  int absMax = 0;           // absolut maximum of the peaks
  u_int8_t indexAbsMax = 0; // index of the maximum peak

  bool flagHeart = false; // flag if heartbeat is detected
  float heartRate = 0;    // final heart rate
  float diaPressure = 0;  // diastolic blood pressure
  float sysPressure = 0;  // systolic blood pressure

  int indexSweep = 0; // index sweep to write in the buffer
  int i = 0;

  // reset buffer arrays
  memset(peakMeas, 0, sizeof(peakMeas));
  memset(tPeakMeas, 0, sizeof(tPeakMeas));
  memset(peakMeasTh, 0, sizeof(peakMeas));
  memset(tPeakMeasTh, 0, sizeof(tPeakMeas));

  Serial.println("Start searching for heartbeat");

  // go through the first 360 values of the filtered data, 360 is the size of the buffer (peakMeas, tPeakMeas, ...)
  while (i < NOS && indexSweep < NOP)
  {

    // find the maximum value of the next peak in the filtered data
    // 0.1 hPa is used as a threshold to decern peaks from valleys
    // HPmeasSample[i]*100
    while (HPmeasSample[i] > 100)
    {
      // If the maximum value of the peak is found, save the timestamp
      if (peakMax < HPmeasSample[i])
      {
        peakMax = HPmeasSample[i];
        tPeakMeas[indexSweep] = i;
      }
      i++;
    }

    // find the minimum of the following valley in the filtered data
    while (HPmeasSample[i] <= 100 && i < NOS)
    {
      // If the minimum value of the valley is found, save the timestamp
      if (peakMin > HPmeasSample[i])
      {
        peakMin = HPmeasSample[i];
      }
      i++;
    }
    // Save the difference between the maximum and the minimum of the heartbeat
    peakMeas[indexSweep] = peakMax - peakMin;
    peakMax = 0;
    peakMin = 100;

    // Serial.println(peakMeas[indexSweep]);
    indexSweep++;
  }

  //  Find the largest peak in the previously found peaks
  for (int l = 0; l < NOP; l++)
  {
    // If the peak is greater than the previous peak, save the index
    if (absMax < peakMeas[l])
    {
      absMax = peakMeas[l];
      indexAbsMax = l;
    }
  }

  // Removing false positives due to noise by threshold detection
  i = 0;
  for (int a = 0; a < NOP; a++)
  {
    delay(1);
    // If the peak is greater than 40% of the maximum peak, save the peak and the timestamp
    if (peakMeas[a] > 0.4 * absMax)
    {
      peakMeasTh[i] = peakMeas[a];
      tPeakMeasTh[i] = tPeakMeas[a];
      i++;
    }
  }

  delay(100);

  size_t heartrate_index = 0;

  Serial.println("Find the first peak after the threshold detection");
  for (int l = 0; l < NOP; l++)
  {
    delay(1);
    if (peakMeasTh[l] == 0)
    {
      heartrate_index = l / 2;
#ifdef DEBUG
      Serial.println("heartrate_index: " + heartrate_index);
#endif

      break;
    }
  }

  delay(100);

  // Calculate the heart rate in bpm
  const int count = 5;
  int16_t time = 0;

  // for (size_t k = heartrate_index; k < heartrate_index + count; k++)
  for (size_t k = 0; k < count; k++)
  {
    if (k + 1 >= NOP)
    {
      break;
    }
    // Calculate the time between the peaks
    time += (tPeakMeasTh[k + 1] - tPeakMeasTh[k]) * measPeriod;

#ifdef DEBUG
    Serial.print("index ");
    Serial.print(k);
    Serial.print(": ");
    Serial.println(time);
#endif // DEBUG
  }

  delay(100);
  // Calculate the average time between the peaks
  time = time / count;
  // Calculate the heart rate in bpm
  heartRate = 60000 / time;

  sysPressure = sysTh * (absMax / 100.0);
  diaPressure = diaTh * (absMax / 100.0);

  Serial.println("done!");
  Serial.print("Number of Peaks: ");
  Serial.println(indexSweep);
  Serial.print("Absolut maxima (hPa): ");
  Serial.println(((float)absMax) / 1000.0);
  Serial.print("Index absolut maxima: ");
  Serial.println(indexAbsMax);
  Serial.print("Systolic blood pressure (mmHg): ");
  Serial.println(hPa_mmHg * sysPressure);
  Serial.print("Diastolic blood pressure (mmHg): ");
  Serial.println(hPa_mmHg * diaPressure);
  Serial.print("Heart-rate /1/min): ");
  Serial.println(round(heartRate));
  Serial.println("Finished printing!");
  Serial.println("\n ################### \n");
  clearDisplay(tft);
  debugPrint(tft, "Heart-rate: " + String(heartRate, 0) + " /1/min", 1);
  debugPrint(tft, "Systolic pressure: " + String(sysPressure, 0) + "mmHg", 3);
  debugPrint(tft, "Diastolic pressure: " + String(diaPressure, 0) + "mmHg", 5);
  delay(100);
  flagInterrupt = false;
  delay(1000);
  while (flagInterrupt == false)
  {
    delay(100);
    Serial.println("Waiting for red button to be pressed!");
  }
  clearDisplay(tft);

#ifdef DEBUG
  Serial.println("PeakMeas:");
  print_array(peakMeas, NOP);

  Serial.println("tPeakMeas:");
  print_array(tPeakMeas, NOP);

  Serial.println("PeakMeasTh:");
  print_array(peakMeasTh, NOP);

  Serial.println("tPeakMeasTh:");
  print_array(tPeakMeasTh, NOP);
#endif // DEBUG
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
  // Set up Advertising Packet
  startAdv();

  // Start Advertising
  // Bluefruit.Advertising.start();
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
  // tft.init(240, 320);
  tft.init(240, 320);
  setupDisplay(tft);
  tft.setTextSize(3);
  tft.setRotation(3);
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
      clearDisplay(tft);
      bigPrint(tft, "Testing your strength...");
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
    clearDisplay(tft);
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
      delay(1000);
      Serial.println("done!");
      delay(1000);
      clearDisplay(tft);
      bigPrint(tft, "Done        ");
    }
    else
    {
      Serial.println("error opening pressureTest.txt");
    }
    // readFlash(fatfs);
    // Serial.println(findHeartRate());
    // Serial.println(getHeartRate());

    Serial.println("Searching for the peaks in the measurement data ... ");
    delay(1000);
    clearDisplay(tft);
    bigPrint(tft, "Calculating...");
    findHeartbeat();
  }
}