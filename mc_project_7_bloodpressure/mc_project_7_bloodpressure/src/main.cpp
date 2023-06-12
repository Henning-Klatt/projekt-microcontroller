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

#define pressureIncrease 280 // put this value to 240+40 as overshoot compensation if you want to measure the blood pressure
#define pressureThreshold 40 // lower threshold, when the cuff is deflated, put this value to 40 for blood pressure measurement

#define settleTime 500 // settle time in ms, when pump/valve is turned on/off
#define maxTime 120000 // after 2 minutes stop the measurement
#define measPeriod 10  // 10ms of sampling time

// Library object for the flash configuration
Adafruit_FlashTransport_QSPI flashTransport;
Adafruit_SPIFlash flash(&flashTransport);

// file system object from SdFat
FatVolume fatfs; // file system
File32 myFile;   // file format for the SD memory
int readFile;    // read the file position, ATTENTION: Output is decoded as integer in ASCII-format!

// Library for the pressure sensor
Adafruit_MPRLS pressureSensor = Adafruit_MPRLS();

Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

float currentPressure[2];            // intermediate pressure values, with buffer of last value
float startPressure = 0;             // intermediate pressure values
volatile bool flagInterrupt = false; // emergency flag from the interrupt
unsigned long startTimer = 0;        // startpoint of the timer
unsigned long endTimer = 0;          // endpoint of the timer
unsigned long startMaxTimer = 0;     // start the timer at the beginning of the whole measurement process, to have a maximum time to stop
float HPbuffer_5Hz[2];               // buffer of last two highpass filter values, with f_3dB = 5Hz
float HPbuffer_0_5Hz[2];             // buffer of last two highpass filter values, with f_3dB = 0.5Hz
uint16_t measSample[12000];          // array for the measured samples, maximum 2 minutes every 10ms -> 12000 entries
int16_t HPmeasSample[12000];         // array of the highpass filtered samples
int writeCount = 0;                  // used for print counter every 500ms and position in array to write to

unsigned long previousMillis = 0;
float pressure_hPa = 0;
unsigned long sensorSpeed = 0;
unsigned long realPeriod = 0;
unsigned long startTime = 0;

volatile bool measurementRunning = false;
volatile bool measurementJustStarted = false;

float relativePressure = 0;    // relative pressure value
float calibrationPressure = 0; // relative pressure value

int SampleCount = 0; // counter for the samples

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
}

void loop()
{
  // refresh memory of sample-buffer
  memset(currentPressure, 0, sizeof(currentPressure));
  memset(measSample, 0, sizeof(measSample));
  memset(HPbuffer_5Hz, 0, sizeof(HPbuffer_5Hz));
  memset(HPbuffer_0_5Hz, 0, sizeof(HPbuffer_0_5Hz));
  memset(HPmeasSample, 0, sizeof(HPmeasSample));
  writeCount = 0;
  Serial.println("Press the green button to start!");

  if (!measurementRunning)
  {
    delay(100);
    debugPrint(tft, "<- Push to start         ", 1);
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
      debugPrint(tft, String(relativePressure, 0) + " hPa / " + String(pressureIncrease) + " hPa", 1);
    }
    else
    {
      digitalWrite(PUMP, LOW); // deactivate Pump
      measurementJustStarted = false;
    }
  }

  startTimer = millis();

  if (startTimer - previousMillis >= measPeriod && !flagInterrupt && measurementRunning)
  {
    previousMillis = startTimer;
    pressure_hPa = pressureSensor.readPressure();
    relativePressure = pressure_hPa - calibrationPressure;
    Serial.println("abs. Pressure: " + String(pressure_hPa, 0)); // Print the measured pressure
    debugPrint(tft, "abs. Pressure: " + String(pressure_hPa, 0), 1);
    sensorSpeed = millis() - startTimer;
    Serial.println("Read-out speed (ms): " + String(sensorSpeed)); // Print the read-out time of one measurement
    realPeriod = startTimer - endTimer;
    Serial.println("Time since last measurement: " + String(realPeriod)); // Print your set measurement interval
    // delay(1000);
    endTimer = millis();
    float elapsed = ((millis() - startTime) / 1000.0);
    Serial.println("Time elapsed: " + String(elapsed));
    // printTimeElapsed(tft, String(elapsed) + " s");
    measSample[SampleCount] = pressure_hPa;
    SampleCount++;
    Serial.println("rel. Pressure: " + String(relativePressure, 0));
    debugPrint(tft, "rel. Pressure: " + String(relativePressure, 0), 2);
    Serial.println("SampleCount: " + String(SampleCount) + " / 12000");
    debugPrint(tft, "Samples: " + String(SampleCount) + " / " + String(sizeof(measSample) / 2), 3);
  }

  /*put your code here*/
  // write measurement array to a .txt file
  if ((SampleCount >= 12000 || (relativePressure < pressureThreshold)) && !flagInterrupt)
  {
    measurementRunning = false; // stop measurement -> save to file
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
    }
    else
    {
      Serial.println("error opening pressureTest.txt");
    }
  }
}