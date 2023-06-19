#ifndef MEASURE_H
#define MEASURE_H

#include <Arduino.h>

// #define pressureIncrease 280 // put this value to 240+40 as overshoot compensation if you want to measure the blood pressure
#define pressureThreshold 40 // lower threshold, when the cuff is deflated, put this value to 40 for blood pressure measurement

#define pressureIncrease 80

#define settleTime 500 // settle time in ms, when pump/valve is turned on/off
#define maxTime 120000 // after 2 minutes stop the measurement
#define measPeriod 10  // 10ms of sampling time

int readFile; // read the file position, ATTENTION: Output is decoded as integer in ASCII-format!

float currentPressure[2]; // intermediate pressure values, with buffer of last value
float startPressure = 0;  // intermediate pressure values

volatile bool flagInterrupt = false; // emergency flag from the interrupt, true if Interrupt was just called
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
unsigned long currentMillis = 0;

volatile bool measurementRunning = false;
volatile bool measurementJustStarted = false;

float relativePressure = 0;    // relative pressure value
float calibrationPressure = 0; // relative pressure value

int SampleCount = 0; // counter for the samples

#endif