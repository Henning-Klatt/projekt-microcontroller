#ifndef EVALUATE_H
#define EVALUATE_H

#include <Arduino.h>
#include <math.h>
#include "Adafruit_SPIFlash.h"

#include "io.h"

#define startButton 12 //*put your pin here*
#define peakTh 100     // threshold if peak is detected
#define peakFilter 0.3 // factor of minimum peak pressure

File32 myFile_read;

// int readFile; // read the file position, ATTENTION: Output is decoded as integer in ASCII-format!

String printData; // print data to console as string
// int16_t HPmeasSample[12000]; // buffer for the highpass filtered samples from the flash memory
int16_t HPsample;        // intermediate buffer value from HPmeasSample
int16_t peakMeas[360];   // buffer for the detected peak, maximum 2 minutes with heart rate of 180/min
int tPeakMeas[360];      // timestamp of the detected peak
int16_t peakMeasTh[360]; // buffer for the detected peak, after threshold detection
int tPeakMeasTh[360];    // timestamp of the buffer
int indexSweep = 0;      // index sweep to write in the buffer
bool flagHeart = false;  // flag if heartbeat is detected
int peakMax = 0;         // intermediate maximum value of the peak
int peakMin = 0;         // intermediate minimum value of the peak
int absMax = 0;          // absolut maximum of the peaks
int indexAbsMax = 0;     // index of the maximum peak
float heartRate = 0;     // final heart rate

float findHeartRate();
void readFlash(FatVolume fatfs);

#endif