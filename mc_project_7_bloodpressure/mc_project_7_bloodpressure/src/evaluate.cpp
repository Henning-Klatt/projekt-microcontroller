#include "evaluate.h"
#include "measure.h"

float findHeartRate()
{
    flagHeart = false;
    absMax = 0;
    indexAbsMax = 0;
    peakMax = 0;
    peakMin = 0;
    indexSweep = 0;
    heartRate = 0;
    memset(peakMeas, 0, sizeof(peakMeas));
    memset(tPeakMeas, 0, sizeof(tPeakMeas));
    Serial.println("Searching for the peaks in the measurement data ... ");
    /*put your code here*/
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
    delay(10);
    return heartRate;
}

void readFlash(FatVolume fatfs)
{
    myFile_read = fatfs.open("HPpressureMeasurement.txt", FILE_READ);
    if (myFile_read)
    {
        Serial.print("Read HPpressureMeasurement.txt...");
        // read from the file until there’s nothing else in it:
        while (myFile_read.available())
        {
            readFile = myFile_read.read();

            if ((char)readFile == ',')
            {
                HPmeasSample[indexSweep] = printData.toInt();
                indexSweep += 1;
                printData = "";
            }
            else
            {
                printData += (char)readFile;
            }
        }
        myFile_read.close();
    }
    else
    {
        Serial.println("error opening HPpressureMeasurement.txt");
    }
    indexSweep = 0;
}