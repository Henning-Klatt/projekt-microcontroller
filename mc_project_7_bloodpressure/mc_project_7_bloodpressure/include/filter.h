#ifndef FILTER_H
#define FILTER_H

#include "Arduino.h"

#define HPnominator_5Hz 0.864244751836367     // filter coefficient of the nominator of the highpass filter, with f_3dB = 5Hz
#define HPdenominator_5Hz 0.728489503672734   // filter coefficient of the denominator of the highpass filter, with f_3dB = 5Hz
#define HPnominator_0_5Hz 0.984534960996653   // filter coefficient of the nominator of the highpass filter, with f_3dB = 0.5Hz
#define HPdenominator_0_5Hz 0.969069921993306 // filter coefficient of the denominator of the highpass filter, with f_3dB = 0.5Hz

void HPfilter(float *pressure, float *HP_5Hz, float *HP_0_5Hz);
void print_array(int16_t *array, int16_t size);

#endif