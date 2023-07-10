#include "filter.h"
#include "Arduino.h"

// function of the highpass filter
void HPfilter(float *pressure, float *HP_5Hz, float *HP_0_5Hz)
{
    *(HP_5Hz + 1) = *(HP_5Hz);     // Shift buffer
    *(HP_0_5Hz + 1) = *(HP_0_5Hz); // Shift buffer
    // Highpass filter in time domain
    *(HP_5Hz) = HPnominator_5Hz * (*(pressure) - *(pressure + 1)) +
                HPdenominator_5Hz * (*(HP_5Hz + 1));
    *(HP_0_5Hz) = HPnominator_0_5Hz * (*(HP_5Hz) - *(HP_5Hz + 1)) +
                  HPdenominator_0_5Hz * (*(HP_0_5Hz + 1));
}

void print_array(int16_t *array, int16_t size)
{
    for (size_t i = 0; i < size; i++)
    {
        if (array[i] != 0)
            Serial.println(array[i]);
    }
}