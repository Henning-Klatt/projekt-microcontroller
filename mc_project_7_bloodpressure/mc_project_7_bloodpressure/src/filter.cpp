#include "filter.h"

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