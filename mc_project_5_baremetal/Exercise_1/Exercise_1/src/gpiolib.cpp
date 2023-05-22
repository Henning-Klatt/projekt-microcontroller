#include "gpiolib.h"

// Make a PIN an output
void GPIO_set_to_Output(uint8_t port, uint8_t pin_number)
{
    uint32_t *tmp = (uint32_t *)((port == 1 ? P1_BASE_ADDRESS : P0_BASE_ADDRESS) +
                                 DIRSET_OFFSET);
    *tmp = (1 << pin_number);
}
// Set the value of an output PIN
void GPIO_set_Output(uint8_t port, uint8_t pin_number, bool value)
{
    uint32_t *tmp = (uint32_t *)((port == 1 ? P1_BASE_ADDRESS : P0_BASE_ADDRESS) + (value ? OUTSET_OFFSET : OUTCLR_OFFSET));
    *tmp = (1 << pin_number);
}
// Make a PIN an input
void GPIO_set_to_Input(uint8_t port, uint8_t pin_number)
{
    uint32_t *tmp = (uint32_t *)((port == 1 ? P1_BASE_ADDRESS : P0_BASE_ADDRESS) + DIRCLR_OFFSET);
    *tmp = (1 << pin_number); // zero has no effect?
}

void GPIO_set_Pullup(uint8_t port, uint8_t pin_number)
{
    uint32_t *tmp = (uint32_t *)((port == 1 ? P1_BASE_ADDRESS : P0_BASE_ADDRESS) + 0x700 + (pin_number * 0x4));
    *tmp = 12; // (1 << pin_number);
}

bool GPIO_get_Input(uint8_t port, uint8_t pin_number)
{
    uint32_t *tmp = (uint32_t *)((port == 1 ? P1_BASE_ADDRESS : P0_BASE_ADDRESS) + IN_OFFSET);
    // return (((1 << pin_number) & *tmp) > 0);
    return (((*tmp) & (1 << pin_number)) > 0);
}

#define OUTPUT_MIN (10)
#define OUTPUT_MAX (90)
#define P_MAX (25) // in psi
#define P_MIN (0)

#define PSI_to_HPA (68.947572932) ///< Constant: PSI to HPA conversion factor

// binary = raw psi
// To Do!
double MPRLS_binary_to_pressure(uint32_t binary)
{
    float pressure = ((((binary - OUTPUT_MIN) * (P_MAX - P_MIN)) / OUTPUT_MAX - OUTPUT_MIN) + 0) * PSI_to_HPA;
    return pressure;
}