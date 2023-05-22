#include <Arduino.h>

// Definitions, cf. Datasheet
#define P0_BASE_ADDRESS 0x50000000
#define P1_BASE_ADDRESS 0x50000300
#define DIRSET_OFFSET 0x518
#define DIRCLR_OFFSET 0x51C
#define OUTSET_OFFSET 0x508 // HIGH
#define OUTCLR_OFFSET 0x50C // LOW
#define IN_OFFSET 0x510

void GPIO_set_to_Output(uint8_t port, uint8_t pin_number);
void GPIO_set_Output(uint8_t port, uint8_t pin_number, bool value);
void GPIO_set_to_Input(uint8_t port, uint8_t pin_number);
bool GPIO_get_Input(uint8_t port, uint8_t pin_number);
void GPIO_set_Pullup(uint8_t port, uint8_t pin_number);
double MPRLS_binary_to_pressure(uint32_t binary);