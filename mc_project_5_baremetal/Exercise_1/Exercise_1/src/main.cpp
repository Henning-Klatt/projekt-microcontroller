#include <Arduino.h>
#include <Adafruit_TinyUSB.h>

#include "gpiolib.h"

// Arduino IDE: Initialization
void setup()
{

  Serial.begin(115200);
  // Define P0.06 as an output
  GPIO_set_to_Output(0, 6);
  // Define P0.06 output to be logic-’1’

  GPIO_set_to_Input(0, 29);
  // pinMode(4, INPUT_PULLDOWN); // P0.29 = 4
  // pinMode(4, INPUT_PULLUP); // P0.29 = 4
  GPIO_set_Pullup(0, 29);
}

void loop()
{
  bool state = !GPIO_get_Input(0, 29);
  // Serial.println(state);
  GPIO_set_Output(0, 06, state);
  // delay(100);
}