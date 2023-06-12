#include <Arduino.h>

#include "io.h"

void setupIO()
{
    pinMode(startButton, INPUT_PULLUP);
    pinMode(interruptButton, INPUT_PULLUP);
    pinMode(PUMP, OUTPUT);
    digitalWrite(PUMP, LOW);
    pinMode(VALVE, OUTPUT);
    digitalWrite(VALVE, LOW);
    pinMode(LED_BTN_GREEN, OUTPUT);
    digitalWrite(LED_BTN_GREEN, LOW);
    pinMode(LED_BTN_RED, OUTPUT);
    digitalWrite(LED_BTN_RED, LOW);
}