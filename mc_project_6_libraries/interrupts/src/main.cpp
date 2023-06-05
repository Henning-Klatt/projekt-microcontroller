#include <Arduino.h>
#include <Adafruit_TinyUSB.h>
#include <Wire.h>

#define LED 3
#define SWITCH 4

int count = 0;                // number of seconds
unsigned long startTimer = 0; // timer to count seconds
unsigned long endTimer = 0;
volatile bool flagStart = false; // flag to signal that an interrupt has been detected
unsigned long previousMillis = 0;

void interruptFunction()
{
  int state = !digitalRead(SWITCH);
  digitalWrite(LED, state);
  if (state)
  {
    flagStart = true;
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(SWITCH, INPUT_PULLUP);
  // Declare the pin as interrupt, interrupts can be detected also by RISING, FALLING and level
  attachInterrupt(digitalPinToInterrupt(SWITCH), interruptFunction, CHANGE);
  pinMode(LED, OUTPUT); // Turn the LED on if the button is pressed
  digitalWrite(LED, LOW);
}
void loop()
{
  startTimer = millis();
  if (startTimer - previousMillis >= 1000)
  {
    previousMillis = startTimer;
    count++;
  }

  if (flagStart)
  {
    count = 0;
    flagStart = false;
  }

  Serial.print("Time (s): ");
  Serial.println(count);
  delay(50);
}
