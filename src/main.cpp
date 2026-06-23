#include "blinky.h"
#include <Arduino.h>

void setup() {
  blinky_init(); // Initialize the LED pin
}

void loop() {
  blinky_on();  // Turn the LED on
  delay(1000);  // Wait for 1 second
  blinky_off(); // Turn the LED off
  delay(1000);  // Wait for 1 second
}
