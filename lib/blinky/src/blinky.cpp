#include "blinky.h"
#include <Arduino.h>

void blinky_init(void) {
  pinMode(LED_PIN, OUTPUT);
}

void blinky_on(void) {
  digitalWrite(LED_PIN, HIGH);
}

void blinky_off(void) {
  digitalWrite(LED_PIN, LOW);
}
