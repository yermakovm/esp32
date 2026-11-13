#include <Arduino.h>

#include "blink_config.h"
#include "led.h"

// Minimal file-scope state: Arduino gives setup()/loop() no owner for it.
static Led led(BlinkConfig::kLedPin, BlinkConfig::kInitialState);
static uint32_t lastBlinkMs = 0;

void setup() { led.init(); }

void loop() {
  const uint32_t now = millis();
  if (now - lastBlinkMs >= BlinkConfig::kBlinkPeriodMs) {
    lastBlinkMs = now;
    led.toggle();
  }
}
