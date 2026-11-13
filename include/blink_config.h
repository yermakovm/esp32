#pragma once

#include <cstdint>

#include "led.h"

// Only compile-time values here: no magic numbers in the code below.
struct BlinkConfig {
  static constexpr uint8_t kLedPin = 2;           // single LED
  static constexpr uint32_t kBlinkPeriodMs = 500; // ms between state changes
  static constexpr LedState kInitialState = LedState::Off;
};
