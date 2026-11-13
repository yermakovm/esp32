#pragma once

#include <Arduino.h>

#include <cstdint>

enum class LedState : uint8_t { Off, On };

class Led {
public:
  Led(uint8_t pin, LedState initialState = LedState::Off)
      : pin_(pin), state_(initialState) {}

  void init() {
    pinMode(pin_, OUTPUT);
    set(state_);
  }

  void set(LedState state) {
    state_ = state;
    digitalWrite(pin_, state == LedState::On ? HIGH : LOW);
  }

  void toggle() { set(state_ == LedState::On ? LedState::Off : LedState::On); }

private:
  uint8_t pin_;
  LedState state_;
};
