#include <Arduino.h>

enum class LedState : uint8_t { Off, On };

// Only compile-time values here: no magic numbers in the code below.
struct BlinkConfig {
  static constexpr uint8_t kLedPin = 2;           // single LED
  static constexpr uint32_t kBlinkPeriodMs = 500; // ms between state changes
  static constexpr LedState kInitialState = LedState::Off;
};

class Led {
public:
  explicit Led(uint8_t pin) : pin_(pin), state_(BlinkConfig::kInitialState) {}

  void init() {
    pinMode(pin_, OUTPUT);
    set(BlinkConfig::kInitialState);
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

// Minimal file-scope state: Arduino gives setup()/loop() no owner for it.
static Led led(BlinkConfig::kLedPin);
static uint32_t lastBlinkMs = 0;

void setup() { led.init(); }

void loop() {
  const uint32_t now = millis();
  if (now - lastBlinkMs >= BlinkConfig::kBlinkPeriodMs) {
    lastBlinkMs = now;
    led.toggle();
  }
}
