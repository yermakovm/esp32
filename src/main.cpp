#include <Arduino.h>

#define YELLOW_LED_PIN 2
#define RED_LED_PIN 35
#define GREEN_LED_PIN 38

static const int ledPins[] = {YELLOW_LED_PIN, RED_LED_PIN, GREEN_LED_PIN};
static const int blinkDelays[] = {
    500, 200, 1000}; // Delay times for each LED in milliseconds
static int lastBlinkTimes[] = {0, 0,
                               0}; // Store the last blink time for each LED

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  pinMode(YELLOW_LED_PIN, OUTPUT);
  pinMode(RED_LED_PIN, OUTPUT);
  pinMode(GREEN_LED_PIN, OUTPUT);
}

void loop() {
  unsigned long currentMillis = millis();

  const int numLeds = sizeof(ledPins) / sizeof(ledPins[0]);
  for (int i = 0; i < numLeds; i++) {
    if (currentMillis - lastBlinkTimes[i] >= blinkDelays[i]) {
      // Toggle the LED state
      digitalWrite(ledPins[i], !digitalRead(ledPins[i]));
      lastBlinkTimes[i] = currentMillis; // Update the last blink time
    }
  }
}
