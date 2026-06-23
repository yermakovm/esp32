#include "unity.h"
#include "blinky.h"
#include <Arduino.h>

void setUp(void) {
}

void tearDown(void) {
}

void test_blinky(void) {
blinky_init();
  blinky_on();
  TEST_ASSERT_EQUAL(HIGH, digitalRead(LED_PIN));
  blinky_off();
  TEST_ASSERT_EQUAL(LOW, digitalRead(LED_PIN));	
}


void runUnityTests(void) {
  UNITY_BEGIN();
  RUN_TEST(test_blinky);
  UNITY_END();
}

/**
  * For Arduino framework
  */
void setup() {
  delay(1000);

  runUnityTests();
}
void loop() {}
