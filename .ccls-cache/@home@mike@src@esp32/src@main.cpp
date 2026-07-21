#include <Arduino.h>

#define PHOTORES_PIN 4

void setup() {
  Serial.begin(115200);
  pinMode(PHOTORES_PIN, INPUT);
}

void loop() {
  int photoresValue = analogRead(PHOTORES_PIN);
  double voltage = (photoresValue / 4095.0) * 3.3;
  double readVoltage = analogReadMilliVolts(PHOTORES_PIN);

  Serial.println();
  Serial.println("+----------+--------------------+-------------------+");
  Serial.println("| Raw      | Calculated (V)     | Read (V)          |");
  Serial.println("+----------+--------------------+-------------------+");
  Serial.printf("| %8d | %18.3f | %17.3f |\n", photoresValue, voltage,
                readVoltage / 1000.0);
  delay(100);
}
