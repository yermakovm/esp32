#include <Adafruit_NeoPixel.h>
#include <Arduino.h>

// Define which pin the data line of your RGB LED is connected to
// Change to 48 if you are using the standard ESP32-S3 DevKit onboard LED
#define PIN 48
#define NUMPIXELS 1 // Number of LEDs in your strip/matrix

Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

void cycleLedRgb(int &red, int &green, int &blue) {
  if (red == 255 && green < 255 && blue == 0) {
    green++;
  } else if (green == 255 && red > 0 && blue == 0) {
    red--;
  } else if (green == 255 && blue < 255 && red == 0) {
    blue++;
  } else if (blue == 255 && green > 0 && red == 0) {
    green--;
  } else if (blue == 255 && red < 255 && green == 0) {
    red++;
  } else if (red == 255 && blue > 0 && green == 0) {
    blue--;
  }
}

void doRainbow() {
  int red = 255;
  int green = 0;
  int blue = 0;
  for (int i = 0; i < 256 * 6; i++) {
    cycleLedRgb(red, green, blue);
    pixels.setPixelColor(0, pixels.Color(red, green, blue));
    pixels.show();
    delay(2);
  }
}

void setup() {
  pixels.begin();
  pixels.setBrightness(50); // Set global brightness (0 to 255)
  pixels.show();
}

void loop() {
  doRainbow();
  delay(100);
}
