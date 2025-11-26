#include <FastLED.h>

#define LED_PIN     7
#define NUM_LEDS    20

CRGB leds[NUM_LEDS];

void setup() {
  FastLED.addLeds<WS2812, LED_PIN, GRB>(leds, NUM_LEDS);
  Serial.begin(9600); // Start serial communication
}

void loop() {
  if (Serial.available()) {
    String input = Serial.readStringUntil('\n');
    input.trim();
    int idx1 = input.indexOf(',');
    int idx2 = input.indexOf(',', idx1 + 1);
    int idx3 = input.indexOf(',', idx2 + 1);
    if (idx1 > 0 && idx2 > idx1 && idx3 > idx2) {
      int ledIndex = input.substring(0, idx1).toInt();
      int r = input.substring(idx1 + 1, idx2).toInt();
      int g = input.substring(idx2 + 1, idx3).toInt();
      int b = input.substring(idx3 + 1).toInt();
      if (ledIndex >= 0 && ledIndex < NUM_LEDS) {
        leds[ledIndex] = CRGB(r, g, b);
        FastLED.show();
      }
    }
  }
} 