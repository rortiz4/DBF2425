#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "strobe.h"
#include "pin_map.h"

#define BLINK_ON_TIME_ms 200
#define BLINK_OFF_TIME_ms 200
#define BLINK_RESET_TIME_ms 800

#define TEST_DELAY_ms 1000
#define LED_BRIGHTNESS_PERCENT 100 // %
#define NUM_LEDS 1

Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, STROBE_LED_PIN, NEO_GRB + NEO_KHZ800);

void init_strobe() {
    leds.begin();
    leds.setBrightness((uint8_t)((LED_BRIGHTNESS_PERCENT/100.0)*255));
    for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
    leds.show();
    Serial.println("Testing LED. Confirm it lights up!");
    delay(TEST_DELAY_ms);
    for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(255, 0, 255));
    leds.show();
    delay(TEST_DELAY_ms);
    for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
    leds.show();
    Serial.println("Strobe LED Initialized.");
}

void blink_strobe(void* pvParameters) {
    while(true) {
        //Serial.println("Strobe Task");
        // Pattern: BlinkBlink......BlinkBlink......
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(255, 0, 0));
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 255, 0));
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
        leds.show();
        vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));
        
        vTaskDelay(pdMS_TO_TICKS(BLINK_RESET_TIME_ms));
    }
}