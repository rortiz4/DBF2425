#include <Arduino.h>
#include "Adafruit_NeoPixel.h"
#include "strobe.h"
#include "pin_map.h"

#define NOT_NEOPIXEL false
#define BLINK_ON_TIME_ms 200
#define BLINK_OFF_TIME_ms 200
#define BLINK_RESET_TIME_ms 0

#define TEST_DELAY_ms 1000
#define LED_BRIGHTNESS_PERCENT 100 // %
#define NUM_LEDS 3

Adafruit_NeoPixel leds = Adafruit_NeoPixel(NUM_LEDS, STROBE_LED_PIN, NEO_GRB + NEO_KHZ800);

void init_strobe() {
    if (NOT_NEOPIXEL) {
        // For Regular LEDs
        pinMode(STROBE_LED_PIN, OUTPUT);
        digitalWrite(STROBE_LED_PIN, HIGH);
        Serial.println("Testing LED. Confirm it lights up!");
        delay(TEST_DELAY_ms);
        //while(true);
        digitalWrite(STROBE_LED_PIN, LOW);
        Serial.println("Strobe LED Initialized (ensure OFF).");
    }
    else {
        // For Addressable LEDs
        leds.begin();
        leds.setBrightness((uint8_t)((LED_BRIGHTNESS_PERCENT/100.0)*255));
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
        leds.show();
        if (NUM_LEDS == 1) Serial.println("Testing LED. Confirm it lights up!");
        else Serial.printf("Testing LEDs. Confirm all %d light up!\n", NUM_LEDS);
        delay(TEST_DELAY_ms);
        //while(true);
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(179, 255, 0));
        leds.show();
        delay(TEST_DELAY_ms);
        for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
        leds.show();
        Serial.println("Strobe LEDs Initialized.");
    }
}

void blink_strobe(void* pvParameters) {
    while(true) {
        // Serial.println("Strobe Task");
        // Pattern: BlinkBlink......BlinkBlink......
        if (NOT_NEOPIXEL) {
            // For Regular LEDs
            digitalWrite(STROBE_LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
            digitalWrite(STROBE_LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));

            digitalWrite(STROBE_LED_PIN, HIGH);
            vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
            digitalWrite(STROBE_LED_PIN, LOW);
            vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));

            vTaskDelay(pdMS_TO_TICKS(BLINK_RESET_TIME_ms));
        }
        else {
            // For Addressable LEDs
            for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(179, 255, 0));
            leds.show();
            vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
            for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
            leds.show();
            vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));
            for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(179, 255, 0));
            leds.show();
            vTaskDelay(pdMS_TO_TICKS(BLINK_ON_TIME_ms));
            for (unsigned int i = 0; i < NUM_LEDS; i++) leds.setPixelColor(i, leds.Color(0, 0, 0));
            leds.show();
            vTaskDelay(pdMS_TO_TICKS(BLINK_OFF_TIME_ms));
            
            vTaskDelay(pdMS_TO_TICKS(BLINK_RESET_TIME_ms));
        }
    }
}