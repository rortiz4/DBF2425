#include <Arduino.h>
#include "strobe.h"
#include "pin_map.h"

#define BLINK_ON_TIME_ms 200
#define BLINK_OFF_TIME_ms 200
#define BLINK_RESET_TIME_ms 1000

void init_strobe() {
    pinMode(STROBE_LED_PIN, OUTPUT);
    digitalWrite(STROBE_LED_PIN, LOW);
    //Serial.println("Strobe LED Initialized.");
}

void blink_strobe(void* pvParameters) {
    // Pattern: BlinkBlink......BlinkBlinnk......
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