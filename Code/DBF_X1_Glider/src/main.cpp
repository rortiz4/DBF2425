#include <Arduino.h>
//#include "soc/soc.h"
//#include "soc/rtc_cntl_reg.h"
#include "esp_sleep.h"
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"
#include "sensors.h"
#include "datalogger.h"
#include "pitcheron_servos.h"
#include "trim_servos.h"
#include "strobe.h"
#include "autopilot.h"
#include "pin_map.h"

#define SERIAL_LOG true // Log Data to Serial
#define SD_LOG true // Log Data to SD Card file (failsafe for SD card popping out if true is included. If that happens, true constant is ignored.)
#define TRIM_SERVOS false // Choose whether to run this program in regular or servo trimming mode
#define SERVO_ACTUATION_TESTS true // Perform pitcheron servo tests during initialization? (ignored if TRIM_SERVOS=true)
#define RELEASE_INIT true // Wait for release before running main code
#define BOOTUP_DELAY 2000 //ms
#define INSTALL_DEBOUNCE_DELAY 250 //ms
#define INSTALL_DELAY 10000
#define RELEASE_DELAY 250 //ms

void setup() {
    delay(BOOTUP_DELAY);
    //WRITE_PERI_REG(RTC_CNTL_BROWN_OUT_REG, 0);
    pinMode(RELEASE_DET_PIN, INPUT);
    esp_sleep_enable_ext0_wakeup((gpio_num_t)RELEASE_DET_PIN, HIGH); 
    init_low_level_hw();
    init_strobe();
    init_queues();
    init_semaphores();
    init_all_sensors();
    
    if (TRIM_SERVOS == false) init_servos(SERVO_ACTUATION_TESTS);
    else trim_servos(); // Note: this instruction is blocking. No further lines of code in this file will execute and RTOS Scheduler never starts.

    init_SD(SERIAL_LOG, SD_LOG); // FORMAT SD CARD TO FAT32 BEFORE FIRST USE
    
    if (RELEASE_INIT) {
        Serial.println("All Systems Initialized. Waiting for GPIO 19 release detection (HIGH=>LOW=>HIGH)...");
        digitalWrite(BUILTIN_LED_PIN, LOW);
        while (true) {
            while (true) {
                if (digitalRead(RELEASE_DET_PIN) == HIGH) delay(INSTALL_DEBOUNCE_DELAY);
                else {
                    delay(INSTALL_DEBOUNCE_DELAY);
                    if (digitalRead(RELEASE_DET_PIN) == LOW) break;
                }
            }

            digitalWrite(BUILTIN_LED_PIN, HIGH);
            delay(INSTALL_DELAY);
            if (digitalRead(RELEASE_DET_PIN) == HIGH) {
                digitalWrite(BUILTIN_LED_PIN, LOW);
                delay(250);
                digitalWrite(BUILTIN_LED_PIN, HIGH);
                continue;
            }
            else break;
        }
        digitalWrite(BUILTIN_LED_PIN, LOW);

        while (digitalRead(RELEASE_DET_PIN) == LOW) {
            esp_light_sleep_start();
            delay(RELEASE_DELAY); // for debouncing release detection magnet
        }
    }
    init_tasks();
    Serial.println("All Systems ONLINE! All Tasks Started Successfully! RTOS Task Scheduler RUNNING!\n");

}

void loop() {
    ;//vTaskStartScheduler();
}

