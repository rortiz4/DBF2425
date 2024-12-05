#include <Arduino.h>
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"
#include "sensors.h"
#include "datalogger.h"
#include "pitcherons_servos.h"

#define SERIAL_LOG true // Log Data to Serial
#define SD_LOG false // Log Data to SD Card file
#define SERVO_ACTUATION_TESTS true // Perform pitcheron servo tests during initialization? 

void setup() {
    delay(3000);
    init_low_level_hw();
    init_queues();
    init_semaphores();
    init_all_sensors();
    init_servos(SERVO_ACTUATION_TESTS);
    init_SD(SERIAL_LOG, SD_LOG); // FORMAT SD CARD TO FAT32 BEFORE FIRST USE
    init_tasks();
    Serial.println("All Systems ONLINE! All Tasks Started Successfully! RTOS Task Scheduler RUNNING!\n");
    /*
    Sample Rate Estimates:
    - IMU Alone: 15Hz
    - Airspeed Sensor Alone: 145Hz
    - All Sensors Together: 10Hz originally (excluding GPS)
    - After Optimizations: 15.34Hz rate (excluding GPS, 5 min. average lines/second)
    */
    // vTaskStartScheduler();
}

void loop() {
    ;
}

