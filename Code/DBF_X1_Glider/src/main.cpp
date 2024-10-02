#include <Arduino.h>
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"
#include "sensors.h"
#include "datalogger.h"

void setup() {
    init_low_level_hw();
    init_queues();
    init_semaphores();
    // init_SD();
    init_all_sensors();
    init_tasks();
    Serial.println("All Tasks Started Successfully. RTOS Task Scheduler Running!\n");
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

