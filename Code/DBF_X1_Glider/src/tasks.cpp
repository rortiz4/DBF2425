#include <Arduino.h>
#include "tasks.h"
#include "sensors.h"
#include "datalogger.h"

#define COMMON_STACK_SIZE 4096 // bytes. All 4 tasks work with 3072, but not 2048, so 4096 chosen to give enough margin.
#define CPU0 0
#define CPU1 1

TaskHandle_t read_imu_task = NULL;
TaskHandle_t read_pitot_task = NULL;
TaskHandle_t read_gps_task = NULL;
TaskHandle_t log_data_task = NULL; // Currently Unused handle

void init_tasks() {
    // 4 Tasks in total: read each of the three sensors and log the data
    xTaskCreatePinnedToCore(
        log_data,
        "Task to log Data to SD Card",
        COMMON_STACK_SIZE,
        NULL,
        1,
        &log_data_task,
        CPU1 // CPU 1 - Logging can happen independently of data collection to speed things up (separate processor)
    );

    Serial.println("SD Logging Task Started");

    xTaskCreatePinnedToCore(
        read_gps,
        "Task to read GPS Data",
        COMMON_STACK_SIZE,
        NULL,
        1,
        &read_gps_task,
        CPU0 // CPU 0 (All sensors on same core since MUTEX needed for I2C bus anyway, also same priority.)
    );

    Serial.println("GPS Data Logging Task Started");

    xTaskCreatePinnedToCore(
        read_bno085,
        "Task to read IMU Data",
        COMMON_STACK_SIZE,
        NULL,
        1,
        &read_imu_task,
        CPU0
    );

    Serial.println("IMU Data Logging Task Started");

    xTaskCreatePinnedToCore(
        read_ms4525do,
        "Task to read Pitot Tube (Airspeed) Data",
        COMMON_STACK_SIZE,
        NULL,
        1,
        &read_pitot_task,
        CPU0
    );

    Serial.println("Pitot Tube Reading Task Started");
}