#include <Arduino.h>
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"
#include "sensors.h"
#include "datalogger.h"
#include "pitcheron_servos.h"
#include "trim_servos.h"

#define SERIAL_LOG true // Log Data to Serial
#define SD_LOG false // Log Data to SD Card file
#define TRIM_SERVOS true // Choose whether to run this program in regular or servo trimming mode
#define SERVO_ACTUATION_TESTS true // Perform pitcheron servo tests during initialization? (ignored if TRIM_SERVOS=true)

void setup() {
    delay(3000);
    init_low_level_hw();
    init_queues();
    init_semaphores();
    init_all_sensors();
    
    if (TRIM_SERVOS == false) init_servos(SERVO_ACTUATION_TESTS);
    else trim_servos(); // Note: this instruction is blocking. No further lines of code in this file will execute and RTOS Scheduler never starts.
    
    init_SD(SERIAL_LOG, SD_LOG); // FORMAT SD CARD TO FAT32 BEFORE FIRST USE
    init_tasks();
    Serial.println("All Systems ONLINE! All Tasks Started Successfully! RTOS Task Scheduler RUNNING!\n");
}

void loop() {
    ;
}

