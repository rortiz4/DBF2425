#ifndef TASKS_H
#define TASKS_H
#include <Arduino.h>

extern TaskHandle_t read_imu_task;
extern TaskHandle_t read_pitot_task;
extern TaskHandle_t read_gps_task;
extern TaskHandle_t log_data_task; // Currently unused handle

void init_tasks();

#endif