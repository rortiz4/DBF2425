#ifndef SEMAPHORES_H
#define SEMAPHORES_H
#include <Arduino.h>

extern SemaphoreHandle_t I2C_MUTEX;

extern SemaphoreHandle_t imu_done;
extern SemaphoreHandle_t airspeed_done;
extern SemaphoreHandle_t gps_done;

void init_semaphores();

#endif