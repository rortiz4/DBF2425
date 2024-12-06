#include "semaphores.h"

SemaphoreHandle_t I2C_MUTEX = NULL;
// SemaphoreHandle_t imu_done = NULL;
// SemaphoreHandle_t airspeed_done = NULL;
// SemaphoreHandle_t gps_done = NULL;

// Note: Mutex is a type of semaphore but with task ownership
void init_semaphores() {
    // Mutex
    I2C_MUTEX = xSemaphoreCreateMutex();

    // Binary Semphores
    // imu_done = xSemaphoreCreateBinary();
    // airspeed_done = xSemaphoreCreateBinary();
    // gps_done = xSemaphoreCreateBinary();
}