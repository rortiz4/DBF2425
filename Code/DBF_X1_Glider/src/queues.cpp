#include "queues.h"
#include "sensors.h"
#include "pitcheron_servos.h"

// Declare queues (allocate space) for externs in queues.h
QueueHandle_t IMU_Queue = NULL;
QueueHandle_t Airspeed_Queue = NULL;
QueueHandle_t GPS_Queue = NULL;
QueueHandle_t Pitcheron_Queue = NULL;


void init_queues() {
    // 1-element queues containing structs defined in queues.h
    IMU_Queue = xQueueCreate(1, sizeof(IMU_Data));
    Airspeed_Queue = xQueueCreate(1, sizeof(Airspeed_Data));
    GPS_Queue = xQueueCreate(1, sizeof(GPS_Data));
    Pitcheron_Queue = xQueueCreate(1, sizeof(Pitcheron_Data));
}