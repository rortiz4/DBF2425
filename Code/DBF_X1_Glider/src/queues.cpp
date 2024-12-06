#include "queues.h"
#include "sensors.h"
#include "pitcheron_servos.h"
#include "autopilot.h"
#include "datalogger.h"

// Declare queues (allocate space) for externs in queues.h
QueueHandle_t IMU_Queue = NULL;
QueueHandle_t Airspeed_Queue = NULL;
QueueHandle_t GPS_Queue = NULL;
QueueHandle_t Autopilot_Queue = NULL;
QueueHandle_t Pitcheron_Queue = NULL;

QueueHandle_t Flight_Data_Queue = NULL; // internal (not datalogged)

void init_queues() {
    // 1-element queues containing structs defined in queues.h
    IMU_Queue = xQueueCreate(1, sizeof(IMU_Data));
    Airspeed_Queue = xQueueCreate(1, sizeof(Airspeed_Data));
    GPS_Queue = xQueueCreate(1, sizeof(GPS_Data));
    Autopilot_Queue = xQueueCreate(1, sizeof(Autopilot_Data));
    Pitcheron_Queue = xQueueCreate(1, sizeof(Pitcheron_Data));

    Flight_Data_Queue = xQueueCreate(1, sizeof(Flight_Data));
}