#ifndef QUEUES_H
#define QUEUES_H
#include <Arduino.h>

// This just lets the compiler know that this queue is declared elsewhere so other files see it when including this header
extern QueueHandle_t IMU_Queue;
extern QueueHandle_t Airspeed_Queue;
extern QueueHandle_t GPS_Queue;
extern QueueHandle_t Pitcheron_Queue;

void init_queues();


#endif