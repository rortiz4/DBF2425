#ifndef QUEUES_H
#define QUEUES_H
#include <Arduino.h>

// Defining containers for data
struct IMU_Data {
    unsigned int sensor_id;
    float acceleration[3]; // x,y,z
    float gyro[3]; // x,y,z
    float magnetic[3]; // x,y,z
    float rotation[4]; // Quaternion - real, i, j, k
};

struct Airspeed_Data {
    unsigned int sensor_id;
    float diff_pressure[2]; // raw, corrected
    float airspeed[2]; // raw, corrected
    float temperature;
};

struct GPS_Data {
    unsigned int sensor_id;
    float latitude;
    float longitude;
    float gnd_speed; // knots
    float altitude;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t hundredths;
    uint8_t satellites;
};

// This just lets the compiler know that this queue is declared elsewhere so other files see it when including this header
extern QueueHandle_t IMU_Queue;
extern QueueHandle_t Airspeed_Queue;
extern QueueHandle_t GPS_Queue;

void init_queues();


#endif