#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>
#include <Wire.h>

bool init_bno085();
bool init_ms4525do();
void init_all_sensors();
void read_bno085(float* output_vector);
void read_ms4525do(float* output_vector);
void read_all_sensors(float* imu_output_vector, float* ms4525do_output_vector);

#endif