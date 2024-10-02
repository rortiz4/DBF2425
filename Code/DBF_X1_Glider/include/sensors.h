#ifndef SENSORS_H
#define SENSORS_H

void init_low_level_hw();
bool init_bno085(); // Unused in main
bool init_ms4525do(); // Unused in main
void init_all_sensors();
void read_bno085(void* pvParameters);
void read_ms4525do(void* pvParameters);
void read_gps(void* pvParameters);

#endif