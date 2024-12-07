#ifndef SENSORS_H
#define SENSORS_H

// Defining containers for data
struct IMU_Data {
    unsigned int sensor_id;
    float lin_accel[3]; // x,y,z
    float euler[3];
    float gyro[3]; // x,y,z
    float magnetic[3]; // x,y,z
    float gravity[3]; // x,y,z
    float rotation[4]; // Quaternion - real, i, j, k
};

struct Airspeed_Data {
    unsigned int sensor_id;
    float diff_pressure;
    float airspeed[2]; // raw, corrected
    float temperature;
};

struct GPS_Data {
    unsigned int sensor_id;
    float latitude;
    float longitude;
    float gnd_speed; // knots
    float altitude;
    float heading;
    uint8_t hours;
    uint8_t minutes;
    uint8_t seconds;
    uint8_t hundredths;
    uint8_t satellites;
};

void init_low_level_hw();
bool init_bno085(); // Unused in main
bool init_abp2(); // Unused in main
void init_all_sensors();
void read_bno085(void* pvParameters);
void read_abp2(void* pvParameters);
void read_gps(void* pvParameters);

#endif