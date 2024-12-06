#ifndef DATALOGGER_H
#define DATALOGGER_H

extern float current_time;
extern bool serial_log;
extern bool SD_log;

// This struct will be used in Autopilot
struct Flight_Data {
    float time; // will probably not be used
    float pitch;
    float roll;
    float yaw;
    float latitude;
    float longitude;
    float heading;
    float airspeed;
    float gnd_speed; // could be used for wind corrections later
};

void init_SD(bool serial_log, bool SD_log);
void log_data(void* pvParameters);

#endif