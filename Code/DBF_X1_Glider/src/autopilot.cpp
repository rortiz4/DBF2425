#include "autopilot.h"
#include "datalogger.h"
#include "queues.h"
#include "pitcheron_servos.h"

void Autopilot_MASTER(void* pvParameters) {
    Flight_Data sensor_data;
    Autopilot_Data AP_log_data;
    while(true) {
        xQueueReceive(Flight_Data_Queue, &sensor_data, portMAX_DELAY);
        
        

        
        
        xQueueSend(Autopilot_Queue, &AP_log_data, portMAX_DELAY);
    }
}

void Autopilot_HDG_SEL_IMU(float yaw, float bearing_change, float max_target_dev) {
    
}

void Autopilot_OP_DES(float airspeed, float target_airspeed, float max_target_dev) {

}

void Autopilot_HDG_SEL_GPS(float current_heading, float current_lat, float current_long, float target_lat, float target_long, float max_target_dev) {

}

void Autopilot_PITCH(float pitch, float target_pitch, float max_target_dev) {

}