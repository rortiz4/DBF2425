#ifndef AUTOPILOT_H
#define AUTOPILOT_H

// This struct will be used by datalogger
struct Autopilot_Data {
    unsigned int sensor_id; // 4
    const char* ap_mode;
    float ap_target_bearing;
    float ap_target_pitch;
};

void Autopilot_MASTER(void* pvParameters); // Calls on each of the below functions to perform manoeuvres as required. Maintains flight envelope protections too.
void Autopilot_HDG_SEL_IMU(float yaw, float bearing_change, float max_target_dev); // Current bearing from IMU "Yaw" Angle. Servo actuation done to turn by bearing_change (e.g. 180degree turn)
void Autopilot_OP_DES(float airspeed, float target_airspeed, float max_target_dev); // Controls pitch to maintain airspeed within +/-max_dev of target
void Autopilot_HDG_SEL_GPS(float current_heading, float current_lat, float current_long, float target_lat, float target_long, float max_target_dev); // Current bearing from GPS+current+target coordinates (used to calculate target bearing for turn). Used after 180 degree turn for homing.
void Autopilot_PITCH(float pitch, float target_pitch, float max_target_dev); // Will probably go unused, but included for completeness (direct pitch control by angle. Dangerous due to high stall potential! Good for levelling off.)
// Note: max_dev is allowed tolerance in each case for bearing/airspeed/pitch from target

#endif