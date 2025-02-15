#ifndef AUTOPILOT_H
#define AUTOPILOT_H

// This struct will be used by datalogger
struct Autopilot_Data {
    unsigned int sensor_id; // 4
    const char* flight_phase;
    const char* ap_mode;
    float ap_target_bearing;
    float ap_target_roll;
    float ap_target_pitch;
};

enum AP_Modes {
    AP_OFF, // 0 (Almost never used except on ground)
    AP_HDG_SEL_IMU, // 1
    AP_HDG_SEL_GPS, // 2...
    AP_SPD_TRIM,
    AP_PITCH_FIXED,
    AP_ROLL_FIXED,
    AP_ENVELOPE_PROT, // Flight envelope protection modes below (AP_ENVELOPE_PROT specifically not logged. See below for logged modes.)
    AP_PROT_ROLL_MIN,
    AP_PROT_ROLL_MAX,
    AP_PROT_PITCH_MIN,
    AP_PROT_PITCH_MAX,
    AP_PROT_STALL,
    AP_PROT_OVERSPEED
};

enum Flight_Phases {
    // Must always alternate between a Roll Mode and a Pitch Mode.
    U_TURN_HDG, // Roll Mode: 180 degree turn after release using IMU only 
    GPS_HOMING, // TBD Roll Mode: Continuous update of target bearing + bearing correction from current GPS coordinates + COG/heading. Target coordinates hard-coded. Same logic as U_TURN_HDG flight phase after that.
    SPD_DESCENT, // Pitch Mode: Primary mode after U-Turn. Will alternate with U_Turn_HDG/GPS_HOMING as needed (common to both roll modes).
    LANDED // Used to return pitcherons to neutral position and turn off autopilot after landing.
};

void Autopilot_MASTER(void* pvParameters); // Calls on each of the below functions to perform manoeuvres as required. Maintains flight envelope protections too.
bool Autopilot_HDG_SEL_IMU(float roll, float yaw, float bearing_change, Autopilot_Data& AP_log_data); // Current bearing from IMU "Yaw" Angle. Servo actuation done to turn by bearing_change (e.g. 180degree turn)
bool Autopilot_SPD_TRIM(float airspeed, float pitch, float target_airspeed, Autopilot_Data& AP_log_data); // Controls pitch to maintain airspeed within +/-max_dev of target.
bool Autopilot_HDG_SEL_GPS(float roll, float current_heading, float current_lat, float current_long, float target_lat, float target_long, Autopilot_Data& AP_log_data); // Current bearing from GPS+current+target coordinates (used to calculate target bearing for turn). Used after 180 degree turn for homing.
bool Autopilot_FLT_ENVELOPE_PROT(float roll, float pitch, float airspeed, Autopilot_Data& AP_log_data);

// Unused in AP_MASTER
bool Autopilot_ROLL_FIXED(float roll, float target_roll, Autopilot_Data& AP_log_data); // Will probably go unused, but included for completeness (direct roll control by angle.
bool Autopilot_PITCH_FIXED(float pitch, float target_pitch, Autopilot_Data& AP_log_data); // Will probably go unused, but included for completeness (direct pitch control by angle. Dangerous due to high stall potential! Good for levelling off.)

// Note: max_dev is allowed tolerance in each case for bearing/airspeed/pitch from target

#endif