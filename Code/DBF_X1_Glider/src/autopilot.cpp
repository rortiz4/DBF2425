#include "autopilot.h"
#include "datalogger.h"
#include "queues.h"
#include "pitcheron_servos.h"

#define AP_ENABLE true // Set true to enable Autopilot, false to disable.
#define GND_TEST true // Disables LANDED state
#define ROLL_PROT_EN true
#define PITCH_PROT_EN true
#define STALL_PROT_EN false
#define PITCH_SPEED_CONTROL true
#define OVSPD_PROT_EN false

// Note: Convention used by autopilot: + means right/up, - means left/down. ALL ANGLES IN DEGREES AND SPEEDS IN ft/s.
// Flight Envelope Limits
#define MAX_PITCHERON_ANGLE SERVO_MAX_ALLOWED // Defined in pitcheron_servos.h
#define STALL_SPEED 30
#define OVERSPEED 60
#define ROLL_LIM_MIN -30
#define ROLL_LIM_MAX 30
#define PITCH_LIM_MIN -20
#define PITCH_LIM_MAX 20

// Autopilot Settings and Control Limits
#define U_TURN_BEARING_CHANGE -180 // deg: From DBF 2024 Competition Rules
#define BULLSEYE_LATITUDE 41.5194//32.2653 // degN: From DBF 2024 Competition Rules
#define BULLSEYE_LONGITUDE -86.2400//-111.2736 // degW: From DBF 2024 Competition Rules
#define HDG_TARGET_DEVIATION_LOW -5
#define HDG_TARGET_DEVIATION_HIGH 5
#define ROLL_TARGET_DEVIATION_LOW -5
#define ROLL_TARGET_DEVIATION_HIGH 5
#define SPD_TARGET 45 // ft/s
#define SPD_TARGET_DEVIATION_LOW -10 // -10 means STALL_SPEED = 40 ft/s with 50 ft/s target
#define SPD_TARGET_DEVIATION_HIGH 10
#define PITCH_TARGET_DEVIATION_LOW -3
#define PITCH_TARGET_DEVIATION_HIGH 3

// PID Proportionality Constants (leave margin for min/max to avoid exceeding flight envelope limits)
#define Kp_ROLL_BEARING_CORR 1// Roll proportional to amount of turning required (yaw change)
#define Kp_PITCH_SPD_CORR -1// MUST BE NEGATIVE!!! Pitch proportional to amount of speed change required (current speed error from target)
#define Kp_SERVO_ANGLE_ROLL 1// Servo Angle proportional to error in roll from target
#define Kp_SERVO_ANGLE_PITCH 1// Servo Angle proportional to error in pitch from target


// Utility Functions for bearings
// Wrap angle to range [0, 360)
float wrap_angle(float angle) {
    float wrapped = fmod(angle, 360.0); // Calculates remainder of angle/360 (wraps angle)
    return wrapped < 0 ? wrapped + 360 : wrapped; // For negative angles
}
// To add bearings: wrap_angle(angle_1+angle_2)
// To subtract bearings: wrap_angle(angle_1-angle_2)
// This function gives the shortest necessary correction for bearings
// e.g. current_bearing = 10deg, target_bearing = 350deg, returns -20deg
// Negative angles mean left turn needed. Positive angles mean right turn needed.
float signed_bearing_correction(float current_bearing, float target_bearing) {
    float brg_corr = wrap_angle(target_bearing - current_bearing);
    if (brg_corr > 180.0) {
        brg_corr -= 360.0;
    }
    return brg_corr;
}
// Function to calculate the bearing from current and target GPS coordinates
float calculate_bearing(float current_lat, float current_long, float target_lat, float target_long, float current_bearing) {
    float target_bearing = current_bearing; // Just setting default condition for variable initialization (modified later).
    // Edge case: Current latitude/longitude = Target latitude/longitude (prevents undefined target bearing)
    if (current_lat == target_lat && current_long == target_long) target_bearing = current_bearing;
    
    else {
        // Convert latitude and longitude from degrees to radians
        current_lat = current_lat * M_PI / 180.0f;
        current_long = current_long * M_PI / 180.0f;
        target_lat = target_lat * M_PI / 180.0f;
        target_long = target_long * M_PI / 180.0f;

        // Calculate the difference in longitude
        float delta_long = target_long - current_long;

        // Calculate the components of the formula
        float x = sinf(delta_long) * cosf(target_lat);
        float y = cosf(current_lat) * sinf(target_lat) - sinf(current_lat) * cosf(target_lat) * cosf(delta_long);

        // Calculate the initial bearing
        float initial_bearing = atan2f(x, y);

        // Convert bearing from radians to degrees and normalize to 0-360
        target_bearing = fmodf((initial_bearing * 180.0f / M_PI) + 360.0f, 360.0f);
    }

    return target_bearing;
}

// Main Autopilot functions
void Autopilot_MASTER(void* pvParameters) {
    Flight_Data sensor_data;
    Autopilot_Data AP_log_data;
    AP_log_data.sensor_id = 3;
    int ap_flight_phase = U_TURN_HDG;
    AP_log_data.flight_phase = "U_TURN_HDG";
    AP_log_data.ap_mode = "AP_OFF";
    bool u_turn_done = false;
    AP_log_data.ap_target_bearing = 0; // just for initialization
    AP_log_data.ap_target_roll = 0; // just for initialization
    AP_log_data.ap_target_pitch = 0; // just for initialization
    if (AP_ENABLE) Serial.println("Autopilot ON!");
    else Serial.println("Autopilot OFF. Maintaining Pitcheron Neutral Position.");
    while(true) {
        //Serial.println("Autopilot Task");
        xQueueReceive(Flight_Data_Queue, &sensor_data, portMAX_DELAY);
        // First use the received data to identify the current phase of flight and AP Mode. But we start with U-Turn immediately
        // Landed
        if (!AP_ENABLE) {
            actuate_pitcherons(0, MAINTAIN_ANGLE); // Do nothing if AP has been disabled through flag.
        }
        else if (sensor_data.airspeed == 0 && !GND_TEST) {
            actuate_pitcherons(0, WINGS_LEVEL);
            AP_log_data.flight_phase = "LANDED";
            AP_log_data.ap_mode = "AP_OFF";
        }
        else if ((ap_flight_phase == U_TURN_HDG) && !u_turn_done) {
            AP_log_data.flight_phase = "U_TURN_HDG";
            // Always verify flight envelope first
            if (Autopilot_FLT_ENVELOPE_PROT(sensor_data.roll, sensor_data.pitch, sensor_data.airspeed, AP_log_data)) {
                if (Autopilot_HDG_SEL_IMU(sensor_data.roll, sensor_data.yaw, U_TURN_BEARING_CHANGE, AP_log_data)) {
                    u_turn_done = true;
                    ap_flight_phase = GPS_HOMING;
                }
                else ap_flight_phase = U_TURN_HDG;
            }
        }
        else if (ap_flight_phase == GPS_HOMING) {
            AP_log_data.flight_phase = "GPS_HOMING";
            // Always verify flight envelope first
            if (Autopilot_FLT_ENVELOPE_PROT(sensor_data.roll, sensor_data.pitch, sensor_data.airspeed, AP_log_data)) {
                if (Autopilot_HDG_SEL_GPS(sensor_data.roll, sensor_data.heading, sensor_data.latitude, sensor_data.longitude, BULLSEYE_LATITUDE, BULLSEYE_LONGITUDE, AP_log_data) && PITCH_SPEED_CONTROL) {
                    ap_flight_phase = SPD_DESCENT;
                }
                else ap_flight_phase = GPS_HOMING;
            } 
        }
        else if (ap_flight_phase == SPD_DESCENT) {
            AP_log_data.flight_phase = "SPD_DESCENT";
            // Always verify flight envelope first
            if (Autopilot_FLT_ENVELOPE_PROT(sensor_data.roll, sensor_data.pitch, sensor_data.airspeed, AP_log_data)) {
                if (Autopilot_HDG_SEL_GPS(sensor_data.roll, sensor_data.heading, sensor_data.latitude, sensor_data.longitude, BULLSEYE_LATITUDE, BULLSEYE_LONGITUDE, AP_log_data)) {
                    if (Autopilot_SPD_TRIM(sensor_data.airspeed, sensor_data.pitch, SPD_TARGET, AP_log_data)) {
                        ap_flight_phase = GPS_HOMING;
                    }
                    else ap_flight_phase = SPD_DESCENT;
                }
                else {
                    ap_flight_phase = GPS_HOMING;
                }
            }
        }
        xQueueSend(Autopilot_Queue, &AP_log_data, portMAX_DELAY);
        vTaskSuspend(NULL);
    }
}

bool Autopilot_HDG_SEL_IMU(float roll, float yaw, float bearing_change, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_HDG_SEL_IMU";
    unsigned int pitcheron_angle = 0;
    static const float target_bearing = wrap_angle(yaw + bearing_change); // Gets target_bearing only from first yaw reading (on release detection).
    float bearing_correction = signed_bearing_correction(yaw, target_bearing); // Continuously recalculated from current bearing.
    float target_roll = Kp_ROLL_BEARING_CORR*bearing_correction; // To turn right, target roll is right
    if (target_roll < ROLL_LIM_MIN) target_roll = ROLL_LIM_MIN;
    else if (target_roll > ROLL_LIM_MAX) target_roll = ROLL_LIM_MAX;
    AP_log_data.ap_target_bearing = target_bearing;
    AP_log_data.ap_target_roll = target_roll;

    // First case: Flight within desired envelope for bearing
    if ((bearing_correction >= HDG_TARGET_DEVIATION_LOW) && (bearing_correction <= HDG_TARGET_DEVIATION_HIGH)) {
        target_roll = 0;
        AP_log_data.ap_target_roll = 0;
        // Verify that roll also within desired envelope for 0 bearing correction
        if ((target_roll-roll >= ROLL_TARGET_DEVIATION_LOW) && (target_roll-roll <= ROLL_TARGET_DEVIATION_HIGH)) {
            actuate_pitcherons(0, MAINTAIN_ANGLE);
            return true; // Don't do anything (Handover to AP_SPD_TRIM)
        }
        // Second case: Flight within desired envelope for bearing but not for roll
        else {
            // Could be greatly simplified to wings level, but doing this initially to match third case
            pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
            if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
            if (target_roll - roll > ROLL_TARGET_DEVIATION_HIGH) {
                // Roll is too low (aircraft is banking too much to the right), correct right
                actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
            } 
            else if (target_roll - roll < -ROLL_TARGET_DEVIATION_HIGH) {
                // Roll is too high (aircraft is banking too much to the left), correct left
                actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
            } 
            else if (target_roll - roll > -ROLL_TARGET_DEVIATION_LOW) {
                // Roll is slightly low, apply small correction right
                actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
            } 
            else if (target_roll - roll < ROLL_TARGET_DEVIATION_LOW) {
                // Roll is slightly high, apply small correction left
                actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
            } 
            else {
                // Roll is within the acceptable range, no correction needed
                actuate_pitcherons(0, MAINTAIN_ANGLE);
            }
            // return false; (by fall through)
        }
    }
    // Third case: Flight outside of envelope for bearing (roll envelope irrelevant)
    else {
        // Figure out whether left or right turn is needed
        // Left Turn Needed = Roll Left
        pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        if (target_roll - roll > ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too low (aircraft is banking too much to the right), correct right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < -ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too high (aircraft is banking too much to the left), correct left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else if (target_roll - roll > -ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly low, apply small correction right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly high, apply small correction left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else {
            // Roll is within the acceptable range, no correction needed
            actuate_pitcherons(0, MAINTAIN_ANGLE);
        }

    }
    return false;
}

bool Autopilot_HDG_SEL_GPS(float roll, float current_heading, float current_lat, float current_long, float target_lat, float target_long, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_HDG_SEL_GPS";
    unsigned int pitcheron_angle = 0;
    static const float target_bearing = calculate_bearing(current_lat, current_long, target_lat, target_long, current_heading); // Gets target_bearing from current and target coordinates (continuously recalculated)
    float bearing_correction = signed_bearing_correction(current_heading, target_bearing); // Continuously recalculated from current bearing.
    float target_roll = Kp_ROLL_BEARING_CORR*bearing_correction; // To turn right, target roll is right
    if (target_roll < ROLL_LIM_MIN) target_roll = ROLL_LIM_MIN;
    else if (target_roll > ROLL_LIM_MAX) target_roll = ROLL_LIM_MAX;
    AP_log_data.ap_target_bearing = target_bearing;
    AP_log_data.ap_target_roll = target_roll;

    // First case: Flight within desired envelope for bearing
    if ((bearing_correction >= HDG_TARGET_DEVIATION_LOW) && (bearing_correction <= HDG_TARGET_DEVIATION_HIGH)) {
        target_roll = 0;
        AP_log_data.ap_target_roll = 0;
        // Verify that roll also within desired envelope for 0 bearing correction
        if ((target_roll-roll >= ROLL_TARGET_DEVIATION_LOW) && (target_roll-roll <= ROLL_TARGET_DEVIATION_HIGH)) {
            actuate_pitcherons(0, MAINTAIN_ANGLE);
            return true; // Don't do anything (Handover to AP_SPD_TRIM)
        }
        // Second case: Flight within desired envelope for bearing but not for roll
        else {
            // Could be greatly simplified to wings level, but doing this initially to match third case
            pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
            if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
            if (target_roll - roll > ROLL_TARGET_DEVIATION_HIGH) {
                // Roll is too low (aircraft is banking too much to the right), correct right
                actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
            } 
            else if (target_roll - roll < -ROLL_TARGET_DEVIATION_HIGH) {
                // Roll is too high (aircraft is banking too much to the left), correct left
                actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
            } 
            else if (target_roll - roll > -ROLL_TARGET_DEVIATION_LOW) {
                // Roll is slightly low, apply small correction right
                actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
            } 
            else if (target_roll - roll < ROLL_TARGET_DEVIATION_LOW) {
                // Roll is slightly high, apply small correction left
                actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
            } 
            else {
                // Roll is within the acceptable range, no correction needed
                actuate_pitcherons(0, MAINTAIN_ANGLE);
            }
            // return false; (by fall through)
        }
    }
    // Third case: Flight outside of envelope for bearing (roll envelope irrelevant)
    else {
        // Figure out whether left or right turn is needed
        // Left Turn Needed = Roll Left
        pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        if (target_roll - roll > ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too low (aircraft is banking too much to the right), correct right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < -ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too high (aircraft is banking too much to the left), correct left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else if (target_roll - roll > -ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly low, apply small correction right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly high, apply small correction left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else {
            // Roll is within the acceptable range, no correction needed
            actuate_pitcherons(0, MAINTAIN_ANGLE);
        }

    }
    return false;
}

bool Autopilot_SPD_TRIM(float airspeed, float pitch, float target_airspeed, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_SPD_TRIM";
    unsigned int pitcheron_angle = 0;
    float speed_correction = target_airspeed - airspeed; // Positive speed correction = need to speed up (too slow) => Pitch nose down.
    float target_pitch = Kp_PITCH_SPD_CORR*speed_correction; // Kp NEGATIVE! e.g. +10 speed correction = -10 target_pitch
    // Limit checking for target_pitch
    if (target_pitch < PITCH_LIM_MIN) target_pitch = PITCH_LIM_MIN;
    else if (target_pitch > PITCH_LIM_MAX) target_pitch = PITCH_LIM_MAX;
    // First case: Flight within desired speed envelope
    if (((speed_correction >= SPD_TARGET_DEVIATION_LOW)) && (speed_correction <= SPD_TARGET_DEVIATION_HIGH)) {
        target_pitch = 0;
        AP_log_data.ap_target_pitch = 0;
        // Verify that pitch also within desired envelope for 0 airspeed correction
        if ((target_pitch-pitch >= PITCH_TARGET_DEVIATION_LOW) && (target_pitch-pitch <= PITCH_TARGET_DEVIATION_HIGH)) {
            actuate_pitcherons(0, MAINTAIN_ANGLE);
            return true; // Don't do anything (handover to AP_HDG_SEL_IMU)
        }
        // Second case: Flight within desired envelope for speed but not for pitch
        else {
            pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
            if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
            if (target_pitch-pitch > PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
            else if (target_pitch-pitch < -PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
            else if (target_pitch-pitch > -PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
            else if (target_pitch-pitch < PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
            else actuate_pitcherons(0, MAINTAIN_ANGLE);
        }
    }
    // Third case: Flight outside of envelope for speed (pitch envelope irrelevant)
    else {
        // Figure out whether pitch up or pitch down is needed
        // Pitch Up Needed = PITCH_NOSE_UP
        pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        if (target_pitch-pitch > PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
        else if (target_pitch-pitch < -PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
        else if (target_pitch-pitch > -PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
        else if (target_pitch-pitch < PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
        else actuate_pitcherons(0, MAINTAIN_ANGLE);
    }
    return false;
}

bool Autopilot_FLT_ENVELOPE_PROT(float roll, float pitch, float airspeed, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_FLT_ENVELOPE_PROT";
    // Prority 1: Roll Protection
    // Priority 2: Pitch Protection
    // Priority 3: Speed Protection
    if ((roll < ROLL_LIM_MIN) && ROLL_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_ROLL_MIN";
        float target_roll = ROLL_LIM_MAX;
        AP_log_data.ap_target_roll = target_roll;
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
    }
    else if ((roll > ROLL_LIM_MAX) && ROLL_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_ROLL_MAX";
        float target_roll = ROLL_LIM_MIN;
        AP_log_data.ap_target_roll = target_roll;
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
    }
    else if ((pitch < PITCH_LIM_MIN) && PITCH_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_PITCH_MIN";
        float target_pitch = PITCH_LIM_MAX;
        AP_log_data.ap_target_pitch = target_pitch;
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
    }
    else if ((pitch > PITCH_LIM_MAX) && PITCH_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_PITCH_MAX";
        float target_pitch = PITCH_LIM_MIN;
        AP_log_data.ap_target_pitch = target_pitch;
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
    }
    else if ((airspeed < STALL_SPEED) && STALL_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_STALL";
        // float target_airspeed = STALL_SPEED+OVERCORRECTION_SPEED;
        float target_pitch = PITCH_LIM_MIN;
        AP_log_data.ap_target_pitch = target_pitch;
        unsigned int pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
    }
    else if ((airspeed > OVERSPEED) && OVSPD_PROT_EN) {
        AP_log_data.ap_mode = "AP_PROT_OVERSPEED";
        // float target_airspeed = OVERSPEED-OVERCORRECTION_SPEED;
        float target_pitch = PITCH_LIM_MAX;
        AP_log_data.ap_target_pitch = target_pitch;
        unsigned int pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
    }
    else return true;
    
    return false;
}

bool Autopilot_ROLL_FIXED(float roll, float target_roll, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_ROLL_FIXED";
    if (target_roll < ROLL_LIM_MIN) target_roll = ROLL_LIM_MIN;
    else if (target_roll > ROLL_LIM_MAX) target_roll = ROLL_LIM_MAX;
    if ((target_roll-roll >= ROLL_TARGET_DEVIATION_LOW) && (target_roll-roll <= ROLL_TARGET_DEVIATION_HIGH)) {
        actuate_pitcherons(0, MAINTAIN_ANGLE);
        return true; // Don't do anything (Handover to AP_SPD_TRIM)
    }
    // Second case: Flight within desired envelope for bearing but not for roll
    else {
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_ROLL*(fabs(target_roll - roll)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        if (target_roll - roll > ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too low (aircraft is banking too much to the right), correct right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < -ROLL_TARGET_DEVIATION_HIGH) {
            // Roll is too high (aircraft is banking too much to the left), correct left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else if (target_roll - roll > -ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly low, apply small correction right
            actuate_pitcherons(pitcheron_angle, ROLL_RIGHT);
        } 
        else if (target_roll - roll < ROLL_TARGET_DEVIATION_LOW) {
            // Roll is slightly high, apply small correction left
            actuate_pitcherons(pitcheron_angle, ROLL_LEFT);
        } 
        else {
            // Roll is within the acceptable range, no correction needed
            actuate_pitcherons(0, MAINTAIN_ANGLE);
        }
    }
    return false;
}

bool Autopilot_PITCH_FIXED(float pitch, float target_pitch, Autopilot_Data& AP_log_data) {
    AP_log_data.ap_mode = "AP_PITCH_FIXED";
    if (target_pitch < PITCH_LIM_MIN) target_pitch = PITCH_LIM_MIN;
    else if (target_pitch > PITCH_LIM_MAX) target_pitch = PITCH_LIM_MAX;
    AP_log_data.ap_target_pitch = target_pitch;
    if ((target_pitch-pitch >= PITCH_TARGET_DEVIATION_LOW) && (target_pitch-pitch <= PITCH_TARGET_DEVIATION_HIGH)) {
        actuate_pitcherons(0, MAINTAIN_ANGLE);
        return true; // Don't do anything (handover to AP_HDG_SEL_IMU)
    } 
    else {
        float pitcheron_angle = (unsigned int)round(Kp_SERVO_ANGLE_PITCH*(fabs(target_pitch - pitch)));
        if (pitcheron_angle > MAX_PITCHERON_ANGLE) pitcheron_angle = MAX_PITCHERON_ANGLE;
        if (target_pitch-pitch > PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
        else if (target_pitch-pitch < -PITCH_TARGET_DEVIATION_HIGH) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
        else if (target_pitch-pitch > -PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_UP);
        else if (target_pitch-pitch < PITCH_TARGET_DEVIATION_LOW) actuate_pitcherons(pitcheron_angle, PITCH_NOSE_DOWN);
        else actuate_pitcherons(0, MAINTAIN_ANGLE);
        // return false; (by fall through)
    }
    return false;
}
