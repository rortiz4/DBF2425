#include <Arduino.h>
#include <ESP32Servo.h>
#include "pin_map.h"
#include "pitcheron_servos.h"
#include "queues.h"

// Basic Assumption: Pitcheron Angle = Servo Angle
// https://cdn.shopify.com/s/files/1/0570/1766/3541/files/X08H_V6.0_Technical_Specifcation.pdf?v=1700472376
// https://kstservos.com/collections/glider-wing-servos/products/x08h-plus-horizontal-lug-servo-5-3kg-cm-0-09s-9-5g-8mm

// Servos l and r are connected to left and right pitcherons as viewed from behind the glider.
// CW_CONVENTION: Clockwise = Positive Angle, Counterclockwise = Negative Angle. Set CONVENTION to -1 if opposite (still assumes 2 identical servos)
#define CW_CONVENTION 1 // 1=Clockwise, -1=Counterclockwise

// CG_CONVENTION: Determine whether fully assembled glider CG is behind or in front of the pitcherons (wingtip test!)
#define CG_CONVENTION 1 // 1 = Pitcherons point up, nose goes down (pitcherons behind CG, like elevators). -1 = Pitcherons point up, nose goes up (pitcherons in front of CG).

// Note: center would be 0 degrees left and right in code, but may not be the case in real life. Trim offset added to center
#define RAW_TRIM_L 0 // deg. Set this to whatever angle must be requested in independent servo tests (actuate_servo_l) to center left servo when testing (regardless of CONVENTION).
#define RAW_TRIM_R 0 // deg. Set this to whatever angle must be requested in independent servo tests (actuate_servo_r) to center right servo when testing (regardless of CONVENTION).
// Define Servo Physical Limits
#define MIN_SERVO_us 1000 // us
#define MAX_SERVO_us 2000 // us
#define PWM_FREQUENCY 333 // Hz

#define ACTUATION_DELAY_ms 1000 // How long to wait after each actuation test

Servo left_servo;
Servo right_servo;

void init_servos_trim(void) {
    Serial.println("Initializing Servos...");
	ESP32PWM::allocateTimer(0); // Allocate timer for both servos
    left_servo.setPeriodHertz(PWM_FREQUENCY);    // 333 Hz servo
    right_servo.setPeriodHertz(PWM_FREQUENCY);    // 333 Hz servo
    left_servo.attach(SERVO_L_PIN); // Attach left servo to pin
    right_servo.attach(SERVO_R_PIN); // Attach right servo to pin
    Serial.println("Servo Initialization Complete. Start Trimming."); 
}

// This function maps angles to microseconds (PWM width) based on servo datasheet
int angle2us(int angle, int angle_min, int angle_max, int us_min, int us_max) {
    if (angle < angle_min) angle = angle_min; // Handle limit
    else if (angle > angle_max) angle = angle_max; // Handle limit
    return us_min + ((angle - angle_min) * (us_max - us_min)) / (angle_max - angle_min);
}

void actuate_servo_l(int raw_angle_servo_l) {
    left_servo.writeMicroseconds(angle2us(raw_angle_servo_l, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_SERVO_us, MAX_SERVO_us));
}

void actuate_servo_r(int raw_angle_servo_r) {
    right_servo.writeMicroseconds(angle2us(raw_angle_servo_r, MIN_SERVO_ANGLE, MAX_SERVO_ANGLE, MIN_SERVO_us, MAX_SERVO_us));
}

// This function initializes the servos, checks range of travel, and then centers servos
void init_servos(bool actuation_test = true) {
    Serial.println("Initializing Servos...");
	ESP32PWM::allocateTimer(0); // Allocate timer for both servos
    left_servo.setPeriodHertz(PWM_FREQUENCY);    // 333 Hz servo
    right_servo.setPeriodHertz(PWM_FREQUENCY);    // 333 Hz servo
    left_servo.attach(SERVO_L_PIN); // Attach left servo to pin
    right_servo.attach(SERVO_R_PIN); // Attach right servo to pin

    // Set servos to 0 position
    Serial.printf("Centering LEFT servo centered with trim @%ddeg.\n", RAW_TRIM_L);
    actuate_servo_l(RAW_TRIM_L+0);
    Serial.printf("Centering RIGHT servo centered with trim @%ddeg.\n", RAW_TRIM_R);
    actuate_servo_r(RAW_TRIM_R+0);
    Serial.println("Servo Initialization Complete! Verify both pitcherons are now correctly trimmed/centered.");
    if (CW_CONVENTION == -1) Serial.println("Note: Using alternate convention for angles (+ => counterclockwise, - => clockwise).");
    else if (CW_CONVENTION != 1) Serial.println("INVALID CW_CONVENTION SPECIFIED! STOP! Roll control will be scaled incorrectly!");
    delay(ACTUATION_DELAY_ms);
    if (actuation_test == true) {
        Serial.println("Testing Servos... Observe movement carefully to verify correct actuation/response.");
        // If both servos turn counterclockwise or clockwise, Pitcherons = Ailerons
        // If one turns clockwise and the other counterclockwise, Pitcherons = Elevator

        // Turn both servos in same direction (counterclockwise: counter-rotating pitcherons). Left pitcheron points down, right points up (roll left)
        actuate_servo_l(RAW_TRIM_L+(SERVO_MIN_ALLOWED*CW_CONVENTION));
        actuate_servo_r(RAW_TRIM_R+(SERVO_MIN_ALLOWED*CW_CONVENTION));
        Serial.printf("Test #1/6: Confirm that LEFT pitcheron points DOWN and RIGHT pitcheron points UP (both@min allowed deflection = %ddeg).\n", SERVO_MIN_ALLOWED);
        delay(ACTUATION_DELAY_ms);

        // Turn both servos in same direction (clockwise: counter-rotating pitcherons). Left pitcheron points up, Right points down (roll right)
        actuate_servo_l(RAW_TRIM_L+(SERVO_MAX_ALLOWED*CW_CONVENTION));
        actuate_servo_r(RAW_TRIM_R+(SERVO_MAX_ALLOWED*CW_CONVENTION));
        Serial.printf("Test #2/6: Confirm that LEFT pitcheron points UP and RIGHT pitcheron points DOWN (both@max allowed deflection = %ddeg)\n.", SERVO_MAX_ALLOWED);
        delay(ACTUATION_DELAY_ms);

        // Center servos
        actuate_servo_l(RAW_TRIM_L+0);
        actuate_servo_r(RAW_TRIM_R+0);
        Serial.printf("Test #3/6: Confirm that BOTH pitcherons are CENTERED (servo_l trim@%ddeg; servo_r trim@%ddeg).\n", RAW_TRIM_L, RAW_TRIM_R);
        delay(ACTUATION_DELAY_ms);

        // Turn servos in opposite directions (L: clockwise, R: counterclockwise). Left pitcheron points up, Right points up (pitch nose down/up)
        actuate_servo_l(RAW_TRIM_L+(SERVO_MAX_ALLOWED*CW_CONVENTION));
        actuate_servo_r(RAW_TRIM_R+(SERVO_MIN_ALLOWED*CW_CONVENTION));
        Serial.println("Test #4/6: Confirm that BOTH pitcherons point UP (max deflection).");
        if (CG_CONVENTION == 1) Serial.println("Pitcherons are BEHIND CG. Confirm this action causes NOSE PITCH DOWN.");
        else if (CG_CONVENTION == -1) Serial.println("Pitcherons are IN FRONT OF CG. Confirm this action causes NOSE PITCH UP.");
        else Serial.println("INVALID CG_CONVENTION SPECIFIED! STOP! There will be no pitch control in-flight (only roll)!");
        delay(ACTUATION_DELAY_ms);

        // Turn servos in opposite directions (L: counterclockwise, R: clockwise). Left pitcheron points down, Right points down (pitch nose up/down)
        actuate_servo_l(RAW_TRIM_L+(SERVO_MIN_ALLOWED*CW_CONVENTION));
        actuate_servo_r(RAW_TRIM_R+(SERVO_MAX_ALLOWED*CW_CONVENTION));
        Serial.println("Test #5/6: Confirm that BOTH pitcherons point DOWN (max deflection).");
        if (CG_CONVENTION == 1) Serial.println("Pitcherons are BEHIND CG. Confirm this action causes NOSE PITCH UP.");
        else if (CG_CONVENTION == -1) Serial.println("Pitcherons are IN FRONT OF CG. Confirm this action causes NOSE PITCH DOWN.");
        else Serial.println("INVALID CG_CONVENTION SPECIFIED! STOP! There will be no pitch control in-flight (only roll)!");
        delay(ACTUATION_DELAY_ms);

        // Center servos
        actuate_servo_l(RAW_TRIM_L+0);
        actuate_servo_r(RAW_TRIM_R+0);
        Serial.printf("Test #6/6: Confirm that BOTH pitcherons are CENTERED (servo_l trim@%ddeg; servo_r trim@%ddeg).\n", RAW_TRIM_L, RAW_TRIM_R);
        delay(ACTUATION_DELAY_ms);

        Serial.println("Servo Testing Complete! Verify both pitcherons are now correctly trimmed/centered.");
    }


}

// This function actuates 2 servos using actuate_servo_l and actuate_servo_r. Specify ROLL_LEFT/ROLL_RIGHT/PITCH_NOSE_UP/PITCH_NOSE_DOWN or WINGS_LEVEL for act_type_direction.
void actuate_pitcherons(unsigned int angle, enum Pitcheron_Actions act_type_direction) {
    Pitcheron_Data new_pitcheron_data;
    new_pitcheron_data.sensor_id = 4;
    new_pitcheron_data.angle_target = angle;
    switch(act_type_direction) {
        case WINGS_LEVEL:
            // Angle ignored (same as doing any action with angle=0)
            actuate_servo_l(RAW_TRIM_L+0);
            actuate_servo_r(RAW_TRIM_R+0);
            new_pitcheron_data.action_target = "WINGS_LEVEL";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L+0;
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R+0;
            break; 
        case ROLL_LEFT:
            // Servo rotation in same direction = Pitcherons actuate in opposite directions
            actuate_servo_l(RAW_TRIM_L-(angle*CW_CONVENTION));
            actuate_servo_r(RAW_TRIM_R-(angle*CW_CONVENTION));
            new_pitcheron_data.action_target = "ROLL_LEFT";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L-(angle*CW_CONVENTION);
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R-(angle*CW_CONVENTION);
            break;
        case ROLL_RIGHT:
            // Servo rotation in same direction = Pitcherons actuate in opposite directions
            actuate_servo_l(RAW_TRIM_L+(angle*CW_CONVENTION));
            actuate_servo_r(RAW_TRIM_R+(angle*CW_CONVENTION));
            new_pitcheron_data.action_target = "ROLL_RIGHT";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L+(angle*CW_CONVENTION);
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R+(angle*CW_CONVENTION);
            break;
        case PITCH_NOSE_UP:
            // Servo rotation in opposite direction = Pitcherons actuate in same directions
            actuate_servo_l(RAW_TRIM_L-(angle*CW_CONVENTION*CG_CONVENTION));
            actuate_servo_r(RAW_TRIM_R+(angle*CW_CONVENTION*CG_CONVENTION));
            new_pitcheron_data.action_target = "PITCH_NOSE_UP";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L-(angle*CW_CONVENTION*CG_CONVENTION);
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R+(angle*CW_CONVENTION*CG_CONVENTION);   
            break;
        case PITCH_NOSE_DOWN:
            // Servo rotation in opposite direction = Pitcherons actuate in same directions 
            actuate_servo_l(RAW_TRIM_L+(angle*CW_CONVENTION*CG_CONVENTION));
            actuate_servo_r(RAW_TRIM_R-(angle*CW_CONVENTION*CG_CONVENTION));
            new_pitcheron_data.action_target = "PITCH_NOSE_DOWN";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L+(angle*CW_CONVENTION*CG_CONVENTION);
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R-(angle*CW_CONVENTION*CG_CONVENTION); 
            break;
        default:
            // Just center and do nothing else (same as WINGS_LEVEL).
            actuate_servo_l(RAW_TRIM_L+0);
            actuate_servo_r(RAW_TRIM_R+0);
            new_pitcheron_data.action_target = "WINGS_LEVEL";
            new_pitcheron_data.raw_angle_l = RAW_TRIM_L+0;
            new_pitcheron_data.raw_angle_r = RAW_TRIM_R+0;
            break;
    }
    xQueueSend(Pitcheron_Queue, &new_pitcheron_data, portMAX_DELAY);
}
