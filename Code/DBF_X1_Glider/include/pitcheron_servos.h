#ifndef PITCHERON_SERVOS_H
#define PITCHERON_SERVOS_H

// https://cdn.shopify.com/s/files/1/0570/1766/3541/files/X08H_V6.0_Technical_Specifcation.pdf?v=1700472376
// https://kstservos.com/collections/glider-wing-servos/products/x08h-plus-horizontal-lug-servo-5-3kg-cm-0-09s-9-5g-8mm

#define MIN_SERVO_ANGLE -60 // deg (unused except for internal angle2us mapping or manual trimming override because dangerous!)
#define MAX_SERVO_ANGLE 60 // deg (unused except for internal angle2us mapping or manual trimming override because dangerous!)
#define SERVO_MIN_ALLOWED -45
#define SERVO_MAX_ALLOWED 45

enum Pitcheron_Actions {
    WINGS_LEVEL, // 0 (could also do WINGS_LEVEL = 0 for custom assignment, and so on.)
    ROLL_LEFT, // 1
    ROLL_RIGHT, // 2
    PITCH_NOSE_UP, // 3
    PITCH_NOSE_DOWN // 4
};

struct Pitcheron_Data {
    unsigned int sensor_id;
    int angle_target; // absolute value only
    const char* action_target; // e.g. WINGS_LEVEL, ROLL_RIGHT, etc.
    int raw_angle_l; // Angle including trim adjustments and CG/CW corrections (raw)
    int raw_angle_r; // Angle including trim adjustments and CG/CW corrections (raw)
    // All angles in degrees.
};

// Functions just for initial testing/trimming (angle specified is directly used for actuation as-is)
void actuate_servo_l(int angle);
void actuate_servo_r(int angle);

void init_servos(bool actuation_test); // Use either init_servos or init_servos_trim in a program, but NOT BOTH.
void init_servos_trim(void); // Do not use, except for in another program (not main()) exclusively to initialize for trimming individual servos.

void actuate_pitcherons(int angle, enum Pitcheron_Actions act_type_direction); // Specify PITCH or ROLL with direction for actuation type + direction (both servos use same angle but potentially different trim offsets)
#endif