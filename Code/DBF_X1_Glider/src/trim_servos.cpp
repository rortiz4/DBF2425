#include "trim_servos.h"
#include "pitcheron_servos.h"

// Mini-program for trimming servos if requested by setting TRIM_SERVOS flag
void trim_servos() {
        init_servos_trim();
        int trim_l = 0;
        int trim_r = 0;
        Serial.println("Servo Trim Program Running. Note down RAW_TRIM_L, RAW_TRIM_R, and CW_CONVENTION needed to CENTER PITCHERONS.");
        Serial.println("Reset/Reprogram with TRIM_SERVOS set to false to disable trim mode. Then update RAW_TRIM flags in pitcheron_servos.cpp.");
        Serial.println("Send all commands over Serial when prompted as Text (ASCII) with \\n line terminator.");
        Serial.println("ALWAYS USE (int)DEGREES for any angles (+/-).");
        actuate_servo_l(0);
        actuate_servo_r(0);
        Serial.printf("Current trim settings: trim_l = %ddeg, trim_r = %ddeg\n", trim_l, trim_r);
        while(true) {
            Serial.printf("Select Servo to Trim [L/R]: ");
            while (Serial.available() <= 0); // Wait for user input
            String user_input = Serial.readStringUntil('\n');
            user_input.trim();

            if ((user_input == "L") || (user_input == "l")) {
                while(true) {
                    Serial.printf("Enter angle in degrees for LEFT servo (range: %ddeg to %ddeg) or press Enter to stop trimming: ", MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
                    while (Serial.available() <= 0); // Wait for user input
                    user_input = Serial.readStringUntil('\n');
                    user_input.trim();
                    if (user_input == "") break;
                    int user_trim = user_input.toInt();
                    // Check limits
                    if (user_trim < MIN_SERVO_ANGLE) {
                        user_trim = MIN_SERVO_ANGLE;
                        Serial.printf("WARNING: Trim value OUT OF RANGE (too LOW). Setting trim_l to Minimum (%ddeg)\n", MIN_SERVO_ANGLE);
                    }
                    else if (user_trim > MAX_SERVO_ANGLE) {
                        user_trim = MAX_SERVO_ANGLE;
                        Serial.printf("WARNING: Trim value OUT OF RANGE (too HIGH). Setting trim_l to Maximum (%ddeg)\n", MAX_SERVO_ANGLE);
                    }
                    trim_l = user_trim;
                    actuate_servo_l(trim_l);
                    Serial.printf("Current trim settings: trim_l = %ddeg, trim_r = %ddeg\n", trim_l, trim_r);
                }
            }

            else if ((user_input == "R") || (user_input == "r")) {
                while(true) {
                    Serial.printf("Enter angle in degrees for RIGHT servo (range: %ddeg to %ddeg) or press Enter to stop trimming: ", MIN_SERVO_ANGLE, MAX_SERVO_ANGLE);
                    while (Serial.available() <= 0); // Wait for user input
                    user_input = Serial.readStringUntil('\n');
                    user_input.trim();
                    if (user_input == "") break;
                    int user_trim = user_input.toInt();
                    // Check limits
                    if (user_trim < MIN_SERVO_ANGLE) {
                        user_trim = MIN_SERVO_ANGLE;
                        Serial.printf("WARNING: Trim value OUT OF RANGE (too LOW). Setting trim_r to Minimum (%ddeg)\n", MIN_SERVO_ANGLE);
                    }
                    else if (user_trim > MAX_SERVO_ANGLE) {
                        user_trim = MAX_SERVO_ANGLE;
                        Serial.printf("WARNING: Trim value OUT OF RANGE (too HIGH). Setting trim_r to Maximum (%ddeg)\n", MAX_SERVO_ANGLE);
                    }
                    trim_r = user_trim;
                    actuate_servo_r(trim_r);
                    Serial.printf("Current trim settings: trim_l = %ddeg, trim_r = %ddeg\n", trim_l, trim_r);
                }
            }

            else {
                Serial.println("Invalid input provided! Try again and enter only 'L' or 'R' (case-insensitive).");
            }
        }
}