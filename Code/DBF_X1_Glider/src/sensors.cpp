/* Include Sensor Libraries */
#include <Adafruit_BNO08x.h>
#include "ms4525do.h"

#define AIR_DENSITY 1.225 // kg/m^3 (for airspeed calculation)

/* Instantiate sensor classes and types */
// BNO085
Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085_value;
// MS4525DO
bfs::Ms4525do ms4525do;

/* Sensor Initialization Functions */
// IMU
bool init_bno085() {
    Serial.print("Initializing BNO085 IMU...");
    // Reports Available: SH2_ACCELEROMETER, SH2_GYROSCOPE_CALIBRATED, SH2_MAGNETIC_FIELD_CALIBRATED,
    // SH2_LINEAR_ACCELERATION, SH2_GRAVITY, SH2_ROTATION_VECTOR, SH2_GEOMAGNETIC_ROTATION_VECTOR,
    // SH2_GAME_ROTATION_VECTOR, SH2_STEP_COUNTER, SH2_STABILITY_CLASSIFIER, SH2_RAW_ACCELEROMETER,
    // SH2_RAW_GYROSCOPE, SH2_RAW_MAGNETOMETER, SH2_SHAKE_DETECTOR, SH2_PERSONAL_ACTIVITY_CLASSIFIER
    if (!bno085.begin_I2C()) {
        Serial.println("\nFailed to find BNO08x chip!");
        return false;
    }
    if (!bno085.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("\nCould not enable rotation vector");
        return false;
    }
    if (!bno085.enableReport(SH2_ACCELEROMETER)) {
        Serial.println("\nCould not enable accelerometer");
        return false;
    }
    if (!bno085.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
        Serial.println("\nCould not enable gyroscope");
        return false;
    }
    if (!bno085.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED)) {
        Serial.println("\nCould not enable magnetic field calibrated");
        return false;
    }

    Serial.println("DONE!");
    return true;
}

// Differential Pressure Sensor (Pitot Tube Airspeed)
bool init_ms4525do() {
    Serial.print("Initializing MS4525DO Differential Pressure/Airspeed Sensor...");
    // I2C address of 0x28, on bus 0, with a -1 to +1 PSI range for pressure transducer
    ms4525do.Config(&Wire, 0x28, 1.0f, -1.0f);
    // Starting communication with the pressure transducer
    if (!ms4525do.Begin()) {
        Serial.println("\nError communicating with sensor!");
        return false;
    }
    Serial.println("DONE!");
    return true;
}

void init_all_sensors() {
    while (!init_bno085()) {
        delay(100);
        Serial.println("BNO085 IMU INITIALIZATION FAILED. RETRYING...");
    }
    delay(100);
    while (!init_ms4525do()) {
        delay(100);
        Serial.println("MS4525DO DIFFERENTIAL PRESSURE SENSOR INITIALIZATION FAILED. RETRYING...");
    }
    Serial.println("All Sensors Initialized Successfully!");
    delay(100);
}


/* Sensor Reading Functions */
// IMU
void read_bno085(float* output_vector) {
    bool rot_read = false;
    bool acc_read = false;
    bool gyro_read = false;
    bool mag_read = false;
    int read_count = 0;

    // Declared in calling function: float output_vector[13] = {0.0};
    // output_vector contains: [rot_real,roti,rotj,rotk,accx,accy,accz,gyrox,gyroy,gyroz,magx,magy,magz]

    while(read_count < 4) {
        // Try to get sensor data
        while (!bno085.getSensorEvent(&bno085_value)) {
            ;
        }
        // Once data is obtained, find out which sensor it belongs to
        switch(bno085_value.sensorId) {
            case SH2_ROTATION_VECTOR:
                // Only read data from a particular sensor once in the while loop
                if(!rot_read) {
                    output_vector[0] = bno085_value.un.rotationVector.real;
                    output_vector[1] = bno085_value.un.rotationVector.i;
                    output_vector[2] = bno085_value.un.rotationVector.j;
                    output_vector[3] = bno085_value.un.rotationVector.k;
                    read_count++;
                    rot_read = true;
                }
                break;
            case SH2_ACCELEROMETER:
                // Only read data from a particular sensor once in the while loop
                if(!acc_read) {
                    output_vector[4] = bno085_value.un.accelerometer.x; // m/s^2
                    output_vector[5] = bno085_value.un.accelerometer.y; // m/s^2
                    output_vector[6] = bno085_value.un.accelerometer.z; // m/s^2
                    read_count++;
                    acc_read = true;
                }
                break;
            case SH2_GYROSCOPE_CALIBRATED:
                // Only read data from a particular sensor once in the while loop
                if(!gyro_read) {
                    output_vector[7] = bno085_value.un.gyroscope.x; // rad/s
                    output_vector[8] = bno085_value.un.gyroscope.y; // rad/s
                    output_vector[9] = bno085_value.un.gyroscope.z; // rad/s
                    read_count++;
                    gyro_read = true;
                }
                break;
            case SH2_MAGNETIC_FIELD_CALIBRATED:
                // Only read data from a particular sensor once in the while loop
                if(!mag_read) {
                    output_vector[10] = bno085_value.un.magneticField.x; // uT
                    output_vector[11] = bno085_value.un.magneticField.y; // uT
                    output_vector[12] = bno085_value.un.magneticField.z; // uT
                    read_count++;
                    mag_read = true;
                }
                break;
        }
    }
}

// MS4525DO
void read_ms4525do(float* output_vector) {
    while(!ms4525do.Read()) {
        ;
    }
    float raw_diff_pressure = ms4525do.pres_pa();
    float temp_C = ms4525do.die_temp_c();

    float raw_airspeed;
    if (raw_diff_pressure < 0) {
        raw_airspeed = -sqrt(-(2*raw_diff_pressure)/AIR_DENSITY); // in m/s from Bernoulli's Equation
    }
    else {
        raw_airspeed = sqrt((2*raw_diff_pressure)/AIR_DENSITY);
    }

    float corr_diff_pressure = raw_diff_pressure; // Temporarily until accurately calibrated
    float corr_airspeed = raw_airspeed; // Temporarily until accurately calibrated

    output_vector[0] = corr_diff_pressure;
    output_vector[1] = corr_airspeed;
    output_vector[2] = raw_diff_pressure;
    output_vector[3] = raw_airspeed;
    output_vector[4] = temp_C;
}

void read_all_sensors(float* imu_output_vector, float* ms4525do_output_vector) {
    read_bno085(imu_output_vector);
    read_ms4525do(ms4525do_output_vector);
}