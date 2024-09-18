#include "sensors.h"

#define I2C_CLOCK_SPEED 100000 // 100 kHz
#define DP 3 // Decimal places of accuracy

void init_i2c() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
}

void setup(void) {
    delay(1000);
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("Adafruit BNO08x IMU Sensor Test!");
    init_i2c();
    init_bno085();

}

void loop() {
    float imu_output_vector[13] = {0.0};
    read_bno085(imu_output_vector);
    Serial.println("\nIMU Data:");

    Serial.printf("Orientation (Quaternion) - real: %.*f, i: %.*f, j: %.*f, k: %.*f\n", \
    DP, imu_output_vector[0], DP, imu_output_vector[1], DP, imu_output_vector[2], DP, imu_output_vector[3]);
    Serial.printf("Acceleration (m/s^2) - x: %.*f, y: %.*f, z: %.*f\n", \
    DP, imu_output_vector[4], DP, imu_output_vector[5], DP, imu_output_vector[6]);
    Serial.printf("Angular Velocity (rad/s) - x: %.*f, y: %.*f, z: %.*f\n", \
    DP, imu_output_vector[7], DP, imu_output_vector[8], DP, imu_output_vector[9]);
    Serial.printf("Magnetic Field (uT) - x: %.*f, y: %.*f, z: %.*f\n", \
    DP, imu_output_vector[10], DP, imu_output_vector[11], DP, imu_output_vector[12]);

  
    delay(250);
}