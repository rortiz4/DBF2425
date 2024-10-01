#include "sensors.h"

#define I2C_CLOCK_SPEED 100000 // 100 kHz
#define DP 3 // Decimal places of accuracy

void init_i2c() {
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
}

void setup() {
    delay(1000);
    Serial.begin(115200);
    while (!Serial)
        delay(10); // will pause Zero, Leonardo, etc until serial console opens

    Serial.println("MS4525DO Sensor Test!");
    init_i2c();
    init_ms4525do();

}

void loop() {
    float ms4525do_output_vector[5] = {0.0};
    read_ms4525do(ms4525do_output_vector);
    Serial.println("\nMS4525DO Data:");

    Serial.printf("Raw / Corrected Differential Pressure (Pa): %.*f / %.*f\n", \
    DP, ms4525do_output_vector[2], DP, ms4525do_output_vector[0]);
    Serial.printf("Raw / Corrected Airspeed (m/s): %.*f / %.*f\n", \
    DP, ms4525do_output_vector[3], DP, ms4525do_output_vector[1]);
    Serial.printf("Temperature (C): %.*f\n", DP, ms4525do_output_vector[4]);

    delay(250);
}