#include<Arduino.h>
#include<Wire.h>

uint8_t id = 0x28; // i2c address
uint8_t data[7]; // holds output data
uint8_t cmd[3] = {0xAA, 0x00, 0x00}; // command to be sent

double press_counts = 0; // digital pressure reading [counts]
double temp_counts = 0; // digital temperature reading [counts]
double pressure = 0; // pressure reading [bar, psi, kPa, etc.]
double temperature = 0; // temperature reading in deg C
double pressurePA = 0;
double airspeed = 0;

double rho = 1.27287; //https://www.omnicalculator.com/physics/air-density#what-is-the-density-of-air

// double outside_temp = 32; // in farenheit
// double airpressure = 100000; // in pascals
// double dewpoint = 28; // in farenheit
// double relative_humidity = 0;

double outputmax = 15099494; // output at maximum pressure [counts]
double outputmin = 1677722; // output at minimum pressure [counts]
double pmax = 1; // maximum value of pressure range in psi
double pmin = -1; // minimum value of pressure range in psi
double PSI_to_pascal = 6894.7572931783;

double percentage = 0; // holds percentage of full scale data
char printBuffer[200], cBuff[20], percBuff[20], pBuff[20], tBuff[20], pasBuff[20], sBuff[20];


void setup() {
    Serial.begin(115200);

    while (!Serial) {
        delay(10);
    }

    Wire.begin();
    sprintf(printBuffer, "\nStatus Register, 24 - bit Sensor data, Digital Pressure Counts,\
    Percentage of full scale pressure, Pressure Output, Temperature\n");
    Serial.println(printBuffer);

}


void loop() {
    Wire.beginTransmission(id);

    int stat = Wire.write (cmd, 3); // write command to the sensor
    stat |= Wire.endTransmission();
    delay(10);
    Wire.requestFrom(id, (int)7); // read back Sensor data 7 bytes
    int i = 0;
    for (i = 0; i < 7; i++) {
        data [i] = Wire.read();
    }

    press_counts = data[3] + data[2] * 256 + data[1] * 65536; // calculate digital pressure counts
    temp_counts = data[6] + data[5] * 256 + data[4] * 65536; // calculate digital temperature counts
    temperature = (temp_counts * 200 / 16777215) - 50; // calculate temperature in deg c
    percentage = (press_counts / 16777215) * 100; // calculate pressure as percentage of full scale

    //calculation of pressure value according to equation 2 of datasheet
    pressure = (((press_counts - outputmin) * (pmax - pmin)) / (outputmax - outputmin)) + pmin;
    pressurePA = pressure * PSI_to_pascal;
    airspeed = sqrt(2 * pressurePA / rho);

    dtostrf(press_counts, 4, 1, cBuff);
    dtostrf(percentage, 4, 3, percBuff);
    dtostrf(pressure, 4, 3, pBuff);
    dtostrf(temperature, 4, 3, tBuff);
    dtostrf(pressurePA, 4, 3, pasBuff);
    dtostrf(airspeed, 4, 3, sBuff);

    /*
    The below code prints the raw data as well as the processed data
    Data format : Status Register, 24-bit Sensor Data, Digital Counts, percentage of full scale
    pressure,
    pressure output, temperature
    */
    sprintf(printBuffer, " % x\t % 2x % 2x % 2x\t % s\t % s\t % s\t % s\t %s\t %s \n", data[0], data[1], data[2],
    data[3], cBuff, percBuff, pBuff, tBuff, pasBuff, sBuff);
    Serial.print(printBuffer);

    delay(10);
}