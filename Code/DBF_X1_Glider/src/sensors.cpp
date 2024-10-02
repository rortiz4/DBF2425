// This file contains all sensor related code functions to initialize and read from sensors.
#include <Arduino.h>
#include "queues.h"
#include "semaphores.h"
#include <Wire.h>
/* Include Sensor Libraries */
#include <Adafruit_BNO08x.h>
#include "ms4525do.h"
#include <SparkFun_u-blox_GNSS_Arduino_Library.h> //Click here to get the library: http://librarymanager/All#SparkFun_u-blox_GNSS

#include <MicroNMEA.h> //http://librarymanager/All#MicroNMEA

#define SERIAL_MONITOR_BAUDRATE 115200 // bits/sec
#define STARTUP_DELAY 5000 // x2
#define I2C_BUS_SPEED 200000 // 100kHz Default
#define GPS_SAMPLE_RATE 10 // Hz (10Hz max)
#define NMEA_BUFFER_SIZE 255
#define INIT_DELAY 100

/* Instantiate sensor classes and types */
// BNO085
Adafruit_BNO08x bno085(-1);
sh2_SensorValue_t bno085_value;
// MS4525DO
bfs::Ms4525do ms4525do;
// GPS
SFE_UBLOX_GNSS myGNSS;
// Create buffer variables for NMEA Sentence Parsing
char nmeaBuffer[NMEA_BUFFER_SIZE];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Initialize Serial and I2C hardware
void init_low_level_hw() {
    // Startup Delay is blocking but that's ok.
    Serial.begin(SERIAL_MONITOR_BAUDRATE);
    delay(STARTUP_DELAY);
    Serial.println("\nESP32 DBF 2025 Payload X1 Glider RTOS Data Collection Software - v1.0");
    Serial.println("By Daniel Noronha & Ricky Ortiz");
    Serial.println("Last Software Update: October 02, 2024");
    Serial.println("Wish Me Luck!!!\n");

    delay(STARTUP_DELAY);
    Wire.begin();
    Wire.setClock(I2C_BUS_SPEED);
    Serial.println("Serial IO & I2C Initialized Successfully!");
}

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

bool init_gps() {
    Serial.print("Initializing GPS...");
    // Starting communication with GPS (assume default I2C Address)
    if (!myGNSS.begin()) {
        Serial.println("\nError communicating with sensor!");
        return false;
    }

    myGNSS.setI2COutput(COM_TYPE_UBX | COM_TYPE_NMEA); //Set the I2C port to output both NMEA and UBX messages
    myGNSS.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT); //Save (only) the communications port settings to flash and BBR
    myGNSS.setProcessNMEAMask(SFE_UBLOX_FILTER_NMEA_GGA | SFE_UBLOX_FILTER_NMEA_RMC); // We only want GGA and RMC NMEA Messages, ignore others
    myGNSS.setNavigationFrequency(GPS_SAMPLE_RATE); // 5 Hz originally

    Serial.println("DONE!");
    return true;
}

void init_all_sensors() {
    while (!init_bno085()) {
        delay(INIT_DELAY);
        Serial.println("BNO085 IMU INITIALIZATION FAILED. RETRYING...");
    }

    while (!init_ms4525do()) {
        delay(INIT_DELAY);
        Serial.println("MS4525DO DIFFERENTIAL PRESSURE SENSOR INITIALIZATION FAILED. RETRYING...");
    }
    while(!init_gps()) {
        delay(INIT_DELAY);
        Serial.println("GPS INITIALIZATION FAILED. RETRYING...");
    }
    Serial.println("All Sensors Initialized Successfully!");
}


/* Sensor Reading Functions */
// IMU
void read_bno085(void* pvParameters) {
    // Initialize IMU Data struct
    IMU_Data new_imu_data;
    new_imu_data.sensor_id = 0;
    while(true) {
        bool rot_read = false;
        bool acc_read = false;
        bool gyro_read = false;
        bool mag_read = false;
        int read_count = 0;

        while(read_count < 4) {
            xSemaphoreTake(I2C_MUTEX, portMAX_DELAY); // This is blocking. xSemaphoreGive() is not
            // Try to get sensor data
            if (!bno085.getSensorEvent(&bno085_value)) {
                xSemaphoreGive(I2C_MUTEX);
                continue;
            }
            // Once data is obtained, find out which sensor it belongs to
            switch(bno085_value.sensorId) {
                case SH2_ROTATION_VECTOR:
                    // Only read data from a particular sensor once in the while loop
                    if(!rot_read) {
                        new_imu_data.rotation[0] = bno085_value.un.rotationVector.real;
                        new_imu_data.rotation[1] = bno085_value.un.rotationVector.i;
                        new_imu_data.rotation[2] = bno085_value.un.rotationVector.j;
                        new_imu_data.rotation[3] = bno085_value.un.rotationVector.k;
                        xSemaphoreGive(I2C_MUTEX);
                        read_count++;
                        rot_read = true;
                    }
                    else xSemaphoreGive(I2C_MUTEX);
                    break;
                case SH2_ACCELEROMETER:
                    // Only read data from a particular sensor once in the while loop
                    if(!acc_read) {
                        new_imu_data.acceleration[0] = bno085_value.un.accelerometer.x; // m/s^2
                        new_imu_data.acceleration[1] = bno085_value.un.accelerometer.y; // m/s^2
                        new_imu_data.acceleration[2] = bno085_value.un.accelerometer.z; // m/s^2
                        xSemaphoreGive(I2C_MUTEX);
                        read_count++;
                        acc_read = true;
                    }
                    else xSemaphoreGive(I2C_MUTEX);
                    break;
                case SH2_GYROSCOPE_CALIBRATED:
                    // Only read data from a particular sensor once in the while loop
                    if(!gyro_read) {
                        new_imu_data.gyro[0] = bno085_value.un.gyroscope.x; // rad/s
                        new_imu_data.gyro[1] = bno085_value.un.gyroscope.y; // rad/s
                        new_imu_data.gyro[2] = bno085_value.un.gyroscope.z; // rad/s
                        xSemaphoreGive(I2C_MUTEX);
                        read_count++;
                        gyro_read = true;
                    }
                    else xSemaphoreGive(I2C_MUTEX);
                    break;
                case SH2_MAGNETIC_FIELD_CALIBRATED:
                    // Only read data from a particular sensor once in the while loop
                    if(!mag_read) {
                        new_imu_data.magnetic[0] = bno085_value.un.magneticField.x; // uT
                        new_imu_data.magnetic[1] = bno085_value.un.magneticField.y; // uT
                        new_imu_data.magnetic[2] = bno085_value.un.magneticField.z; // uT
                        xSemaphoreGive(I2C_MUTEX);
                        read_count++;
                        mag_read = true;
                    }
                    else xSemaphoreGive(I2C_MUTEX);
                    break;
                default:
                    xSemaphoreGive(I2C_MUTEX);
                    break;
            }
        }
        xQueueSend(IMU_Queue, &new_imu_data, portMAX_DELAY);
        xSemaphoreGive(imu_done);
        vTaskSuspend(NULL); // Data logging task will resume this as soon as all data has been logged.
        // If queue sending fails, it will try again without suspending.
    }
}

// MS4525DO
void read_ms4525do(void* pvParameters) {
    // Initialize Airspeed_Data struct
    Airspeed_Data new_airspeed_data;
    new_airspeed_data.sensor_id = 1;
    while(true) {
        xSemaphoreTake(I2C_MUTEX, portMAX_DELAY);
        if(!ms4525do.Read()) {
            xSemaphoreGive(I2C_MUTEX);
            continue;
        }
        // Must initialize a vector like so: float output_vector[5]; to store data.
        float raw_diff_pressure = ms4525do.pres_pa();
        float temp_C = ms4525do.die_temp_c();
        float raw_airspeed = ms4525do.aspeed();

        xSemaphoreGive(I2C_MUTEX);

        float corr_diff_pressure = raw_diff_pressure; // Temporarily until accurately calibrated
        float corr_airspeed = raw_airspeed + 9.441615431; // Temporarily until accurately calibrated

        new_airspeed_data.diff_pressure[0] = raw_diff_pressure;
        new_airspeed_data.diff_pressure[1] = corr_diff_pressure;
        new_airspeed_data.airspeed[0] = raw_airspeed;
        new_airspeed_data.airspeed[1] = corr_airspeed;
        new_airspeed_data.temperature = temp_C;

        xQueueSend(Airspeed_Queue, &new_airspeed_data, portMAX_DELAY);
        xSemaphoreGive(airspeed_done);
        vTaskSuspend(NULL); // Data logging task will resume this as soon as all data has been logged.
    }
}

void read_gps(void* pvParameters) {
    // Initialize GPS_Data struct
    GPS_Data new_gps_data;
    bool first_fix = false;
    new_gps_data.sensor_id = 2;
    while(true) {
        xSemaphoreTake(I2C_MUTEX, portMAX_DELAY);
        myGNSS.checkUblox();
        // Fetch GPS data character by character
        if(!nmea.isValid()) {
            xSemaphoreGive(I2C_MUTEX);
            if (!first_fix) {
                new_gps_data.latitude = 0;
                new_gps_data.longitude = 0;
                new_gps_data.heading = 0;
                new_gps_data.gnd_speed = 0;
                new_gps_data.altitude = 0;
                new_gps_data.hours = 0;
                new_gps_data.minutes = 0;
                new_gps_data.seconds = 0;
                new_gps_data.hundredths = 0;
                new_gps_data.satellites = 0;
                xQueueSend(GPS_Queue, &new_gps_data, portMAX_DELAY);
                xSemaphoreGive(gps_done);
                vTaskSuspend(NULL);
            }
            //xQueueSend(GPS_Queue, &new_gps_data, portMAX_DELAY);
            //xSemaphoreGive(gps_done);
            //vTaskSuspend(NULL);
            continue;
        }
        first_fix = true;
        // Store NMEA parsed data (with consistent type-casting)
        long alt_long;
        long heading_long;
        float latitude_mdeg = (float)nmea.getLatitude();
        float longitude_mdeg = (float)nmea.getLongitude();
        float heading = (float)nmea.getCourse();
        float gnd_speed = (float)nmea.getSpeed();
        bool altitude = nmea.getAltitude(alt_long);
        float alt = (float)alt_long;
        uint8_t hours = (nmea.getHour() - 4); // EST is 4 hours behind UTC
        uint8_t minutes = nmea.getMinute();
        uint8_t seconds = nmea.getSecond();
        uint8_t hundredths = nmea.getHundredths();
        uint8_t num_sats = (float)nmea.getNumSatellites(); // Can be int but makes queue implementation much easier
        // Clear nmea buffer
        nmea.clear(); // We already stored the data in variables above.
        xSemaphoreGive(I2C_MUTEX);
        // Adjusting Entries!
        latitude_mdeg = latitude_mdeg / 1000000;
        longitude_mdeg = longitude_mdeg / 1000000;
        gnd_speed = gnd_speed * (1.68781 / 1000); // Knots to ft/s
        alt = alt / 1000;
        heading = heading / 1000;

        // Store Data in struct, then send to queue
        new_gps_data.latitude = latitude_mdeg;
        new_gps_data.longitude = longitude_mdeg;
        new_gps_data.heading = heading;
        new_gps_data.gnd_speed = gnd_speed;
        new_gps_data.altitude = alt;
        new_gps_data.hours = hours;
        new_gps_data.minutes = minutes;
        new_gps_data.seconds = seconds;
        new_gps_data.hundredths = hundredths;
        new_gps_data.satellites = num_sats;

        xQueueSend(GPS_Queue, &new_gps_data, portMAX_DELAY);
        xSemaphoreGive(gps_done);
        vTaskSuspend(NULL);
    }
    
}

//This function gets called from the SparkFun u-blox Arduino Library
//As each NMEA character comes in you can specify what to do with it
//Useful for passing to other libraries like tinyGPS, MicroNMEA, or even
//a buffer, radio, etc.
void SFE_UBLOX_GNSS::processNMEA(char incoming)
{
  //Take the incoming char from the u-blox I2C port and pass it on to the MicroNMEA lib
  //for sentence cracking
  nmea.process(incoming);
}

