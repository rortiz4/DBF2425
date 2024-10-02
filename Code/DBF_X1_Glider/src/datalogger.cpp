// This file contains all SD card related functions to initialize and write to the SD Card
#include <SPI.h>
#include <SD.h>
#include "datalogger.h"
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"

#define SD_CS_PIN 10
#define FILE_COUNT_START 0
#define INIT_DELAY_SD 100
#define DP_DATA 3 // Decimal places to record
#define DP_GPS 6 // Latitude/Longitude decimal places

#define BUILTIN_LED_PIN 15

File datafile;                              // File object to handle file writing

void init_SD() {
    unsigned int file_counter = FILE_COUNT_START;        // Start the file counter at 1
    char filename[16];
    Serial.print("Initializing SD Card...");
    pinMode(SD_CS_PIN, OUTPUT);
    while (!SD.begin(SD_CS_PIN)) {
        Serial.println("Card failed, or not present. Retrying..."); // Retry if SD card can't be initialized
    }
    Serial.println("DONE!");
    Serial.println("Searching for next available filename");
    // Get next filenumber for filename 
    sprintf(filename, "data%03d.csv", file_counter); // Create the filename with the counter
    while(SD.exists(filename)) { 
        file_counter++;
        if (file_counter == 1000) {
            file_counter = FILE_COUNT_START;
            Serial.println("Overwriting file data000.csv... 1000 files in storage. Please DELETE SOME!!!");
            sprintf(filename, "data%03d.csv", file_counter); // Create the filename with the counter
            break;
        }
        else sprintf(filename, "data%03u.csv", file_counter); // Create the filename with the counter
    }

    datafile = SD.open(filename, FILE_WRITE);
    // Open File
    if (datafile) {
        Serial.printf("Writing to file: data%03u.csv\n", file_counter);
    }
    else {
        Serial.printf("Failed to Open File: data%03u.csv for writing!\n", file_counter);
        while(!datafile) {
            Serial.println("Retrying file open...");
            delay(INIT_DELAY_SD);
            datafile = SD.open(filename, FILE_WRITE);
        }
    }
    Serial.println("Writing .csv header:");
    // Write Header to file
    const char csv_header[] =   "line_Number,ESP_Time,ID0,Acc_x,Acc_y,Acc_z,gyro_x,gyro_y,gyro_z,mag_x,mag_y,mag_z,"
                                "rot_re,rot_i,rot_j,rot_k,ID1,raw_press,corr_press,raw_airspeed,corr_airspeed,temp,"
                                "ID2,latitude,longitude,gnd_speed,altitude,hours,mins,secs,hundredths,satellites\n";
    Serial.print(csv_header);
    Serial.flush(); 
    datafile.print(csv_header);
    datafile.flush(); 
    Serial.println(".csv Header Writing DONE!");
}

void log_data(void* pvParameters) {
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    IMU_Data imu;
    Airspeed_Data pitot;
    GPS_Data gps;
    bool led_on = false;
    unsigned long line_num = 1; // These are only initialized once
    Serial.flush();
    while(true) {
        digitalWrite(BUILTIN_LED_PIN, LOW);
        /* From queues.h (just for reference)
            // Defining containers for data
        struct IMU_Data {
            unsigned int sensor_id;
            float acceleration[3]; // x,y,z
            float gyro[3]; // x,y,z
            float magnetic[3]; // x,y,z
            float rotation[4]; // Quaternion - real, i, j, k
        };

        struct Airspeed_Data {
            unsigned int sensor_id;
            float diff_pressure[2]; // raw, corrected
            float airspeed[2]; // raw, corrected
            float temperature;
        };

        struct GPS_Data {
            unsigned int sensor_id;
            float latitude;
            float longitude;
            float gnd_speed; // knots
            float altitude;
            uint8_t hours;
            uint8_t minutes;
            uint8_t seconds;
            uint8_t hundredths;
            uint8_t satellites;
        };
        */
        // Check GPS first since that is the slowest (others should already be taken)
        bool gps_taken = xSemaphoreTake(gps_done, portMAX_DELAY);
        bool imu_taken = xSemaphoreTake(imu_done, portMAX_DELAY);
        bool airspeed_taken = xSemaphoreTake(airspeed_done, portMAX_DELAY);
        if (airspeed_taken) { //imu_take && airspeed_taken && gps_taken) {
            xQueueReceive(GPS_Queue, &gps, portMAX_DELAY);
            vTaskResume(read_gps_task);
            xQueueReceive(IMU_Queue, &imu, portMAX_DELAY);
            vTaskResume(read_imu_task);
            xQueueReceive(Airspeed_Queue, &pitot, portMAX_DELAY);
            vTaskResume(read_pitot_task);
        }
        static unsigned long start_time = micros();
        float current_time = (float)((micros() - start_time)/1000000.0); // Convert us (micros) to s
        // Line Num + ESP Time + IMU Data
        Serial.printf("%lu,%.*f,%u,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,", line_num, 6, current_time, imu.sensor_id, \
        DP_DATA, imu.acceleration[0], \
        DP_DATA, imu.acceleration[1], \
        DP_DATA, imu.acceleration[2], \
        DP_DATA, imu.gyro[0], \
        DP_DATA, imu.gyro[1], \
        DP_DATA, imu.gyro[2], \
        DP_DATA, imu.magnetic[0], \
        DP_DATA, imu.magnetic[1], \
        DP_DATA, imu.magnetic[2], \
        DP_DATA, imu.rotation[0], \
        DP_DATA, imu.rotation[1], \
        DP_DATA, imu.rotation[2], \
        DP_DATA, imu.rotation[3]);

        // Airspeed/Pitot Tube Data
        Serial.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,", pitot.sensor_id, \
        DP_DATA, pitot.diff_pressure[0], \
        DP_DATA, pitot.diff_pressure[1], \
        DP_DATA, pitot.airspeed[0], \
        DP_DATA, pitot.airspeed[1], \
        DP_DATA, pitot.temperature);
        // GPS Data
        Serial.printf("%u,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u\n", gps.sensor_id, \
        DP_GPS, gps.latitude, \
        DP_GPS, gps.longitude,
        DP_DATA, gps.gnd_speed, \
        DP_GPS, gps.altitude, \
        gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

        Serial.flush();
        /*
        // Line Num + ESP Time + IMU Data
        datafile.printf("%lu,%lu,%u,%.*f,%.*f,%.*f,%.*f,", line_num, micros()-start_time, imu.sensor_id, \
        DP_DATA, imu.acceleration[0], \
        DP_DATA, imu.gyro[0], \
        DP_DATA, imu.magnetic[0], \
        DP_DATA, imu.rotation[0]);

        // Airspeed/Pitot Tube Data
        datafile.printf("%u,%.*f,%.*f,%.*f,", pitot.sensor_id, \
        DP_DATA, pitot.diff_pressure[0], \
        DP_DATA, pitot.airspeed[0], \
        DP_DATA, pitot.temperature);

        // GPS Data
        datafile.printf("%u,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u\n", gps.sensor_id, \
        DP_GPS, gps.latitude, \
        DP_GPS, gps.longitude,
        DP_DATA, gps.gnd_speed, \
        DP_GPS, gps.altitude, \
        gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

        datafile.flush();
        */

        // Resume suspended reading tasks after logging data to SD card and loop again.
        line_num++;
        if(led_on) {
            led_on = false;
            digitalWrite(BUILTIN_LED_PIN, LOW);
        }
        else {
            led_on = true;
            digitalWrite(BUILTIN_LED_PIN, HIGH);
        }
    }
}

