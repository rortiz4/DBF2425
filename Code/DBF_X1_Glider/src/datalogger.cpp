// This file contains all SD card related functions to initialize and write to the SD Card
#include <SPI.h>
#include <SD.h>
#include "datalogger.h"
#include "sensors.h"
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"

#define SD_CS_PIN 5
#define FILE_COUNT_START 0
#define INIT_DELAY_SD 100
#define LINE_NUM_START 1
#define DP_DATA 3 // Decimal places to record
#define DP_GPS 6 // Latitude/Longitude decimal places
#define GPS_FIX_DELAY_THRESHOLD 0.100 // If more than 100ms passed since last fix, take data from other sensors again

#define BUILTIN_LED_PIN 15

bool log_to_serial = true;
bool log_to_SD = false;
float current_time = 0;
File datafile;                              // File object to handle file writing

void init_SD(bool serial_log, bool SD_log) {
    if (serial_log) log_to_serial = true;
    else log_to_serial = false;
    if (SD_log) log_to_SD = true;
    else log_to_SD = false;
    if (!log_to_SD) return;
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
    sprintf(filename, "/data%03u.csv", file_counter); // Create the filename with the counter
    while(SD.exists(filename)) { 
        file_counter++;
        if (file_counter >= 1000) {
            file_counter = FILE_COUNT_START;
            Serial.println("Overwriting file /data000.csv... 1000+ files in storage. Please DELETE SOME!!!");
            sprintf(filename, "/data%03u.csv", file_counter); // Create the filename with the counter
            break;
        }
        else sprintf(filename, "/data%03u.csv", file_counter); // Create the filename with the counter
    }

    datafile = SD.open(filename, FILE_WRITE);
    // Open File
    if (datafile) {
        Serial.printf("Writing to file: /data%03u.csv\n", file_counter);
    }
    else {
        Serial.printf("Failed to Open File: /data%03u.csv for writing!\n", file_counter);
        while(!datafile) {
            Serial.println("Retrying file open...");
            delay(INIT_DELAY_SD);
            datafile = SD.open(filename, FILE_WRITE);
        }
    }
    // Write Header to file
    const char csv_header[] =   "Line_Num,ESP32_Time,ID0,LinAcc_x,LinAcc_y,LinAcc_z,Pitch_deg,Roll_deg,Yaw_deg,Gyro_deg_x,Gyro_deg_y,Gyro_deg_z,Magnet_uT_x,Magnet_uT_y,Magnet_uT_z,"
                                "Grav_x,Grav_y,Grav_z,Quat_re,Quat_i,Quat_j,Quat_k,ID1,RawPress_Pa,CorrPressPa,RawAirspeed,CorrAirspeed,temp_C,"
                                "ID2,latitude,longitude,heading,gnd_speed,altitude,hours,mins,secs,hundredths,satellites\n";

    Serial.println("Writing .csv header:");
    datafile.print(csv_header);
    datafile.flush(); 
    if (log_to_serial) {
        Serial.print(csv_header);
        Serial.flush();
    }
    Serial.println(".csv Header Writing DONE!");
}

void log_data(void* pvParameters) {
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    IMU_Data imu;
    Airspeed_Data pitot;
    GPS_Data gps;
    digitalWrite(BUILTIN_LED_PIN, LOW);
    bool led_on = false;
    unsigned long line_num = LINE_NUM_START; // These are only initialized once
    Serial.flush();
    while(true) {
        // See queues.h for definition of structs

        static unsigned long start_time = micros();
        float prev_time = 0;
        float almost_current_time = 0;
        // Check GPS first since that is the slowest (others should already be taken)
        prev_time = (float)((micros() - start_time)/1000000.0);
        xSemaphoreTake(gps_done, portMAX_DELAY);
        almost_current_time = (float)((micros() - start_time)/1000000.0); // Convert us (micros) to s
        xQueueReceive(GPS_Queue, &gps, portMAX_DELAY);
        vTaskResume(read_gps_task);

// If GPS regains fix after a long time (more than 1s), take IMU/Airspeed readings twice
        if ((almost_current_time - prev_time) > GPS_FIX_DELAY_THRESHOLD) {
            Serial.printf("GPS fix lost, then regained after %f seconds!\n", (almost_current_time - prev_time));
            xSemaphoreTake(imu_done, portMAX_DELAY);
            xQueueReceive(IMU_Queue, &imu, portMAX_DELAY);
            vTaskResume(read_imu_task);
            xSemaphoreTake(imu_done, portMAX_DELAY);
            xQueueReceive(IMU_Queue, &imu, portMAX_DELAY);
            vTaskResume(read_imu_task);
            
            xSemaphoreTake(airspeed_done, portMAX_DELAY);
            xQueueReceive(Airspeed_Queue, &pitot, portMAX_DELAY);
            vTaskResume(read_pitot_task);
            xSemaphoreTake(airspeed_done, portMAX_DELAY);
            xQueueReceive(Airspeed_Queue, &pitot, portMAX_DELAY);
            vTaskResume(read_pitot_task);
        }
        else {
            xSemaphoreTake(imu_done, portMAX_DELAY);
            xQueueReceive(IMU_Queue, &imu, portMAX_DELAY);
            vTaskResume(read_imu_task);

            xSemaphoreTake(airspeed_done, portMAX_DELAY);
            xQueueReceive(Airspeed_Queue, &pitot, portMAX_DELAY);
            vTaskResume(read_pitot_task);
        }

        current_time = (float)((micros() - start_time)/1000000.0);

        // Log Data to datafile
        if (log_to_SD) {
            // Line Num + ESP Time + IMU Data
            datafile.printf("%lu,%.*f,%u,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,", line_num, 6, current_time, imu.sensor_id, \
            DP_DATA, imu.lin_accel[0], \
            DP_DATA, imu.lin_accel[1], \
            DP_DATA, imu.lin_accel[2], \
            DP_DATA, imu.euler[0], \
            DP_DATA, imu.euler[1], \
            DP_DATA, imu.euler[2], \
            DP_DATA, imu.gyro[0], \
            DP_DATA, imu.gyro[1], \
            DP_DATA, imu.gyro[2], \
            DP_DATA, imu.magnetic[0], \
            DP_DATA, imu.magnetic[1], \
            DP_DATA, imu.magnetic[2], \
            DP_DATA, imu.gravity[0], \
            DP_DATA, imu.gravity[1], \
            DP_DATA, imu.gravity[2], \
            DP_DATA, imu.rotation[0], \
            DP_DATA, imu.rotation[1], \
            DP_DATA, imu.rotation[2], \
            DP_DATA, imu.rotation[3]);

            // Airspeed/Pitot Tube Data
            datafile.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,", pitot.sensor_id, \
            DP_DATA, pitot.diff_pressure[0], \
            DP_DATA, pitot.diff_pressure[1], \
            DP_DATA, pitot.airspeed[0], \
            DP_DATA, pitot.airspeed[1], \
            DP_DATA, pitot.temperature);
            // GPS Data
            datafile.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u\n", gps.sensor_id, \
            DP_GPS, gps.latitude, \
            DP_GPS, gps.longitude, \
            DP_GPS, gps.heading, \
            DP_DATA, gps.gnd_speed, \
            DP_GPS, gps.altitude, \
            gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

            datafile.flush();
        }
        
        if (log_to_serial) {
            // Line Num + ESP Time + IMU Data
            Serial.printf("%lu,%.*f,%u,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,%.*f,", line_num, 6, current_time, imu.sensor_id, \
            DP_DATA, imu.lin_accel[0], \
            DP_DATA, imu.lin_accel[1], \
            DP_DATA, imu.lin_accel[2], \
            DP_DATA, imu.euler[0], \
            DP_DATA, imu.euler[1], \
            DP_DATA, imu.euler[2], \
            DP_DATA, imu.gyro[0], \
            DP_DATA, imu.gyro[1], \
            DP_DATA, imu.gyro[2], \
            DP_DATA, imu.magnetic[0], \
            DP_DATA, imu.magnetic[1], \
            DP_DATA, imu.magnetic[2], \
            DP_DATA, imu.gravity[0], \
            DP_DATA, imu.gravity[1], \
            DP_DATA, imu.gravity[2], \
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
            Serial.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u\n", gps.sensor_id, \
            DP_GPS, gps.latitude, \
            DP_GPS, gps.longitude, \
            DP_DATA, gps.heading, \
            DP_GPS, gps.gnd_speed, \
            DP_DATA, gps.altitude, \
            gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

            Serial.flush();
        }

        line_num++;
        // Resume suspended reading tasks after logging data to SD card and loop again.
        if(led_on) {
            led_on = false;
            digitalWrite(BUILTIN_LED_PIN, LOW);
        }
        else {
            led_on = true;
            digitalWrite(BUILTIN_LED_PIN, HIGH);
        }
        // vTaskDelay(500/portTICK_PERIOD_MS); // Extra Delay to make serial monitor data human-readable. Too fast otherwise!
    }
}

