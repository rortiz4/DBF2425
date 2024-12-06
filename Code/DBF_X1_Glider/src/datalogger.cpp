// This file contains all SD card related functions to initialize and write to the SD Card
#include <SPI.h>
#include <SD.h>
#include "pin_map.h"
#include "datalogger.h"
#include "sensors.h"
#include "autopilot.h"
#include "pitcheron_servos.h"
#include "tasks.h"
#include "queues.h"
#include "semaphores.h"

#define FILE_COUNT_START 0
#define INIT_DELAY_SD 100
#define LINE_NUM_START 1
#define DP_DATA 3 // Decimal places to record
#define DP_GPS 6 // Latitude/Longitude decimal places
#define GPS_FIX_DELAY_THRESHOLD 0.250 // If more than 250ms passed since last fix, take data from other sensors again

bool log_to_serial;
bool log_to_SD;
File datafile;                              // File object to handle file writing
SPIClass mySPI(VSPI);

void init_SD(bool serial_log, bool SD_log) {
    if (serial_log) log_to_serial = true;
    else log_to_serial = false;
    if (SD_log) log_to_SD = true;
    else log_to_SD = false;
    if (!log_to_SD) return;
    unsigned int file_counter = FILE_COUNT_START;        // Start the file counter at 1
    char filename[16];
    Serial.print("Initializing SD Card...");
    pinMode(SD_CS, OUTPUT);
    digitalWrite(SD_CS, HIGH);
    mySPI.begin(SD_SCK, SD_MISO, SD_MOSI, SD_CS);
    // SPI.setFrequency(1000000);
    while (!SD.begin(SD_CS, mySPI, 80000000)) {
        Serial.println("Card failed, or not present. Retrying..."); // Retry if SD card can't be initialized
        delay(500);
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
    const char csv_header[] =   "Line_Num,ESP32_Time_s,ID0_IMU,LinAcc_x,LinAcc_y,LinAcc_z,Pitch,Roll,Yaw,Gyro_x,Gyro_y,Gyro_z,Magnet_uT_x,Magnet_uT_y,Magnet_uT_z,"
                                "Grav_x,Grav_y,Grav_z,Quat_re,Quat_i,Quat_j,Quat_k,ID1_ASPD,RawPress_Pa,temp_C,RawAirspeed,CorrAirspeed,"
                                "ID2_GPS,latitude,longitude,heading,gnd_speed,altitude,hours,mins,secs,hundredths,satellites,"
                                "ID3_AP,AP_Mode,AP_HDGSEL,AP_PITCH,"
                                "ID4_SERVO,servo_L_angle,servo_R_angle,servo_angle_target,servo_action_target\n";

    Serial.println("Writing .csv header:");
    datafile.print(csv_header);
    datafile.flush(); 
    if (log_to_serial) {
        Serial.print(csv_header);
    }
    Serial.println(".csv Header Writing DONE!");
}

void log_data(void* pvParameters) {
    float current_time = 0;
    pinMode(BUILTIN_LED_PIN, OUTPUT);
    IMU_Data imu;
    Airspeed_Data pitot;
    GPS_Data gps;
    Flight_Data ap_input_data;
    Autopilot_Data AP_data;
    Pitcheron_Data pitcherons;
    digitalWrite(BUILTIN_LED_PIN, LOW);
    bool led_on = false;
    unsigned long line_num = LINE_NUM_START; // These are only initialized once
    while(true) {
        static unsigned long start_time = micros();
        // Serial.println("Taking GPS Semaphore");
        // xSemaphoreTake(gps_done, portMAX_DELAY);
        xQueueReceive(GPS_Queue, &gps, portMAX_DELAY);

        // Serial.println("Taking IMU Semaphore");
        // xSemaphoreTake(imu_done, portMAX_DELAY);
        xQueueReceive(IMU_Queue, &imu, portMAX_DELAY);

        // Serial.println("Taking Airspeed Semaphore");
        // xSemaphoreTake(airspeed_done, portMAX_DELAY);
        xQueueReceive(Airspeed_Queue, &pitot, portMAX_DELAY);

        current_time = (float)((micros() - start_time)/1000000.0);
        // Load received data onto Flight_Data_Queue for Autopilot
        ap_input_data.time = current_time;
        ap_input_data.pitch = imu.euler[0];
        ap_input_data.roll = imu.euler[1];
        ap_input_data.yaw = imu.euler[2];
        ap_input_data.airspeed = pitot.airspeed[1]; // Corrected airspeed
        ap_input_data.gnd_speed = gps.gnd_speed;
        ap_input_data.latitude = gps.latitude;
        ap_input_data.longitude = gps.longitude;
        ap_input_data.heading = gps.heading;

        xQueueSend(Flight_Data_Queue, &ap_input_data, portMAX_DELAY);

        // Serial.println("Taking Autopilot Semaphore");
        xQueueReceive(Autopilot_Queue, &AP_data, portMAX_DELAY);
        xQueueReceive(Pitcheron_Queue, &pitcherons, portMAX_DELAY);


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
            datafile.printf("%u,%.*f,%.*f,%.*f,%.*f,", pitot.sensor_id, \
            DP_DATA, pitot.diff_pressure, \
            DP_DATA, pitot.temperature, \
            DP_DATA, pitot.airspeed[0], \
            DP_DATA, pitot.airspeed[1]);

            // GPS Data
            datafile.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u,", gps.sensor_id, \
            DP_GPS, gps.latitude, \
            DP_GPS, gps.longitude, \
            DP_GPS, gps.heading, \
            DP_DATA, gps.gnd_speed, \
            DP_GPS, gps.altitude, \
            gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

            // Autopilot Data
            datafile.printf("%u,%s,%.*f,%.*f,", AP_data.sensor_id, \
            AP_data.ap_mode, DP_DATA, AP_data.ap_target_bearing, DP_DATA, AP_data.ap_target_pitch);

            // Servo Data
            datafile.printf("%u,%d,%d,%d,%s\n", pitcherons.sensor_id, \
            pitcherons.raw_angle_l, \
            pitcherons.raw_angle_r, \
            pitcherons.angle_target, \ 
            pitcherons.action_target); // Do not use comma in action target String!

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

            // Serial.flush();

            // Airspeed/Pitot Tube Data
            Serial.printf("%u,%.*f,%.*f,%.*f,%.*f,", pitot.sensor_id, \
            DP_DATA, pitot.diff_pressure, \
            DP_DATA, pitot.temperature, \
            DP_DATA, pitot.airspeed[0], \
            DP_DATA, pitot.airspeed[1]);
            // Serial.flush();

            // GPS Data
            Serial.printf("%u,%.*f,%.*f,%.*f,%.*f,%.*f,%u,%u,%u,%u,%u,", gps.sensor_id, \
            DP_GPS, gps.latitude, \
            DP_GPS, gps.longitude, \
            DP_DATA, gps.heading, \
            DP_GPS, gps.gnd_speed, \
            DP_DATA, gps.altitude, \
            gps.hours, gps.minutes, gps.seconds, gps.hundredths, gps.satellites);

            // Autopilot Data
            Serial.printf("%u,%s,%.*f,%.*f,", AP_data.sensor_id, \
            AP_data.ap_mode, DP_DATA, AP_data.ap_target_bearing, DP_DATA, AP_data.ap_target_pitch);

            // Servo Data
            Serial.printf("%u,%d,%d,%d,%s\n", pitcherons.sensor_id, \
            pitcherons.raw_angle_l, \
            pitcherons.raw_angle_r, \
            pitcherons.angle_target, \ 
            pitcherons.action_target); // Do not use comma in action target String!
            
            // Serial.flush();
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
        //vTaskDelay(250/portTICK_PERIOD_MS); // Extra Delay (in milliseconds) to make serial monitor data human-readable. Too fast otherwise!
        vTaskResume(read_gps_task);
        vTaskResume(read_imu_task);
        vTaskResume(read_pitot_task);
    }
}

