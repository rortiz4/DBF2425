#include <Arduino.h>
#include "flash_filesystem.h"
#include "sensors.h"

#define DP 3 // Decimal places to record
#define SERIAL_BAUDRATE 115200
#define I2C_CLOCK_SPEED 100000 // 100 kHz
#define CONFIG_FILE_PATH "/config_DO_NOT_DELETE.sys" // File contains only filenumber of next file to be written
#define DATAFILE_LINE_BUFFER_SIZE 256
#define MAX_STORAGE_BYTES 2650000 // 4MB-1.32MB w/ extra buffer for config file in /, other folders (not /data), etc.

#define LOOP_DELAY 250 // milliseconds to wait between processing data.

#define LED_PIN 15

char datafile[32]; // Note that filename cannot be more than 31 +\0 characters long.
unsigned long line_num = 1;
unsigned long current_storage_used = 0;
bool led_state = false;

void init_serial_i2c() {
    Serial.begin(SERIAL_BAUDRATE);
    Wire.begin();
    Wire.setClock(I2C_CLOCK_SPEED);
}

void float_data_to_string(float* data, int data_size, char * output_string, bool serial_print=false) {
    sprintf(output_string, "%lu,%.*f,%.*f,%.*f,%.*f,%.*f\n", line_num, DP, data[0], DP, data[1], \
    DP, data[2], DP, data[3], DP, data[4]);
    if(serial_print) {
        Serial.print(output_string);
    }
}

void setup() {
    init_serial_i2c();
    pinMode(LED_PIN, OUTPUT);
    delay(3000);
    init_fs(); // Initialize littlefs
    init_ms4525do(); // Initialize Diff Pressure Sensor for Airspeed
    digitalWrite(LED_PIN, true);
    delay(2000);
    digitalWrite(LED_PIN, false);

    current_storage_used = list_files("/data");

    const char csv_header[] = "Line_No.,Corr_DP(Pa),Corr_Airspeed(m/s),DP(Pa),Airspeed(m/s),Temperature(C)\n";
    int current_datafile_number;
    int next_datafile_number;
    const char current_datafile_number_str[] = "000";
    const char next_datafile_number_str[] = "000";
    // Filenumber Management
    read_file(CONFIG_FILE_PATH, current_datafile_number_str, true, true); // Read string filenumber
    sscanf(current_datafile_number_str, "%d", &current_datafile_number); // Convert filenumber to int
    next_datafile_number = current_datafile_number;
    next_datafile_number++; // Increment filenumber for next reboot
    if (next_datafile_number > 999) {
        next_datafile_number = 0; // Loopback and overwrite
    }
    sprintf((char*)next_datafile_number_str, "%03d", next_datafile_number); // Convert int to padded string
    write_file(CONFIG_FILE_PATH, next_datafile_number_str); // Store Padded string in CONFIG file.

    current_storage_used += strlen(csv_header);
    if (current_storage_used > MAX_STORAGE_BYTES) {
        while(1) {
            Serial.println("STORAGE FULL! Please clear up some space.");
            digitalWrite(LED_PIN, false);
        }
    }

    sprintf((char*)datafile, "/data/data%03d.csv", current_datafile_number); // Create filepath
    write_file(datafile, csv_header, false); // Write file header
}

void loop() {
    char csv_output_string[DATAFILE_LINE_BUFFER_SIZE] = "";
    const int ms4525do_field_count = 5;
    float ms4525do_output[ms4525do_field_count] = {0.0}; // Initialize to 0 every loop time.
    read_ms4525do(ms4525do_output);
    float_data_to_string(ms4525do_output,ms4525do_field_count,csv_output_string, false);
    // Serial.println(datafile);
    // Serial.println(csv_output_string);
    current_storage_used += strlen(csv_output_string);
    if (current_storage_used > MAX_STORAGE_BYTES) {
        while(1) {
            Serial.println("STORAGE FULL! Please clear up some space.");
            digitalWrite(LED_PIN, false);
        }
    }
    append_file(datafile, csv_output_string, false);
    line_num++;
    led_state = ~led_state;
    digitalWrite(LED_PIN, led_state);
    delay(LOOP_DELAY);
}