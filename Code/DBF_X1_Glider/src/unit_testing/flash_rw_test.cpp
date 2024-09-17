#include <LittleFS.h> // Older/Deprecated alternative: SPIFFS.h
#include "flash_filesystem.h"

void setup() {
  Serial.begin(115200);
  // Initialize LittleFS
  init_fs();
}

void loop() {
  // Call the write and read functions in the loop
  write_file("/test_data.txt", "Hello, ESP32 with LittleFS!");
  delay(1000);  // Small delay to simulate real-time usage
  append_file("/test_data.txt", "Appended: Hello, ESP32 with LittleFS!");
  delay(1000);
  read_file("/test_data.txt");
  delay(5000);  // Delay between reading/writing
}

