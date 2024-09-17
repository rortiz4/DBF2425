#include <LittleFS.h> // Older/Deprecated alternative: SPIFFS.h

void init_fs() {
  // Initialize LittleFS
  if (!LittleFS.begin(true)) {
    Serial.println("An error occurred while mounting LittleFS");
    return;
  }
  Serial.println("Flash Memory Initialized: LittleFS mounted successfully!");
}

// Function to read data from a file in LittleFS
void read_file(const char* path) {
  Serial.printf("Reading from file: %s\n", path);
  File file = LittleFS.open(path);
  if (!file) {
    Serial.println("Failed to open file for reading");
    return;
  }
  while (file.available()) {
    Serial.write(file.read());  // Print the content to Serial
  }
  file.close();  // Always close the file
  Serial.println("\nRead complete!");
}

// Function to write data to a file in LittleFS
void write_file(const char* path, const char* message) {
  Serial.printf("Writing to file: %s\n", path);
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for writing");
    return;
  }
  file.println(message);  // Write the message to the file
  file.close();  // Always close the file
  Serial.println("Write successful!");
}

void append_file(const char* path, const char* message) {
  // If file does not exist, it will be created.
  Serial.printf("Appending to file: %s\n", path);
  File file = LittleFS.open(path, FILE_WRITE);
  if (!file) {
    Serial.println("Failed to open file for appending");
    return;
  }
  file.seek(file.size());
  file.println(message);  // Write the message to the file
  file.close();  // Always close the file
  Serial.println("Append successful!");
}

void delete_file(const char* path) {
  if (LittleFS.remove(path)) {
    Serial.printf("File deleted: %s\n", path);
  } else {
    Serial.printf("Failed to delete file: %s\n", path);
  }
}

void clear_directory(const char* path) {
  File root = LittleFS.open(path);
  
  // Check if the directory opened successfully
  if (!root || !root.isDirectory()) {
    Serial.println("Failed to open directory or directory doesn't exist");
    return;
  }

  // Iterate through all files in the directory
  File file = root.openNextFile();
  while (file) {
    // Check if it's a regular file (not a subdirectory)
    if (!file.isDirectory()) {
      Serial.print("Deleting file: ");
      Serial.println(file.name());
      LittleFS.remove(file.name());  // Delete the file
    }
    file = root.openNextFile();  // Move to the next file
  }
}

/* Test Code
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
*/
