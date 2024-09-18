#include <LittleFS.h> // Older/Deprecated alternative: SPIFFS.h

void init_fs() {
    // Initialize LittleFS
    if (!LittleFS.begin(true)) {
        Serial.println("An error occurred while mounting LittleFS");
        return;
    }
    Serial.println("Flash Memory Initialized: LittleFS mounted successfully!");
}
// Function to create directories
void create_dir(const char* dir_path) {
  if (LittleFS.mkdir(dir_path)) {
    Serial.print("Directory created: ");
    Serial.println(dir_path);
  } else {
    Serial.print("Failed to create directory: ");
    Serial.println(dir_path);
  }
}

void list_files(const char* dir_path) {
    // Function to list files in the specified directory. MUST PROVIDE DIRECTORY AS PARAMETER, NOT FILE.
    File dir = LittleFS.open(dir_path);
    if (!dir || !dir.isDirectory()) {
        Serial.printf("Failed to open directory \"%s\" or directory doesn't exist\n", dir_path);
        return;
    }
    else {
        File file = dir.openNextFile();
        Serial.print("Listing files & folders in directory: ");
        Serial.println(dir_path);
        while (file) {
            if (file.isDirectory()) {
                Serial.print("Folder: ");
                Serial.println(file.name());
                file = dir.openNextFile();
            }
            else {
                Serial.print("File: ");
                Serial.print(file.name());
                Serial.print(" | Size (Bytes): ");
                Serial.println(file.size());
                file = dir.openNextFile();
            }
        }
    }
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
    Serial.println("\nRead complete!\n");
}

// Function to write data to a file in LittleFS
void write_file(const char* path, const char* message, bool add_newline) {
    Serial.printf("Writing to file: %s\n", path);
    File file = LittleFS.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return;
    }
    if (add_newline) {
        file.println(message);  // Write the message to the file
    }
    else {
        file.print(message);
    }
    file.close();  // Always close the file
    Serial.println("Write successful!");
}

void append_file(const char* path, const char* message, bool add_newline) {
    // If file does not exist, it will be created.
    Serial.printf("Appending to file: %s\n", path);
    File file = LittleFS.open(path, FILE_APPEND);
    if (!file) {
        File file = LittleFS.open(path, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open file for writing");
            return;
        }
        Serial.println("File does not exist, but will be created for writing.");
    }
    // file.seek(file.size());
    if (add_newline) {
        file.println(message);  // Write the message to the file
    }
    else {
        file.print(message);
    }
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

void delete_dir(const char* dir_path) {
    File root = LittleFS.open(dir_path);
    
    // Check if the directory opened successfully
    if (!root) {
        Serial.printf("Failed to open directory \"%s\" or directory doesn't exist\n", dir_path);
        return;
    }

    Serial.printf("Clearing Directory: \"%s\"\n", dir_path);
    
    // Iterate through all files in the directory
    File file = root.openNextFile();
    while (file) {
        String filePath = String(dir_path) + "/" + file.name();
        if (!file.isDirectory()) {
            file.close();
            delete_file(filePath.c_str());
            file = root.openNextFile();
        }
        else {
            file.close();
            delete_dir(filePath.c_str());
        }
    }

    root.close();

    // Once all contents are deleted, remove the empty directory (exception: '/' root directory which will always remain)
    if (dir_path != "/") {
        if (LittleFS.rmdir(dir_path)) {
            Serial.printf("Deleted directory: \"%s\"\n", dir_path);
        } else {
            Serial.printf("Failed to delete directory: \"%s\"\n", dir_path);
        }
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
