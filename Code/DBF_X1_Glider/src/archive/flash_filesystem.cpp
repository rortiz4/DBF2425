#include <LittleFS.h> // Older/Deprecated alternative: SPIFFS.h
#define MAX_FILE_SIZE 4000000 // Max filesize is 4MB

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

unsigned long list_files(const char* dir_path) {
    unsigned long folder_size = 0;
    // Function to list files in the specified directory. MUST PROVIDE DIRECTORY AS PARAMETER, NOT FILE.
    File dir = LittleFS.open(dir_path);
    if (!dir || !dir.isDirectory()) {
        Serial.printf("Failed to open directory \"%s\" or directory doesn't exist\n", dir_path);
        return 0;
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
                folder_size += file.size();
                file = dir.openNextFile();
            }
        }
    }
    Serial.print("Total current directory size (excluding subfolders) = ");
    Serial.print(folder_size);
    Serial.println(" Bytes");
    return folder_size;
}

// Function to read data from a file in LittleFS
unsigned int read_file(const char* path, const char* file_contents, bool serial_output, bool var_output, int read_delay, bool read_complete) {
    Serial.printf("Reading from file: %s\n", path);
    char* file_contents_char_array = const_cast<char*>(file_contents);

    File file = LittleFS.open(path, "r");
    if (!file) {
        Serial.println("Failed to open file for reading");
        return 0;
    }

    if (var_output) {
        unsigned int index = 0;
        while (file.available() && index < MAX_FILE_SIZE - 1) { // Leave space for null terminator
            file_contents_char_array[index++] = (char)file.read();
        }
        file_contents_char_array[index] = '\0'; // Null-terminate the string
        file_contents = (const char*)file_contents_char_array;

        if (serial_output) {
            Serial.write(file_contents); // Print the content to Serial
        }
    }
    else if (serial_output) {
        while(file.available()) {
            char file_byte = (char)file.read();
            Serial.write(file_byte);
            if(file_byte == '\n') delay(read_delay);
        }
    }
    else {
        Serial.println("No Read Output Sink Specified. File reading skipped.");
        return 0;
    }
    
    
    unsigned int byte_count = file.size();
    file.close(); // Always close the file
    if (read_complete) Serial.println("\nRead complete!\n");
    return byte_count;
}

// Function to write data to a file in LittleFS
unsigned int write_file(const char* path, const char* message, bool add_newline) {
    Serial.printf("Writing to file: %s\n", path);
    File file = LittleFS.open(path, FILE_WRITE);
    if (!file) {
        Serial.println("Failed to open file for writing");
        return 0;
    }
    if (add_newline) {
        file.println(message);  // Write the message to the file
    }
    else {
        file.print(message);
    }
    unsigned int byte_count = file.size();
    file.close();  // Always close the file
    Serial.println("Write successful!");
    return byte_count;
}

unsigned int append_file(const char* path, const char* message, bool add_newline) {
    // If file does not exist, it will be created.
    Serial.printf("Appending to file: %s\n", path);
    File file = LittleFS.open(path, FILE_APPEND);
    if (!file) {
        File file = LittleFS.open(path, FILE_WRITE);
        if (!file) {
            Serial.println("Failed to open file for writing");
            return 0;
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
    unsigned int byte_count = file.size();
    file.close();  // Always close the file
    Serial.println("Append successful!");
    return byte_count;
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
    if (strcmp(dir_path, "/")) {
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
