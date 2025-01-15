#include <Arduino.h>
#include "flash_filesystem.h"

void setup() {
    Serial.begin(115200);
    // Initialize LittleFS
    init_fs();
    Serial.println();
    list_files("/");
    Serial.println();
    
    write_file("/testfile.txt", "ABC");
    read_file("/testfile.txt");
    append_file("/testfile.txt", "DEFGHIJK", false);
    read_file("/testfile.txt");

    create_dir("/testfolder");
    Serial.println();
    write_file("/testfolder/testfile1.txt", "ABC");
    Serial.println();
    write_file("/testfolder/testfile2.txt", "ABCD");
    Serial.println();
    write_file("/testfolder/testfile3.txt", "ABCDE");
    Serial.println();
    create_dir("/testfolder/testsubfolder");
    write_file("/testfolder/testsubfolder/testfile3.txt", "ABCDEF");
    Serial.println();
    list_files("/");
    Serial.println();
    list_files("/testfolder");
    Serial.println();
    list_files("/testfolder/testsubfolder");
    Serial.println();
    read_file("/testfolder/testfile1.txt");
    delete_file("/testfolder/testfile1.txt");
    Serial.println();
    list_files("/testfolder");
    delete_dir("/testfolder");
    Serial.println();
    list_files("/");
    read_file("/testfile.txt");
    delete_dir("/");
    list_files("/");
}

void loop() {
    // Call the write and read functions in the loop
    //list_files("/");
    //write_file("/test_data.txt", "Hello, ESP32 with LittleFS!ABCDE");
    //delay(1000);  // Small delay to simulate real-time usage
    //append_file("/test_data.txt", "Appended: Hello, ESP32 with LittleFS!");
    //delay(1000);
    //read_file("/test_data.txt");
    //delay(5000);  // Delay between reading/writing
}

