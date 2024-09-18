#include <Arduino.h>
#include "flash_filesystem.h"

#define SERIAL_SPEED 115200         // Define serial communication speed
#define INPUT_BUFFER_SIZE 200       // Define memory to reserve for input string
#define COMMAND_LINE_TERMINATOR '\n' // Define the command line terminator
#define STARTING_DIRECTORY "/"       // Define the starting directory

String inputString = "";          // String to hold incoming data
bool stringComplete = false;      // Flag when input is complete

String currentDirectory = STARTING_DIRECTORY;  // Variable to keep track of the current directory

// Function to display help message and available commands
void help() {
    Serial.println("**********************************************************************");
    Serial.println("*                                                                    *");
    Serial.println("*               ESP32 LittleFS File Manager v2.0                     *");
    Serial.println("*                                                                    *");
    Serial.println("*                         By Daniel Noronha                          *");
    Serial.println("*                                                                    *");
    Serial.println("**********************************************************************");
  
    Serial.println();
    Serial.print("Available (case-insensitive) commands that must terminate with \\n:");

    Serial.println("pwd                     : Prints the current directory");
    Serial.println("                          Function Used: print_working_directory()");

    Serial.println("cd [</path/to/folder>]  : Changes the current directory");
    Serial.println("                          Function Used: change_directory(const char* dir_path)");

    Serial.println("ls [</path/to/folder>]  : Lists files in the specified directory or current directory if none provided");
    Serial.println("                          Function Used: list_files(const char* dir_path)");

    Serial.println("mkdir </path/to/folder> : Creates a directory at the specified path");
    Serial.println("                          Function Used: create_dir(const char* dir_path)");

    Serial.println("cat </path/to/file>     : Reads and prints the content of the file");
    Serial.println("                          Function Used: read_file(const char* path)");

    Serial.println("rm </path/to/file>      : Deletes the specified file");
    Serial.println("                          Function Used: delete_file(const char* path)");

    Serial.println("rmdir </path/to/folder> : Removes the specified directory");
    Serial.println("                          Function Used: delete_dir(const char* dir_path)");

    Serial.println("help                    : Displays this help message");
    Serial.println("                          Function Used: help()");

    Serial.println(); // Print a blank line
    Serial.print("Current directory: "); // Display the current directory
    Serial.println(currentDirectory);
    Serial.println(); // Print another blank line
}

// Function to print the current working directory
void print_working_directory() {
    Serial.print("Current directory: ");
    Serial.println(currentDirectory);
}

// Function to change the current directory
void change_directory(const char* dir_path) {
    if (strcmp(dir_path, "..") == 0) {
        // Simulate changing to the parent directory
        int lastSlash = currentDirectory.lastIndexOf('/');
        if (lastSlash > 0) {
            currentDirectory = currentDirectory.substring(0, lastSlash);
            if (currentDirectory.length() == 0) {
                currentDirectory = STARTING_DIRECTORY;
            }
        }
    } else {
        // Simulate changing to the specified directory
        if (dir_path[0] == '/') {
            currentDirectory = dir_path;  // Absolute path
        } else {
            currentDirectory += "/";
            currentDirectory += dir_path;  // Relative path
        }
    }
    Serial.print("Changed directory to: ");
    Serial.println(currentDirectory);
}

// Function to split the input string and process the command
void processCommand(String input) {
    // Convert command to lowercase and extract it
    int index = input.indexOf(' ');
    String commandStr = input.substring(0, index);
    commandStr.toLowerCase();
    const char* command = commandStr.c_str(); // Convert command to const char*

    // Extract argument (if any)
    String argument = input.substring(index + 1);
    const char* arg = argument.c_str(); // Convert argument to const char*

    // Process the command using switch-case
    if (strcmp(command, "mkdir") == 0) {
        if (argument.length() == 0) {
            Serial.println("ERROR: NO DIRECTORY PATH PROVIDED. PLEASE TRY AGAIN!");
        } else {
            create_dir(arg);
        }
    } else if (strcmp(command, "ls") == 0) {
        if (argument.length() > 0) {
            list_files(arg);
        } else {
            list_files(currentDirectory.c_str());
        }
    } else if (strcmp(command, "cat") == 0) {
        if (argument.length() == 0) {
            Serial.println("ERROR: NO FILE PATH PROVIDED. PLEASE TRY AGAIN!");
        } else {
            read_file(arg);
        }
    } else if (strcmp(command, "rm") == 0) {
        if (argument.length() == 0) {
            Serial.println("ERROR: NO FILE PATH PROVIDED. PLEASE TRY AGAIN!");
        } else {
            delete_file(arg);
        }
    } else if (strcmp(command, "rmdir") == 0) {
        if (argument.length() == 0) {
            Serial.println("ERROR: NO DIRECTORY PATH PROVIDED. PLEASE TRY AGAIN!");
        } else {
            delete_dir(arg);
        }
    } else if (strcmp(command, "cd") == 0) {
        if (argument.length() == 0) {
            // No argument provided, change to STARTING_DIRECTORY
            change_directory(STARTING_DIRECTORY);
        } else {
            change_directory(arg);
        }
    } else if (strcmp(command, "pwd") == 0) {
        print_working_directory();
    } else if (strcmp(command, "help") == 0) {
        help();
    } else {
        Serial.println("ERROR: UNKNOWN COMMAND PROVIDED. PLEASE TRY AGAIN!");
        Serial.println("Use help for a list of possible commands and usage.");
    }
}

// This function is called automatically whenever there is serial input
void serialEvent() {
    while (Serial.available()) {
        // Get the new byte
        char inChar = (char)Serial.read();

        // Add it to the input string
        inputString += inChar;

        // If the input ends with the command line terminator, mark the string as complete
        if (inChar == COMMAND_LINE_TERMINATOR) {
            stringComplete = true;
        }
    }
}

void setup() {
    // Start the serial communication
    Serial.begin(SERIAL_SPEED);
    inputString.reserve(INPUT_BUFFER_SIZE);  // Reserve memory to avoid dynamic allocation

    // Display help message and available commands
    help();
}

void loop() {
    // Check if the string is complete
    if (stringComplete) {
        // Process the command
        processCommand(inputString);

        // Clear the string and reset the flag
        inputString = "";
        stringComplete = false;
    }

    // Continue reading serial data
    serialEvent();
}
