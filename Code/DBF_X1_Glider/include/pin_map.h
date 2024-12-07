#ifndef PIN_MAP_H
#define PIN_MAP_H

// For datalogger (SPI pins)
#define SD_CS 5
#define SD_MISO 25
#define SD_MOSI 26
#define SD_SCK 27

// For all sensors (I2C pins)
#define SDA_PIN 21
#define SCL_PIN 22

// Outputs (LEDs+Servos)
#define BUILTIN_LED_PIN 2

#define RELEASE_DET_PIN 19
#define STROBE_LED_PIN 20

// Separate left and right servo pin defs
#define SERVO_L_PIN 13
#define SERVO_R_PIN 14

#endif