#include <Arduino.h>
#define LED 15

void setup() {
  // put your setup code here, to run once:
    pinMode(LED, OUTPUT);
    Serial.begin(115200);
    delay(5000); // Allows time to open the serial monitor on UART after programming.
}

void loop() {
    Serial.println("Daniel Noronha");
    digitalWrite(LED, HIGH);
    Serial.println("LED is on");
    delay(2000);
    digitalWrite(LED, LOW);
    Serial.println("LED is off");
    delay(2000);
}

