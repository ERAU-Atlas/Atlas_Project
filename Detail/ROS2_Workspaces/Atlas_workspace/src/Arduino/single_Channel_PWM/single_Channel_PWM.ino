#include <Arduino.h>

void setup() {
    pinMode(2, OUTPUT);
}

void loop() {
    analogWrite(2, map(50, 0, 100, 0, 255)); // Example usage with input of 50%
}


