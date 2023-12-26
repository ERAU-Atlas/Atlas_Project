#include <Arduino.h>
// #include <AnalogIO.h>

// void setup_pwm(void);

void setup() {
    pinMode(2, OUTPUT);
    // setup_pwm();
}

void loop() {
    analogWrite(2, map(51, 0, 100, 0, 255)); // Example usage with input of 0.5
}


