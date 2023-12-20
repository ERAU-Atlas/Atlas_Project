#include <Arduino.h>

const int motorPin = 9; // Pin connected to the motor

void setup() {
    pinMode(motorPin, OUTPUT);
}

void loop() {
    // Set the motor speed using PWM
    analogWrite(motorPin, 128); // Set PWM duty cycle to 50% (0-255 range)

    // Add any additional logic or control algorithms here

    delay(1000); // Delay for 1 second
}
