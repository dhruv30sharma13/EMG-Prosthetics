#include <Servo.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>

// Threshold for servo motor control with muscle sensor.
#define THRESHOLD 100

// Pin number where the sensor is connected. (Analog 0)
#define EMG_PIN 5

// Pin number where the servo motor is connected. (Digital PWM 3)
#define SERVO_PIN 3
int f = 0;
int angle = 0;

// Define Servo motor
Servo SERVO_1;

void setup()
{

    // BAUDRATE set to 115200
    Serial.begin(115200);
    while (!Serial)
        ; // optionally wait for serial terminal to open
    Serial.println("MyoWare Example_01_analogRead_SINGLE");

    // Set servo motor to digital pin 3
    SERVO_1.attach(SERVO_PIN);
}

void loop()
{
    // Moving average
    int value = 0;
    int average_window = 1000 for (int i = 0; i < average_window; i++)
    {
        value += analogRead(EMG_PIN);
        delay(1);
    }
    value /= average_window;

    // If the sensor value is GREATER than the THRESHOLD
    if (value > THRESHOLD && f == 0)
    {
        int angle = anlge + 10 > 90 ? 90 : angle + 10;
        SERVO_1.write(angle);
        delay(100);
    }

    // If the sensor is LESS than the THRESHOLD
    else
    {
        //  f = 0;
        angle = angle - 10 > 0 ? angle - 10 : 0;
        SERVO_1.write(angle);
        delay(100);
    }

    // You can use serial monitor to set THRESHOLD properly, comparing the values shown when you open and close your hand.
    Serial.println(value);

    // update f from input pin
}