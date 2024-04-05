#include <Servo.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Threshold for servo motor control with muscle sensor.
// #define THRESHOLD 100

// Pin number where the sensor is connected. (Analog 0)
#define EMG_PIN 5

// Pin number where the servo motor is connected. (Digital PWM 3)
#define SERVO_PIN 3
int f = 0;
int angle = 0;

// Define Servo motor
Servo SERVO_1;

struct node
{
    int data;
    struct node *next;
};

struct node *head = NULL;
struct node *tail = NULL;

void insertatend(int data)
{
    // create a link
    struct node *lk = (struct node *)malloc(sizeof(struct node));
    lk->data = data;
    lk->next = NULL;

    if (head == NULL)
    {
        // if list is empty, this new node is the first node
        head = lk;
        tail = lk;
    }
    else
    {
        // otherwise, append the new node at the end and update the tail
        tail->next = lk;
        tail = lk;
    }
}

void deleteatbegin()
{
    if (head != NULL)
    {
        struct node *temp = head;
        head = head->next;
        free(temp);
    }
}

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

int loopCount = 0;
int caliberationTime = 1000;
int caliberationValue = 100;
int averageEMG = 0;
long long threshold = 0;

int windowSize = 100;
int afterCalCount = 0;
long long windowSum = 0;

void loop()
{
    int value = analogRead(EMG_PIN);
    if (loopCount <= caliberationTime)
    {
        threshold += value;
    }
    else if (loopCount == caliberationTime + 1)
    {
        threshold /= caliberationTime;
        threshold += caliberationValue;
    }
    else
    {
        insertatend(value);
        if (afterCalCount <= windowSize)
        {
            windowSum += value;
        }

        else
        {
            windowSum -= head->data;
            windowSum += value;
            deleteatbegin();

            int normalisedValue = windowSum / windowSize;

            // If the sensor value is GREATER than the THRESHOLD
            if (normalisedValue > threshold && f == 0)
            {
                angle = angle + 10 > 90 ? 90 : angle + 10;
                SERVO_1.write(angle);
            }

            // If the sensor is LESS than the THRESHOLD
            else
            {
                //  f = 0;
                angle = angle - 10 > 0 ? angle - 10 : 0;
                SERVO_1.write(angle);
            }
            delay(100);

            // update f from input pin
        }

        afterCalCount++;
    }
    // You can use serial monitor to set THRESHOLD properly, comparing the values shown when you open and close your hand.
    Serial.println(value);
    delay(10);
    loopCount++;
}
