#include <Servo.h>
#include <limits.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Muscle sensor 1
#define EMG_PIN_1 A5
#define THRESHOLD_1 100 // Adjust as needed

// Muscle sensor 2
#define EMG_PIN_2 A4
#define THRESHOLD_2 100 // Adjust as needed

// Servo 1
#define SERVO_PIN_1 3
Servo SERVO_1;

// Servo 2
#define SERVO_PIN_2 9
Servo SERVO_2;

// Linked list variables (shared for both sensors)
struct node
{
    int data;
    struct node *next;
};
struct node *head = NULL;
struct node *tail = NULL;

// Linked list functions
void insertatend(int data)
{
    struct node *lk = (struct node *)malloc(sizeof(struct node));
    lk->data = data;
    lk->next = NULL;

    if (head == NULL)
    {
        head = lk;
        tail = lk;
    }
    else
    {
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

// Calibration variables
int loopCount = 0;
int caliberationTime = 1000;
int caliberationValue = 2;

// Variables for sensor 1
int averageEMG_1 = 0;
long long threshold_1 = 0;
int windowSize_1 = 10;
int afterCalCount_1 = 0;
long long windowSum_1 = 0;

// Variables for sensor 2
int averageEMG_2 = 0;
long long threshold_2 = 0;
int windowSize_2 = 10;
int afterCalCount_2 = 0;
long long windowSum_2 = 0;

void setup()
{
    Serial.begin(115200);
    while (!Serial)
        ;
    Serial.println("MyoWare Example_01_analogRead_SINGLE");

    SERVO_1.attach(SERVO_PIN_1);
    SERVO_2.attach(SERVO_PIN_2);
}

void loop()
{
    // Sensor 1 readings
    int value_1 = analogRead(EMG_PIN_1);
    calibration(value_1, &threshold_1, &afterCalCount_1, &windowSum_1, windowSize_1, loopCount, caliberationTime, caliberationValue);
    controlServo(value_1, &windowSum_1, windowSize_1, threshold_1, &SERVO_1);

    // Sensor 2 readings
    int value_2 = analogRead(EMG_PIN_2);
    calibration(value_2, &threshold_2, &afterCalCount_2, &windowSum_2, windowSize_2, loopCount, caliberationTime, caliberationValue);
    controlServo(value_2, &windowSum_2, windowSize_2, threshold_2, &SERVO_2);

    // Serial monitoring
    Serial.print(value_1);
    Serial.print("\t");
    Serial.println(value_2);

    delay(10);
    loopCount++;
}

// Helper function for calibration
void calibration(int value, long long *threshold, int *afterCalCount, long long *windowSum, int windowSize, int loopCount, int caliberationTime, int caliberationValue)
{
    if (loopCount <= caliberationTime && loopCount > 500)
    {
        *threshold += value;
    }
    else if (loopCount == caliberationTime + 1)
    {
        *threshold /= (caliberationTime / 2);
        *threshold += caliberationValue;
    }
    else
    {
        insertatend(value);
        if (*afterCalCount <= windowSize)
        {
            *windowSum += value;
        }
        else
        {
            *windowSum -= head->data;
            *windowSum += value;
            deleteatbegin();
        }
        *afterCalCount++;
    }
}

// Helper function for servo control
void controlServo(int value, long long *windowSum, int windowSize, long long threshold, Servo *servo)
{
    int normalisedValue = *windowSum / windowSize;

    if (normalisedValue > threshold)
    {
        int angle = servo->read(); // Get current angle
        angle = angle + 10 > 90 ? 90 : angle + 10;
        servo->write(angle);
    }
    else
    {
        int angle = servo->read(); // Get current angle
        angle = angle - 10 > 0 ? angle - 10 : 0;
        servo->write(angle);
    }
    delay(50);
}
