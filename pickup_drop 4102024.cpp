#include <Arduino.h>
#include "EMGFilters.h"

#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

//  Analog 0 sensor input
#define SensorInputPin A0
#define SERVOMIN  150 
#define SERVOMAX  600 
#define STOP 375
#define SERVO_FREQ 50 

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
EMGFilters myFilter;

SAMPLE_FREQUENCY sampleRate = SAMPLE_FREQ_1000HZ;
NOTCH_FREQUENCY humFreq = NOTCH_FREQ_50HZ;

struct Joint180 {
    int pin;
    int min_pos;
    int max_pos;
    int cur_pos;
    bool active;
};

struct Joint360 {
    int pin;
    int cur_state;
    bool active;
};

//  creating different joints for arm
Joint180 shoulderR;
Joint180 shoulderUD;

Joint360 elbow1;
Joint360 elbow2;

Joint180 wrist;
Joint180 gripper;

//  Time interval for processing the input signal
unsigned long interval = 1000000UL / static_cast<unsigned long>(sampleRate);

//  Circular buffer for moving average
const int winSize = 100;
int buffer[winSize];
int bufferIndex = 0;
long winSum = 0;

void setup() {
    myFilter.init(sampleRate, humFreq, true, true, true);
    Serial.begin(115200);

    //  Initialize circular buffer
    for (int i = 0; i < winSize; i++) {
        buffer[i] = 0;
    }

    //  Setup of motor
    pwm.begin();
    pwm.setPWMFreq(SERVO_FREQ);

    /*
        initializing joints
        angles correct as per arm setup on 4-10-2024
    */
    shoulderR.pin = 0; shoulderR.min_pos = 40; shoulderR.max_pos = 90; shoulderR.active = true;
    shoulderUD.pin = 1; shoulderUD.min_pos = 20; shoulderUD.max_pos = 60; shoulderUD.active = true;
    // elbow1.pin = 3; elbow1.active = true;
    // elbow2.pin = 3; elbow2.active = true;
    wrist.pin = 3; wrist.max_pos = 90; wrist.min_pos = -30; wrist.active = true;
    gripper.pin = 4; gripper.min_pos = 80; gripper.max_pos = 140; gripper.active = true;

    shoulderR.cur_pos = shoulderR.min_pos;
    shoulderUD.cur_pos = shoulderUD.min_pos;
}

//  functions for motor drive

void setServoAngle(uint8_t servoNum, int angle) {
    //  set motor to angle "angle"
    int pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);
    pwm.setPWM(servoNum, 0, pulseWidth);
}

void sweep_joint(Joint180 &joint, int target_angle) {
    /*
        set 180 motor to "target_angle"
        constrain between max and min of that joint
        delay
    */
    target_angle = constrain(target_angle, joint.min_pos, joint.max_pos);

    if (joint.active) {
        setServoAngle(joint.pin, target_angle);  // Set servo on channel for joint
        delay(10);

        joint.cur_pos = target_angle;
    }
}

//  as it is functions to drive 360 motors
void sweep_cont1(Joint360 joint, int fwd_time, int stop_time, int bwd_time, int dir = 0) {
  // forward
  // -- t0/t2 = 1000/725 -- //
  if (!joint.active)
  return;

  pwm.setPWM(joint.pin, 0, SERVOMAX);
  delay(0);

  pwm.setPWM(joint.pin, 0, SERVOMIN);
  delay(fwd_time);

  // stop
  pwm.setPWM(joint.pin, 0, 0);
  delay(stop_time);

  // // backward
  // pwm.setPWM(joint.pin, 0, SERVOMIN);
  // delay(bwd_time);

  // // stop
  // pwm.setPWM(joint.pin, 0, 0);
  // delay(stop_time);
}

void sweep_cont2(Joint360 joint, int fwd_time, int stop_time, int bwd_time, int dir = 0) {
  // forward
  // -- t0/t2 = 1000/725 -- //
  if (!joint.active)
  return;

  // pwm.setPWM(joint.pin, 0, SERVOMAX);
  // // delay(1);

  // // stop
  pwm.setPWM(joint.pin, 0, STOP);
  delay(int(1.5*bwd_time));

  // backward
  pwm.setPWM(joint.pin, 0, SERVOMIN);
  delay(0);

  // stop
  // pwm.setPWM(joint.pin, 0, STOP);
  // delay(stop_time);

  pwm.setPWM(joint.pin, 0, 0);
  delay(stop_time);
}

//  threshold detection limit
float threshold = 3.0;

void loop() {
    unsigned long timeStamp = micros();

    int data = analogRead(SensorInputPin);
    int dataAfterFilter = myFilter.update(data);

    // Update winSum and circular buffer
    winSum = winSum - buffer[bufferIndex] + abs(dataAfterFilter);
    buffer[bufferIndex] = abs(dataAfterFilter);
    bufferIndex = (bufferIndex + 1) % winSize;

    // Calculate winNorm
    int winNorm = winSum / winSize;

    //  to plot on Arduino serial plotter
    Serial.println(winNorm);

    //  to keep 180 motor at wrist at fixed position for pickup-drop action
    sweep_joint(wrist, -30);

    //  drive action
    int target_angle1, target_angle2;
    if (winNorm > threshold)
    {
        /*
            if above threshold:
                1. for first half move up and right(left)
                2. for second hald move down and right(left)
        */

        //  when one side movement at its end ~ almost, close grip for holding object
        if (shoulderR.cur_pos == shoulderR.max_pos - 1) {
          sweep_joint(gripper, gripper.min_pos);
        }
        if (shoulderR.cur_pos < 65)
        {
            target_angle1 = min(shoulderR.cur_pos + 1, shoulderR.max_pos);
            target_angle2 = min(shoulderUD.cur_pos + 2, shoulderUD.max_pos);
        }
        else
        {
            target_angle1 = min(shoulderR.cur_pos + 1, shoulderR.max_pos);
            target_angle2 = max(shoulderUD.cur_pos - 2, shoulderUD.min_pos);
        }
    }
    else
    {
        /*
            if below threshold:
                1. for first half move up and left(right)
                2. for second hald move down and left(right)
        */

        //  when other side movement at its end ~ almost, open grip for dropping object
        if (shoulderR.cur_pos == shoulderR.min_pos + 1) {
          sweep_joint(gripper, gripper.max_pos);
        }
        if (shoulderR.cur_pos < 65)
        {
            target_angle1 = max(shoulderR.cur_pos - 1, shoulderR.min_pos);
            target_angle2 = max(shoulderUD.cur_pos - 2, shoulderUD.min_pos);
        }
        else
        {
            target_angle1 = max(shoulderR.cur_pos - 1, shoulderR.min_pos);
            target_angle2 = min(shoulderUD.cur_pos + 2, shoulderUD.max_pos);
        }
    }
    sweep_joint(shoulderR, target_angle1);
    sweep_joint(shoulderUD, target_angle2);
}
