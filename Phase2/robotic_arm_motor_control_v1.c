#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

#define SERVOMIN  150 
#define SERVOMAX  600 
#define STOP 375
#define SERVO_FREQ 50 

class Joint180 {
public:
  int pin;
  int min_pos;
  int max_pos;
  int cur_pos;
  bool active;

  // int Joint180 () {

  // }
};

class Joint360 {
public:
  int pin;
  int cur_state;
  bool active;
};

Joint180 shoulderR;
Joint180 shoulderUD;
Joint360 elbow1;
Joint360 elbow2;
Joint180 wrist;
Joint180 gripper;   // rotate between 45 and 120

void setup() {
  pwm.begin();
  pwm.setPWMFreq(SERVO_FREQ);

  shoulderR.pin = 0; shoulderR.min_pos = -30; shoulderR.max_pos = 180; shoulderR.active = true;
  shoulderUD.pin = 1; shoulderUD.min_pos = 10; shoulderUD.max_pos = 160; shoulderUD.active = true;
  elbow1.pin = 0; elbow1.active = true;
  elbow2.pin = 0; elbow2.active = true;
  wrist.pin = 3; wrist.max_pos = 90; wrist.min_pos = -30; wrist.active = true;
  gripper.pin = 4; gripper.min_pos = 45; gripper.max_pos = 120; gripper.active = true;
}

void setServoAngle(uint8_t servoNum, int angle) {
  int pulseWidth = map(angle, 0, 180, SERVOMIN, SERVOMAX);
  pwm.setPWM(servoNum, 0, pulseWidth);
}

// dir = 0 -> only min to max, dir == 1 only max to min, dir == 2 both sweep
void sweep_joint(Joint180 joint, int angle_min, int angle_max, int at_step, int dir = 0) {

  if (joint.active) {
    if (dir == 0 || dir == 2) {
      for (int angle = angle_min; angle <= angle_max; angle += at_step) {  // Sweep from 0 to angle_max degrees
        setServoAngle(joint.pin, angle);  // Set servo on channel for joint
        delay(500);
      }
      joint.cur_pos = angle_max;
    }

    if (dir == 1 || dir == 2){
      for (int angle = angle_max; angle >= angle_min; angle -= at_step) {  // Sweep back from angle_max to 0 degrees
        setServoAngle(joint.pin, angle);  // Set servo on channel for joint
        delay(500);
      }
      joint.cur_pos = angle_min;
    }
  }
}


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

void loop() {
  sweep_cont1(elbow1, 1500, 1500, 1500);  
  elbow1.active = false;

  delay(1000);

  sweep_cont2(elbow2, 1500, 1500, 1500);
  elbow2.active = false;
}
