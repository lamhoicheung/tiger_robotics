#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <math.h>


// PIN CONFIG

#define C620_PWM_1 26
#define C620_PWM_2 25
#define SERVO_LEFT 12
#define SERVO_RIGHT 14
#define STEPPER_PUL 33
#define STEPPER_DIR 32
#define STEPPER_LOWER_LIMIT 26
#define C620_CALIBRATION 18

// SERVO ANGLE

#define LEFT_SERVO_OPEN 65
#define LEFT_SERVO_CLOSE 90
#define RIGHT_SERVO_OPEN 125
#define RIGHT_SERVO_CLOSE 100

// ROBOT STATE DEFINITION

#define GRIPPER_UP 1
#define GRIPPER_DOWN 2
#define GRIPPER_STOP 3
#define GRIPPER_OPEN 4
#define GRIPPER_CLOSE 5
#define GRIPPER_SPEED 28000

// DEBUG
bool DEBUG_SPEED_CALCULATION = false;

// DEFINE OBJECTS

Servo leftHand, rightHand; 
Servo leftWheel, rightWheel;


// FUNCTIONS

void driveServo(int mode) {

  if (mode == GRIPPER_OPEN) {
    leftHand.write(LEFT_SERVO_OPEN);
    rightHand.write(RIGHT_SERVO_OPEN);
  }
  else if (mode == GRIPPER_CLOSE) {
    leftHand.write(LEFT_SERVO_CLOSE);
    rightHand.write(RIGHT_SERVO_CLOSE);    
  }

}

void driveStepper(int mode) {
  if (mode == GRIPPER_UP) {
    digitalWrite(STEPPER_DIR, HIGH);
    tone(STEPPER_PUL, GRIPPER_SPEED);    
  }
  // else if (mode == GRIPPER_DOWN && digitalRead(STEPPER_LOWER_LIMIT) == 1) {
  //   digitalWrite(STEPPER_DIR, LOW);
  //   tone(STEPPER_PUL, GRIPPER_SPEED);
  // }
  else if (mode == GRIPPER_DOWN) {
    digitalWrite(STEPPER_DIR, LOW);
    tone(STEPPER_PUL, GRIPPER_SPEED);
  }
  else {
    noTone(STEPPER_PUL);
  }
}

void calculateSpeed(int &left_motor_speed, int &right_motor_speed) {

  double left, right;
  double r, t; // temp var for calculations

  double x = map(PS4.LStickY(),-128,128,-1000,1000) ; // normalize x and y
  double y = map(PS4.LStickX(),-128,128,-1000,1000) ; 

  r = hypot(x, y);
  t = atan2(y, x);

  t += M_PI / 4.;

  left = r * cos(t);
  right = r * sin(t);

  // Serial.print(PS4.right_joy_x); Serial.print('\t');
  // Serial.print(PS4.right_joy_y); Serial.print('\t');
  // Serial.print(x); Serial.print('\t');
  // Serial.print(y); Serial.print('\t');
  // Serial.print(r); Serial.print('\t');
  // Serial.print(t); Serial.print('\t');
  // Serial.print(left); Serial.print('\t');
  // Serial.print(right); Serial.print('\t');

  left *= sqrt(2);
  right *= sqrt(2);

  // Serial.print(left); Serial.print('\t');
  // Serial.print(right); Serial.print('\t');
  // Serial.println();

  // rescale
  left += 1500;
  right += 1500;

  // clamp then rescale
  left = constrain(left, 1000, 2000);
  right = constrain(right, 1000, 2000);

  left = map(left, 1000, 2000, 1600, 1400);
  right = map(right, 1000, 2000, 1400, 1600);

  // if (DEBUG_SPEED_CALCULATION) {
  //   Serial.print(left); Serial.print('\t');
  //   Serial.print(right);
  //   Serial.println();
  // }

  left_motor_speed = left;
  right_motor_speed = right;
}

void driveMotor(int left_motor_speed, int right_motor_speed) {
  leftWheel.writeMicroseconds(left_motor_speed);
  rightWheel.writeMicroseconds(right_motor_speed);
}

// SETUP AND LOOP

void setup() {

  Serial.begin(115200);

  // PS4
  PS4.begin();

  pinMode(STEPPER_PUL, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);
  pinMode(STEPPER_LOWER_LIMIT, INPUT_PULLUP);

  leftWheel.attach(C620_PWM_1);
  rightWheel.attach(C620_PWM_2);

  leftHand.attach(SERVO_LEFT);
  rightHand.attach(SERVO_RIGHT);

  pinMode(C620_CALIBRATION, INPUT_PULLUP);

}

void loop() {

  // PS4
  if (!PS4.isConnected()) {
    // turn off all motors

    return;
  }

  // control servo

  if (PS4.Square() == 1) driveServo(GRIPPER_CLOSE);
  else if (PS4.Circle() == 1) driveServo(GRIPPER_OPEN);

  // control stepper

  if (PS4.Triangle() == 1) driveStepper(GRIPPER_UP);
  else if (PS4.Cross() == 1) driveStepper(GRIPPER_DOWN);
  else driveStepper(GRIPPER_STOP);

  // control motors

  int left_motor_speed, right_motor_speed;
  calculateSpeed(left_motor_speed, right_motor_speed);
  if (digitalRead(C620_CALIBRATION) == LOW) {
    left_motor_speed = PS4.LStickY();
    right_motor_speed = PS4.LStickY();
  }
  driveMotor(left_motor_speed, right_motor_speed);

  delay(10);
}
