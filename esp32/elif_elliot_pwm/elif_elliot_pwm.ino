// LIBRARIES
#include <PS4Controller.h>
#include <ESP32Servo.h> // Servo available on: 2,4,5,12-19,21-23,25-27,32-33

/*
  Start up sequence

  1. Switch to Calibration Mode
  2. Connect PS4
  3. Push Right Y stick to top
  4. Turn on motor power
  5. Once you see solid green light on C620, push Right Y stick to bottom, hold until you see blinking ornage
  6. Release right stick
  7. When you see blinking green, okay
  8. Switch to operation mode

*/


// #############################
// DEFINITIONS, GLOBAL VARIABLES
// #############################


// PS4
bool debugPS4 = false;
bool lastSpeedUpButtonState = 0;
bool lastSpeedDownButtonState = 0;


// SHOOTER (BLDC), TAKE 5V SIGNALS
Servo topRoller, bottomRoller;
#define TOP_ROLLER 19
#define BOTTOM_ROLLER 18
int rollerSpeeds[4] = {0, 65, 83, 102};
int speedSelect = 0;

// PUSHER (STEPPER), TAKE 5V SIGNALS
#define STEPPER_DIR 16
#define STEPPER_PUL 17

int stepper_speed = 32000;


// C620, take 3.3V PWM signals
#define C620_PWM_1 33
#define C620_PWM_2 25
#define C620_PWM_3 26
#define C620_PWM_4 27
#define CALIBRATION_SWITCH 32

#define FORWARD 0
#define BACKWARD 1
#define LEFT 2
#define RIGHT 3
#define FORWARD_LEFT 4
#define FORWARD_RIGHT 5
#define BACKWARD_LEFT 6
#define BACKWARD_RIGHT 7
#define ROTATE_CW 8
#define ROTATE_CCW 9
#define STOP 10

Servo frontLeftWheel;
Servo backLeftWheel;
Servo frontRightWheel;
Servo backRightWheel;


void calibrateC620() {

  int wheelSpeed = map(PS4.RStickY(), -128, 128, 1000, 2000);

  // Serial.println(wheelSpeed);

  frontLeftWheel.writeMicroseconds(wheelSpeed);
  backLeftWheel.writeMicroseconds(wheelSpeed);
  frontRightWheel.writeMicroseconds(wheelSpeed);
  backRightWheel.writeMicroseconds(wheelSpeed);

}


// #############################
// SETUP
// #############################



void setup() {

  Serial.begin(115200);

  // PS4
  PS4.begin();

  if (!PS4.isConnected()) delay(500);

  // C620
  pinMode(CALIBRATION_SWITCH, INPUT_PULLUP);
  frontLeftWheel.attach(C620_PWM_1);
  backLeftWheel.attach(C620_PWM_2);
  frontRightWheel.attach(C620_PWM_3);
  backRightWheel.attach(C620_PWM_4);


  // -------------------- PUSHER (STEPPER) --------------------
  pinMode(STEPPER_PUL, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);


  // -------------------- SHOOTER (BLDC) --------------------
  topRoller.attach(TOP_ROLLER);
  bottomRoller.attach(BOTTOM_ROLLER);
  topRoller.writeMicroseconds(1500);
  bottomRoller.writeMicroseconds(1500);
}



// #############################
// LOOP
// #############################



void loop() {

  while (debugPS4) {
    int up = PS4.Up();
    int leftX = PS4.LStickX();
    int square = PS4.Square();

    Serial.print(up); Serial.print('/t');
    Serial.print(leftX); Serial.print('/t');
    Serial.print(square); Serial.print('/t');
    Serial.println();

    delay(200);
  }

  while (digitalRead(CALIBRATION_SWITCH) == 1) {
    calibrateC620();
  }

  // -------------------- SHOOTER (BLDC) --------------------

  if (PS4.Triangle() != lastSpeedUpButtonState) {
    if (PS4.Triangle() == 1) {
      speedSelect++;
      speedSelect = constrain(speedSelect, 0, 3);
    }
  }
  lastSpeedUpButtonState = PS4.Triangle();


  if (PS4.Cross() != lastSpeedDownButtonState) {
    if (PS4.Cross() == 1) {
      speedSelect--;
      speedSelect = constrain(speedSelect, 0, 3);
    }
  }
  lastSpeedDownButtonState = PS4.Cross();

  // Serial.println(speedSelect);


  // // if (PS4.Triangle()) rollerSpeed += 10; // need long delay and small increment otherwise speed changes rapidly, causing motor jerk
  // // else if (PS4.Cross()) rollerSpeed -= 10;

  // // rollerSpeed = constrain(rollerSpeed, 0, 500);
  topRoller.writeMicroseconds(1500 + rollerSpeeds[speedSelect]);
  bottomRoller.writeMicroseconds(1500 + rollerSpeeds[speedSelect]);


  // -------------------- PUSHER (STEPPER) --------------------
  if (PS4.RStickY() > 50) {
    digitalWrite(STEPPER_DIR, LOW);
    tone(STEPPER_PUL, stepper_speed); 
  }
  else if (PS4.RStickY() < -50) {
    digitalWrite(STEPPER_DIR, HIGH);
    tone(STEPPER_PUL, stepper_speed);   
  }
  else noTone(STEPPER_PUL);



  // -------------------- C620 --------------------

  // read PS4 and set drive mode
  const int deadband = 30;
  int driveMode = STOP;

  if (PS4.L1() == 0 && PS4.R1() == 0) {
    if (abs(PS4.LStickX()) <= deadband && PS4.LStickY() >= deadband) driveMode = FORWARD;
    else if (abs(PS4.LStickX()) <= deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD;
    else if (abs(PS4.LStickY()) <= deadband && PS4.LStickX() >= deadband) driveMode = RIGHT;
    else if (abs(PS4.LStickY()) <= deadband && PS4.LStickX() <= -deadband) driveMode = LEFT;
    else if (PS4.LStickX() <= -deadband && PS4.LStickY() >= deadband) driveMode = FORWARD_LEFT;
    else if (PS4.LStickX() <= -deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD_LEFT;
    else if (PS4.LStickX() >= deadband && PS4.LStickY() >= deadband) driveMode = FORWARD_RIGHT;
    else if (PS4.LStickX() >= deadband && PS4.LStickY() <= -deadband) driveMode = BACKWARD_RIGHT;
  }
  else if (PS4.L1() == 1 && PS4.R1() == 0 && 
          abs(PS4.LStickX()) <= deadband && abs(PS4.LStickY()) <= deadband) driveMode = ROTATE_CW;
  else if (PS4.L1() == 0 && PS4.R1() == 1 && 
          abs(PS4.LStickX()) <= deadband && abs(PS4.LStickY()) <= deadband) driveMode = ROTATE_CCW;


    // calculate motor speeds according to drive mode
  int nominalSpeed = 60;
  int speedMultiplier = 1;

  if (PS4.Square() == 1) speedMultiplier = 2;
  else speedMultiplier = 1;

  short c620targetSpeeds[4];

  switch (driveMode) {
    case FORWARD:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case BACKWARD:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case LEFT:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case RIGHT:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case FORWARD_LEFT:
      c620targetSpeeds[0] = 0;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = 0;
      break;

    case FORWARD_RIGHT:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = 0;
      c620targetSpeeds[2] = 0;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case BACKWARD_LEFT:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = 0;
      c620targetSpeeds[2] = 0;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    case BACKWARD_RIGHT:
      c620targetSpeeds[0] = 0;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = 0;
      break;

    case ROTATE_CW:
      c620targetSpeeds[0] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = -nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = -nominalSpeed * speedMultiplier;
      break;

    case ROTATE_CCW:
      c620targetSpeeds[0] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[1] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[2] = nominalSpeed * speedMultiplier;
      c620targetSpeeds[3] = nominalSpeed * speedMultiplier;
      break;

    default:
      c620targetSpeeds[0] = 0;    
      c620targetSpeeds[1] = 0;    
      c620targetSpeeds[2] = 0;    
      c620targetSpeeds[3] = 0;    
      break;
  }  

  // Serial.println(driveMode);
  // Serial.println(c620targetSpeeds[1] + 1500);

  frontLeftWheel.writeMicroseconds(c620targetSpeeds[0] + 1500);
  backLeftWheel.writeMicroseconds(c620targetSpeeds[1] + 1500);
  frontRightWheel.writeMicroseconds(c620targetSpeeds[2] + 1500);
  backRightWheel.writeMicroseconds(c620targetSpeeds[3] + 1500);


  // delay(10);
}