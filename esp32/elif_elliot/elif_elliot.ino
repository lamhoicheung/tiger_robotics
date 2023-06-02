// LIBRARIES
#include <PS4Controller.h>
#include <ESP32Servo.h>
#include <CytronMotorDriver.h> // need to modify the header or cpp file, comment out ledWrite, directly use analogWrite


// PIN DEFINITIONS

// GLOBAL VARIABLES

  // PS4

  // C620
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

  // CYTRON
  #define CYTRON_DIR 33
  #define CYTRON_PWM 32
  CytronMD platformLift(PWM_DIR, CYTRON_PWM, CYTRON_DIR);
  int liftSpeed = 0;


  // STEPPER, take 5V signals, still has some problem, fail to change direction
  #define STEPPER_DIR 16
  #define STEPPER_PUL 17
  int stepper_speed = 28000;

  // BLDC, take 5V signals
  Servo topRoller, bottomRoller;
  #define TOP_ROLLER 18
  #define BOTTOM_ROLLER 19
  int rollerSpeed = 0;

// GLOBAL FUNCTIONS

  // PS4

  // C620

  // CYTRON

  // STEPPER

  // BLDC


void setup() {

  Serial.begin(115200);

  // PS4
  PS4.begin();

  // C620

  // CYTRON

  // STEPPER

  // BLDC
  topRoller.attach(TOP_ROLLER);
  bottomRoller.attach(BOTTOM_ROLLER);
  topRoller.writeMicroseconds(1500);
  bottomRoller.writeMicroseconds(1500);
  delay(3000);
}

void loop() {

  // PS4
  if (!PS4.isConnected()) {
    // turn off all motors

    return;
  }

  // C620

  // CYTRON
  if (PS4.Up()) liftSpeed = -255;
  else if (PS4.Down()) liftSpeed = 255;
  else liftSpeed = 0;
  platformLift.setSpeed(liftSpeed); // range -255 to 255

  // STEPPER
  if (PS4.R1()) {
    digitalWrite(STEPPER_DIR, HIGH);
    tone(STEPPER_PUL, stepper_speed); 
  }
  else if (PS4.L1()) {
    digitalWrite(STEPPER_DIR, LOW);
    tone(STEPPER_PUL, stepper_speed);   
  }
  else noTone(STEPPER_PUL);

  // BLDC
  if (PS4.Triangle()) rollerSpeed += 10; // need long delay and small increment otherwise speed changes rapidly, causing motor jerk
  else if (PS4.Cross()) rollerSpeed -= 10;

  rollerSpeed = constrain(rollerSpeed, 0, 500);
  topRoller.writeMicroseconds(1500 + rollerSpeed);
  // bottomRoller.writeMicroseconds(1500 - rollerSpeed);

  delay(50); // delay required otherwise cytron will not work

}