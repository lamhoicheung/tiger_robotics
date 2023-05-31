// LIBRARIES

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
  #define CYTRON_DIR 2
  #define CYTRON_PWM 3
  CytronMD motor(PWM_DIR, CYTRON_PWM, CYTRON_DIR);


  // STEPPER
  #define STEPPER_DIR 4
  #define STEPPER_PUL 5
  int stepper_speed = 28000;

// GLOBAL FUNCTIONS

void setup() {

  // PS4

  // C620

  // CYTRON

  // STEPPER

}

void loop() {

  // PS4

  // C620

  // CYTRON
  motor.setSpeed(100); // range -255 to 255

  // STEPPER
  // up
  digitalWrite(STEPPER_DIR, HIGH);
  tone(STEPPER_PUL, stepper_speed);   
  // down
  digitalWrite(STEPPER_DIR, LOW);
  tone(STEPPER_PUL, stepper_speed);   
  // stop
  noTone(STEPPER_PUL);

}