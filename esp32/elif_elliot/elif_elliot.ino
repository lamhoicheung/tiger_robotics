// LIBRARIES
#include <PS4Controller.h>
#include <driver/twai.h>
#include <ESP32Servo.h>
#include <CytronMotorDriver.h> // need to modify the header or cpp file, comment out ledWrite, directly use analogWrite


// PIN DEFINITIONS

// GLOBAL VARIABLES

  // PS4

  // C620
  #define RX_PIN 5
  #define TX_PIN 4

  const short currentLowerLimit = -1024;
  const short currentUpperLimit = 1024;

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
    // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    // Install TWAI driver
  if (twai_driver_install(&g_config, &t_config, &f_config) == ESP_OK) {
    Serial.println("Driver installed");
  } else {
    Serial.println("Failed to install driver");
    return;
  }

    // Start TWAI driver
  if (twai_start() == ESP_OK) {
    Serial.println("Driver started");
  } else {
    Serial.println("Failed to start driver");
    return;
  }


  // CYTRON

  // STEPPER
  pinMode(STEPPER_PUL, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);

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

    // read PS4 and set drive mode
  const int deadband = 30;
  int driveMode = STOP;

  if (PS4.L1() == 1 && PS4.R1() == 1) {
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
  int nominalAccel = 1000;
  int accelMultiplier = 1;

  if (PS4.Square() == 0) accelMultiplier = 2;
  else accelMultiplier = 1;

  short wheelAccel[4] = {0, 0, 0, 0};

  switch (driveMode) {
    case FORWARD:
      wheelAccel[0] = nominalAccel * accelMultiplier;
      wheelAccel[1] = nominalAccel * accelMultiplier;
      wheelAccel[2] = -nominalAccel * accelMultiplier;
      wheelAccel[3] = -nominalAccel * accelMultiplier;
      break;

    case BACKWARD:
      wheelAccel[0] = -nominalAccel * accelMultiplier;
      wheelAccel[1] = -nominalAccel * accelMultiplier;
      wheelAccel[2] = nominalAccel * accelMultiplier;
      wheelAccel[3] = nominalAccel * accelMultiplier;
      break;

    case LEFT:
      wheelAccel[0] = -nominalAccel * accelMultiplier;
      wheelAccel[1] = nominalAccel * accelMultiplier;
      wheelAccel[2] = -nominalAccel * accelMultiplier;
      wheelAccel[3] = nominalAccel * accelMultiplier;
      break;

    case RIGHT:
      wheelAccel[0] = nominalAccel * accelMultiplier;
      wheelAccel[1] = -nominalAccel * accelMultiplier;
      wheelAccel[2] = nominalAccel * accelMultiplier;
      wheelAccel[3] = -nominalAccel * accelMultiplier;
      break;

    case FORWARD_LEFT:
      wheelAccel[0] = 0;
      wheelAccel[1] = nominalAccel * accelMultiplier;
      wheelAccel[2] = -nominalAccel * accelMultiplier;
      wheelAccel[3] = 0;
      break;

    case FORWARD_RIGHT:
      wheelAccel[0] = nominalAccel * accelMultiplier;
      wheelAccel[1] = 0;
      wheelAccel[2] = 0;
      wheelAccel[3] = -nominalAccel * accelMultiplier;
      break;

    case BACKWARD_LEFT:
      wheelAccel[0] = -nominalAccel * accelMultiplier;
      wheelAccel[1] = 0;
      wheelAccel[2] = 0;
      wheelAccel[3] = nominalAccel * accelMultiplier;
      break;

    case BACKWARD_RIGHT:
      wheelAccel[0] = 0;
      wheelAccel[1] = -nominalAccel * accelMultiplier;
      wheelAccel[2] = nominalAccel * accelMultiplier;
      wheelAccel[3] = 0;
      break;

    case ROTATE_CW:
      wheelAccel[0] = nominalAccel * accelMultiplier;
      wheelAccel[1] = nominalAccel * accelMultiplier;
      wheelAccel[2] = nominalAccel * accelMultiplier;
      wheelAccel[3] = nominalAccel * accelMultiplier;
      break;

    case ROTATE_CCW:
      wheelAccel[0] = -nominalAccel * accelMultiplier;
      wheelAccel[1] = -nominalAccel * accelMultiplier;
      wheelAccel[2] = -nominalAccel * accelMultiplier;
      wheelAccel[3] = -nominalAccel * accelMultiplier;
      break;

    default:
      wheelAccel[0] = 0;    
      wheelAccel[1] = 0;    
      wheelAccel[2] = 0;    
      wheelAccel[3] = 0;    
      break;
  }  


    // split into bytes
  byte currentBytes[4][2];

  for (int i = 0; i < 4; i++) {
    byte currentHB = (wheelAccel[i] >> 8) & 0xFF;
    byte currentLB = wheelAccel[i] & 0xFF;   

    currentBytes[i][0] = currentHB;
    currentBytes[i][1] = currentLB;
  }

    //Configure message to transmit
  twai_message_t command;
  command.identifier = 0x200;
  command.extd = 0;
  command.data_length_code = 8;
  command.data[0] = currentBytes[0][0]; // motor 1
  command.data[1] = currentBytes[0][1];
  command.data[2] = currentBytes[1][0]; // motor 2
  command.data[3] = currentBytes[1][1];
  command.data[4] = currentBytes[2][0]; // motor 3
  command.data[5] = currentBytes[2][1];
  command.data[6] = currentBytes[3][0]; // motor 4
  command.data[7] = currentBytes[3][1];

  //Queue message for transmission
  if (twai_transmit(&command, pdMS_TO_TICKS(1000)) == ESP_OK) {
      Serial.println("Message queued for transmission");
  } else {
      Serial.println("Failed to queue message for transmission");
  }



  // CYTRON
  if (PS4.Up()) liftSpeed = -255;
  else if (PS4.Down()) liftSpeed = 255;
  else liftSpeed = 0;
  platformLift.setSpeed(liftSpeed); // range -255 to 255



  // STEPPER
  if (PS4.RStickY() > 200) {
    digitalWrite(STEPPER_DIR, HIGH);
    tone(STEPPER_PUL, stepper_speed); 
  }
  else if (PS4.RStickY() < -200) {
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