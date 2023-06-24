// LIBRARIES
#include <PS4Controller.h>
#include <driver/twai.h>
#include <ESP32Servo.h>



// #############################
// DEFINITIONS, GLOBAL VARIABLES
// #############################



// SHOOTER (BLDC), TAKE 5V SIGNALS
Servo topRoller, bottomRoller;
#define TOP_ROLLER 19
#define BOTTOM_ROLLER 18
int rollerSpeed = 0;


// PUSHER (STEPPER), TAKE 5V SIGNALS
#define STEPPER_DIR 16
#define STEPPER_PUL 17

int stepper_speed = 32000;


// C620
#define CAN_RX_PIN 5
#define CAN_TX_PIN 4

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

short c620StatusBuffer[4][2];

class C620 {
  private:
    double feedback, setpoint;
    double Kp, Ki, Kd;
    int T;
    unsigned long lastTime;
    double totalErr, lastErr;
    int controlLimit;

  public:
    int id;
    double control;
    short position;
    short velocity;

    C620(int id) {
      this->id = id;

      Kp = 1;
      Ki = 0.005;
      Kd = 0;

      T = 10;

      lastTime = 0;
      totalErr = 0;
      lastErr = 0;
      controlLimit = 10000;
    }

    void read() {

      position = c620StatusBuffer[id - 0x201][0];
      velocity = c620StatusBuffer[id - 0x201][1];

    }

    void setSpeed(short rpm) {
      // PID
      setpoint = rpm;
      feedback = velocity;

      unsigned long now = millis();

      if (now - lastTime >= T) {

        double err = setpoint - feedback;

        totalErr += err;
        totalErr = constrain(totalErr, -controlLimit, controlLimit);

        double deltaErr = err - lastErr;

        control = Kp*err + (Ki*T)*totalErr + (Kd/T)*deltaErr;

        control = constrain(control, -controlLimit, controlLimit);

        lastErr = err;
        lastTime = millis();
      }
    }
};

C620 frontLeftWheel(0x201);
C620 backLeftWheel(0x202);
C620 frontRightWheel(0x203);
C620 backRightWheel(0x204);



// #############################
// SETUP
// #############################



void setup() {

  Serial.begin(115200);


  // PS4
  PS4.begin();


  // C620
    // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)CAN_TX_PIN, (gpio_num_t)CAN_RX_PIN, TWAI_MODE_NORMAL);
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


  // -------------------- PUSHER (STEPPER) --------------------
  pinMode(STEPPER_PUL, OUTPUT);
  pinMode(STEPPER_DIR, OUTPUT);


  // -------------------- SHOOTER (BLDC) --------------------
  topRoller.attach(TOP_ROLLER);
  bottomRoller.attach(BOTTOM_ROLLER);
  topRoller.writeMicroseconds(1500);
  bottomRoller.writeMicroseconds(1500);

  delay(500);
}



// #############################
// LOOP
// #############################



void loop() {

  // PS4
  if (!PS4.isConnected()) {
    return;
  }


  // -------------------- SHOOTER (BLDC) --------------------
  if (PS4.Triangle()) rollerSpeed += 10; // need long delay and small increment otherwise speed changes rapidly, causing motor jerk
  else if (PS4.Cross()) rollerSpeed -= 10;

  rollerSpeed = constrain(rollerSpeed, 0, 500);
  topRoller.writeMicroseconds(1500 + rollerSpeed);
  bottomRoller.writeMicroseconds(1500 + rollerSpeed);


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
  int nominalSpeed = 2000;
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

  // update c620 status
  //Wait for message to be received
  twai_message_t message;
  if (twai_receive(&message, pdMS_TO_TICKS(10000)) == ESP_OK) {
      printf("Message received\n");
  } else {
      printf("Failed to receive message\n");
      return;
  }

  //Process received message
  short posHB = message.data[0] << 8;
  short posLB = message.data[1];
  short pos = posHB | posLB;
  pos = map(pos, 0, 8191, 0, 360);

  short rpmHB = message.data[2] << 8;
  short rpmLB = message.data[3];
  short rpm = rpmHB | rpmLB;

  // c620StatusBuffer[message.identifier - 0x201][0] = pos;
  c620StatusBuffer[message.identifier - 0x201][1] = rpm;

  frontLeftWheel.read();
  frontRightWheel.read();
  backLeftWheel.read();
  backRightWheel.read();

  // set speed
  frontLeftWheel.setSpeed(c620targetSpeeds[0]);
  backLeftWheel.setSpeed(c620targetSpeeds[1]);
  frontRightWheel.setSpeed(c620targetSpeeds[2]);
  backRightWheel.setSpeed(c620targetSpeeds[3]);

  // split into bytes
  int c620targetAccels[4];
  byte currentBytes[4][2];

  c620targetAccels[0] = frontLeftWheel.control;
  c620targetAccels[1] = backLeftWheel.control;
  c620targetAccels[2] = frontRightWheel.control;
  c620targetAccels[3] = backRightWheel.control;

  for (int i = 0; i < 4; i++) {
    byte currentHB = (c620targetAccels[i] >> 8) & 0xFF;
    byte currentLB = c620targetAccels[i] & 0xFF;   

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

}