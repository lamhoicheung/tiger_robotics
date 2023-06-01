// LIBRARIES
#include <PS4Controller.h>
#include <driver/twai.h>
#include <ESP32Servo.h>
#include <CytronMotorDriver.h>


// PIN DEFINITIONS

// GLOBAL VARIABLES

  // PS4

  // C620
  #define RX_PIN 5
  #define TX_PIN 4
  #define POLLING_RATE_MS 1000
  static bool driver_installed = false;

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


  // STEPPER, take 5V signals
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
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();  //Look in the api-reference for other speed sets.
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

    // Reconfigure alerts to detect frame receive, Bus-Off error and RX queue full states
  uint32_t alerts_to_enable = TWAI_ALERT_RX_DATA | TWAI_ALERT_ERR_PASS | TWAI_ALERT_BUS_ERROR | TWAI_ALERT_RX_QUEUE_FULL;
  if (twai_reconfigure_alerts(alerts_to_enable, NULL) == ESP_OK) {
    Serial.println("CAN Alerts reconfigured");
  } else {
    Serial.println("Failed to reconfigure alerts");
    return;
  }

    // TWAI driver is now successfully installed and started
  driver_installed = true;


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
  if (!driver_installed) {
    // Driver not installed
    delay(1000);
    return;
  }
  // Check if alert happened
  uint32_t alerts_triggered;
  twai_read_alerts(&alerts_triggered, pdMS_TO_TICKS(POLLING_RATE_MS));
  twai_status_info_t twaistatus;
  twai_get_status_info(&twaistatus);

  // Handle alerts
  if (alerts_triggered & TWAI_ALERT_ERR_PASS) {
    Serial.println("Alert: TWAI controller has become error passive.");
  }
  if (alerts_triggered & TWAI_ALERT_BUS_ERROR) {
    Serial.println("Alert: A (Bit, Stuff, CRC, Form, ACK) error has occurred on the bus.");
    Serial.printf("Bus error count: %d\n", twaistatus.bus_error_count);
  }
  if (alerts_triggered & TWAI_ALERT_RX_QUEUE_FULL) {
    Serial.println("Alert: The RX queue is full causing a received frame to be lost.");
    Serial.printf("RX buffered: %d\t", twaistatus.msgs_to_rx);
    Serial.printf("RX missed: %d\t", twaistatus.rx_missed_count);
    Serial.printf("RX overrun %d\n", twaistatus.rx_overrun_count);
  }

  // Check if message is received
  if (alerts_triggered & TWAI_ALERT_RX_DATA) {
    // One or more messages received. Handle all.
    twai_message_t message;
    while (twai_receive(&message, 0) == ESP_OK) {
      handle_rx_message(message);
      transmit_message();
    }
  }



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
  if (PS4.Triangle()) rollerSpeed += 10; // need long delay and small increment otherwise speed changes rapidly, cause motor jerk
  else if (PS4.Cross()) rollerSpeed -= 10;

  rollerSpeed = constrain(rollerSpeed, 0, 500);
  topRoller.writeMicroseconds(1500 + rollerSpeed);
  // bottomRoller.writeMicroseconds(1500 - rollerSpeed);

  delay(50); // delay required otherwise cytron will not work

}