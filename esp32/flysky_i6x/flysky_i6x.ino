// Require logic shifter
// Use 5V to power the receiver

// Doc: https://github.com/bmellink/IBusBM

#include <Arduino.h>
#include <IBusBM.h>


IBusBM ibus;
const int RC_READ_RATE = 200;

struct Command {
  int left_joy_x;
  int left_joy_y;
  int right_joy_x;
  int right_joy_y;
  int switch_1;
  int switch_2;
  int switch_3;
  int switch_4;
  int pot_1;
  int pot_2;
};

Command cmd;

void taskReadRC(void* pvParam) {
  ibus.begin(Serial2, 1);  // iBUS object connected to serial2 RX2 pin (pin 16) using timer 1
  // use Serial1 will crash

  while (1) {
    // Value range: 1000-2000
    cmd.left_joy_x = ibus.readChannel(3);
    cmd.left_joy_y = ibus.readChannel(2);
    cmd.right_joy_x = ibus.readChannel(0);
    cmd.right_joy_y = ibus.readChannel(1);
    cmd.switch_1 = ibus.readChannel(4);
    cmd.switch_2 = ibus.readChannel(5);
    cmd.switch_3 = ibus.readChannel(6);
    cmd.switch_4 = ibus.readChannel(7);
    cmd.pot_1 = ibus.readChannel(8);
    cmd.pot_2 = ibus.readChannel(9);

    Serial.print(cmd.left_joy_x);
    Serial.print('\t');
    Serial.print(cmd.left_joy_y);
    Serial.print('\t');
    Serial.print(cmd.right_joy_x);
    Serial.print('\t');
    Serial.print(cmd.right_joy_y);
    Serial.print('\t');
    Serial.print(cmd.switch_1);
    Serial.print('\t');
    Serial.print(cmd.switch_2);
    Serial.print('\t');
    Serial.print(cmd.switch_3);
    Serial.print('\t');
    Serial.print(cmd.switch_4);
    Serial.print('\t');
    Serial.print(cmd.pot_1);
    Serial.print('\t');
    Serial.print(cmd.pot_2);
    Serial.println();

    vTaskDelay(RC_READ_RATE / portTICK_PERIOD_MS);
  }
}


void setup() {
  Serial.begin(115200);
  xTaskCreate(taskReadRC, "task read RC", 1500, NULL, 1, NULL);
}

void loop() {
}
