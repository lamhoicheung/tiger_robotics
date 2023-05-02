// Require logic shifter
// Use 5V to power the receiver

// Doc: https://github.com/bmellink/IBusBM


#include <IBusBM.h>

IBusBM ibus;

struct Command {
  int left_joy_x;
  int left_joy_y;
  int right_joy_x;
  int right_joy_y;
  int switch_a;
  int switch_b;
  int switch_c;
  int switch_d;
  int pot_a;
  int pot_b;
};

Command cmd;

void readRC() {

  cmd.left_joy_x = ibus.readChannel(3);
  cmd.left_joy_y = ibus.readChannel(2);
  cmd.right_joy_x = ibus.readChannel(0);
  cmd.right_joy_y = ibus.readChannel(1);
  cmd.switch_a = ibus.readChannel(4);
  cmd.switch_b = ibus.readChannel(5);
  cmd.switch_c = ibus.readChannel(6);
  cmd.switch_d = ibus.readChannel(7);
  cmd.pot_a = ibus.readChannel(8);
  cmd.pot_b = ibus.readChannel(9);

  Serial.print(cmd.left_joy_x); Serial.print('\t');
  Serial.print(cmd.left_joy_y); Serial.print('\t');
  Serial.print(cmd.right_joy_x); Serial.print('\t');
  Serial.print(cmd.right_joy_y); Serial.print('\t');
  Serial.print(cmd.switch_a); Serial.print('\t');
  Serial.print(cmd.switch_b); Serial.print('\t');
  Serial.print(cmd.switch_c); Serial.print('\t');
  Serial.print(cmd.switch_d); Serial.print('\t');
  Serial.print(cmd.pot_a); Serial.print('\t');
  Serial.print(cmd.pot_b); Serial.println();

}

void setup() {

  Serial.begin(115200);
  ibus.begin(Serial2, 1);  // iBUS object connected to serial2 RX2 pin (pin 16) using timer 1

}

void loop() {

  readRC();
  delay(100);

}
