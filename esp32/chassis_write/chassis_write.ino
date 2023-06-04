#include "driver/twai.h"
#define RX_PIN 5
#define TX_PIN 4
 


void setup() {

  Serial.begin(115200);
// C620
    // Initialize configuration structures using macro initializers
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT((gpio_num_t)TX_PIN, (gpio_num_t)RX_PIN, TWAI_MODE_NORMAL);
  twai_timing_config_t t_config = TWAI_TIMING_CONFIG_1MBITS();
  twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

}

void loop() {
short wheelAccel[4] = {800, 800, 800, 800};
  byte currentBytes[4][2];

    // split into bytes
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

}