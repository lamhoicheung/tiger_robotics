/*

  Known issue: Serial will not work sometimes (i.e. cannot print stuff on serial monitor)
  Solve: Reconnect USB, reselect port

  Just use 3.3 to power TJA1050

  known issue 2: C620 will freeze sometimes, use esp32 works, esp32s3 will freeze c620 always
  Solve: Power off c620 and turn back on.

*/

#include "driver/twai.h"

// Pins used to connect to CAN bus transceiver:
#define RX_PIN 5
#define TX_PIN 4

// Intervall:
#define POLLING_RATE_MS 1000

static bool driver_installed = false;

const short currentLowerLimit = -1024;
const short currentUpperLimit = 1024;

void setup() {
  // Start Serial:
  Serial.begin(115200);

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
}

static void transmit_message() {
  //Configure message to transmit
  twai_message_t command;
  command.identifier = 0x200;
  command.extd = 0;
  command.data_length_code = 8;
  command.data[0] = 0x00;
  command.data[1] = 0x00;
  command.data[2] = 0x00;
  command.data[3] = 0x00;
  command.data[4] = 0x00;
  command.data[5] = 0x00;
  command.data[6] = 0x00;
  command.data[7] = 0x00;

  short current = 300;
  current = constrain(current, currentLowerLimit, currentUpperLimit);
  
  byte currentHB = (current >> 8) & 0xFF;
  byte currentLB = current & 0xFF;

  command.data[0] = currentHB;
  command.data[1] = currentLB;

  //Queue message for transmission
  if (twai_transmit(&command, pdMS_TO_TICKS(1000)) == ESP_OK) {
      printf("Message queued for transmission\n");
  } else {
      printf("Failed to queue message for transmission\n");
  }

}

static void handle_rx_message(twai_message_t& message) {
  // Process received message
  short posHB = message.data[0] << 8;
  short posLB = message.data[1];
  short pos = posHB | posLB;
  Serial.print(map(pos, 0, 8191, 0, 360));
  Serial.print(" ");

  short rpmHB = message.data[2] << 8;
  short rpmLB = message.data[3];
  short rpm = rpmHB | rpmLB;
  Serial.println(rpm); 


  // if (message.extd) {
  //   Serial.println("Message is in Extended Format");
  // } else {
  //   Serial.println("Message is in Standard Format");
  // }
  // Serial.printf("ID: %x\nByte:", message.identifier);
  // if (!(message.rtr)) {
  //   for (int i = 0; i < message.data_length_code; i++) {
  //     Serial.printf(" %d = %02x,", i, message.data[i]);
  //   }
  //   Serial.println("");
  // }
}

void loop() {
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
}
