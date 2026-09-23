#include "Bigbrother.h"

Bigbrother::Bigbrother(uint8_t addr)
  : ADDR_BIGBRO(addr), alone(true) {}

void Bigbrother::begin() {
  // Pin initialisation
  pinMode(LED, OUTPUT);
  pinMode(PWR_CTRL, OUTPUT);
  pinMode(TRIG, INPUT);
  pinMode(UART_RX, INPUT);
  pinMode(UART_TX, INPUT);

  digitalWrite(PWR_CTRL, LOW); // Ensure intial off camera

  // current measure init
  ina219.begin();
  ina219.setCalibration_32V_1A();
  measureCurrent();

  blinkLED(3, 100);

  alone = digitalRead(TRIG);

  if (!alone) {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX, CAN_RX, TWAI_MODE_NORMAL);
    twai_timing_config_t t_config = TWAI_TIMING_CONFIG_500KBITS();  // 500 kbps
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
      while (1) {
        digitalWrite(LED, !digitalRead(LED));
        delay(100);
      }
    }

    if (twai_start() != ESP_OK) {
      while (1) {
        digitalWrite(LED, !digitalRead(LED));
        delay(500);
      }
    }
  }
}

////////////////////////////////////////////
// Private methods
////////////////////////////////////////////
void Bigbrother::UARTinit() {
  Serial1.begin(115200, SERIAL_8N1, UART_RX, UART_TX);
}

void Bigbrother::UARToff() {
  Serial1.end();
  pinMode(UART_RX, INPUT);
  pinMode(UART_TX, INPUT);
}

////////////////////////////////////////////
// Camera control
////////////////////////////////////////////
void Bigbrother::cameraON() {
  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    digitalWrite(PWR_CTRL, HIGH);
    delay(2000);

    if (isON()) {
      UARTinit();
      delay(2000);
      base_current = measureCurrent();
      return;
    }

    digitalWrite(PWR_CTRL, LOW);
    delay(100);
  }

  end();
}

void Bigbrother::cameraOFF() {
  UARToff();

  for (uint8_t attempt = 0; attempt < 5; attempt++) {
    digitalWrite(PWR_CTRL, LOW);
    delay(20);

    if (!isON()) {
      return;
    }
  }

  end();
}

bool Bigbrother::isON() {
  return (measureCurrent() >= PWR_TRESHOLD);
}

bool Bigbrother::isRec() {
  return (measureCurrent() - base_current >= REC_TRESHOLD);
}

////////////////////////////////////////////
// Tools methods
////////////////////////////////////////////
void Bigbrother::blinkLED(uint8_t rep, int duration) {
  for (uint8_t i = 0; i < rep; i++) {
    digitalWrite(LED, LOW);
    delay(duration);
    digitalWrite(LED, HIGH);
    delay(duration);
  }
}

float Bigbrother::measureCurrent(uint8_t rep) {
  float total = 0.0;
  for (uint8_t i = 0; i < rep; i++) {
    total += ina219.getCurrent_mA();
    delay(10);
  }
  return total / rep;
}

void Bigbrother::end() {
  UARToff();
  digitalWrite(PWR_CTRL, LOW);
  digitalWrite(LED, LOW);
}

////////////////////////////////////////////
// UART communication
////////////////////////////////////////////
void Bigbrother::send_UART_CMD(const uint8_t *data, size_t len) {
  Serial1.write(data, len);
  blinkLED(1, 50);
}

////////////////////////////////////////////
// CAN communication
////////////////////////////////////////////
void Bigbrother::send_health_packet(bool recording_status) {
  twai_message_t tx_msg = { 0 };

  // Construction de l'ID : [ TYPE_HEALTH ] [ ADDR_CAM ]
  tx_msg.identifier = (static_cast<uint16_t>(MessageType::HEALTH) << 2) | ADDR_BIGBRO;
  tx_msg.extd = 0;
  tx_msg.rtr = 0;
  tx_msg.data_length_code = 1;
  tx_msg.data[0] = recording_status ? 1 : 0;  // 1 = On/Record, 0 = Off/Standby

  twai_transmit(&tx_msg, pdMS_TO_TICKS(50));
}

Bigbrother::MessageType Bigbrother::listen_CAN() {
  twai_message_t rx_msg;

  bool messageRecu = (twai_receive(&rx_msg, pdMS_TO_TICKS(10)) == ESP_OK);

  uint16_t msg_type = messageRecu ? ((rx_msg.identifier >> 2) & 0x1FF) : 0;
  uint8_t sender_addr = messageRecu ? (rx_msg.identifier & 0x03) : 0;
  uint16_t validation = messageRecu ? ((uint16_t(rx_msg.data[0]) << 8) | rx_msg.data[1]) : 0;

  if (messageRecu && sender_addr == ADDR_FLIGHT_COMP && msg_type == static_cast<uint16_t>(MessageType::START_REC) && validation == VALID_START_REC) {
    return MessageType::START_REC;
  }

  else if (messageRecu && sender_addr == ADDR_FLIGHT_COMP && msg_type == static_cast<uint16_t>(MessageType::STOP_REC) && validation == VALID_STOP_REC) {
    return MessageType::STOP_REC;
  }

  else if (messageRecu && sender_addr == ADDR_FLIGHT_COMP && msg_type == static_cast<uint16_t>(MessageType::HEALTH)) {
    return MessageType::HEALTH;
  }

  return MessageType::NONE;
}

////////////////////////////////////////////
// Getter
////////////////////////////////////////////
bool Bigbrother::get_mode() {
  return alone;
}