#include "Camera.h"

Camera::Camera()
  : _board(nullptr) {}

void Camera::begin(Bigbrother& board) {
  _board = &board;
}

////////////////////////////////////////////
// UART msg composition
////////////////////////////////////////////
void Camera::toggleRecording() {
  uint8_t pkt[5];
  pkt[0] = RUNCAM_HEADER;
  pkt[1] = RUNCAM_CMD_CAM_CTRL;
  pkt[2] = RUNCAM_TOGGLE;
  pkt[3] = runcam_crc(pkt[0], pkt[1], pkt[2]);
  pkt[4] = RUNCAM_TAIL;

  if (_board == nullptr)
    return;
  _board->send_UART_CMD(pkt, 5);
  delay(3000);
}

uint8_t Camera::runcam_crc(uint8_t header, uint8_t cmd, uint8_t toggle) {
  uint8_t buffer[5];

  buffer[0] = RUNCAM_HEADER;
  buffer[1] = RUNCAM_CMD_CAM_CTRL;
  buffer[2] = RUNCAM_TOGGLE;
  buffer[3] = RUNCAM_TAIL;

  uint8_t crc = crc_high_first(buffer, 4);

  return crc;
}

uint8_t Camera::crc_high_first(uint8_t* ptr, uint8_t len) {
  uint8_t i;
  uint8_t crc = 0x00;
  while (len--) {
    crc ^= *ptr++;
    for (i = 8; i > 0; --i) {
      if (crc & 0x80)
        crc = (crc << 1) ^ 0x31;
      else
        crc = (crc << 1);
    }
  }
  return (crc);
}