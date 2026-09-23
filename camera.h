/*
The Camera class is an abstraction of the camera used with the controller.
It defines the communication protocol and commands required to control the camera.

Each Camera object is associated with a controller, which is responsible for managing
the underlying hardware interfaces and communication buses.
*/

#pragma once

#include <Arduino.h>
#include "Bigbrother.h"

class Camera {
private:
  // constant cmd bytes for uart msg
  static constexpr uint8_t RUNCAM_HEADER = 0x55;
  static constexpr uint8_t RUNCAM_TAIL = 0xaa;
  static constexpr uint8_t RUNCAM_CMD_CAM_CTRL = 0x01;
  static constexpr uint8_t RUNCAM_TOGGLE = 0x02;

  Bigbrother* _board;

  // Methods used to compute byte integrity for msg
  uint8_t runcam_crc(uint8_t header, uint8_t cmd, uint8_t toggle);
  uint8_t crc_high_first(uint8_t* ptr, uint8_t len);

public:
  Camera();  // Constructor
  void begin(Bigbrother& board);

  void toggleRecording();
};