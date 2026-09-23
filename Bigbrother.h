/*
The BigBrother class is an abstraction of the camera controller PCB, 
also called BigBrother. The PCB is based on a Seeed Studio XIAO ESP32-C3 
and integrates several peripherals dedicated to different functions.

This class defines the complete hardware architecture of the PCB, 
including GPIOs, UART interfaces, and CAN communication. It also provides 
the methods required to control the main controller features, such as current measurement, 
power control, and other hardware functions.
*/

#pragma once

#include <Arduino.h>
#include <Adafruit_INA219.h>
#include "driver/twai.h"

class Bigbrother {
private:
  static constexpr uint8_t ADDR_FLIGHT_COMP = 0b11;  // FC CAN adrr

  static constexpr uint16_t VALID_START_REC = 0x42A5;  // msg verification codes
  static constexpr uint16_t VALID_STOP_REC = 0xA542;

  // GPIO definition
  static constexpr gpio_num_t LED = GPIO_NUM_2;
  static constexpr gpio_num_t PWR_CTRL = GPIO_NUM_3;
  static constexpr gpio_num_t TRIG = GPIO_NUM_10;

  static constexpr gpio_num_t CAN_RX = GPIO_NUM_4;
  static constexpr gpio_num_t CAN_TX = GPIO_NUM_5;

  static constexpr gpio_num_t UART_RX = GPIO_NUM_20;
  static constexpr gpio_num_t UART_TX = GPIO_NUM_21;

  static constexpr uint16_t PWR_TRESHOLD = 200.0f;  // Power on current TRESHOLD
  static constexpr uint8_t REC_TRESHOLD = 20.0f;    // Record current TRESHOLD

  Adafruit_INA219 ina219;

  const uint8_t ADDR_BIGBRO;  // BigBrother adrr depending on location on LV
  bool alone;

  float base_current = 0.0f;

  // Methods used to manage UART bus
  void UARTinit();
  void UARToff();

public:
  // CAN cmd
  enum class MessageType : uint16_t {
    START_REC = 0b111000000,
    STOP_REC = 0b000111000,
    HEALTH = 0b000000111,
    NONE = 0b000000000
  };

  Bigbrother(uint8_t addr);  // Constructor
  void begin();              // Initialisation

  void cameraON();
  void cameraOFF();

  bool isON();
  bool isRec();

  void blinkLED(uint8_t rep, int duration);
  float measureCurrent(uint8_t rep = 10);
  void end();

  void send_UART_CMD(const uint8_t *data, size_t len);
  void send_health_packet(bool recording_status);
  MessageType listen_CAN();

  bool get_mode();
};