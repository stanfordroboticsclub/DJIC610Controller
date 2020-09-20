#pragma once

#include <Arduino.h>
#include <C610.h>
#include <FlexCAN_T4.h>
#include <Streaming.h>

template <CAN_DEV_TABLE _bus = CAN1>
class C610Bus {
 public:
  static const uint8_t SIZE = 8;

 private:
  static const uint32_t ZERO_TO_THREE_COMMAND_ID = 0x200;
  static const uint32_t FOUR_TO_SEVEN_COMMAND_ID = 0x1FF;
  static const uint32_t RECEIVE_BASE_ID = 0x200;
  static const uint32_t COUNTS_PER_REV = 8192;

  bool is_initialized = false;
  C610 _controllers[SIZE];
  FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16> _can;

  void initializeCAN();

 public:
  C610Bus();
  void pollCAN();
  void callback(const CAN_message_t &msg);
  void commandTorques(const int32_t torque0, const int32_t torque1 = 0,
                      const int32_t torque2 = 0, const int32_t torque3 = 0,
                      const uint8_t subbus = 0);
  C610 &get(const uint8_t i);
};

#include "C610Bus.tpp"