#pragma once

#include <Arduino.h>
#include <C610.h>
#include <FlexCAN_T4.h>

enum class C610Subbus { kIDZeroToThree, kIDFourToSeven };

template <CAN_DEV_TABLE _bus = CAN1>
class C610Bus {
 public:
  static const uint8_t kSize = 8;

 private:
  static const uint32_t kIDZeroToThreeCommandID = 0x200;
  static const uint32_t kFourToSevenCommandID = 0x1FF;
  static const uint32_t kReceiveBaseID = 0x200;
  static const uint32_t kCountsPerRev = 8192;

  bool is_initialized_ = false;
  C610 controllers_[kSize];
  FlexCAN_T4<_bus, RX_SIZE_256, TX_SIZE_16> can_;

  void InitializeCAN();

  static void TorqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower);

 public:
  C610Bus();
  void PollCAN();
  void Callback(const CAN_message_t &msg);
  void CommandTorques(const int32_t torque0, const int32_t torque1 = 0,
                      const int32_t torque2 = 0, const int32_t torque3 = 0,
                      C610Subbus subbus = 0);
  C610 &Get(const uint8_t i);
};

#include "C610Bus.tpp"