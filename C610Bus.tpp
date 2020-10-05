#include "Arduino.h"
#include "C610Bus.h"

template <CAN_DEV_TABLE _bus>
C610Bus<_bus>::C610Bus() {
  for (uint8_t i = 0; i < kSize; i++) {
    controllers_[i] = C610();
  }
  InitializeCAN();
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::TorqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower) {
  upper = (torque >> 8) & 0xFF;
  lower = torque & 0xFF;
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::PollCAN() {
  can_.events();
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::InitializeCAN() {
  can_.begin();
  can_.setBaudRate(1000000);
  can_.setMaxMB(16);
  can_.enableFIFO();
  can_.enableFIFOInterrupt();
  can_.mailboxStatus();
  can_.onReceive([this](const CAN_message_t &msg) { this->Callback(msg); });
  is_initialized_ = true;
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::Callback(const CAN_message_t &msg) {
  if (msg.id >= kReceiveBaseID + 1 && msg.id <= kReceiveBaseID + kSize) {
    uint8_t esc_index =
        msg.id - kReceiveBaseID - 1;  // ESC 1 corresponds to index 0
    C610Feedback f = C610::InterpretMessage(msg);
    controllers_[esc_index].UpdateState(f);
  } else {
    Serial.print("Invalid ID for feedback message: ");
    Serial.println(msg.id);
    return;
  }
}

template <CAN_DEV_TABLE _bus>
void C610Bus<_bus>::CommandTorques(const int32_t torque0, const int32_t torque1,
                                   const int32_t torque2, const int32_t torque3,
                                   C610Subbus subbus) {
  if (!is_initialized_) {
    Serial.println("Bus must be initialized before use.");
  }
  // IDs 0 through 3 go on ID 0x200
  // IDs 4 through 7 go on ID 0x1FF

  int16_t t0 =
      constrain(torque0, -32000, 32000);  // prevent overflow of int16_t
  int16_t t1 = constrain(torque1, -32000, 32000);
  int16_t t2 = constrain(torque2, -32000, 32000);
  int16_t t3 = constrain(torque3, -32000, 32000);

  CAN_message_t msg;
  if (subbus == C610Subbus::kIDZeroToThree) {
    msg.id = kIDZeroToThreeCommandID;
  } else if (subbus == C610Subbus::kIDFourToSeven) {
    msg.id = kFourToSevenCommandID;
  } else {
    Serial.println("Invalid ESC subbus.");
    return;
  }
  TorqueToBytes(t0, msg.buf[0], msg.buf[1]);
  TorqueToBytes(t1, msg.buf[2], msg.buf[3]);
  TorqueToBytes(t2, msg.buf[4], msg.buf[5]);
  TorqueToBytes(t3, msg.buf[6], msg.buf[7]);
  can_.write(msg);
}

template <CAN_DEV_TABLE _bus>
C610 &C610Bus<_bus>::Get(const uint8_t i) {
  return controllers_[i];
}