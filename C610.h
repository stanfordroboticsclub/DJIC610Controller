#pragma once

#include <Arduino.h>
#include <FlexCAN_T4.h>

struct C610Feedback {
  int32_t counts, rpm, torque;
};

class C610 {
 private:
  uint8_t _initialized_mechanical_angle;
  int32_t _rotations;
  int32_t _last_pos_measurement;
  int32_t _counts;
  int32_t _rpm;
  int32_t _torque;

 public:
  static const int32_t COUNTS_PER_REV = 8192;

  static void torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower);
  static C610Feedback interpretMessage(const CAN_message_t &msg);

  C610();
  void updateState(C610Feedback feedback);
  int32_t counts();
  int32_t rpm();
  int32_t torque();
};
