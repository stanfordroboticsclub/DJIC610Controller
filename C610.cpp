#include "C610.h"

#include <FlexCAN_T4.h>

C610::C610() {
  _initialized_mechanical_angle = false;
  _rotations = 0;
  _last_pos_measurement = 0;
  _counts = 0;
  _rpm = 0;
  _torque = 0;
}

void C610::torqueToBytes(int16_t torque, uint8_t &upper, uint8_t &lower) {
  upper = (torque >> 8) & 0xFF;
  lower = torque & 0xFF;
}

C610Feedback C610::interpretMessage(const CAN_message_t &msg) {
  // See C610 manual for protocol details
  C610Feedback r = {.counts = uint16_t((msg.buf[0] << 8) | msg.buf[1]),
                    .rpm = int16_t((msg.buf[2] << 8) | msg.buf[3]),
                    .torque = int16_t((msg.buf[4] << 8) | msg.buf[5])};
  return r;
}

int32_t C610::counts() { return _counts; }

int32_t C610::rpm() { return _rpm; }

int32_t C610::torque() { return _torque; }

void C610::updateState(C610Feedback f) {
  // Initial setup
  if (!_initialized_mechanical_angle) {
    _initialized_mechanical_angle = true;
    _last_pos_measurement = f.counts;
  }

  // Position
  int32_t delta = f.counts - _last_pos_measurement;
  if (delta > COUNTS_PER_REV /
                  2) {  // Crossed from >= 0 counts to <= 8191 counts. Could
                        // also trigger if spinning super fast (>2000rps)
    _rotations -= 1;
  } else if (delta <
             -COUNTS_PER_REV /
                 2) {  // Crossed from <= 8191 counts to >= 0 counts. Could
                       // also trigger if spinning super fast (>2000rps)
    _rotations += 1;
  }
  _counts = _rotations * COUNTS_PER_REV + f.counts;
  _last_pos_measurement = f.counts;

  // Velocity
  _rpm = f.rpm;

  // Torque
  _torque = f.torque;
}