#include "C610.h"

#include <FlexCAN_T4.h>

namespace C610Helper {
  int sign(float val) { return (val > 0) - (val < 0); }
}

C610::C610() {
  _initialized_mechanical_angle = false;
  _rotations = 0;
  _last_pos_measurement = 0;
  _counts = 0;
  _rpm = 0;
  _current = 0;
}

C610Feedback C610::InterpretMessage(const CAN_message_t &msg) {
  // See C610 manual for protocol details
  C610Feedback r = {.counts = uint16_t((msg.buf[0] << 8) | msg.buf[1]),
                    .rpm = int16_t((msg.buf[2] << 8) | msg.buf[3]),
                    .current = int16_t((msg.buf[4] << 8) | msg.buf[5])};
  return r;
}

float C610::Position() { return _counts / kCountsPerRad; }

float C610::Velocity() { return _rpm / kRPMPerRadS; }

float C610::ElectricalPower() {
  return Current() * Current() * kResistance + kVoltageConstant * Velocity() * Current();
}

// backdriving: torque = -67.297289572517 [mNm] + -2.769829798378095 * w[rad/s] + 0.30812359545248624 * i[mA]
// forward: torque = -13.649400446367881 [mNm] + -4.94437013621672 * w[rad/s] + 0.17862214298313905 * i[mA]
float C610::Torque() {
  if (_rpm * _current > 0) {
    return -0.0673 * C610Helper::sign(Velocity()) - 0.00277 * Velocity() + 0.000308 * _current;
  } else {
    return -0.0136 * C610Helper::sign(Velocity()) - 0.00494 * Velocity() + 0.000179 * _current;
  }
}

float C610::MechanicalPower() {
  return Torque() * Velocity();
}

float C610::Current() { return _current / kMilliAmpPerAmp; }

void C610::UpdateState(C610Feedback f) {
  // Initial setup
  if (!_initialized_mechanical_angle) {
    _initialized_mechanical_angle = true;
    _last_pos_measurement = f.counts;
  }

  // Position
  int32_t delta = f.counts - _last_pos_measurement;
  if (delta > kCountsPerRev /
                  2) {  // Crossed from >= 0 counts to <= 8191 counts. Could
                        // also trigger if spinning super fast (>2000rps)
    _rotations -= 1;
  } else if (delta <
             -kCountsPerRev /
                 2) {  // Crossed from <= 8191 counts to >= 0 counts. Could
                       // also trigger if spinning super fast (>2000rps)
    _rotations += 1;
  }
  _counts = _rotations * kCountsPerRev + f.counts;
  _last_pos_measurement = f.counts;

  // Velocity
  _rpm = f.rpm;

  // Torque
  _current = f.current;
}