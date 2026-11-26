#include "MotorOutput.hpp"

#include <algorithm>
#include <Arduino.h>

#include "drone/board_config.hpp"

namespace {
constexpr uint8_t motorPins[] = {
    drone::board::motorFrontLeft,
    drone::board::motorFrontRight,
    drone::board::motorRearLeft,
    drone::board::motorRearRight,
};
}

uint32_t MotorOutput::pulseToDuty(int pulseWidthUs) {
  return (pulseWidthUs * 1024) / 2404;
}

bool MotorOutput::begin() {
  bool attached = true;
  for (uint8_t pin : motorPins) {
    attached = ledcAttach(pin, drone::board::pwmFrequencyHz,
                          drone::board::pwmResolutionBits) && attached;
  }
  stop();
  return attached;
}

void MotorOutput::stop() {
  for (uint8_t pin : motorPins) {
    ledcWrite(pin, pulseToDuty(drone::board::minPulseUs));
  }
}

void MotorOutput::write(int basePulseUs, float roll, float pitch, float yaw,
                        bool armed, bool calibrating, int flightMode, int yawBias,
                        int trimFrontLeftUs, int trimFrontRightUs,
                        int trimRearLeftUs, int trimRearRightUs) {
  if (!armed || calibrating) {
    stop();
    return;
  }

  int frontLeft;
  int frontRight;
  int rearLeft;
  int rearRight;
  if (flightMode == 0) {
    frontLeft = basePulseUs + roll - pitch - yaw;
    frontRight = basePulseUs - roll - pitch + yaw;
    rearLeft = basePulseUs + roll + pitch + yaw;
    rearRight = basePulseUs - roll + pitch - yaw;
  } else {
    const float correctedYaw = yaw + yawBias;
    const float yawFrontLeft = correctedYaw < 0.0f ? -correctedYaw : 0.0f;
    const float yawFrontRight = correctedYaw > 0.0f ? correctedYaw : 0.0f;
    const float yawRearLeft = correctedYaw > 0.0f ? correctedYaw : 0.0f;
    const float yawRearRight = correctedYaw < 0.0f ? -correctedYaw : 0.0f;
    frontLeft = basePulseUs + roll - pitch + yawFrontLeft;
    frontRight = basePulseUs - roll - pitch + yawFrontRight;
    rearLeft = basePulseUs + roll + pitch + yawRearLeft;
    rearRight = basePulseUs - roll + pitch + yawRearRight;
  }

  const int outputs[] = {
      frontLeft + trimFrontLeftUs,
      frontRight + trimFrontRightUs,
      rearLeft + trimRearLeftUs,
      rearRight + trimRearRightUs,
  };
  for (size_t i = 0; i < 4; ++i) {
    const int bounded = std::clamp(outputs[i], static_cast<int>(drone::board::minPulseUs),
                                   static_cast<int>(drone::board::maxPulseUs));
    ledcWrite(motorPins[i], pulseToDuty(bounded));
  }
}
