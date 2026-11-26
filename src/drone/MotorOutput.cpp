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
  const uint32_t periodUs = 1000000UL / drone::board::pwmFrequencyHz;
  const uint32_t maxDuty = (1UL << drone::board::pwmResolutionBits) - 1UL;
  const int boundedPulse = std::clamp(pulseWidthUs,
                                      static_cast<int>(drone::board::minPulseUs),
                                      static_cast<int>(drone::board::maxPulseUs));
  return static_cast<uint32_t>((static_cast<uint64_t>(boundedPulse) * maxDuty) / periodUs);
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
                        bool armed, bool calibrating, int flightMode, int yawBias) {
  if (!armed || calibrating) {
    stop();
    return;
  }

  int frontLeft;
  int frontRight;
  int rearLeft;
  int rearRight;
  if (flightMode == 0) {
    frontLeft = basePulseUs + static_cast<int>(roll - pitch - yaw);
    frontRight = basePulseUs + static_cast<int>(-roll - pitch + yaw);
    rearLeft = basePulseUs + static_cast<int>(roll + pitch + yaw);
    rearRight = basePulseUs + static_cast<int>(-roll + pitch - yaw);
  } else {
    const float correctedYaw = yaw + yawBias;
    const int yawFrontLeft = correctedYaw < 0.0f ? static_cast<int>(-correctedYaw) : 0;
    const int yawFrontRight = correctedYaw > 0.0f ? static_cast<int>(correctedYaw) : 0;
    const int yawRearLeft = correctedYaw > 0.0f ? static_cast<int>(correctedYaw) : 0;
    const int yawRearRight = correctedYaw < 0.0f ? static_cast<int>(-correctedYaw) : 0;
    frontLeft = basePulseUs + static_cast<int>(roll - pitch) + yawFrontLeft;
    frontRight = basePulseUs + static_cast<int>(-roll - pitch) + yawFrontRight;
    rearLeft = basePulseUs + static_cast<int>(roll + pitch) + yawRearLeft;
    rearRight = basePulseUs + static_cast<int>(-roll + pitch) + yawRearRight;
  }

  const int outputs[] = {
      frontLeft + drone::board::trimFrontLeftUs,
      frontRight + drone::board::trimFrontRightUs,
      rearLeft + drone::board::trimRearLeftUs,
      rearRight + drone::board::trimRearRightUs,
  };
  for (size_t i = 0; i < 4; ++i) {
    const int bounded = std::clamp(outputs[i], static_cast<int>(drone::board::minPulseUs),
                                   static_cast<int>(drone::board::maxPulseUs));
    ledcWrite(motorPins[i], pulseToDuty(bounded));
  }
}
