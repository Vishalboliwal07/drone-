#pragma once

#include <cstdint>

namespace drone::board {
constexpr uint8_t powerLatch = 4;
constexpr uint8_t led1 = 18;
constexpr uint8_t led2 = 19;
constexpr uint8_t motorFrontLeft = 12;
constexpr uint8_t motorRearLeft = 13;
constexpr uint8_t motorRearRight = 14;
constexpr uint8_t motorFrontRight = 15;
constexpr uint16_t pwmFrequencyHz = 416;
constexpr uint8_t pwmResolutionBits = 10;
constexpr uint16_t minPulseUs = 1000;
constexpr uint16_t maxPulseUs = 2000;
constexpr float seaLevelPressureHpa = 1013.25f;
constexpr float maxAltitudeMeters = 10.0f;
}
