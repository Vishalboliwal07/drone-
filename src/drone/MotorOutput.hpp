#pragma once

#include <cstdint>

class MotorOutput {
 public:
  bool begin();
  void stop();
  void write(int basePulseUs, float roll, float pitch, float yaw,
             bool armed, bool calibrating, int flightMode, int yawBias,
             int trimFrontLeftUs, int trimFrontRightUs,
             int trimRearLeftUs, int trimRearRightUs);

 private:
  static uint32_t pulseToDuty(int pulseWidthUs);
};
