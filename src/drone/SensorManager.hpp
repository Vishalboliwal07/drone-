#pragma once

struct SensorReadings {
  float accelX;
  float accelY;
  float accelZ;
  float gyroX;
  float gyroY;
  float gyroZ;
};

class SensorManager {
 public:
  bool begin();
  bool read(SensorReadings& readings);
  float altitudeMeters(float seaLevelPressureHpa);
};
