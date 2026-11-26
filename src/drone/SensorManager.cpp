#include "SensorManager.hpp"

#include <Arduino.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Wire.h>
#include <cmath>

namespace {
Adafruit_LSM6DS3TRC imu;
Adafruit_BMP280 barometer;
}

bool SensorManager::begin() {
  Wire.begin();
  Wire.setTimeOut(3000);
  if (!imu.begin_I2C(0x6A)) return false;
  imu.setAccelRange(LSM6DS_ACCEL_RANGE_8_G);
  imu.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  imu.setAccelDataRate(LSM6DS_RATE_208_HZ);
  imu.setGyroDataRate(LSM6DS_RATE_208_HZ);
  return barometer.begin(0x76);
}

bool SensorManager::read(SensorReadings& readings) {
  sensors_event_t accel;
  sensors_event_t gyro;
  sensors_event_t temperature;
  if (!imu.getEvent(&accel, &gyro, &temperature)) return false;

  readings = {
      accel.acceleration.x,
      accel.acceleration.y,
      accel.acceleration.z,
      gyro.gyro.x,
      gyro.gyro.y,
      gyro.gyro.z,
  };
  return std::isfinite(readings.accelX) && std::isfinite(readings.accelY) &&
         std::isfinite(readings.accelZ) && std::isfinite(readings.gyroX) &&
         std::isfinite(readings.gyroY) && std::isfinite(readings.gyroZ);
}

float SensorManager::altitudeMeters(float seaLevelPressureHpa) {
  return barometer.readAltitude(seaLevelPressureHpa);
}
