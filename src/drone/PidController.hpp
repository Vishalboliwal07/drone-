#pragma once

class PidController {
 public:
  void configure(float kp, float ki, float kd, float feedForward);
  void reset();
  void resetIntegral();
  float update(float setpoint, float measurement, float dtSeconds,
               float throttle, float derivativeCutoffHz = 60.0f);

 private:
  float _kp = 0.0f;
  float _ki = 0.0f;
  float _kd = 0.0f;
  float _feedForward = 0.0f;
  float _integral = 0.0f;
  float _previousMeasurement = 0.0f;
  float _filteredDerivative = 0.0f;
};
