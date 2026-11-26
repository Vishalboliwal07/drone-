#include "PidController.hpp"

#include <algorithm>
#include <cmath>

void PidController::configure(float kp, float ki, float kd, float feedForward) {
  _kp = kp;
  _ki = ki;
  _kd = kd;
  _feedForward = feedForward;
  reset();
}

void PidController::reset() {
  _integral = 0.0f;
  _previousMeasurement = 0.0f;
  _filteredDerivative = 0.0f;
}

float PidController::update(float setpoint, float measurement, float dtSeconds,
                            float throttle, float derivativeCutoffHz) {
  const float dt = (dtSeconds > 0.0f && std::isfinite(dtSeconds)) ? dtSeconds : 0.005f;
  const float error = setpoint - measurement;

  if (throttle > 50.0f) {
    _integral = std::clamp(_integral + error * _ki * dt, -150.0f, 150.0f);
  } else {
    _integral = 0.0f;
  }

  const float cutoff = std::max(derivativeCutoffHz, 0.01f);
  const float rc = 1.0f / (2.0f * 3.14159265358979323846f * cutoff);
  const float alpha = dt / (dt + rc);
  const float rawDerivative = (measurement - _previousMeasurement) / dt;
  _filteredDerivative = alpha * rawDerivative + (1.0f - alpha) * _filteredDerivative;
  _previousMeasurement = measurement;

  const float output = _kp * error + _integral - _kd * _filteredDerivative + _feedForward * setpoint;
  return std::clamp(output, -300.0f, 300.0f);
}
