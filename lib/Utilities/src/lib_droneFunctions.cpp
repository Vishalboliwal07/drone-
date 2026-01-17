#include "lib_droneFunctions.h"

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataPtr, int len)
{
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    lastRecvTime = millis();
}

void setupESPNow()
{
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (esp_now_init() != ESP_OK)
        return;
    esp_now_register_recv_cb(OnDataRecv);
}

void setupMotors()
{
    ledcAttach(MOTOR_FL, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_FR, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_RL, PWM_FREQ, PWM_RES);
    ledcAttach(MOTOR_RR, PWM_FREQ, PWM_RES);
    ledcWrite(MOTOR_FL, pulseToDuty(MIN_PULSE));
    ledcWrite(MOTOR_FR, pulseToDuty(MIN_PULSE));
    ledcWrite(MOTOR_RL, pulseToDuty(MIN_PULSE));
    ledcWrite(MOTOR_RR, pulseToDuty(MIN_PULSE));
}

uint32_t pulseToDuty(int pulseWidth)
{
    return (pulseWidth * 1024) / 2404;
}


void updateMotors(int basePulse, int baseYaw, int basePitch, int baseRoll)
{
  // int fl = basePulse;
  // int fr = basePulse;
  // int rl = basePulse;
  // int rr = basePulse;

  int fl = basePulse + testControlScaling * baseRoll - testControlScaling * basePitch - testControlScaling * baseYaw;
  int fr = basePulse - testControlScaling * baseRoll - testControlScaling * basePitch + testControlScaling * baseYaw;
  int rl = basePulse + testControlScaling * baseRoll + testControlScaling * basePitch + testControlScaling * baseYaw;
  int rr = basePulse - testControlScaling * baseRoll + testControlScaling * basePitch - testControlScaling * baseYaw;

  // fl += -(x[0] * kp) + (x[2] * kp);
  // fr += +(x[0] * kp) + (x[2] * kp);
  // rl += -(x[0] * kp) - (x[2] * kp);
  // rr += +(x[0] * kp) - (x[2] * kp);
//   fl += +(RollPid.Output()) - (PitchPid.Output());
//   fr += -(RollPid.Output()) - (PitchPid.Output());
//   rl += +(RollPid.Output()) + (PitchPid.Output());
//   rr += -(RollPid.Output()) + (PitchPid.Output());

  fl = constrain(fl, MIN_PULSE, MAX_PULSE);
  fr = constrain(fr, MIN_PULSE, MAX_PULSE);
  rl = constrain(rl, MIN_PULSE, MAX_PULSE);
  rr = constrain(rr, MIN_PULSE, MAX_PULSE);

  ledcWrite(MOTOR_FL, pulseToDuty(fl));
  ledcWrite(MOTOR_FR, pulseToDuty(fr));
  ledcWrite(MOTOR_RL, pulseToDuty(rl));
  ledcWrite(MOTOR_RR, pulseToDuty(rr));
}

void flightTask(void *p)
{
  while (1)
  {
    // RollPid.Update(0.0,x[0],(millis()/1000));
    // PitchPid.Update(0.0,x[2],(millis()/1000));
    // YawPid.Update(0.0,x[0],millis()/1000);
    int basePulse = map(incomingData.throttle, 0, 100, MIN_PULSE, MAX_PULSE);
    updateMotors(basePulse, incomingData.yaw, incomingData.pitch, incomingData.roll);
    vTaskDelay(pdMS_TO_TICKS(2)); // 500 Hz
  }
}