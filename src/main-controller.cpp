#include "lib_globalConstants.h"
#include "lib_functions.h"

void setup()
{
  Serial.begin(115200);
  setCpuFrequencyMhz(240);

  initButtons();

  EspNowSetup();

  CalibrateCenter();

  xTaskCreate(
      ButtonTask,
      "ButtonTask",
      2048,
      NULL,
      1,
      NULL);

  xTaskCreate(
      ControllerTask,
      "ControllerTask",
      2048,
      NULL,
      5,
      NULL);

  StartupSound();
}

void loop()
{
}