#include <Arduino.h>

#include "lib_droneConstants.h"
#include "lib_droneFunctions.h"

void setup()
{
    Serial.begin(115200);
    setCpuFrequencyMhz(240);
    setupESPNow();
    setupMotors();
    xTaskCreatePinnedToCore(flightTask, "FlightTask", 6144, NULL, 5, NULL, 1);
}

void loop()
{
}
