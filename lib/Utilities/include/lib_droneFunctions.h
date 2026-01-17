#pragma once

#include "lib_droneConstants.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <Adafruit_LSM6DS3TRC.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>


void flightTask(void *p);

void setupESPNow();

void OnDataRecv(const esp_now_recv_info_t *info, const uint8_t *incomingDataPtr, int len);

void setupMotors();

uint32_t pulseToDuty(int pulseWidth);

void updateMotors(int basePulse, int baseYaw, int basePitch, int baseRoll);