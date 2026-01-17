#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include "lib_globalConstants.h"
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>



void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status);

int mapJoystick(int val, int centerVal, int minOut, int maxOut);

bool buttonPressed(Button &btn);

void beep();

void initButtons();

void ButtonTask(void *pvParameters);

void ControllerTask(void *pvParameters);


void CalibrateCenter();

void StartupSound();

void EspNowSetup();

