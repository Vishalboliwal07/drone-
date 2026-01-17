#pragma once

#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>



uint8_t inline droneMacAddress[] = {0x68, 0x25, 0xDD, 0xCC, 0xB8, 0x04};

#define CONTROLLER_TASK_HZ 100
#define CONTROLLER_TASK_PERIOD_MS (1000 / CONTROLLER_TASK_HZ)

// === JOYSTICKS ===
#define PIN_THROTTLE 34
#define PIN_YAW 35
#define PIN_ROLL 36
#define PIN_PITCH 39

// === FUNCTION BUTTONS ===
#define PIN_ARM 26
#define PIN_POWER 13
#define PIN_BUZZER 15
#define PIN_LED 21

// === STEP SIZE BUTTONS ===
#define PIN_S1_INC 12   // Increase Step
#define PIN_S2_DEC 25   // Decrease Step
#define PIN_S3_RESET 14 // RESET ALL TRIMS (NEW!)

// === TRIM BUTTONS ===
#define PIN_S5_RIGHT 16
#define PIN_S6_LEFT 18
#define PIN_S7_BACK 17
#define PIN_S8_FRONT 19

#define DEBOUNCE_MS 30
#define BUTTON_PERIOD 10 // task polling period (ms)

esp_now_peer_info_t inline peerInfo;


typedef struct
{
    uint8_t pin;
    bool stableState;
    bool lastReading;
    unsigned long lastChangeTime;
} Button;

unsigned long inline lastPowerLedTime;
bool inline powerLedState = false;

struct struct_message
{
    int throttle;
    int yaw;
    int pitch;
    int roll;
    bool armed;
};

struct_message inline myData;
bool inline outputOff = true;


// Logic Variables
bool inline isArmed = false;
bool inline isPowered = false;
bool inline lastArmBtnState = HIGH;
unsigned long inline buttonTimer = 0;
bool inline buttonActive = false;

// Trim Storage
int inline trimRoll = 0;
int inline trimPitch = 0;
int inline trimYaw = 0;

// Step Size Logic
int inline stepValues[] = {5, 10, 20, 25, 50, 75, 100};
int inline stepIndex = 0; // Start at 5
int inline currentStepSize = 5;

// Button States (Memory)
bool inline lastS1 = HIGH;
bool inline lastS2 = HIGH;
bool inline lastS3 = HIGH;
bool inline lastS5 = HIGH;
bool inline lastS6 = HIGH;
bool inline lastS7 = HIGH;
bool inline lastS8 = HIGH;

// Calibration Centers
int inline yawCenter = 0;
int inline pitchCenter = 0;
int inline rollCenter = 0;

int inline rawThrottle = 0;
int inline rawYaw = 0;
int inline rawPitch = 0;
int inline rawRoll = 0;