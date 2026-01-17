#pragma once
#include <iostream>
#include <Arduino.h>

#define PWM_FREQ 50     // ESC frequency
#define PWM_RES 16      // Resolution
#define PWM_ARM_US 1000 // ESC OFF
#define PWM_RUN_US 1200 // Low speed test

static inline int escPins[4] = {12, 13, 14, 15};

class BldcMotor
{
private:
public:
    BldcMotor();
    ~BldcMotor();
    void StopAllMotors();
    uint32_t GetDutyCycle(uint32_t us);
    void ArmESC();
    void RunMotor(int motorNumber, uint32_t pwmDutyCycle);
};


