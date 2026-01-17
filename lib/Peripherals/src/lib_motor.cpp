#include "../include/lib_motor.h"

BldcMotor::BldcMotor()
{
    std::cout << "Class created" << std::endl;
}

BldcMotor::~BldcMotor()
{
}

void BldcMotor::StopAllMotors()
{
    for (int i = 0; i < 4; i++)
    {
        ledcWrite(escPins[i], GetDutyCycle(PWM_ARM_US));
    }
}

uint32_t BldcMotor::GetDutyCycle(uint32_t microSeconds)
{
    uint32_t maxDuty = (1 << PWM_RES) - 1;
    uint32_t periodUs = 1000000 / PWM_FREQ;
    return (microSeconds * maxDuty) / periodUs;
}

void BldcMotor::ArmESC()
{
    for (int i = 0; i < 4; i++)
    {
        ledcAttach(escPins[i], PWM_FREQ, PWM_RES);
        ledcWrite(escPins[i], GetDutyCycle(PWM_ARM_US));
    }
}

void BldcMotor::RunMotor(int motorNumber,uint32_t pwmDutyCycle){
    ledcWrite(escPins[motorNumber], GetDutyCycle(pwmDutyCycle));
}