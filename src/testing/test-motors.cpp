#include <Arduino.h>
#include "lib_motor.h"

    BldcMotor* bldcMotor = new BldcMotor();

void setup()
{
    Serial.begin(115200);
    delay(2000);

    Serial.println("ESC ARMING...");
    bldcMotor->ArmESC();

    // Attach ESC pins

    delay(5000); // ESC arming time
    Serial.println("ESC ARMED");
}

void loop()
{

    for (int i = 0; i < 4; i++)
    {
        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.println(" ON");

        bldcMotor->StopAllMotors();
        delay(1000);

        bldcMotor->RunMotor(i,PWM_RUN_US);
        delay(4000);

        Serial.print("Motor ");
        Serial.print(i + 1);
        Serial.println(" OFF");

        bldcMotor->StopAllMotors();
        delay(3000);
    }
}