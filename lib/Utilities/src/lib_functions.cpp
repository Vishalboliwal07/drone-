#include "lib_functions.h"

Button buttonPower = {PIN_POWER, HIGH, HIGH, 0};
Button buttonArm = {PIN_ARM, HIGH, HIGH, 0};
Button btnS5 = {PIN_S5_RIGHT, HIGH, HIGH, 0};
Button btnS6 = {PIN_S6_LEFT, HIGH, HIGH, 0};
Button btnS7 = {PIN_S7_BACK, HIGH, HIGH, 0};
Button btnS8 = {PIN_S8_FRONT, HIGH, HIGH, 0};
Button btnS1 = {PIN_S1_INC, HIGH, HIGH, 0};
Button btnS2 = {PIN_S2_DEC, HIGH, HIGH, 0};
Button btnS3 = {PIN_S3_RESET, HIGH, HIGH, 0};

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

int mapJoystick(int val, int centerVal, int minOut, int maxOut)
{
    int diff = val - centerVal;
    if (abs(diff) < 100)
        return 0; // Deadzone
    return map(diff, -2048, 2048, minOut, maxOut);
}

bool buttonPressed(Button &btn)
{
    bool reading = digitalRead(btn.pin);
    unsigned long now = millis();

    if (reading != btn.lastReading)
    {
        btn.lastChangeTime = now;
    }

    if ((now - btn.lastChangeTime) > DEBOUNCE_MS)
    {
        if (reading != btn.stableState)
        {
            btn.stableState = reading;

            // Detect falling edge (pressed)
            if (btn.stableState == LOW)
            {
                btn.lastReading = reading;
                return true;
            }
        }
    }

    btn.lastReading = reading;
    return false;
}

void CalibrateCenter()
{
    Serial.println("Calibrating... Keep Sticks Centered.");
    long ySum = 0, pSum = 0, rSum = 0;
    for (int i = 0; i < 50; i++)
    {
        ySum += analogRead(PIN_YAW);
        pSum += analogRead(PIN_PITCH);
        rSum += analogRead(PIN_ROLL);
        delay(10);
    }
    yawCenter = ySum / 50;
    pitchCenter = pSum / 50;
    rollCenter = rSum / 50;
}

void beep()
{
    digitalWrite(PIN_BUZZER, HIGH);
    vTaskDelay(pdMS_TO_TICKS(20));
    digitalWrite(PIN_BUZZER, LOW);
}

void initButtons()
{
    pinMode(PIN_S5_RIGHT, INPUT_PULLUP);
    pinMode(PIN_S6_LEFT, INPUT_PULLUP);
    pinMode(PIN_S7_BACK, INPUT_PULLUP);
    pinMode(PIN_S8_FRONT, INPUT_PULLUP);

    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);

    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_LED, HIGH);

    pinMode(PIN_THROTTLE, INPUT);
    pinMode(PIN_YAW, INPUT);
    pinMode(PIN_ROLL, INPUT);
    pinMode(PIN_PITCH, INPUT);

    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_POWER, INPUT_PULLUP);

    pinMode(PIN_S1_INC, INPUT_PULLUP);
    pinMode(PIN_S2_DEC, INPUT_PULLUP);
    pinMode(PIN_S3_RESET, INPUT_PULLUP); // NEW
}

void ButtonTask(void *pvParameters)
{
    while (true)
    {
        if (buttonPressed(btnS1))
        {
            if (stepIndex < 6)
            {
                stepIndex++;
                currentStepSize = stepValues[stepIndex];
                beep();
            }
        }

        if (buttonPressed(btnS2))
        {
            if (stepIndex > 6)
            {
                stepIndex--;
                currentStepSize = stepValues[stepIndex];
                beep();
            }
        }

        if (buttonPressed(btnS3))
        {
            trimRoll = 0;
            trimPitch = 0;
            Serial.println("!!! TRIMS RESET TO ZERO !!!");
            beep();
        }

        if (buttonPressed(btnS5))
        {
            trimYaw -= currentStepSize;
            Serial.printf("S5 RIGHT -> Yaw: %d\n", trimYaw);
            beep();
        }

        if (buttonPressed(btnS6))
        {
            trimYaw += currentStepSize;
            Serial.printf("S6 LEFT -> Yaw: %d\n", trimYaw);
            beep();
        }

        if (buttonPressed(btnS7))
        {
            trimPitch += currentStepSize;
            Serial.printf("Trim Forward (%d) -> Pitch: %d\n", currentStepSize, trimPitch);
            beep();
        }

        if (buttonPressed(buttonPower))
        {
            isPowered = !isPowered;
            Serial.printf("Power toggle\n");
            beep();
        }

        if (buttonPressed(buttonArm))
        {
            isArmed = !isArmed;
            Serial.printf("Arming toggle\n");
            beep();
        }

        if (buttonPressed(btnS8))
        {
            trimPitch -= currentStepSize;
            Serial.printf("Trim Backward (%d) -> Pitch: %d\n", currentStepSize, trimPitch);
            beep();
        }
        if (isPowered)
        {
            if (isArmed)
            {
                if (outputOff){
                    outputOff = false;
                }
                digitalWrite(PIN_LED, HIGH);
            }
            else
            {
                if(!outputOff){
                    outputOff = true;
                }
                unsigned long now = millis();

                if ((now - lastPowerLedTime) > 500)
                {

                    digitalWrite(PIN_LED, powerLedState);
                    powerLedState = !powerLedState;
                    lastPowerLedTime = millis();
                }
            }
        }
        else
        {
            isArmed = false;
            if (!outputOff){
                outputOff = true;
            }
            digitalWrite(PIN_LED, LOW);
        }

        vTaskDelay(pdMS_TO_TICKS(BUTTON_PERIOD));
    }
}

void StartupSound()
{
    for (int i = 0; i < 4; i++)
    {
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_BUZZER, HIGH);
        vTaskDelay(pdMS_TO_TICKS(50));
        digitalWrite(PIN_BUZZER, LOW);
    }
}

void EspNowSetup()
{
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK)
        return;
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

    memcpy(peerInfo.peer_addr, droneMacAddress, 6);
    peerInfo.channel = 1;
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
}

void ControllerTask(void *pvParameters)
{
    const TickType_t period =
        pdMS_TO_TICKS(CONTROLLER_TASK_PERIOD_MS);
    TickType_t lastWakeTime = xTaskGetTickCount();

    while (true)
    {
        if (!outputOff){

            rawThrottle = analogRead(PIN_THROTTLE);
            rawYaw = analogRead(PIN_YAW);
            rawPitch = analogRead(PIN_PITCH);
            rawRoll = analogRead(PIN_ROLL);
            int t = map(rawThrottle, 0, 4095, 0, 100);
            if (t < 15)
                t = 0;
            myData.throttle = t;
            myData.yaw = mapJoystick(rawYaw, yawCenter, -75, 75) + trimYaw;
    
            myData.pitch = mapJoystick(rawPitch, pitchCenter, -75, 75) + trimPitch;
            myData.roll = mapJoystick(rawRoll, rollCenter, 75, -75) + trimRoll;
            esp_now_send(droneMacAddress, (uint8_t *)&myData, sizeof(myData));
            vTaskDelayUntil(&lastWakeTime, period);
        }
        else{
            myData.throttle = 0;
            myData.yaw = 0;
            myData.pitch = 0;
            myData.roll = 0;
            esp_now_send(droneMacAddress, (uint8_t *)&myData, sizeof(myData));
            vTaskDelayUntil(&lastWakeTime, period);
        }
    }
}
