#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ================= HARDWARE CONFIGURATION =================
uint8_t droneMacAddress[] = {0x68, 0x25, 0xDD, 0xCC, 0x9E, 0xD4}; 

// === JOYSTICKS ===
#define PIN_THROTTLE 34  
#define PIN_YAW      35  
#define PIN_ROLL     36  
#define PIN_PITCH    39  

// === BUTTONS ===
#define PIN_ARM      26  
#define PIN_POWER    13  
#define PIN_BUZZER   15  
#define PIN_LED      21  

// === STEP / RESET BUTTONS ===
#define PIN_S1_INC   12  // Increase Step
#define PIN_S3_FUNC  14  // Short=Dec Step, Long=Reset Trims

// === TRIM BUTTONS ===
#define PIN_S5_RIGHT 16
#define PIN_S6_LEFT  18
#define PIN_S7_BACK  17
#define PIN_S8_FRONT 19

// =========================================================

typedef struct struct_message {
    int throttle;
    int yaw;
    int pitch;
    int roll;
    bool armed;
    int flightMode; 
    int yawBias; 
} struct_message;

struct_message myData;
esp_now_peer_info_t peerInfo;

// Logic Variables
bool isArmed = false;       
bool lastArmBtnState = HIGH;

// Power / Mode Button Logic
unsigned long buttonTimer = 0;
bool buttonActive = false;
bool longPressHandled = false; 

// FLIGHT MODES
int flightMode = 0; 
int lockedThrottle = 0; 

// TRIM STORAGE
int trimRoll = 0;
int trimPitch = 0;
int trimYaw = 0; 

// !!! CHANGE: Step Size Logic (Min is 1) !!!
int stepValues[] = {1, 5, 10, 20, 25, 50, 100}; // Changed 5 to 1 at start
int stepIndex = 1; // Default start at 5 (Index 1)
int currentStepSize = 5;

// Button States
bool lastS1 = HIGH; 
unsigned long timerS3 = 0; bool activeS3 = false; bool longS3 = false;
unsigned long timerS5 = 0; bool activeS5 = false; bool longS5 = false;
unsigned long timerS6 = 0; bool activeS6 = false; bool longS6 = false;
bool lastS7 = HIGH; bool lastS8 = HIGH;

// Calibration
int yawCenter = 0; int pitchCenter = 0; int rollCenter = 0;

void OnDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {}

int mapJoystick(int val, int centerVal, int minOut, int maxOut) {
    int diff = val - centerVal;
    if (abs(diff) < 100) return 0; 
    float norm = (float)diff / 2048.0;
    float curve = norm * norm * norm; 
    int range = (maxOut - minOut) / 2; 
    return (int)(curve * range);
}

void setup() {
    Serial.begin(115200);
    
    pinMode(PIN_BUZZER, OUTPUT); digitalWrite(PIN_BUZZER, LOW); 
    pinMode(PIN_LED, OUTPUT);    digitalWrite(PIN_LED, HIGH); 

    pinMode(PIN_THROTTLE, INPUT);
    pinMode(PIN_YAW, INPUT);
    pinMode(PIN_ROLL, INPUT);
    pinMode(PIN_PITCH, INPUT);
    
    pinMode(PIN_ARM, INPUT_PULLUP);
    pinMode(PIN_POWER, INPUT_PULLUP);
    
    pinMode(PIN_S1_INC, INPUT_PULLUP);
    pinMode(PIN_S3_FUNC, INPUT_PULLUP); 

    pinMode(PIN_S5_RIGHT, INPUT_PULLUP);
    pinMode(PIN_S6_LEFT, INPUT_PULLUP);
    pinMode(PIN_S7_BACK, INPUT_PULLUP);
    pinMode(PIN_S8_FRONT, INPUT_PULLUP);
    
    esp_sleep_enable_ext0_wakeup((gpio_num_t)PIN_POWER, 0);

    Serial.println("Calibrating...");
    long ySum=0, pSum=0, rSum=0;
    for(int i=0; i<50; i++) {
        ySum += analogRead(PIN_YAW);
        pSum += analogRead(PIN_PITCH);
        rSum += analogRead(PIN_ROLL);
        delay(10);
    }
    yawCenter = ySum / 50;
    pitchCenter = pSum / 50;
    rollCenter = rSum / 50;
    
    for(int i=0; i<3; i++) { digitalWrite(PIN_BUZZER, HIGH); delay(50); digitalWrite(PIN_BUZZER, LOW); delay(50); }

    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);

    if (esp_now_init() != ESP_OK) return;
    esp_now_register_send_cb((esp_now_send_cb_t)OnDataSent);

    memcpy(peerInfo.peer_addr, droneMacAddress, 6);
    peerInfo.channel = 1;  
    peerInfo.encrypt = false;
    esp_now_add_peer(&peerInfo);
    
    Serial.println("Ready. Mode 1.");
}

void loop() {
    // === S10 LOGIC (Power & Mode) ===
    if (digitalRead(PIN_POWER) == LOW) {
        if (!buttonActive) { 
            buttonActive = true; buttonTimer = millis(); longPressHandled = false;
        }
        if (millis() - buttonTimer > 2000 && !longPressHandled) {
            longPressHandled = true; 
            Serial.println("Shutting Down...");
            myData.flightMode = 99; // Kill Drone
            myData.armed = false;
            myData.throttle = 0;
            esp_now_send(droneMacAddress, (uint8_t *) &myData, sizeof(myData));
            delay(100); 

            for(int i=0; i<3; i++) { digitalWrite(PIN_LED, LOW); delay(100); digitalWrite(PIN_LED, HIGH); delay(100); }
            digitalWrite(PIN_LED, LOW); 
            while(digitalRead(PIN_POWER) == LOW) { delay(10); } 
            esp_deep_sleep_start();
        }
    } else { 
        if (buttonActive) {
            if (!longPressHandled && (millis() - buttonTimer < 2000)) {
                flightMode = (flightMode == 0) ? 1 : 0;
                if(flightMode == 1) { 
                    Serial.println("MODE 2: Yaw Active");
                    digitalWrite(PIN_BUZZER,HIGH); delay(50); digitalWrite(PIN_BUZZER,LOW); delay(50); digitalWrite(PIN_BUZZER,HIGH); delay(50); digitalWrite(PIN_BUZZER,LOW); 
                } else { 
                    Serial.println("MODE 1: Normal");
                    digitalWrite(PIN_BUZZER,HIGH); delay(100); digitalWrite(PIN_BUZZER,LOW); 
                }
            }
            buttonActive = false; 
        }
    }

    // === S1 (INCREASE STEP) ===
    bool s1 = digitalRead(PIN_S1_INC);
    if (s1 == LOW && lastS1 == HIGH) { 
        if(stepIndex < 6) {
            stepIndex++; 
            currentStepSize = stepValues[stepIndex]; 
            Serial.printf("Step Size: %d\n", currentStepSize);
            digitalWrite(PIN_BUZZER,HIGH); delay(50); digitalWrite(PIN_BUZZER,LOW); delay(50); digitalWrite(PIN_BUZZER,HIGH); delay(50); digitalWrite(PIN_BUZZER,LOW);
        } else {
            Serial.println("Step Size: MAX (100)");
            digitalWrite(PIN_BUZZER,HIGH); delay(200); digitalWrite(PIN_BUZZER,LOW);
        } 
    } 
    lastS1 = s1;

    // === S3 (SHORT=DECREASE STEP, LONG=RESET TRIMS) ===
    if (digitalRead(PIN_S3_FUNC) == LOW) {
        if (!activeS3) { activeS3 = true; timerS3 = millis(); longS3 = false; }
        // HELD FOR 2 SECONDS -> RESET TRIMS
        if (!longS3 && (millis() - timerS3 > 2000)) { 
            longS3 = true;
            trimRoll = 0; trimPitch = 0; trimYaw = 0; 
            Serial.println("!!! ALL TRIMS RESET !!!");
            digitalWrite(PIN_BUZZER, HIGH); delay(500); digitalWrite(PIN_BUZZER, LOW);
        }
    } else {
        if (activeS3) {
            // RELEASED QUICKLY -> DECREASE STEP
            if (!longS3) { 
                if(stepIndex > 0) {
                    stepIndex--; 
                    currentStepSize = stepValues[stepIndex]; 
                    Serial.printf("Step Size: %d\n", currentStepSize);
                    digitalWrite(PIN_BUZZER,HIGH); delay(50); digitalWrite(PIN_BUZZER,LOW);
                } else {
                    Serial.println("Step Size: MIN (1)");
                    digitalWrite(PIN_BUZZER,HIGH); delay(200); digitalWrite(PIN_BUZZER,LOW);
                } 
            }
            activeS3 = false;
        }
    }

    // === S5 (SHORT=ROLL RIGHT, LONG=YAW CCW/Anti-Clockwise) ===
    if (digitalRead(PIN_S5_RIGHT) == LOW) {
        if (!activeS5) { activeS5 = true; timerS5 = millis(); longS5 = false; }
        if (!longS5 && (millis() - timerS5 > 2000)) { 
            longS5 = true; trimYaw -= currentStepSize; 
            Serial.printf("YAW CCW (Anti-Clock) Bias: %d\n", trimYaw);
            digitalWrite(PIN_BUZZER, HIGH); delay(300); digitalWrite(PIN_BUZZER, LOW);
        }
    } else {
        if (activeS5) { if (!longS5) { trimRoll -= currentStepSize; Serial.printf("Roll Right: %d\n", trimRoll); digitalWrite(PIN_BUZZER, HIGH); delay(20); digitalWrite(PIN_BUZZER, LOW); } activeS5 = false; }
    }

    // === S6 (SHORT=ROLL LEFT, LONG=YAW CW/Clockwise) ===
    if (digitalRead(PIN_S6_LEFT) == LOW) {
        if (!activeS6) { activeS6 = true; timerS6 = millis(); longS6 = false; }
        if (!longS6 && (millis() - timerS6 > 2000)) { 
            longS6 = true; trimYaw += currentStepSize; 
            Serial.printf("YAW CW (Clockwise) Bias: %d\n", trimYaw);
            digitalWrite(PIN_BUZZER, HIGH); delay(300); digitalWrite(PIN_BUZZER, LOW);
        }
    } else {
        if (activeS6) { if (!longS6) { trimRoll += currentStepSize; Serial.printf("Roll Left: %d\n", trimRoll); digitalWrite(PIN_BUZZER, HIGH); delay(20); digitalWrite(PIN_BUZZER, LOW); } activeS6 = false; }
    }

    bool s7 = digitalRead(PIN_S7_BACK); 
    if (s7 == LOW && lastS7 == HIGH) { trimPitch += currentStepSize; Serial.printf("Pitch Forward: %d\n", trimPitch); digitalWrite(PIN_BUZZER,HIGH); delay(20); digitalWrite(PIN_BUZZER,LOW); } lastS7 = s7;

    bool s8 = digitalRead(PIN_S8_FRONT);
    if (s8 == LOW && lastS8 == HIGH) { trimPitch -= currentStepSize; Serial.printf("Pitch Backward: %d\n", trimPitch); digitalWrite(PIN_BUZZER,HIGH); delay(20); digitalWrite(PIN_BUZZER,LOW); } lastS8 = s8;

    // Flight Control Logic
    int rawThr = analogRead(PIN_THROTTLE);
    int rawYaw = analogRead(PIN_YAW);
    int rawPit = analogRead(PIN_PITCH);
    int rawRol = analogRead(PIN_ROLL);
    
    if (flightMode == 0) { 
        int t = map(rawThr, 0, 4095, 0, 1000); if(t<50) t=0; 
        myData.throttle = t; lockedThrottle = t; myData.yaw = 0;
    } else { 
        myData.throttle = lockedThrottle; myData.yaw = mapJoystick(rawYaw, yawCenter, 75, -75);
    }
    
    myData.pitch = mapJoystick(rawPit, pitchCenter, -75, 75) + trimPitch;
    myData.roll  = mapJoystick(rawRol, rollCenter, 75, -75) + trimRoll;
    myData.flightMode = flightMode; 
    myData.yawBias = trimYaw; 

    bool currentBtnState = digitalRead(PIN_ARM);
    if (lastArmBtnState == HIGH && currentBtnState == LOW) {
        isArmed = !isArmed; digitalWrite(PIN_BUZZER, HIGH); delay(50); digitalWrite(PIN_BUZZER, LOW);
    }
    lastArmBtnState = currentBtnState;
    myData.armed = isArmed; 

    if (isArmed) digitalWrite(PIN_LED, HIGH);
    else { int b = (flightMode == 0) ? 500 : 100; digitalWrite(PIN_LED, (millis() / b) % 2); }

    esp_now_send(droneMacAddress, (uint8_t *) &myData, sizeof(myData));
    delay(20); 
}
