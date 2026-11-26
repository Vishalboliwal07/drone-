#include <Arduino.h>
#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>
#include <freertos/FreeRTOS.h>
#include <cmath>
#include <cstring>

#include "common/command_packet.hpp"
#include "drone/board_config.hpp"
#include "FlightController.hpp"
#include "MotorOutput.hpp"
#include "PidController.hpp"
#include "SensorManager.hpp"

// ================= HARDWARE CONFIGURATION =================
#define PIN_POWER_LATCH drone::board::powerLatch
#define PIN_LED1 drone::board::led1
#define PIN_LED2 drone::board::led2

// ================= MANUAL MOTOR TRIMS =================
int TRIM_FL = 0;
int TRIM_FR = 0;
int TRIM_RL = 0;
int TRIM_RR = 0;

// ================= SETTINGS =================
#undef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 1000 
#define PWM_FREQ drone::board::pwmFrequencyHz
#define PWM_RES drone::board::pwmResolutionBits
#define MIN_PULSE drone::board::minPulseUs
#define MAX_PULSE drone::board::maxPulseUs

#define D_TERM_FILTER_FREQ  60.0f 
#define ROLL_PITCH_FF  0.8f 
#define YAW_FF         0.5f

// --- Altitude Settings ---
#define SEALEVEL_PRESSURE_HPA drone::board::seaLevelPressureHpa
#define ALT_MAX_LIMIT drone::board::maxAltitudeMeters

// ================= OBJECTS =================
SensorManager sensors;
SimpleKalmanFilter gyroXFilter(1, 1, 0.02);
SimpleKalmanFilter gyroYFilter(1, 1, 0.02);
SimpleKalmanFilter gyroZFilter(1, 0, 0.02);
SimpleKalmanFilter pitch_filter(0.1, 0.1, 0.01);
SimpleKalmanFilter roll_filter(0.1, 0.1, 0.01);

SimpleKalmanFilter pressureKalman(2, 2, 0.08);

typedef struct { float rollAngle, pitchAngle, yawAngle; } Orientation;
typedef struct State { volatile bool armed; volatile bool calibrating; volatile int throttle; volatile int pitch; volatile int roll; volatile int yaw; } State;

CommandPacket incomingData;
State state;
Orientation orientation = {0.0, 0.0, 0.0};
PidController rollPID, pitchPID, yawPID, altPID;
MotorOutput motors;

float accelBiasX=0, accelBiasY=0, accelBiasZ=0;
float gyroBiasX=0, gyroBiasY=0, gyroBiasZ=0;
float rollOffsetDeg=0.0f, pitchOffsetDeg=0.0f;
float dt=0.005;
unsigned long lastRecvTime = 0;
unsigned long ledTimer = 0;
bool ledState = false;

float groundReferenceAlt = 0;
float filteredAltitude = 0;
float targetAltitude = 0;
bool altitudeHoldActive = false;
int lastThrottleReading = 0;

// ================= CALLBACK =================
void OnDataRecv(const esp_now_recv_info_t * info, const uint8_t *incomingDataPtr, int len) {
    memcpy(&incomingData, incomingDataPtr, sizeof(incomingData));
    state.throttle = incomingData.throttle;
    state.yaw      = incomingData.yaw;
    state.pitch    = incomingData.pitch;
    state.roll     = incomingData.roll;

    if (incomingData.flightMode == 99) {
        digitalWrite(PIN_POWER_LATCH, LOW);
        while(1);
    }

    if (!incomingData.armed) state.armed = false;
    else state.armed = true;
    lastRecvTime = millis();
}

// ================= HELPERS =================
void setupESPNow() {
    WiFi.mode(WIFI_STA);
    esp_wifi_set_promiscuous(true);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_set_promiscuous(false);
    if (esp_now_init() != ESP_OK) return;
    esp_now_register_recv_cb(OnDataRecv);
}

float expoMovingAverage(float current, float previous, float alpha = 0.3464) { return alpha * previous + (1 - alpha) * current; }
void setupMotors() { motors.begin(); }

void flightTask(void *p) {
    vTaskDelay(pdMS_TO_TICKS(1000));
    state.calibrating = true;
    
    // --- IMU CALIBRATION ---
    float accSumX=0, accSumY=0, accSumZ=0;
    for (int i=0; i<300; i++) { 
        SensorReadings readings{};
        if (sensors.read(readings)) {
            gyroBiasX += readings.gyroX; gyroBiasY += readings.gyroY; gyroBiasZ += readings.gyroZ;
            accSumX += readings.accelX; accSumY += readings.accelY; accSumZ += readings.accelZ;
        }
        vTaskDelay(2);
    }
    gyroBiasX/=300.0; gyroBiasY/=300.0; gyroBiasZ/=300.0;
    accSumX/=300.0; accSumY/=300.0; accSumZ/=300.0;
    pitchOffsetDeg = atan2f(-accSumX, sqrtf(accSumY*accSumY + accSumZ*accSumZ)) * 57.2958f;
    rollOffsetDeg  = atanf(accSumY / accSumZ) * 57.2958f;   
    
    // --- BARO CALIBRATION ---
    float pressSum = 0;
    for(int i=0; i<100; i++) {
        sensors.altitudeMeters(SEALEVEL_PRESSURE_HPA);
        vTaskDelay(10);
    }
    for(int i=0; i<100; i++) {
        pressSum += sensors.altitudeMeters(SEALEVEL_PRESSURE_HPA);
        vTaskDelay(10);
    }
    groundReferenceAlt = pressSum / 100.0f;
    
    state.calibrating = false;

    rollPID.configure(1.3f, 0.5f, 0.04f, ROLL_PITCH_FF);
    pitchPID.configure(1.3f, 0.5f, 0.04f, ROLL_PITCH_FF);
    yawPID.configure(1.3f, 0.35f, 0.04f, YAW_FF);
    altPID.configure(1.2f, 0.20f, 0.25f, 0.0f);

    unsigned long time_now = micros();
    unsigned long time_prev = time_now;
    TickType_t xLastTick = xTaskGetTickCount();
    
    float imu_roll_dot_prev=0, imu_pitch_dot_prev=0, imu_yaw_dot_prev=0;
    float accelRoll_prev=0, accelPitch_prev=0;

    while (1) {
        time_now = micros();
        dt = (time_now - time_prev) / 1000000.0f;
        time_prev = time_now;
        if (dt > 0.1f) dt = 0.005f;
        
        if (millis() - lastRecvTime > 60000) { digitalWrite(PIN_POWER_LATCH, LOW); }
        if (millis() - lastRecvTime > 1000) state.armed = false;

        SensorReadings readings{};
        if (sensors.read(readings)) {
            // 1. Gyro Calculations
            float gx = (readings.gyroX - gyroBiasX) * 57.2958;
            float gy = (readings.gyroY - gyroBiasY) * 57.2958;
            float gz = (readings.gyroZ - gyroBiasZ) * 57.2958;
            float f_gx = expoMovingAverage(gx, imu_roll_dot_prev);  
            float f_gy = expoMovingAverage(gy, imu_pitch_dot_prev); 
            float f_gz = expoMovingAverage(gz, imu_yaw_dot_prev);   
            float k_gx = gyroXFilter.updateEstimate(f_gx);
            float k_gy = gyroYFilter.updateEstimate(f_gy);
            float k_gz = gyroZFilter.updateEstimate(f_gz);
            imu_roll_dot_prev=k_gx; imu_pitch_dot_prev=k_gy; imu_yaw_dot_prev=k_gz;

            // 2. Accel Calculations
            float calAccX = readings.accelX - accelBiasX;
            float calAccY = readings.accelY - accelBiasY;
            float calAccZ = readings.accelZ - accelBiasZ;
            float accRoll  = atan2f(calAccY, calAccZ) * 57.2958f - rollOffsetDeg;
            float accPitch = atanf(-calAccX / sqrtf(calAccY*calAccY + calAccZ*calAccZ)) * 57.2958f - pitchOffsetDeg;
            
            float k_accRoll = roll_filter.updateEstimate(expoMovingAverage(accRoll, accelRoll_prev));
            float k_accPitch = pitch_filter.updateEstimate(expoMovingAverage(accPitch, accelPitch_prev));
            accelRoll_prev=k_accRoll; accelPitch_prev=k_accPitch;

            // 3. Fusion
            float alpha = 0.98;
            orientation.rollAngle  = alpha * (orientation.rollAngle + k_gx * dt) + (1.0f - alpha) * k_accRoll;
            orientation.pitchAngle = alpha * (orientation.pitchAngle + k_gy * dt) + (1.0f - alpha) * k_accPitch;

            if (abs(orientation.rollAngle) > 75.0 || abs(orientation.pitchAngle) > 75.0) {
                state.armed = false;
            }

            // --- ALTITUDE LOGIC ---
            float absAlt = sensors.altitudeMeters(SEALEVEL_PRESSURE_HPA);
            filteredAltitude = pressureKalman.updateEstimate(absAlt - groundReferenceAlt);
            
            if (abs(state.throttle - lastThrottleReading) < 20) {
                if (!altitudeHoldActive && state.throttle > 200) { 
                    targetAltitude = filteredAltitude;
                    altitudeHoldActive = true;
                }
            } else {
                altitudeHoldActive = false;
            }
            lastThrottleReading = state.throttle;
            
            if (filteredAltitude >= ALT_MAX_LIMIT) {
                targetAltitude = ALT_MAX_LIMIT;
                altitudeHoldActive = true;
            }
            
            float altPID_Output = 0;
            if (state.armed && altitudeHoldActive) {
                altPID_Output = altPID.update(targetAltitude, filteredAltitude, dt, state.throttle);
                altPID_Output = constrain(altPID_Output, -200, 200); 
            } else {
                altPID.resetIntegral();
                altPID_Output = 0;
            }
            // ---------------------
            
            float targetRoll  = state.roll * 0.3f;
            float targetPitch = state.pitch * 0.3f;
            float targetRollRate  = 4.0f * (targetRoll - orientation.rollAngle);
            float targetPitchRate = 4.0f * (targetPitch - orientation.pitchAngle);
            float targetYawRate   = 3.0f * (state.yaw); 

            float rollOut = rollPID.update(targetRollRate, k_gx, dt, state.throttle);
            float pitchOut = pitchPID.update(targetPitchRate, k_gy, dt, state.throttle);
            float yawOut = yawPID.update(targetYawRate, k_gz, dt, state.throttle);
            
            // ==========================================
            // --- NEW: THROTTLE PID ADJUSTMENTS START ---
            // ==========================================

            // 1. LOW THROTTLE CUTOFF (Below 5% -> 50)
            if (state.throttle < 50) {
                rollOut = 0;
                pitchOut = 0;
                yawOut = 0;
                // Important: Reset I-terms so they don't accumulate on ground
                rollPID.resetIntegral();
                pitchPID.resetIntegral();
                yawPID.resetIntegral();
            }
            // 2. HIGH THROTTLE ATTENUATION (Above 80% -> 800)
            // Scales from 100% PID strength at 800 throttle to 80% PID strength at 1000 throttle
            else if (state.throttle > 800) {
                // Formula: 1.0 - (throttle overshoot * scaling factor)
                // Range 800-1000 is 200 units. We want to drop 0.2 (20%).
                // Factor = (state.throttle - 800) * 0.001
                float tpaFactor = 1.0f - ((float)(state.throttle - 800) * 0.001f);
                
                rollOut  *= tpaFactor;
                pitchOut *= tpaFactor;
                yawOut   *= tpaFactor;
            }

            // ========================================
            // --- NEW: THROTTLE PID ADJUSTMENTS END ---
            // ========================================

            int basePulse = map(state.throttle, 0, 1000, MIN_PULSE, MAX_PULSE);
            basePulse += (int)altPID_Output;
            
            motors.write(basePulse, rollOut, pitchOut, yawOut, state.armed,
                         state.calibrating, incomingData.flightMode, incomingData.yawBias,
                         TRIM_FL, TRIM_FR, TRIM_RL, TRIM_RR);
        }
        vTaskDelayUntil(&xLastTick, pdMS_TO_TICKS(5));
    }
}

void flightControllerSetup() {
    setCpuFrequencyMhz(240);
    Serial.begin(115200);
    pinMode(PIN_POWER_LATCH, OUTPUT); digitalWrite(PIN_POWER_LATCH, HIGH); 
    pinMode(PIN_LED1, OUTPUT); pinMode(PIN_LED2, OUTPUT);
    for(int i=0; i<3; i++) { digitalWrite(PIN_LED1, HIGH); digitalWrite(PIN_LED2, HIGH); delay(100); digitalWrite(PIN_LED1, LOW); digitalWrite(PIN_LED2, LOW); delay(100); }
    pinMode(21, INPUT_PULLUP); pinMode(22, OUTPUT);
    for(int i=0; i<8; i++) { digitalWrite(22, LOW); delayMicroseconds(5); digitalWrite(22, HIGH); delayMicroseconds(5); }
    pinMode(22, INPUT);
    Wire.begin(); Wire.setTimeOut(3000);
    setupMotors(); sensors.begin(); setupESPNow();
    Serial.println("System Ready.");
    xTaskCreatePinnedToCore(flightTask, "FlightTask", 20480, NULL, 5, NULL, 1);
}

void flightControllerLoop() {
    if (state.armed) { digitalWrite(PIN_LED1, HIGH); digitalWrite(PIN_LED2, HIGH); } 
    else if (state.calibrating) { if (millis() - ledTimer > 100) { ledTimer = millis(); ledState = !ledState; digitalWrite(PIN_LED1, ledState); digitalWrite(PIN_LED2, !ledState); } }
    else { if (millis() - ledTimer > 500) { ledTimer = millis(); ledState = !ledState; digitalWrite(PIN_LED1, ledState); digitalWrite(PIN_LED2, !ledState); } }
    delay(20); 
}
