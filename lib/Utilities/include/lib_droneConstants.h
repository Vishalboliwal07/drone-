#pragma once


#define PIN_POWER_LATCH 4
#define PIN_LED1 18
#define PIN_LED2 19

#define MOTOR_FL 12
#define MOTOR_RL 13
#define MOTOR_RR 14
#define MOTOR_FR 15

#undef CONFIG_FREERTOS_HZ
#define CONFIG_FREERTOS_HZ 1000
#define PWM_FREQ 416
#define PWM_RES 10
#define MIN_PULSE 1000 
#define MAX_PULSE 2000

unsigned long inline lastRecvTime = 0;

static float inline testControlScaling = 2.5;
static float inline testControlPID = 5.0;
static const int inline kpTesting = 100;



struct State
{
  volatile bool armed;
  volatile bool calibrating;
  volatile int throttle;
  volatile int pitch;
  volatile int roll;
  volatile int yaw;
};

typedef struct struct_message
{
  int throttle;
  int yaw;
  int pitch;
  int roll;
  bool armed;
} struct_message;

struct_message inline incomingData;
State inline controllerCommand;
State inline droneState;