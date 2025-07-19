#ifndef PIN_DEFINITIONS_H
#define PIN_DEFINITIONS_H

#include <Arduino.h>

// =============================================================================
// MOTOR DRIVER PIN DEFINITIONS
// =============================================================================
// Motor Driver 1
extern const int MOTOR1_LPWM;
extern const int MOTOR1_RPWM;
extern const int MOTOR1_L_IS;
extern const int MOTOR1_R_IS;

// Motor Driver 2
extern const int MOTOR2_LPWM;
extern const int MOTOR2_RPWM;
extern const int MOTOR2_L_IS;
extern const int MOTOR2_R_IS;

// Motor Driver 3
extern const int MOTOR3_LPWM;
extern const int MOTOR3_RPWM;
extern const int MOTOR3_L_IS;
extern const int MOTOR3_R_IS;

// =============================================================================
// DEBUG UART2 PINS
// =============================================================================
extern const int UART2_TX_PIN;
extern const int UART2_RX_PIN;

// =============================================================================
// PWM CONFIGURATION
// =============================================================================
extern const int PWM_FREQUENCY;
extern const int PWM_RESOLUTION;

// PWM Channels
extern const int MOTOR1_LPWM_CHANNEL;
extern const int MOTOR1_RPWM_CHANNEL;
extern const int MOTOR2_LPWM_CHANNEL;
extern const int MOTOR2_RPWM_CHANNEL;
extern const int MOTOR3_LPWM_CHANNEL;
extern const int MOTOR3_RPWM_CHANNEL;

// =============================================================================
// CURRENT SENSING CONFIGURATION
// =============================================================================
extern const int ADC_RESOLUTION;
extern const float ADC_VOLTAGE_REF;
extern const float CURRENT_SENSE_RATIO;

// =============================================================================
// DEBUG CONFIGURATION
// =============================================================================
extern const int DEBUG_BAUD_RATE;
extern const bool ENABLE_DEBUG_UART2;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void initializePins();
void setupPWM();
void setupDebugUART2();
void setMotorSpeed(int motor, int speed);
int readCurrentSense(int motor, bool isReverse);
void stopAllMotors();
void debugPrint(String message);
void debugPrintf(const char* format, ...);

#endif // PIN_DEFINITIONS_H 