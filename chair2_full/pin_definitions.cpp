/*
 * Pin Definitions for Stewart Platform Chair Control
 * ESP32 DOIT DevKit v1 controlling 3x BTS7960 Motor Drivers
 * 
 * Hardware Configuration:
 * - ESP32 receives data via USB UART (Serial)
 * - 3x BTS7960 motor drivers for linear actuators
 * - E and F pins on drivers are shorted (always enabled)
 */

#include <Arduino.h>

// =============================================================================
// USB UART CONFIGURATION
// =============================================================================
// Using built-in Serial for USB UART communication
// TX: GPIO1 (built-in USB UART TX)
// RX: GPIO3 (built-in USB UART RX)
// Baud rate: 115200 (as defined in platformio.ini)

// =============================================================================
// MOTOR DRIVER 1 - PIN DEFINITIONS
// =============================================================================
// Motor Driver 1 (e.g., Front Left Actuator)
const int MOTOR1_LPWM = 16;    // GPIO16 - Left/Reverse PWM
const int MOTOR1_RPWM = 17;    // GPIO17 - Right/Forward PWM
const int MOTOR1_L_IS = 34;    // GPIO34 - Left/Reverse Current Sense (ADC1_CH6)
const int MOTOR1_R_IS = 35;    // GPIO35 - Right/Forward Current Sense (ADC1_CH7)

// =============================================================================
// MOTOR DRIVER 2 - PIN DEFINITIONS  
// =============================================================================
// Motor Driver 2 (e.g., Front Right Actuator)
const int MOTOR2_LPWM = 18;    // GPIO18 - Left/Reverse PWM
const int MOTOR2_RPWM = 19;    // GPIO19 - Right/Forward PWM
const int MOTOR2_L_IS = 32;    // GPIO32 - Left/Reverse Current Sense (ADC1_CH4)
const int MOTOR2_R_IS = 33;    // GPIO33 - Right/Forward Current Sense (ADC1_CH5)

// =============================================================================
// MOTOR DRIVER 3 - PIN DEFINITIONS
// =============================================================================
// Motor Driver 3 (e.g., Rear Actuator)
const int MOTOR3_LPWM = 25;    // GPIO25 - Left/Reverse PWM
const int MOTOR3_RPWM = 26;    // GPIO26 - Right/Forward PWM
const int MOTOR3_L_IS = 36;    // GPIO36 - Left/Reverse Current Sense (ADC1_CH0)
const int MOTOR3_R_IS = 39;    // GPIO39 - Right/Forward Current Sense (ADC1_CH3)

// =============================================================================
// SHARED POWER CONNECTIONS
// =============================================================================
// VCC (5V): Connect to ESP32 5V pin or external 5V supply
// GND: Connect to ESP32 GND
// Motor Power (B+/B-): Connect to external 12V-24V supply

// =============================================================================
// PWM CONFIGURATION
// =============================================================================
const int PWM_FREQUENCY = 1000;  // 1kHz PWM frequency
const int PWM_RESOLUTION = 8;    // 8-bit resolution (0-255)

// PWM Channels (ESP32 has 16 PWM channels)
const int MOTOR1_LPWM_CHANNEL = 0;
const int MOTOR1_RPWM_CHANNEL = 1;
const int MOTOR2_LPWM_CHANNEL = 2;
const int MOTOR2_RPWM_CHANNEL = 3;
const int MOTOR3_LPWM_CHANNEL = 4;
const int MOTOR3_RPWM_CHANNEL = 5;

// =============================================================================
// CURRENT SENSING CONFIGURATION
// =============================================================================
const int ADC_RESOLUTION = 12;   // 12-bit ADC resolution (0-4095)
const float ADC_VOLTAGE_REF = 3.3; // ESP32 ADC reference voltage
const float CURRENT_SENSE_RATIO = 1.0; // Adjust based on BTS7960 datasheet

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void initializePins();
void setupPWM();
void setMotorSpeed(int motor, int speed); // speed: -255 to +255
int readCurrentSense(int motor, bool isReverse);
void stopAllMotors();

// =============================================================================
// WIRING SUMMARY
// =============================================================================
/*
 * ESP32 to BTS7960 Motor Driver Connections:
 * 
 * MOTOR DRIVER 1:
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO34
 * - D (R_IS) -> ESP32 GPIO35  
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO16
 * - H (RPWM) -> ESP32 GPIO17
 * 
 * MOTOR DRIVER 2:
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO32
 * - D (R_IS) -> ESP32 GPIO33
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO18
 * - H (RPWM) -> ESP32 GPIO19
 * 
 * MOTOR DRIVER 3:
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO36
 * - D (R_IS) -> ESP32 GPIO39
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO25
 * - H (RPWM) -> ESP32 GPIO26
 * 
 * MOTOR POWER CONNECTIONS:
 * - 1 (B-) -> Motor Power Supply Negative (12V-24V)
 * - 2 (B+) -> Motor Power Supply Positive (12V-24V)
 * - 3 (M+) -> Linear Actuator Positive
 * - 4 (M-) -> Linear Actuator Negative
 * 
 * USB UART:
 * - Connect ESP32 to PC via USB cable
 * - Serial communication on GPIO1 (TX) and GPIO3 (RX)
 * - Baud rate: 115200
 */ 