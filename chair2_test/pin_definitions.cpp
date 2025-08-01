/*
 * Pin Definitions for Stewart Platform Chair Control
 * ESP32 DOIT DevKit v1 controlling 3x BTS7960 Motor Drivers
 * 
 * Hardware Configuration:
 * - ESP32 receives data via Bluetooth 5.0
 * - 3x BTS7960 motor drivers for linear actuators
 * - E and F pins on drivers are shorted (always enabled)
 * - UART0 (Serial) used for debugging output
 */

#include <Arduino.h>
#include <cstdarg>  // For va_list, va_start, va_end
#include "pin_definitions.h"

// =============================================================================
// UART CONFIGURATION
// =============================================================================
// USB UART (Serial) - Built-in USB communication
// TX: GPIO1 (built-in USB UART TX)
// RX: GPIO3 (built-in USB UART RX)
// Baud rate: 115200 (for Bluetooth status and main debugging)

// UART2 (Serial2) - Additional debugging interface
// TX: GPIO17 (UART2 TX)
// RX: GPIO16 (UART2 RX)
// Baud rate: 115200 (for detailed IMU/motor debugging)

// =============================================================================
// MOTOR DRIVER 1 - PIN DEFINITIONS
// =============================================================================
// Motor Driver 1 (e.g., Front Left Actuator)
// CHANGED: Moved from GPIO16/17 to GPIO4/5 to free up UART2 pins
const int MOTOR1_LPWM = 4;     // GPIO4 - Left/Reverse PWM
const int MOTOR1_RPWM = 5;     // GPIO5 - Right/Forward PWM
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
// DEBUG UART2 PINS (Now Available)
// =============================================================================
const int UART2_TX_PIN = 17;   // GPIO17 - UART2 TX (freed from motor control)
const int UART2_RX_PIN = 16;   // GPIO16 - UART2 RX (freed from motor control)

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
// DEBUG CONFIGURATION
// =============================================================================
const int DEBUG_BAUD_RATE = 115200;  // UART0 baud rate for debugging
const bool ENABLE_DEBUG_UART0 = true; // Enable/disable UART0 debugging

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void initializePins();
void setupPWM();
void setupDebugUART0();
void setMotorSpeed(int motor, int speed); // speed: -255 to +255
int readCurrentSense(int motor, bool isReverse);
void stopAllMotors();
void debugPrint(String message); // Helper function for UART0 debugging
void debugPrintf(const char* format, ...); // Printf-style debugging

// =============================================================================
// WIRING SUMMARY
// =============================================================================
/*
 * ESP32 to BTS7960 Motor Driver Connections:
 * 
 * MOTOR DRIVER 1: **UPDATED PINS**
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO34
 * - D (R_IS) -> ESP32 GPIO35  
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO4  **CHANGED FROM GPIO16**
 * - H (RPWM) -> ESP32 GPIO5  **CHANGED FROM GPIO17**
 * 
 * MOTOR DRIVER 2: (Unchanged)
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO32
 * - D (R_IS) -> ESP32 GPIO33
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO18
 * - H (RPWM) -> ESP32 GPIO19
 * 
 * MOTOR DRIVER 3: (Unchanged)
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
 * UART CONNECTIONS:
 * - USB UART: Built-in via USB cable (GPIO1 TX, GPIO3 RX) - Debug output
 */ 

// =============================================================================
// UART0 DEBUG FUNCTIONS IMPLEMENTATION
// =============================================================================
void setupDebugUART0() {
  if (ENABLE_DEBUG_UART0) {
    Serial.begin(DEBUG_BAUD_RATE);
    delay(100); // Allow Serial to initialize
    Serial.println("\n==== UART0 Debug Interface Started ====");
    Serial.printf("Debug UART0 initialized at %d baud\n", DEBUG_BAUD_RATE);
    Serial.println("Available for IMU and motor debugging");
    Serial.println("==========================================");
  }
}

void debugPrint(String message) {
  if (ENABLE_DEBUG_UART0) {
    Serial.println("[DEBUG] " + message);
  }
}

void debugPrintf(const char* format, ...) {
  if (ENABLE_DEBUG_UART0) {
    char buffer[256];
    va_list args;
    va_start(args, format);
    vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);
    Serial.print("[DEBUG] ");
    Serial.println(buffer);
  }
} 

// =============================================================================
// MOTOR CONTROL FUNCTIONS IMPLEMENTATION
// =============================================================================
void setMotorSpeed(int motor, int speed) {
  // speed: -255 (full reverse) to +255 (full forward)
  int pwmValue = abs(speed);
  bool direction = (speed >= 0); // true for forward (RPWM), false for reverse (LPWM)
  
  switch (motor) {
    case 1: // Motor 1
      ledcWrite(MOTOR1_LPWM_CHANNEL, direction ? 0 : pwmValue);
      ledcWrite(MOTOR1_RPWM_CHANNEL, direction ? pwmValue : 0);
      break;
      
    case 2: // Motor 2  
      ledcWrite(MOTOR2_LPWM_CHANNEL, direction ? 0 : pwmValue);
      ledcWrite(MOTOR2_RPWM_CHANNEL, direction ? pwmValue : 0);
      break;
      
    case 3: // Motor 3
      ledcWrite(MOTOR3_LPWM_CHANNEL, direction ? 0 : pwmValue);
      ledcWrite(MOTOR3_RPWM_CHANNEL, direction ? pwmValue : 0);
      break;
  }
}

void stopAllMotors() {
  for (int i = 1; i <= 3; i++) {
    setMotorSpeed(i, 0);
  }
}

// =============================================================================
// PIN INITIALIZATION FUNCTIONS IMPLEMENTATION
// =============================================================================
void initializePins() {
  // Set current sense pins as inputs
  pinMode(MOTOR1_L_IS, INPUT);
  pinMode(MOTOR1_R_IS, INPUT);
  pinMode(MOTOR2_L_IS, INPUT);
  pinMode(MOTOR2_R_IS, INPUT);
  pinMode(MOTOR3_L_IS, INPUT);
  pinMode(MOTOR3_R_IS, INPUT);
  
  // Analog read resolution
  analogReadResolution(ADC_RESOLUTION);
}

void setupPWM() {
  // Setup PWM channels for all motors
  ledcSetup(MOTOR1_LPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR1_RPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR2_LPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR2_RPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR3_LPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  ledcSetup(MOTOR3_RPWM_CHANNEL, PWM_FREQUENCY, PWM_RESOLUTION);
  
  // Attach pins to PWM channels
  ledcAttachPin(MOTOR1_LPWM, MOTOR1_LPWM_CHANNEL);
  ledcAttachPin(MOTOR1_RPWM, MOTOR1_RPWM_CHANNEL);
  ledcAttachPin(MOTOR2_LPWM, MOTOR2_LPWM_CHANNEL);
  ledcAttachPin(MOTOR2_RPWM, MOTOR2_RPWM_CHANNEL);
  ledcAttachPin(MOTOR3_LPWM, MOTOR3_LPWM_CHANNEL);
  ledcAttachPin(MOTOR3_RPWM, MOTOR3_RPWM_CHANNEL);
} 

// =============================================================================
// CURRENT SENSING FUNCTIONS IMPLEMENTATION
// =============================================================================
int readCurrentSense(int motor, bool isReverse) {
  int pin = -1;
  
  // Select the appropriate current sense pin based on motor and direction
  switch (motor) {
    case 1:
      pin = isReverse ? MOTOR1_L_IS : MOTOR1_R_IS;
      break;
    case 2:
      pin = isReverse ? MOTOR2_L_IS : MOTOR2_R_IS;
      break;
    case 3:
      pin = isReverse ? MOTOR3_L_IS : MOTOR3_R_IS;
      break;
    default:
      return 0; // Invalid motor number
  }
  
  // Read the analog value from the current sense pin
  return analogRead(pin);
} 