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

// =============================================================================
// UART CONFIGURATION
// =============================================================================
// USB UART (Serial) - Built-in USB communication
// TX: GPIO1 (built-in USB UART TX)
// RX: GPIO3 (built-in USB UART RX)
// Baud rate: 115200 (for debugging output)

// =============================================================================
// MOTOR DRIVER 1 - PIN DEFINITIONS
// =============================================================================
// Motor Driver 1 (e.g., Front Left Actuator)
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
 * MOTOR DRIVER 1:
 * - A (GND) -> ESP32 GND
 * - B (VCC) -> ESP32 5V
 * - C (L_IS) -> ESP32 GPIO34
 * - D (R_IS) -> ESP32 GPIO35  
 * - E (L_EN) -> Shorted to F
 * - F (R_EN) -> Shorted to E
 * - G (LPWM) -> ESP32 GPIO4
 * - H (RPWM) -> ESP32 GPIO5
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
 * DEBUG CONNECTIONS:
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