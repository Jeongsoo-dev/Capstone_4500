/*
 * current_sense_test.cpp
 *
 * This program is designed to test and monitor the current sensing feedback
 * from three BTS7960 motor drivers connected to an ESP32. It initializes
 * the necessary pins and periodically reads the analog values from the L_IS
 * and R_IS pins of each motor driver.
 *
 * The primary purpose of this test is to establish a baseline for the
 * current draw of the actuators under normal and stalled conditions. By
 * observing the output values, you can determine an appropriate threshold
 to detect when an actuator is blocked or under excessive load.
 *
 * This helps in implementing safety features like automatic shutdown to
 * prevent damage to the motors or the power supply.
 *
 * Hardware:
 * - ESP32 DOIT DevKit v1
 * - 3x BTS7960 Motor Drivers
 * - Up to 3 linear actuators
 *
 * The program will print the raw ADC values from the current sense pins
 * to the default Serial port for monitoring.
 */

#include "pin_definitions.h"
#include <Arduino.h>

// Define the interval for printing the current sense values (in milliseconds)
const unsigned long PRINT_INTERVAL = 500;
unsigned long previousMillis = 0;

// === State Machine for Motor Movement ===
enum MotorState {
  MOVING_UP,
  MOVING_DOWN
};
MotorState currentState = MOVING_UP;
unsigned long stateStartTime = 0;

const int MOTOR_SPEED = 128; // Speed for motors (0-255)
const unsigned long DURATION_UP = 10000; // 10 seconds
const unsigned long DURATION_DOWN = 20000; // 20 seconds
// =======================================

void setup() {
  // Initialize the debug serial port
  setupDebugUART2();
  
  // Add immediate test output to verify UART2 is working
  delay(1000); // Give UART2 time to initialize
  debugPrint("=== UART2 TEST MESSAGE ===");
  debugPrint("If you can see this, UART2 is working!");

  // Initialize all the GPIO pins and PWM settings
  initializePins();
  debugPrint("Pins initialized");
  
  setupPWM();
  debugPrint("PWM setup complete");

  debugPrint("==========================================");
  debugPrint("  Motor Current Sense Test Initialized");
  debugPrint("==========================================");
  debugPrint("Reading current sense values from all motors...");
  debugPrintf("Motor 1: L_IS(%d), R_IS(%d) | Motor 2: L_IS(%d), R_IS(%d) | Motor 3: L_IS(%d), R_IS(%d)",
              MOTOR1_L_IS, MOTOR1_R_IS, MOTOR2_L_IS, MOTOR2_R_IS, MOTOR3_L_IS, MOTOR3_R_IS);
              
  debugPrint("Starting sequence: 10 seconds UP, then 20 seconds DOWN.");
  
  // Start the initial state (moving up)
  stateStartTime = millis();
  setMotorSpeed(1, MOTOR_SPEED);
  setMotorSpeed(2, MOTOR_SPEED);
  setMotorSpeed(3, MOTOR_SPEED);
  debugPrint("Motors started - moving UP");
}

void loop() {
  unsigned long currentMillis = millis();
  
  // Add periodic status output to verify loop is running
  static unsigned long lastStatusTime = 0;
  if (currentMillis - lastStatusTime >= 5000) { // Every 5 seconds
    debugPrintf("Loop running - State: %s, Time in state: %lu ms", 
                currentState == MOVING_UP ? "UP" : "DOWN", 
                currentMillis - stateStartTime);
    lastStatusTime = currentMillis;
  }

  // --- State Machine Logic ---
  if (currentState == MOVING_UP) {
    if (currentMillis - stateStartTime >= DURATION_UP) {
      debugPrint("10s UP complete. Moving DOWN for 20s.");
      currentState = MOVING_DOWN;
      stateStartTime = currentMillis; // Reset timer for new state
      // Set motors to move down
      setMotorSpeed(1, -MOTOR_SPEED);
      setMotorSpeed(2, -MOTOR_SPEED);
      setMotorSpeed(3, -MOTOR_SPEED);
    }
  } else if (currentState == MOVING_DOWN) {
    if (currentMillis - stateStartTime >= DURATION_DOWN) {
      debugPrint("20s DOWN complete. Moving UP for 10s.");
      currentState = MOVING_UP;
      stateStartTime = currentMillis; // Reset timer for new state
      // Set motors to move up
      setMotorSpeed(1, MOTOR_SPEED);
      setMotorSpeed(2, MOTOR_SPEED);
      setMotorSpeed(3, MOTOR_SPEED);
    }
  }

  // --- Current Sensing Logic (runs independently) ---
  if (currentMillis - previousMillis >= PRINT_INTERVAL) {
    previousMillis = currentMillis;

    // Read the current sense values for each motor
    // The second parameter 'true' for isReverse for L_IS and 'false' for R_IS
    int motor1_L_IS_val = readCurrentSense(1, true);
    int motor1_R_IS_val = readCurrentSense(1, false);

    int motor2_L_IS_val = readCurrentSense(2, true);
    int motor2_R_IS_val = readCurrentSense(2, false);

    int motor3_L_IS_val = readCurrentSense(3, true);
    int motor3_R_IS_val = readCurrentSense(3, false);

    // Print the values to the debug UART
    debugPrintf("M1 L:%-4d R:%-4d | M2 L:%-4d R:%-4d | M3 L:%-4d R:%-4d",
                  motor1_L_IS_val, motor1_R_IS_val,
                  motor2_L_IS_val, motor2_R_IS_val,
                  motor3_L_IS_val, motor3_R_IS_val);
  }
} 