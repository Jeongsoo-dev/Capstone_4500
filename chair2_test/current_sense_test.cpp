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

void setup() {
  // Initialize the Serial monitor for output
  Serial.begin(115200);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  }

  // Initialize all the GPIO pins and PWM settings
  initializePins();
  setupPWM();

  // Optionally, set up the debug UART if you want to use it
  // setupDebugUART2();

  Serial.println("==========================================");
  Serial.println("  Motor Current Sense Test Initialized");
  Serial.println("==========================================");
  Serial.println("Reading current sense values from all motors...");
  Serial.println("Motor 1: L_IS(34), R_IS(35) | Motor 2: L_IS(32), R_IS(33) | Motor 3: L_IS(36), R_IS(39)");
}

void loop() {
  unsigned long currentMillis = millis();

  // Check if it's time to print the sensor values again
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

    // Print the values to the Serial monitor in a formatted way
    Serial.printf("M1 L:%-4d R:%-4d | M2 L:%-4d R:%-4d | M3 L:%-4d R:%-4d\n",
                  motor1_L_IS_val, motor1_R_IS_val,
                  motor2_L_IS_val, motor2_R_IS_val,
                  motor3_L_IS_val, motor3_R_IS_val);
  }

  // You can add motor movement commands here to test current draw
  // under load. For example, to run motor 1 forward at half speed:
  // setMotorSpeed(1, 128);

  // To stop all motors:
  // stopAllMotors();
} 