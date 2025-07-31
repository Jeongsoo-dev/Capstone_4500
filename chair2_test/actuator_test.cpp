#include <Arduino.h>
#include "pin_definitions.h"

// ===== Pins =====
#define PWM_PIN 12
#define DIR_PIN 13
#define PWM_CHANNEL 0

// ===== Actuator Motion Settings =====
const float actuatorSpeed = 84.0; // mm/s
const float strokeLength = 300.0; // mm
const int pwmDuty = 200;          // out of 255

void setup() {
  setupDebugUART0();
  delay(1000);
  debugPrint("\n[BOOT] ESP32 Actuator Test Starting");

  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, 20000, 8); 
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  debugPrint("[✓] PWM and DIR initialized");
  
  // Move to neutral state during initialization
  // For single actuator test, move to average neutral length
  const float neutralLength = (670 + 735 + 735) / 3.0; // Average of l3, l1, l2
  debugPrintf("[*] Moving to neutral state (%.0f mm from minimum)", neutralLength - 550);
  
  // Move from minimum (550mm) to neutral position
  moveActuator(neutralLength - 550, true); // Move up to neutral
  debugPrint("[✓] Actuator at neutral state");
}

void moveActuator(float distanceMM, bool direction) {
  float durationSeconds = distanceMM / actuatorSpeed;
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL, pwmDuty);

  debugPrintf("Moving %.0f mm %s for %.2f seconds", distanceMM, direction ? "↑" : "↓", durationSeconds);
  delay((int)(durationSeconds * 1000));

  ledcWrite(PWM_CHANNEL, 0);
  debugPrint("✓ Movement complete");
}

void loop() {
  // Example Cycle: Move Up → Down → Stop
  moveActuator(100, true);   // Move up 100 mm
  delay(1000);
  moveActuator(100, false);  // Move down 100 mm
  delay(1000);
}
