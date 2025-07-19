#include <Arduino.h>

// ===== Pins =====
#define PWM_PIN 12
#define DIR_PIN 13
#define PWM_CHANNEL 0

// ===== Actuator Motion Settings =====
const float actuatorSpeed = 84.0; // mm/s
const float strokeLength = 300.0; // mm
const int pwmDuty = 200;          // out of 255

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n[BOOT] ESP32 Actuator Test Starting");

  pinMode(DIR_PIN, OUTPUT);
  ledcSetup(PWM_CHANNEL, 20000, 8); 
  ledcAttachPin(PWM_PIN, PWM_CHANNEL);

  Serial.println("[✓] PWM and DIR initialized");
}

void moveActuator(float distanceMM, bool direction) {
  float durationSeconds = distanceMM / actuatorSpeed;
  digitalWrite(DIR_PIN, direction ? HIGH : LOW);
  ledcWrite(PWM_CHANNEL, pwmDuty);

  Serial.printf("Moving %.0f mm %s for %.2f seconds\n", distanceMM, direction ? "↑" : "↓", durationSeconds);
  delay((int)(durationSeconds * 1000));

  ledcWrite(PWM_CHANNEL, 0);
  Serial.println("✓ Movement complete");
}

void loop() {
  // Example Cycle: Move Up → Down → Stop
  moveActuator(100, true);   // Move up 100 mm
  delay(1000);
  moveActuator(100, false);  // Move down 100 mm
  delay(1000);
}
