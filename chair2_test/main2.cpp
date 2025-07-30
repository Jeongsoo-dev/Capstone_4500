#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include "pin_definitions.h" // Use the new pin definitions

// =============================================================================
// CONFIGURATION
// =============================================================================
// -- Wi-Fi Credentials
const char* ssid = "StewartPlatform_Playback";
const char* password = "esp32vrchair";

// -- Stewart Platform Physical Dimensions
const int minLength = 550;      // mm - minimum actuator length
const int maxLength = 850;      // mm - maximum actuator length
const int homeLength = 700;     // mm - neutral/level position
const float armRadius = 200.0;  // mm - radius from center to actuator mount

// -- Control Loop  
// VERIFIED: actuatorSpeed = 84.0 mm/s corresponds to 100% duty cycle (PWM 255)
// This means: PWM = (desired_speed_mm_per_sec / 84.0) * 255
const float actuatorSpeed = 84.0; // mm/s - max actuator speed at 100% duty cycle
const int updateInterval = 10;  // ms - control loop interval (100 Hz)

// -- Motion Cueing Factors
// How much vertical heave to apply based on Z acceleration
const float heaveScaleFactor = 20.0; // mm of heave per G of acceleration
// How much to tilt based on angular velocity (for fast motion cues)
const float pitchCueFactor = 0.1; // degrees of extra pitch per °/s
const float rollCueFactor = 0.1;  // degrees of extra roll per °/s

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
// -- WebSocket Server
WebSocketsServer webSocket = WebSocketsServer(81);

// -- Actuator Position Tracking
float currentLength[3] = {minLength, minLength, minLength}; // Current lengths [A, B, C]
float targetLength[3] = {homeLength, homeLength, homeLength}; // Target lengths [A, B, C]
const float actuatorAngles[3] = {0, 120, 240}; // Actuator positions in degrees

// -- Timing
unsigned long lastUpdate = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length);
float computeActuatorLength(float pitchDeg, float rollDeg, float heave, float angleDeg);
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
// Motor control functions are now in pin_definitions.cpp/h

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  setupDebugUART0();
  delay(1000);
  debugPrint("\n==== Stewart Platform Playback Firmware (main2.cpp) ====");

  // Initialize motor drivers and PWM from pin_definitions.cpp
  initializePins();
  setupPWM();
  stopAllMotors();
  debugPrint("[✓] Motor drivers initialized");

  // Start Wi-Fi Access Point
  if (WiFi.softAP(ssid, password)) {
    debugPrint("[✓] WiFi AP started");
    debugPrint("    SSID: " + String(ssid));
    debugPrint("    IP: " + WiFi.softAPIP().toString());
  } else {
    debugPrint("[!] WiFi AP failed to start");
  }

  // STEP 1: Move actuators down to shortest position (time-based)
  debugPrint("[*] Moving actuators down to shortest position (4 seconds at full speed)...");
  
  // Move all actuators down at full speed for reliable initialization
  // Use 100% of max speed: 84 mm/s = PWM 255
  const int moveDownSpeed = -255; // Negative for downward movement (100% of max speed)
  
  // Move down for 4 seconds at full speed
  unsigned long moveStartTime = millis();
  while (millis() - moveStartTime < 4000) { // 4 seconds = 4000ms
    for (int i = 1; i <= 3; i++) {
      setMotorSpeed(i, moveDownSpeed); // -255 PWM = 100% speed downward
    }
    delay(updateInterval);
  }
  
  // Stop all motors
  stopAllMotors();
  debugPrint("[✓] All actuators at shortest position");
  
  // STEP 2: Move actuators up to home position
  debugPrint("[*] Moving to home position...");
  
  // Set targets to home length and move there
  for (int i = 0; i < 3; i++) {
    targetLength[i] = homeLength;
    currentLength[i] = minLength; // Set current position to minimum since we're at bottom
  }
  
  // Move actuators to home position and wait until they actually reach it
  const float positionTolerance = 5.0; // mm - how close to home position is "close enough"
  const unsigned long maxHomeTime = 8000; // Max 8 seconds to reach home (safety)
  unsigned long homeStartTime = millis();
  bool allAtHome = false;
  
  while (!allAtHome && (millis() - homeStartTime < maxHomeTime)) {
    updateActuatorsSmoothly();
    
    // Check if all actuators are close enough to home position
    bool actuator1AtHome = abs(currentLength[0] - homeLength) <= positionTolerance;
    bool actuator2AtHome = abs(currentLength[1] - homeLength) <= positionTolerance;
    bool actuator3AtHome = abs(currentLength[2] - homeLength) <= positionTolerance;
    
    if (actuator1AtHome && actuator2AtHome && actuator3AtHome) {
      allAtHome = true;
      debugPrintf("All actuators reached home position - Lengths: %.1f, %.1f, %.1f mm", 
                  currentLength[0], currentLength[1], currentLength[2]);
    }
    
    delay(updateInterval);
  }
  
  if (!allAtHome) {
    debugPrint("[!] Warning: Not all actuators reached home position within time limit");
    debugPrintf("Current positions: %.1f, %.1f, %.1f mm (target: %.1f mm)", 
                currentLength[0], currentLength[1], currentLength[2], homeLength);
  }
  
  // Hold at home position for 1 second to ensure stability
  debugPrint("[*] Stabilizing at home position...");
  unsigned long stabilizeStartTime = millis();
  while (millis() - stabilizeStartTime < 1000) { // 1 second
    updateActuatorsSmoothly(); // Continue fine-tuning position
    delay(updateInterval);
  }
  
  // Stop all motors
  stopAllMotors();
  
  debugPrint("[✓] Initialization complete - platform stable at home position");

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  debugPrint("[✓] WebSocket server running on port 81");
  debugPrint("System ready. Waiting for Python client...");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastUpdate >= updateInterval) {
    updateActuatorsSmoothly();
    lastUpdate = now;
  }
}

// =============================================================================
// WEBSOCKET EVENT HANDLER
// =============================================================================
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      debugPrintf("[WS] Client #%u disconnected", client);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(client);
      debugPrintf("[WS] Client #%u connected from %d.%d.%d.%d", client, ip[0], ip[1], ip[2], ip[3]);
      webSocket.sendTXT(client, "{\"status\":\"connected\",\"message\":\"Stewart Platform Playback Ready\"}");
      break;
    }
    
    case WStype_TEXT: {
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, payload);

      if (error) {
        debugPrintf("[WS] JSON parse error: %s", error.c_str());
        webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
        return;
      }

      // --- Extract motion data from JSON ---
      JsonObject motion = doc["motion"];
      float pitch = motion["orientation"]["pitch"];
      float roll = motion["orientation"]["roll"];
      
      float acc_x = motion["acceleration"]["x"];
      float acc_y = motion["acceleration"]["y"];
      float acc_z = motion["acceleration"]["z"];
      
      float ang_vel_x = motion["angular_velocity"]["x"];
      float ang_vel_y = motion["angular_velocity"]["y"];

      // --- Calculate Motion Components ---
      
      // 1. Heave (Vertical Motion) from Z-axis acceleration
      // Subtract gravity (9.81 m/s^2) to get net vertical acceleration
      float vertical_accel_ms2 = acc_z - 9.81; 
      float heave_offset = vertical_accel_ms2 * heaveScaleFactor;

      // 2. Motion Cueing from Angular Velocity
      // Add a small, temporary tilt based on how fast the IMU is rotating.
      // This creates a sensation of faster movement.
      float pitch_with_cue = pitch + (ang_vel_y * pitchCueFactor);
      float roll_with_cue = roll + (ang_vel_x * rollCueFactor);

      // --- Compute Target Actuator Lengths ---
      for (int i = 0; i < 3; i++) {
        targetLength[i] = computeActuatorLength(pitch_with_cue, roll_with_cue, heave_offset, actuatorAngles[i]);
      }
      
      // Optional: Log received data to serial for debugging
      // debugPrintf("P:%.1f R:%.1f | H:%.1f | T:%.1f,%.1f,%.1f", pitch, roll, heave_offset, targetLength[0], targetLength[1], targetLength[2]);

      // Acknowledge receipt (optional, can be disabled for performance)
      // webSocket.sendTXT(client, "{\"status\":\"ok\"}");
      break;
    }
    default:
      // Handle other WebSocket events if necessary
      break;
  }
}

// =============================================================================
// STEWART PLATFORM KINEMATICS
// =============================================================================
// Computes the required length of an actuator based on platform tilt and heave.
float computeActuatorLength(float pitchDeg, float rollDeg, float heave, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad = radians(rollDeg);
  float angleRad = radians(angleDeg);

  // Calculate vertical displacement (dz) due to platform tilt
  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  
  // Calculate final target length including home position, tilt, and heave
  float target = homeLength + dz + heave;

  // Ensure the target length is within the physical limits of the actuator
  return constrain(target, minLength, maxLength);
}

// =============================================================================
// SMOOTH ACTUATOR CONTROL
// =============================================================================
// Moves actuators towards their target lengths incrementally.
void updateActuatorsSmoothly() {
  for (int i = 0; i < 3; i++) {
    // Calculate required speed based on position error and maximum actuator speed
    float positionError = targetLength[i] - currentLength[i];
    
    // Calculate desired speed in mm/s based on position error
    // Use proportional control: larger error = faster speed, up to actuatorSpeed limit
    float maxAllowedSpeed = actuatorSpeed; // 84 mm/s at 100% duty cycle (PWM 255)
    float desiredSpeed_mmPerSec;
    
    // Proportional control with saturation
    const float proportionalGain = 8.0; // Adjust this for responsiveness vs stability
    desiredSpeed_mmPerSec = positionError * proportionalGain;
    
    // Limit to maximum actuator speed
    desiredSpeed_mmPerSec = constrain(desiredSpeed_mmPerSec, -maxAllowedSpeed, maxAllowedSpeed);
    
    // Convert desired speed (mm/s) to PWM value (0-255)
    // Since actuatorSpeed (84 mm/s) corresponds to PWM 255 (100% duty cycle)
    int pwmValue = (int)((abs(desiredSpeed_mmPerSec) / actuatorSpeed) * 255.0);
    pwmValue = constrain(pwmValue, 0, 255);
    
    // Apply direction (positive = extend, negative = retract)
    int motorSpeed = (desiredSpeed_mmPerSec >= 0) ? pwmValue : -pwmValue;
    
    // Update current position based on actual movement that will occur
    // Calculate actual speed that will be achieved with this PWM
    float actualSpeed_mmPerSec = (motorSpeed / 255.0) * actuatorSpeed;
    float actualMovement_mm = actualSpeed_mmPerSec * (updateInterval / 1000.0);
    currentLength[i] += actualMovement_mm;
    
    // Prevent overshoot - if we're very close, just set to target
    if (abs(targetLength[i] - currentLength[i]) < 0.5) { // Within 0.5mm
      currentLength[i] = targetLength[i];
      motorSpeed = 0; // Stop motor when at target
    }
    
    // Motors are numbered 1, 2, 3 in setMotorSpeed
    setMotorSpeed(i + 1, motorSpeed); 
  }
}

// Helper function to move a value towards a target by a maximum step.
float stepTowards(float current, float target, float maxStep) {
  if (abs(target - current) <= maxStep) {
    return target;
  }
  return current + (target > current ? maxStep : -maxStep);
} 