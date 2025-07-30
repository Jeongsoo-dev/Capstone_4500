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
const float actuatorSpeed = 84.0; // mm/s - max actuator speed (which is the nature of the actuator, can't be faster)
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

  // Move to lowest position and hold for 4 seconds
  debugPrint("[*] Moving to lowest position...");
  
  // Set targets to minimum length (lowest position)
  for (int i = 0; i < 3; i++) {
    targetLength[i] = minLength;
  }
  
  // Move actuators to lowest position for 4 seconds
  unsigned long startTime = millis();
  while (millis() - startTime < 4000) { // 4 seconds = 4000ms
    updateActuatorsSmoothly();
    delay(updateInterval); // Use same update interval as main loop
  }
  
  debugPrint("[*] Moving to home position...");
  
  // Set targets to home length and move there
  for (int i = 0; i < 3; i++) {
    targetLength[i] = homeLength;
  }
  
  // Move actuators to home position for 3 seconds
  startTime = millis();
  while (millis() - startTime < 3000) { // 3 seconds = 3000ms
    updateActuatorsSmoothly();
    delay(updateInterval);
  }
  
  // Stop all motors
  stopAllMotors();
  
  debugPrint("[✓] Initialization complete - platform at home position");

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
  // Use a more aggressive step size for faster movement
  // Since actuatorSpeed is just the physical max speed, we use a fraction of it for control
  float maxStep = (actuatorSpeed * 0.5) * (updateInterval / 1000.0); // Use 50% of max speed
  
  for (int i = 0; i < 3; i++) {
    // Calculate the next incremental position for the actuator
    float previousLength = currentLength[i];
    currentLength[i] = stepTowards(currentLength[i], targetLength[i], maxStep);
    
    // Determine the required motor speed and direction based on the change in length
    float delta = currentLength[i] - previousLength;
    
    // Scale the change in length to a PWM value with much higher scaling factor
    // Higher scaling factor = more aggressive/faster movement
    int speed = constrain(delta * 500, -255, 255); 
    
    // Motors are numbered 1, 2, 3 in setMotorSpeed
    setMotorSpeed(i + 1, speed); 
  }
}

// Helper function to move a value towards a target by a maximum step.
float stepTowards(float current, float target, float maxStep) {
  if (abs(target - current) <= maxStep) {
    return target;
  }
  return current + (target > current ? maxStep : -maxStep);
} 