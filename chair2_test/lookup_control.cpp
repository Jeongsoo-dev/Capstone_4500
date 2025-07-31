#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include "pin_definitions.h" // Use the same pin definitions as main2.cpp

// =============================================================================
// CONFIGURATION
// =============================================================================
// -- Wi-Fi Credentials
const char* ssid = "StewartPlatform_Lookup";
const char* password = "esp32vrchair";

// -- Stewart Platform Physical Dimensions
const int minLength = 550;      // mm - minimum actuator length
const int maxLength = 850;      // mm - maximum actuator length
const int homeLength = 700;     // mm - neutral/level position

// -- Control Loop  
// VERIFIED: actuatorSpeed = 84.0 mm/s corresponds to 100% duty cycle (PWM 255)
// This means: PWM = (desired_speed_mm_per_sec / 84.0) * 255
const float actuatorSpeed = 84.0; // mm/s - max actuator speed at 100% duty cycle
const int updateInterval = 10;  // ms - control loop interval (100 Hz)

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
// -- WebSocket Server
WebSocketsServer webSocket = WebSocketsServer(81);

// -- Actuator Position Tracking
float currentLength[3] = {minLength, minLength, minLength}; // Current lengths [A, B, C]
float targetLength[3] = {homeLength, homeLength, homeLength}; // Target lengths [A, B, C]

// -- Timing
unsigned long lastUpdate = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length);
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
// Motor control functions are in pin_definitions.cpp/h

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  setupDebugUART0();
  delay(1000);
  debugPrint("\n==== Stewart Platform Lookup Control Firmware (actuator_control.cpp) ====");

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
      webSocket.sendTXT(client, "{\"status\":\"connected\",\"message\":\"Stewart Platform Lookup Control Ready\"}");
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

      // Check if this is a direct actuator length command
      if (doc.containsKey("command") && doc["command"] == "set_actuator_lengths") {
        
        // Extract actuator lengths from JSON
        JsonObject lengths = doc["actuator_lengths"];
        if (!lengths.containsKey("l1") || !lengths.containsKey("l2") || !lengths.containsKey("l3")) {
          debugPrint("[WS] Error: Missing actuator length values");
          webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Missing actuator lengths\"}");
          return;
        }
        
        float l1 = lengths["l1"];
        float l2 = lengths["l2"];
        float l3 = lengths["l3"];
        
        // Validate actuator lengths are within safe operating range
        if (l1 < minLength || l1 > maxLength ||
            l2 < minLength || l2 > maxLength ||
            l3 < minLength || l3 > maxLength) {
          debugPrintf("[WS] Error: Actuator lengths out of range [%d-%d mm]: %.1f, %.1f, %.1f", 
                      minLength, maxLength, l1, l2, l3);
          webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Actuator lengths out of safe range\"}");
          return;
        }
        
        // Set new target lengths
        targetLength[0] = l1;
        targetLength[1] = l2;
        targetLength[2] = l3;
        
        debugPrintf("[WS] New targets set: L1=%.1f, L2=%.1f, L3=%.1f mm", l1, l2, l3);
        
        // Send acknowledgment
        webSocket.sendTXT(client, "{\"status\":\"success\",\"message\":\"Target lengths updated\"}");
        
      } else {
        // Handle legacy motion data format for backward compatibility
        JsonObject motion = doc["motion"];
        if (motion.containsKey("orientation")) {
          debugPrint("[WS] Warning: Received legacy motion data format - not supported in lookup control mode");
          webSocket.sendTXT(client, "{\"status\":\"warning\",\"message\":\"Use direct actuator length commands\"}");
        } else {
          debugPrint("[WS] Error: Unknown command format");
          webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Unknown command format\"}");
        }
      }
      
      break;
    }
    default:
      // Handle other WebSocket events if necessary
      break;
  }
}

// =============================================================================
// SMOOTH ACTUATOR CONTROL
// =============================================================================
// Moves actuators towards their target lengths incrementally.
// This is identical to main2.cpp for consistent behavior
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