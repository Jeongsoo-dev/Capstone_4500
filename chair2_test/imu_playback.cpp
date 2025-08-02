#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include <FS.h>
#include <SPIFFS.h>
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
// Neutral state lengths based on corrected mathematical model
const int neutralLength[3] = {735, 735, 670}; // mm - Motor 1(l1), Motor 2(l2), Motor 3(l3)
const float armRadius = 200.0;  // mm - radius from center to actuator mount

// -- Lookup Table Configuration
const char* LOOKUP_TABLE_FILE = "/lookup_table.txt";
const int MAX_LOOKUP_ENTRIES = 2000;  // Reduced to prevent memory issues

// -- Workspace Constraints (matching Python validation)
const float PITCH_MIN = -10.0;   // degrees
const float PITCH_MAX = 15.0;    // degrees
const float ROLL_MIN = -15.0;    // degrees
const float ROLL_MAX = 15.0;     // degrees

// -- Control Loop  
// VERIFIED: actuatorSpeed = 84.0 mm/s corresponds to 100% duty cycle (PWM 255)
// This means: PWM = (desired_speed_mm_per_sec / 84.0) * 255
const float actuatorSpeed = 84.0; // mm/s - max actuator speed at 100% duty cycle
const int updateInterval = 10;  // ms - control loop interval (100 Hz)

// -- Motion Cueing Factors
// Heave disabled - focusing only on pitch and roll
// const float heaveScaleFactor = 20.0; // mm of heave per G of acceleration (DISABLED)
// How much to tilt based on angular velocity (for fast motion cues)
const float pitchCueFactor = 0.1; // degrees of extra pitch per °/s
const float rollCueFactor = 0.1;  // degrees of extra roll per °/s

// -- Smoothing Parameters
const float IMU_SMOOTHING_FACTOR = 0.3; // Low-pass filter strength (0.1-0.3, lower = smoother)
const float TARGET_RATE_LIMIT = 2.0; // mm/s max rate of target change
const float DEADBAND_THRESHOLD = 0.5; // mm - ignore changes smaller than this
const float REDUCED_GAIN = 4.0; // Reduced from 8.0 for smoother response

// =============================================================================
// LOOKUP TABLE STRUCTURES
// =============================================================================
struct LookupEntry {
  float roll_deg;
  float pitch_deg;
  float l1;
  float l2;
  float l3;
  float height;
};

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
// -- WebSocket Server
WebSocketsServer webSocket = WebSocketsServer(81);

// -- Actuator Position Tracking
float currentLength[3] = {minLength, minLength, minLength}; // Current lengths [A, B, C]
float targetLength[3] = {neutralLength[0], neutralLength[1], neutralLength[2]}; // Target lengths [A, B, C]
const float actuatorAngles[3] = {0, 120, 240}; // Actuator positions in degrees

// -- Timing
unsigned long lastUpdate = 0;

// -- Lookup Table Data
LookupEntry* lookupTable = nullptr;
int lookupTableSize = 0;
float* uniquePitches = nullptr;
float* uniqueRolls = nullptr;
int numPitches = 0;
int numRolls = 0;
bool lookupTableLoaded = false;

// -- Smoothing Variables
float filteredPitch = 0.0;
float filteredRoll = 0.0;
// float filteredAccZ = 9.81; // Removed - heave disabled
float filteredAngVelX = 0.0;
float filteredAngVelY = 0.0;
float previousTargets[3] = {neutralLength[0], neutralLength[1], neutralLength[2]};
unsigned long lastIMUTime = 0;
bool smoothingInitialized = false;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length);
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg); // Removed heave parameter
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
// Motor control functions are now in pin_definitions.cpp/h

// Lookup table functions
bool loadLookupTable();
bool validateWorkspaceConstraints(float pitch_deg, float roll_deg);
bool getActuatorLengthsFromLookup(float pitch_deg, float roll_deg, float& l1, float& l2, float& l3); // Now clamps input values automatically
bool validateActuatorLengths(float l1, float l2, float l3);
float bilinearInterpolate(float x, float y, float x1, float y1, float x2, float y2, 
                         float f11, float f12, float f21, float f22);
void cleanupLookupTable();

// Smoothing functions
void applyIMUSmoothing(float& pitch, float& roll, float& ang_vel_x, float& ang_vel_y); // Removed acc_z - heave disabled
void applyTargetRateLimiting(float newTargets[3], float previousTargets[3], float maxRate_mm_per_s, float deltaTime_s);

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  setupDebugUART0();
  delay(1000);
  debugPrint("\n==== Stewart Platform Playback Firmware (main2.cpp) ====");

  // Initialize motor drivers and PWM from pin_definitions.cpp first
  initializePins();
  setupPWM();
  stopAllMotors();
  debugPrint("[✓] Motor drivers initialized");

  // Initialize SPIFFS for file system access after motor setup
  if (!SPIFFS.begin(true)) {
    debugPrint("[!] SPIFFS mount failed");
    // Continue without lookup table - will use fallback
  } else {
    debugPrint("[✓] SPIFFS mounted");
    
    // Load lookup table after basic initialization is complete
    if (loadLookupTable()) {
      debugPrint("[✓] Lookup table loaded successfully");
    } else {
      debugPrint("[!] Failed to load lookup table - using mathematical fallback");
    }
  }

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
  
  // STEP 2: Move actuators up to neutral positions
  debugPrint("[*] Moving to neutral positions...");
  
  // Set targets to neutral lengths and move there
  for (int i = 0; i < 3; i++) {
    targetLength[i] = neutralLength[i];
    currentLength[i] = minLength; // Set current position to minimum since we're at bottom
  }
  
  // Move actuators to neutral positions and wait until they actually reach it
  const float positionTolerance = 5.0; // mm - how close to neutral position is "close enough"
  const unsigned long maxNeutralTime = 8000; // Max 8 seconds to reach neutral (safety)
  unsigned long neutralStartTime = millis();
  bool allAtNeutral = false;
  
  while (!allAtNeutral && (millis() - neutralStartTime < maxNeutralTime)) {
    updateActuatorsSmoothly();
    
    // Check if all actuators are close enough to neutral positions
    bool actuator1AtNeutral = abs(currentLength[0] - neutralLength[0]) <= positionTolerance;
    bool actuator2AtNeutral = abs(currentLength[1] - neutralLength[1]) <= positionTolerance;
    bool actuator3AtNeutral = abs(currentLength[2] - neutralLength[2]) <= positionTolerance;
    
    if (actuator1AtNeutral && actuator2AtNeutral && actuator3AtNeutral) {
      allAtNeutral = true;
      debugPrintf("All actuators reached neutral positions - Lengths: %.1f, %.1f, %.1f mm", 
                  currentLength[0], currentLength[1], currentLength[2]);
    }
    
    delay(updateInterval);
  }
  
  if (!allAtNeutral) {
    debugPrint("[!] Warning: Not all actuators reached neutral position within timeout");
    debugPrintf("Current positions - Motor 1: %.1f mm, Motor 2: %.1f mm, Motor 3: %.1f mm", 
                currentLength[0], currentLength[1], currentLength[2]);
  } else {
    debugPrint("[✓] Stewart Platform at NEUTRAL STATE and ready for control");
  }
  
  // Hold at neutral position for 1 second to ensure stability
  debugPrint("[*] Stabilizing at neutral position...");
  unsigned long stabilizeStartTime = millis();
  while (millis() - stabilizeStartTime < 1000) { // 1 second
    updateActuatorsSmoothly(); // Continue fine-tuning position
    delay(updateInterval);
  }
  
  // Stop all motors
  stopAllMotors();
  
  debugPrint("[✓] Initialization complete - platform stable at neutral state");

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
      
      // Note: Acceleration data no longer used (heave disabled)
      // float acc_x = motion["acceleration"]["x"];
      // float acc_y = motion["acceleration"]["y"];
      // float acc_z = motion["acceleration"]["z"];
      
      float ang_vel_x = motion["angular_velocity"]["x"];
      float ang_vel_y = motion["angular_velocity"]["y"];

      // --- Apply IMU Smoothing ---
      applyIMUSmoothing(pitch, roll, ang_vel_x, ang_vel_y);

      // --- Calculate Motion Components ---
      
      // Heave (vertical motion) disabled - focusing only on pitch and roll
      
      // Motion Cueing from Angular Velocity
      // Add a small, temporary tilt based on how fast the IMU is rotating.
      // This creates a sensation of faster movement.
      float pitch_with_cue = pitch + (ang_vel_y * pitchCueFactor);
      float roll_with_cue = roll + (ang_vel_x * rollCueFactor);

      // --- Compute Target Actuator Lengths ---
      
      // Always try lookup table first with clamping for continuous movement
      float lookup_l1, lookup_l2, lookup_l3;
      bool lookupSuccess = false;
      
      if (lookupTableLoaded) {
        // Use lookup table with automatic clamping for continuous movement
        // Values will be clamped to valid workspace instead of rejected
        lookupSuccess = getActuatorLengthsFromLookup(pitch_with_cue, roll_with_cue, 
                                                    lookup_l1, lookup_l2, lookup_l3);
      }
      
      // Store current time for rate limiting
      unsigned long currentTime = millis();
      float deltaTime_s = (currentTime - lastIMUTime) / 1000.0;
      lastIMUTime = currentTime;
      
      // Calculate new target lengths
      float newTargets[3];
      
      if (lookupSuccess) {
        // Use lookup table results directly (no heave offset)
        newTargets[0] = lookup_l1;  // Motor 1 (l1)
        newTargets[1] = lookup_l2;  // Motor 2 (l2)
        newTargets[2] = lookup_l3;  // Motor 3 (l3)
        
        // Ensure within physical limits
        for (int i = 0; i < 3; i++) {
          newTargets[i] = constrain(newTargets[i], minLength, maxLength);
        }
      } else {
        // Only fallback to mathematical computation if lookup table failed to load
        // or had a serious error (not just out-of-range values)
        debugPrint("[!] Lookup table unavailable - using mathematical fallback");
        for (int i = 0; i < 3; i++) {
          newTargets[i] = computeActuatorLength(pitch_with_cue, roll_with_cue, actuatorAngles[i]);
        }
      }
      
      // Apply rate limiting for smoother movement
      if (deltaTime_s > 0 && deltaTime_s < 1.0) { // Valid time delta
        applyTargetRateLimiting(newTargets, previousTargets, TARGET_RATE_LIMIT, deltaTime_s);
      }
      
      // Update target lengths with smoothed values
      for (int i = 0; i < 3; i++) {
        targetLength[i] = newTargets[i];
        previousTargets[i] = newTargets[i];
      }
      
      // Optional: Log received data to serial for debugging
      // debugPrintf("P:%.1f R:%.1f | T:%.1f,%.1f,%.1f", pitch, roll, targetLength[0], targetLength[1], targetLength[2]);

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
// Computes the required length of an actuator based on platform tilt only.
// This is the mathematical fallback when lookup table is not available
// Heave disabled - focusing only on pitch and roll
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad = radians(rollDeg);
  float angleRad = radians(angleDeg);

  // Calculate vertical displacement (dz) due to platform tilt
  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  
  // Use neutral length as baseline instead of undefined homeLength
  // For this mathematical approximation, we'll use a weighted average of neutral lengths
  float baseLength = (neutralLength[0] + neutralLength[1] + neutralLength[2]) / 3.0;
  
  // Calculate final target length including base position and tilt (no heave)
  float target = baseLength + dz;

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
    
    // Proportional control with saturation (reduced gain for smoother response)
    desiredSpeed_mmPerSec = positionError * REDUCED_GAIN;
    
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

// =============================================================================
// LOOKUP TABLE IMPLEMENTATION
// =============================================================================

bool loadLookupTable() {
  debugPrint("[*] Loading lookup table from SPIFFS...");
  debugPrintf("[*] Free heap before loading: %d bytes", ESP.getFreeHeap());
  
  File file = SPIFFS.open(LOOKUP_TABLE_FILE, "r");
  if (!file) {
    debugPrint("[!] Failed to open lookup table file");
    return false;
  }
  
  // Allocate memory for lookup table
  lookupTable = (LookupEntry*)malloc(MAX_LOOKUP_ENTRIES * sizeof(LookupEntry));
  if (!lookupTable) {
    debugPrint("[!] Failed to allocate memory for lookup table");
    file.close();
    return false;
  }
  
  // Read and parse CSV file
  String line;
  int lineCount = 0;
  bool isHeader = true;
  
  while (file.available() && lookupTableSize < MAX_LOOKUP_ENTRIES) {
    line = file.readStringUntil('\n');
    line.trim();
    
    // Skip header line
    if (isHeader) {
      isHeader = false;
      continue;
    }
    
    // Skip empty lines
    if (line.length() == 0) continue;
    
    // Yield periodically to prevent watchdog timeout
    if (lineCount % 100 == 0) {
      yield();
    }
    
    // Parse CSV line: roll_deg,pitch_deg,l1,l2,l3,height
    int commaIndices[5];
    int commaCount = 0;
    
    // Find comma positions
    for (int i = 0; i < line.length() && commaCount < 5; i++) {
      if (line.charAt(i) == ',') {
        commaIndices[commaCount++] = i;
      }
    }
    
    if (commaCount != 5) {
      debugPrintf("[!] Invalid CSV line %d: wrong number of columns", lineCount);
      continue;
    }
    
    // Extract values
    try {
      lookupTable[lookupTableSize].roll_deg = line.substring(0, commaIndices[0]).toFloat();
      lookupTable[lookupTableSize].pitch_deg = line.substring(commaIndices[0] + 1, commaIndices[1]).toFloat();
      lookupTable[lookupTableSize].l1 = line.substring(commaIndices[1] + 1, commaIndices[2]).toFloat();
      lookupTable[lookupTableSize].l2 = line.substring(commaIndices[2] + 1, commaIndices[3]).toFloat();
      lookupTable[lookupTableSize].l3 = line.substring(commaIndices[3] + 1, commaIndices[4]).toFloat();
      lookupTable[lookupTableSize].height = line.substring(commaIndices[4] + 1).toFloat();
      
      lookupTableSize++;
    } catch (...) {
      debugPrintf("[!] Failed to parse line %d", lineCount);
    }
    
    lineCount++;
  }
  
  file.close();
  
  if (lookupTableSize == 0) {
    debugPrint("[!] No valid data found in lookup table");
    free(lookupTable);
    lookupTable = nullptr;
    return false;
  }
  
  debugPrintf("[✓] Loaded %d lookup table entries", lookupTableSize);
  
  // Extract unique pitch and roll values for interpolation
  // Use dynamic allocation to avoid stack overflow
  const int MAX_UNIQUE_VALUES = 200; // Reduced from 1000 to be more reasonable
  float* pitches = (float*)malloc(MAX_UNIQUE_VALUES * sizeof(float));
  float* rolls = (float*)malloc(MAX_UNIQUE_VALUES * sizeof(float));
  
  if (!pitches || !rolls) {
    debugPrint("[!] Failed to allocate memory for unique value extraction");
    if (pitches) free(pitches);
    if (rolls) free(rolls);
    free(lookupTable);
    lookupTable = nullptr;
    return false;
  }
  
  int pitchCount = 0, rollCount = 0;
  
  // Find unique pitches
  for (int i = 0; i < lookupTableSize; i++) {
    float pitch = lookupTable[i].pitch_deg;
    bool found = false;
    for (int j = 0; j < pitchCount; j++) {
      if (abs(pitches[j] - pitch) < 0.01) {
        found = true;
        break;
      }
    }
    if (!found && pitchCount < MAX_UNIQUE_VALUES) {
      pitches[pitchCount++] = pitch;
    }
  }
  
  // Find unique rolls
  for (int i = 0; i < lookupTableSize; i++) {
    float roll = lookupTable[i].roll_deg;
    bool found = false;
    for (int j = 0; j < rollCount; j++) {
      if (abs(rolls[j] - roll) < 0.01) {
        found = true;
        break;
      }
    }
    if (!found && rollCount < MAX_UNIQUE_VALUES) {
      rolls[rollCount++] = roll;
    }
  }
  
  // Sort arrays (simple bubble sort)
  for (int i = 0; i < pitchCount - 1; i++) {
    for (int j = 0; j < pitchCount - i - 1; j++) {
      if (pitches[j] > pitches[j + 1]) {
        float temp = pitches[j];
        pitches[j] = pitches[j + 1];
        pitches[j + 1] = temp;
      }
    }
  }
  
  for (int i = 0; i < rollCount - 1; i++) {
    for (int j = 0; j < rollCount - i - 1; j++) {
      if (rolls[j] > rolls[j + 1]) {
        float temp = rolls[j];
        rolls[j] = rolls[j + 1];
        rolls[j + 1] = temp;
      }
    }
  }
  
  // Allocate and copy unique values
  uniquePitches = (float*)malloc(pitchCount * sizeof(float));
  uniqueRolls = (float*)malloc(rollCount * sizeof(float));
  
  if (!uniquePitches || !uniqueRolls) {
    debugPrint("[!] Failed to allocate memory for unique values");
    // Clean up temporary arrays before returning
    free(pitches);
    free(rolls);
    if (uniquePitches) free(uniquePitches);
    if (uniqueRolls) free(uniqueRolls);
    free(lookupTable);
    lookupTable = nullptr;
    return false;
  }
  
  memcpy(uniquePitches, pitches, pitchCount * sizeof(float));
  memcpy(uniqueRolls, rolls, rollCount * sizeof(float));
  numPitches = pitchCount;
  numRolls = rollCount;
  
  // Clean up temporary arrays
  free(pitches);
  free(rolls);
  
  debugPrintf("[✓] Pitch range: [%.1f°, %.1f°] with %d points", 
              uniquePitches[0], uniquePitches[numPitches-1], numPitches);
  debugPrintf("[✓] Roll range: [%.1f°, %.1f°] with %d points", 
              uniqueRolls[0], uniqueRolls[numRolls-1], numRolls);
  
  debugPrintf("[*] Free heap after loading: %d bytes", ESP.getFreeHeap());
  
  lookupTableLoaded = true;
  return true;
}

bool validateWorkspaceConstraints(float pitch_deg, float roll_deg) {
  // Check basic ranges
  if (pitch_deg < PITCH_MIN || pitch_deg > PITCH_MAX) {
    return false;
  }
  if (roll_deg < ROLL_MIN || roll_deg > ROLL_MAX) {
    return false;
  }
  
  // Check constraint pattern: pitch >= abs(roll) - 10
  if (pitch_deg < abs(roll_deg) - 13) {
    return false;
  }
  
  return true;
}

bool validateActuatorLengths(float l1, float l2, float l3) {
  return (l1 >= minLength && l1 <= maxLength &&
          l2 >= minLength && l2 <= maxLength &&
          l3 >= minLength && l3 <= maxLength);
}

float bilinearInterpolate(float x, float y, float x1, float y1, float x2, float y2, 
                         float f11, float f12, float f21, float f22) {
  float denom = (x2 - x1) * (y2 - y1);
  if (abs(denom) < 0.0001) return f11; // Avoid division by zero
  
  float result = (f11 * (x2 - x) * (y2 - y) +
                  f21 * (x - x1) * (y2 - y) +
                  f12 * (x2 - x) * (y - y1) +
                  f22 * (x - x1) * (y - y1)) / denom;
  
  return result;
}

bool getActuatorLengthsFromLookup(float pitch_deg, float roll_deg, float& l1, float& l2, float& l3) {
  if (!lookupTableLoaded || !lookupTable) {
    return false;
  }
  
  // Clamp values to workspace constraints instead of rejecting them
  float clamped_pitch = pitch_deg;
  float clamped_roll = roll_deg;
  
  // Clamp to basic pitch/roll ranges
  clamped_pitch = constrain(clamped_pitch, PITCH_MIN, PITCH_MAX);
  clamped_roll = constrain(clamped_roll, ROLL_MIN, ROLL_MAX);
  
  // Apply constraint: pitch >= abs(roll) - 13
  // If pitch is too low for the given roll, increase pitch to minimum allowed
  float min_pitch_for_roll = abs(clamped_roll) - 13.0;
  if (clamped_pitch < min_pitch_for_roll) {
    clamped_pitch = min_pitch_for_roll;
    // Re-clamp pitch to stay within absolute limits
    clamped_pitch = constrain(clamped_pitch, PITCH_MIN, PITCH_MAX);
  }
  
  // Clamp to lookup table range
  clamped_pitch = constrain(clamped_pitch, uniquePitches[0], uniquePitches[numPitches-1]);
  clamped_roll = constrain(clamped_roll, uniqueRolls[0], uniqueRolls[numRolls-1]);
  
  // Log if significant clamping occurred (for debugging, but not too frequently)
  static unsigned long lastClampLog = 0;
  if ((abs(clamped_pitch - pitch_deg) > 0.5 || abs(clamped_roll - roll_deg) > 0.5) && 
      (millis() - lastClampLog > 1000)) { // Log max once per second
    debugPrintf("[*] Clamped (%.1f°, %.1f°) → (%.1f°, %.1f°)", 
                pitch_deg, roll_deg, clamped_pitch, clamped_roll);
    lastClampLog = millis();
  }
  
  // Use clamped values for lookup
  pitch_deg = clamped_pitch;
  roll_deg = clamped_roll;
  
  // Find bounding grid points
  int pitch_i1 = 0, pitch_i2 = 0;
  int roll_j1 = 0, roll_j2 = 0;
  
  // Find pitch indices
  for (int i = 0; i < numPitches - 1; i++) {
    if (pitch_deg >= uniquePitches[i] && pitch_deg <= uniquePitches[i + 1]) {
      pitch_i1 = i;
      pitch_i2 = i + 1;
      break;
    }
  }
  
  // Find roll indices
  for (int j = 0; j < numRolls - 1; j++) {
    if (roll_deg >= uniqueRolls[j] && roll_deg <= uniqueRolls[j + 1]) {
      roll_j1 = j;
      roll_j2 = j + 1;
      break;
    }
  }
  
  // Find the four corner entries in lookup table
  float f11_l1 = 0, f12_l1 = 0, f21_l1 = 0, f22_l1 = 0;
  float f11_l2 = 0, f12_l2 = 0, f21_l2 = 0, f22_l2 = 0;
  float f11_l3 = 0, f12_l3 = 0, f21_l3 = 0, f22_l3 = 0;
  
  bool found11 = false, found12 = false, found21 = false, found22 = false;
  
  for (int i = 0; i < lookupTableSize; i++) {
    float p = lookupTable[i].pitch_deg;
    float r = lookupTable[i].roll_deg;
    
    if (abs(p - uniquePitches[pitch_i1]) < 0.01 && abs(r - uniqueRolls[roll_j1]) < 0.01) {
      f11_l1 = lookupTable[i].l1; f11_l2 = lookupTable[i].l2; f11_l3 = lookupTable[i].l3;
      found11 = true;
    }
    if (abs(p - uniquePitches[pitch_i1]) < 0.01 && abs(r - uniqueRolls[roll_j2]) < 0.01) {
      f12_l1 = lookupTable[i].l1; f12_l2 = lookupTable[i].l2; f12_l3 = lookupTable[i].l3;
      found12 = true;
    }
    if (abs(p - uniquePitches[pitch_i2]) < 0.01 && abs(r - uniqueRolls[roll_j1]) < 0.01) {
      f21_l1 = lookupTable[i].l1; f21_l2 = lookupTable[i].l2; f21_l3 = lookupTable[i].l3;
      found21 = true;
    }
    if (abs(p - uniquePitches[pitch_i2]) < 0.01 && abs(r - uniqueRolls[roll_j2]) < 0.01) {
      f22_l1 = lookupTable[i].l1; f22_l2 = lookupTable[i].l2; f22_l3 = lookupTable[i].l3;
      found22 = true;
    }
  }
  
  if (!found11 || !found12 || !found21 || !found22) {
    debugPrint("[!] Could not find all corner points for interpolation");
    return false;
  }
  
  // Perform bilinear interpolation
  l1 = bilinearInterpolate(pitch_deg, roll_deg, 
                          uniquePitches[pitch_i1], uniqueRolls[roll_j1],
                          uniquePitches[pitch_i2], uniqueRolls[roll_j2],
                          f11_l1, f12_l1, f21_l1, f22_l1);
  
  l2 = bilinearInterpolate(pitch_deg, roll_deg, 
                          uniquePitches[pitch_i1], uniqueRolls[roll_j1],
                          uniquePitches[pitch_i2], uniqueRolls[roll_j2],
                          f11_l2, f12_l2, f21_l2, f22_l2);
  
  l3 = bilinearInterpolate(pitch_deg, roll_deg, 
                          uniquePitches[pitch_i1], uniqueRolls[roll_j1],
                          uniquePitches[pitch_i2], uniqueRolls[roll_j2],
                          f11_l3, f12_l3, f21_l3, f22_l3);
  
  return validateActuatorLengths(l1, l2, l3);
}

void cleanupLookupTable() {
  if (lookupTable) {
    free(lookupTable);
    lookupTable = nullptr;
  }
  if (uniquePitches) {
    free(uniquePitches);
    uniquePitches = nullptr;
  }
  if (uniqueRolls) {
    free(uniqueRolls);
    uniqueRolls = nullptr;
  }
  lookupTableSize = 0;
  numPitches = 0;
  numRolls = 0;
  lookupTableLoaded = false;
}

// =============================================================================
// SMOOTHING FUNCTIONS
// =============================================================================

void applyIMUSmoothing(float& pitch, float& roll, float& ang_vel_x, float& ang_vel_y) {
  // Initialize smoothing on first call
  if (!smoothingInitialized) {
    filteredPitch = pitch;
    filteredRoll = roll;
    // filteredAccZ removed - heave disabled
    filteredAngVelX = ang_vel_x;
    filteredAngVelY = ang_vel_y;
    smoothingInitialized = true;
    debugPrint("[*] IMU smoothing initialized (pitch/roll focus)");
    return;
  }
  
  // Apply low-pass filtering (exponential moving average)
  // New_value = alpha * raw_value + (1-alpha) * previous_filtered_value
  filteredPitch = IMU_SMOOTHING_FACTOR * pitch + (1.0 - IMU_SMOOTHING_FACTOR) * filteredPitch;
  filteredRoll = IMU_SMOOTHING_FACTOR * roll + (1.0 - IMU_SMOOTHING_FACTOR) * filteredRoll;
  // filteredAccZ removed - heave disabled
  filteredAngVelX = IMU_SMOOTHING_FACTOR * ang_vel_x + (1.0 - IMU_SMOOTHING_FACTOR) * filteredAngVelX;
  filteredAngVelY = IMU_SMOOTHING_FACTOR * ang_vel_y + (1.0 - IMU_SMOOTHING_FACTOR) * filteredAngVelY;
  
  // Update input values with filtered values
  pitch = filteredPitch;
  roll = filteredRoll;
  // acc_z removed - heave disabled
  ang_vel_x = filteredAngVelX;
  ang_vel_y = filteredAngVelY;
}

void applyTargetRateLimiting(float newTargets[3], float previousTargets[3], float maxRate_mm_per_s, float deltaTime_s) {
  // Limit how fast targets can change to prevent jerky movement
  float maxChange_mm = maxRate_mm_per_s * deltaTime_s;
  
  for (int i = 0; i < 3; i++) {
    float targetChange = newTargets[i] - previousTargets[i];
    
    // Apply deadband - ignore very small changes to reduce noise
    if (abs(targetChange) < DEADBAND_THRESHOLD) {
      newTargets[i] = previousTargets[i]; // No change
      continue;
    }
    
    // Rate limit the target change
    if (abs(targetChange) > maxChange_mm) {
      if (targetChange > 0) {
        newTargets[i] = previousTargets[i] + maxChange_mm;
      } else {
        newTargets[i] = previousTargets[i] - maxChange_mm;
      }
    }
  }
} 