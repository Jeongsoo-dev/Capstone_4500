#include <Arduino.h>

// Include pin definitions
#include "pin_definitions.cpp"

// =============================================================================
// STEWART PLATFORM CONFIGURATION
// =============================================================================
const int minLength = 550;      // mm - minimum actuator length
const int maxLength = 850;      // mm - maximum actuator length
const int homeLength = 700;     // mm - neutral/level position
const float actuatorSpeed = 84.0; // mm/s - actuator movement speed
const float armRadius = 200.0;  // mm - radius from center to actuator mount
const int updateInterval = 10;  // ms - control loop interval

// =============================================================================
// ACTUATOR POSITION TRACKING
// =============================================================================
float currentLength[3] = {minLength, minLength, minLength};  // Current lengths [A, B, C]
float targetLength[3] = {homeLength, homeLength, homeLength};   // Target lengths [A, B, C]
float actuatorAngles[3] = {0, 120, 240};   // Actuator positions in degrees
bool systemHomed = false;  // Track if system has completed homing

// =============================================================================
// IMU DATA PARSING
// =============================================================================
struct IMUData {
  float roll = 0.0;   // X-axis angle (degrees)
  float pitch = 0.0;  // Y-axis angle (degrees) 
  float yaw = 0.0;    // Z-axis angle (degrees)
  bool dataValid = false;
};

IMUData imuData;
unsigned long lastUpdate = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void initializeSystem();
void performHomingSequence();
bool parseIMUPacket();
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg);
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
void setMotorSpeed(int motor, int speed);
void stopAllMotors();

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==== Stewart Platform IMU Control Starting ====");
  
  initializeSystem();
  performHomingSequence();
  
  Serial.println("[✓] System initialized and homed");
  Serial.println("[INFO] Waiting for IMU data packets (0x55 0x61)...");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  // Only process IMU data after homing is complete
  if (systemHomed) {
    // Parse incoming IMU data
    if (parseIMUPacket()) {
      // Convert IMU angles to actuator lengths
      targetLength[0] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[0]);
      targetLength[1] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[1]);
      targetLength[2] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[2]);
      
      Serial.printf("[IMU] Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°\n", 
                    imuData.roll, imuData.pitch, imuData.yaw);
      Serial.printf("[TARGET] A: %.1f, B: %.1f, C: %.1f\n", 
                    targetLength[0], targetLength[1], targetLength[2]);
    }
    
    // Update actuators at regular intervals
    unsigned long now = millis();
    if (now - lastUpdate >= updateInterval) {
      updateActuatorsSmoothly();
      lastUpdate = now;
    }
  }
}

// =============================================================================
// SYSTEM INITIALIZATION
// =============================================================================
void initializeSystem() {
  // Initialize motor driver pins
  initializePins();
  setupPWM();
  
  // Stop all motors initially
  stopAllMotors();
  
  Serial.println("[✓] Motor drivers initialized");
}

// =============================================================================
// HOMING SEQUENCE
// =============================================================================
void performHomingSequence() {
  Serial.println("[INFO] Starting homing sequence...");
  Serial.printf("[INFO] Moving actuators from %d mm to %d mm (neutral position)\n", 
                minLength, homeLength);
  
  // Calculate expected homing time
  float homingDistance = homeLength - minLength;
  float homingTime = homingDistance / actuatorSpeed;
  
  Serial.printf("[INFO] Expected homing time: %.1f seconds\n", homingTime);
  
  // Move actuators to home position smoothly
  unsigned long homingStart = millis();
  
  while (!systemHomed) {
    unsigned long now = millis();
    if (now - lastUpdate >= updateInterval) {
      updateActuatorsSmoothly();
      
      // Check if all actuators reached home position
      bool allAtHome = true;
      for (int i = 0; i < 3; i++) {
        if (abs(currentLength[i] - homeLength) > 1.0) { // 1mm tolerance
          allAtHome = false;
          break;
        }
      }
      
      if (allAtHome) {
        systemHomed = true;
        stopAllMotors();
        Serial.println("[✓] Homing complete - platform level");
        Serial.printf("[INFO] Final positions: A=%.1f, B=%.1f, C=%.1f mm\n",
                      currentLength[0], currentLength[1], currentLength[2]);
      }
      
      // Safety timeout
      if ((now - homingStart) > (homingTime * 1000 * 2)) {
        Serial.println("[WARNING] Homing timeout - proceeding anyway");
        systemHomed = true;
        stopAllMotors();
        break;
      }
      
      lastUpdate = now;
    }
  }
}

// =============================================================================
// IMU DATA PARSING
// =============================================================================
bool parseIMUPacket() {
  static uint8_t buffer[32];
  static int bufferIndex = 0;
  
  // Read available bytes
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    // Look for packet header
    if (bufferIndex == 0 && byte == 0x55) {
      buffer[0] = byte;
      bufferIndex = 1;
    }
    // Check for flag byte (acceleration + angle data)
    else if (bufferIndex == 1 && byte == 0x61) {
      buffer[1] = byte;
      bufferIndex = 2;
    }
    // Collect data bytes
    else if (bufferIndex >= 2 && bufferIndex < 20) {
      buffer[bufferIndex] = byte;
      bufferIndex++;
      
      // Process complete packet (header + flag + 18 data bytes)
      if (bufferIndex == 20) {
        // Extract angle data (bytes 12-17)
        int16_t rollRaw = (buffer[15] << 8) | buffer[14];   // Roll high/low
        int16_t pitchRaw = (buffer[17] << 8) | buffer[16];  // Pitch high/low  
        int16_t yawRaw = (buffer[19] << 8) | buffer[18];    // Yaw high/low
        
        // Convert to degrees according to protocol
        imuData.roll = (float)rollRaw / 32768.0 * 180.0;
        imuData.pitch = (float)pitchRaw / 32768.0 * 180.0;
        imuData.yaw = (float)yawRaw / 32768.0 * 180.0;
        imuData.dataValid = true;
        
        bufferIndex = 0; // Reset for next packet
        return true;
      }
    }
    else {
      // Invalid sequence, reset
      bufferIndex = 0;
    }
  }
  
  return false;
}

// =============================================================================
// STEWART PLATFORM KINEMATICS
// =============================================================================
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad = radians(rollDeg);
  float angleRad = radians(angleDeg);
  
  // Compute vertical displacement for this actuator position
  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  
  // Calculate target length (home position + displacement)
  float target = homeLength + dz;
  
  // Constrain to actuator limits
  return constrain(target, minLength, maxLength);
}

// =============================================================================
// SMOOTH ACTUATOR CONTROL
// =============================================================================
void updateActuatorsSmoothly() {
  float maxStep = actuatorSpeed * updateInterval / 1000.0;
  
  for (int i = 0; i < 3; i++) {
    currentLength[i] = stepTowards(currentLength[i], targetLength[i], maxStep);
    
    // Calculate speed for this motor (-255 to +255)
    static float lastLength[3] = {minLength, minLength, minLength};
    float delta = currentLength[i] - lastLength[i];
    int speed = constrain(delta * 50, -255, 255); // Scale factor for responsiveness
    
    setMotorSpeed(i + 1, speed); // Motors are numbered 1, 2, 3
    lastLength[i] = currentLength[i];
  }
}

float stepTowards(float current, float target, float maxStep) {
  float delta = target - current;
  if (abs(delta) <= maxStep) return target;
  return current + (delta > 0 ? maxStep : -maxStep);
}

// =============================================================================
// MOTOR CONTROL FUNCTIONS
// =============================================================================
void setMotorSpeed(int motor, int speed) {
  // speed: -255 (full reverse) to +255 (full forward)
  int pwmValue = abs(speed);
  bool direction = (speed >= 0);
  
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
// PIN INITIALIZATION FUNCTIONS (from pin_definitions.cpp)
// =============================================================================
void initializePins() {
  // Set PWM pins as outputs (automatically handled by ledcAttachPin)
  // Set current sense pins as inputs
  pinMode(MOTOR1_L_IS, INPUT);
  pinMode(MOTOR1_R_IS, INPUT);
  pinMode(MOTOR2_L_IS, INPUT);
  pinMode(MOTOR2_R_IS, INPUT);
  pinMode(MOTOR3_L_IS, INPUT);
  pinMode(MOTOR3_R_IS, INPUT);
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