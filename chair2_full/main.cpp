#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>

// Include pin definitions
#include "pin_definitions.h"

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
// COMMUNICATION MODE CONFIGURATION
// =============================================================================
// Set to true for Bluetooth, false for UART/Serial communication
const bool USE_BLUETOOTH_MODE = true;

// BLE Configuration - REPLACE WITH YOUR IMU'S ACTUAL UUIDs
// NOTE: These are placeholder UUIDs - you MUST replace them with your device's actual UUIDs
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9b34fb");    // Update for your device
static BLEUUID CHAR_UUID("0000ffe9-0000-1000-8000-00805f9b34fb");       // Update for your device

// BLE Variables
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
bool bleConnected = false;
bool bleConnecting = false;

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
  
  // Optional time data (when available)
  struct {
    uint8_t year = 0;
    uint8_t month = 0; 
    uint8_t day = 0;
    uint8_t hour = 0;
    uint8_t minute = 0;
    uint8_t second = 0;
    uint16_t millisecond = 0;
    bool timeValid = false;
  } timestamp;
};

IMUData imuData;
unsigned long lastUpdate = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
void initializeSystem();
void performHomingSequence();
bool initializeBLE();
void scanForIMU();
bool connectToIMU();
bool parseIMUPacket(uint8_t* data, size_t length);
bool parseIMUPacketUART();  // For UART mode
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg);
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
void setMotorSpeed(int motor, int speed);
void stopAllMotors();
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void processIMUData();  // Common IMU processing

// =============================================================================
// BLE CALLBACK CLASSES
// =============================================================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    debugPrint("BLE connection established with IMU device");
    bleConnected = true;
    bleConnecting = false;
    
    // Set up the service and characteristic
    connectToIMU();
  }

  void onDisconnect(BLEClient* pclient) override {
    debugPrint("BLE connection lost - will attempt to reconnect");
    bleConnected = false;
    bleConnecting = false;
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    debugPrintf("Found device: %s", advertisedDevice.toString().c_str());
    
    // Check if this device has our service UUID
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      debugPrint("Found IMU device! Attempting to connect...");
      BLEDevice::getScan()->stop();
      
      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new MyClientCallback());
      
      if (pClient->connect(&advertisedDevice)) {
        debugPrint("BLE connection established");
      } else {
        debugPrint("BLE connection failed");
        bleConnecting = false;
      }
    }
  }
};

// =============================================================================
// SETUP FUNCTION
// =============================================================================
void setup() {
  // Initialize debug interface first
  setupDebugUART2();
  delay(1000);
  
  if (USE_BLUETOOTH_MODE) {
    debugPrint("==== Stewart Platform IMU Control (Bluetooth 5.0) Starting ====");
    
    initializeSystem();
    
    // Initialize BLE
    debugPrint("Initializing Bluetooth Low Energy...");
    if (initializeBLE()) {
      debugPrint("BLE initialized successfully");
      scanForIMU();
    } else {
      debugPrint("Failed to initialize BLE");
      return;
    }
  } else {
    debugPrint("==== Stewart Platform IMU Control (UART) Starting ====");
    
    // Initialize Serial for UART communication with IMU
    Serial.begin(115200);
    debugPrint("UART initialized for IMU communication at 115200 baud");
    
    initializeSystem();
  }
  
  performHomingSequence();
  
  debugPrint("System initialized and homed");
  if (USE_BLUETOOTH_MODE) {
    debugPrint("Waiting for IMU data packets via BLE...");
  } else {
    debugPrint("Waiting for IMU data packets via UART (0x55 0x61)...");
  }
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  if (USE_BLUETOOTH_MODE) {
    // Handle BLE connection state
    if (!bleConnected && !bleConnecting && systemHomed) {
      debugPrint("Attempting to reconnect to IMU...");
      scanForIMU();
      delay(2000); // Wait before retry
    }
  } else {
    // UART mode - check for incoming data
    if (systemHomed && parseIMUPacketUART()) {
      processIMUData();
    }
  }
  
  // Only process control updates after homing is complete
  if (systemHomed) {
    // Update actuators at regular intervals
    unsigned long now = millis();
    if (now - lastUpdate >= updateInterval) {
      updateActuatorsSmoothly();
      lastUpdate = now;
    }
  }
  
  delay(10);
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
  
  debugPrint("Motor drivers initialized");
}

// =============================================================================
// BLE INITIALIZATION
// =============================================================================
bool initializeBLE() {
  BLEDevice::init("Stewart_Platform_Control");
  return true;
}

void scanForIMU() {
  if (bleConnecting) return;
  
  bleConnecting = true;
  debugPrint("Scanning for IMU device...");
  debugPrintf("Looking for service UUID: %s", SERVICE_UUID.toString().c_str());
  
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(10, false); // Scan for 10 seconds
}

bool connectToIMU() {
  if (!bleConnected || pClient == nullptr) return false;
  
  debugPrint("Getting IMU service...");
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  if (pRemoteService == nullptr) {
    debugPrint("Failed to find IMU service");
    return false;
  }
  
  debugPrint("Getting IMU characteristic...");
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHAR_UUID);
  if (pRemoteCharacteristic == nullptr) {
    debugPrint("Failed to find IMU characteristic");
    return false;
  }
  
  // Register for notifications
  if (pRemoteCharacteristic->canNotify()) {
    pRemoteCharacteristic->registerForNotify(onNotify);
    debugPrint("BLE notifications registered successfully");
    return true;
  }
  
  debugPrint("Characteristic does not support notifications");
  return false;
}

// BLE notification callback
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("BLE notification received: %d bytes", length);
  
  if (parseIMUPacket(pData, length)) {
    processIMUData();
  } else {
    debugPrint("Failed to parse IMU packet");
    // Print raw data for debugging
    debugPrint("Raw packet data:");
    for(int i = 0; i < length && i < 32; i++) {
      Serial2.printf("0x%02X ", pData[i]);
      if((i + 1) % 8 == 0) Serial2.println();
    }
    Serial2.println();
  }
}

// =============================================================================
// HOMING SEQUENCE
// =============================================================================
void performHomingSequence() {
  debugPrint("Starting homing sequence...");
  debugPrintf("Moving actuators from %d mm to %d mm (neutral position)", 
                minLength, homeLength);
  
  // Calculate expected homing time
  float homingDistance = homeLength - minLength;
  float homingTime = homingDistance / actuatorSpeed;
  
  debugPrintf("Expected homing time: %.1f seconds", homingTime);
  
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
        debugPrint("Homing complete - platform level");
        debugPrintf("Final positions: A=%.1f, B=%.1f, C=%.1f mm",
                      currentLength[0], currentLength[1], currentLength[2]);
      }
      
      // Safety timeout
      if ((now - homingStart) > (homingTime * 1000 * 2)) {
        debugPrint("Homing timeout - proceeding anyway");
        systemHomed = true;
        stopAllMotors();
        break;
      }
      
      lastUpdate = now;
    }
  }
}

// =============================================================================
// IMU DATA PARSING - BLE MODE
// =============================================================================
bool parseIMUPacket(uint8_t* data, size_t length) {
  // Check for minimum packet size (header + flag + 18 data bytes)
  if (length < 20) return false;
  
  // Look for packet header (0x55 0x61)
  if (data[0] == 0x55 && data[1] == 0x61) {
    // Extract angle data (bytes 14-19)
    int16_t rollRaw = (data[15] << 8) | data[14];   // Roll high/low
    int16_t pitchRaw = (data[17] << 8) | data[16];  // Pitch high/low  
    int16_t yawRaw = (data[19] << 8) | data[18];    // Yaw high/low
    
    // Convert to degrees according to protocol
    imuData.roll = (float)rollRaw / 32768.0 * 180.0;
    imuData.pitch = (float)pitchRaw / 32768.0 * 180.0;
    imuData.yaw = (float)yawRaw / 32768.0 * 180.0;
    imuData.dataValid = true;
    
    // Parse optional time data if available (28-byte packet)
    if (length >= 28) {
      imuData.timestamp.year = data[20];
      imuData.timestamp.month = data[21];
      imuData.timestamp.day = data[22];
      imuData.timestamp.hour = data[23];
      imuData.timestamp.minute = data[24];
      imuData.timestamp.second = data[25];
      imuData.timestamp.millisecond = (data[27] << 8) | data[26]; // MSH MSL
      imuData.timestamp.timeValid = true;
      
      debugPrintf("Time: %02d/%02d/%02d %02d:%02d:%02d.%03d", 
                  imuData.timestamp.year, imuData.timestamp.month, imuData.timestamp.day,
                  imuData.timestamp.hour, imuData.timestamp.minute, imuData.timestamp.second,
                  imuData.timestamp.millisecond);
    } else {
      imuData.timestamp.timeValid = false;
    }
    
    return true;
  }
  
  return false;
}

// =============================================================================
// IMU DATA PARSING - UART MODE
// =============================================================================
bool parseIMUPacketUART() {
  static uint8_t buffer[32];
  static int bufferIndex = 0;
  
  // Read available bytes from Serial (UART0)
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
    // Collect data bytes - handle both 20-byte and 28-byte packets
    else if (bufferIndex >= 2 && bufferIndex < 28) {
      buffer[bufferIndex] = byte;
      bufferIndex++;
      
      // Process complete basic packet (20 bytes: header + flag + 18 data bytes)
      if (bufferIndex == 20) {
        // Extract angle data (bytes 14-19)
        int16_t rollRaw = (buffer[15] << 8) | buffer[14];   // Roll high/low
        int16_t pitchRaw = (buffer[17] << 8) | buffer[16];  // Pitch high/low  
        int16_t yawRaw = (buffer[19] << 8) | buffer[18];    // Yaw high/low
        
        // Convert to degrees according to protocol
        imuData.roll = (float)rollRaw / 32768.0 * 180.0;
        imuData.pitch = (float)pitchRaw / 32768.0 * 180.0;
        imuData.yaw = (float)yawRaw / 32768.0 * 180.0;
        imuData.dataValid = true;
        imuData.timestamp.timeValid = false; // No time data yet
        
        // Don't reset bufferIndex yet - check if more data is coming for time
      }
      
      // Process extended packet with time data (28 bytes total)
      else if (bufferIndex == 28) {
        // Parse time data (bytes 20-27)
        imuData.timestamp.year = buffer[20];
        imuData.timestamp.month = buffer[21];
        imuData.timestamp.day = buffer[22];
        imuData.timestamp.hour = buffer[23];
        imuData.timestamp.minute = buffer[24];
        imuData.timestamp.second = buffer[25];
        imuData.timestamp.millisecond = (buffer[27] << 8) | buffer[26]; // MSH MSL
        imuData.timestamp.timeValid = true;
        
        debugPrintf("UART Time: %02d/%02d/%02d %02d:%02d:%02d.%03d", 
                    imuData.timestamp.year, imuData.timestamp.month, imuData.timestamp.day,
                    imuData.timestamp.hour, imuData.timestamp.minute, imuData.timestamp.second,
                    imuData.timestamp.millisecond);
        
        bufferIndex = 0; // Reset for next packet
        return true;
      }
    }
    else {
      // Invalid sequence, reset
      bufferIndex = 0;
    }
  }
  
  // Handle timeout for incomplete packets - if we have basic data but no more bytes coming
  static unsigned long lastByteTime = 0;
  if (bufferIndex == 20 && (millis() - lastByteTime > 10)) {
    // Timeout waiting for time data - process basic packet
    bufferIndex = 0;
    return true;
  }
  
  if (bufferIndex > 0) lastByteTime = millis();
  
  return false;
}

// =============================================================================
// COMMON IMU DATA PROCESSING
// =============================================================================
void processIMUData() {
  // Convert IMU angles to actuator lengths
  targetLength[0] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[0]);
  targetLength[1] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[1]);
  targetLength[2] = computeActuatorLength(imuData.pitch, imuData.roll, actuatorAngles[2]);
  
  debugPrintf("IMU Data - Roll: %.2f°, Pitch: %.2f°, Yaw: %.2f°", 
              imuData.roll, imuData.pitch, imuData.yaw);
  debugPrintf("Actuator Targets - A: %.1fmm, B: %.1fmm, C: %.1fmm", 
              targetLength[0], targetLength[1], targetLength[2]);
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