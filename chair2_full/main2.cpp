#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <FS.h>
#include <SPIFFS.h>
#include "pin_definitions.h"

// =============================================================================
// STEWART PLATFORM REALTIME IMU CONTROL - FINAL VERSION WITH REGULAR BLE
// =============================================================================
// Integrates:
// - Regular BLE IMU connectivity (from imu_monitor.cpp)
// - Lookup table with bilinear interpolation
// - Smoothing and rate limiting for smooth motion
// - Focus on pitch/roll only (no heave)
// - UART0 debugging

// =============================================================================
// CONFIGURATION
// =============================================================================
// -- Stewart Platform Physical Dimensions
const int minLength = 550;      // mm - minimum actuator length
const int maxLength = 850;      // mm - maximum actuator length
// Neutral state lengths based on corrected mathematical model [[memory:4862545]]
const int neutralLength[3] = {735, 735, 670}; // mm - Motor 1(l1), Motor 2(l2), Motor 3(l3)
const float armRadius = 200.0;  // mm - radius from center to actuator mount

// -- Control Loop  
const float actuatorSpeed = 84.0; // mm/s - max actuator speed at 100% duty cycle
const int updateInterval = 10;  // ms - control loop interval (100 Hz)

// -- Motion Cueing Factors (heave disabled)
const float pitchCueFactor = 0.1; // degrees of extra pitch per °/s
const float rollCueFactor = 0.1;  // degrees of extra roll per °/s

// -- Smoothing Parameters
const float IMU_SMOOTHING_FACTOR = 0.3; // Low-pass filter strength (user adjusted)
const float TARGET_RATE_LIMIT = 50.0; // mm/s max rate of target change (INCREASED FOR DEBUG)
const float DEADBAND_THRESHOLD = 3; // mm - ignore changes smaller than this (reasonable for IMU noise)
const float REDUCED_GAIN = 4.0; // Reduced gain for smoother response

// -- Advanced Motion Control Parameters (TEMPORARILY DISABLED FOR DEBUG)
const bool ENABLE_DATA_RATE_LIMITING = true;  // Disabled when sequential targeting is active
const unsigned long IMU_DATA_INTERVAL_MS = 50;  // Process IMU data max every 500ms (2Hz)
const bool ENABLE_SEQUENTIAL_TARGETING = false;   // Individual actuators wait to reach target before accepting new ones

// -- Lookup Table Configuration
const char* LOOKUP_TABLE_FILE = "/lookup_table.txt";
const int MAX_LOOKUP_ENTRIES = 2000;  // Reduced to prevent memory issues

// -- Workspace Constraints (matching Python validation)
const float PITCH_MIN = -10.0;   // degrees
const float PITCH_MAX = 15.0;    // degrees
const float ROLL_MIN = -15.0;    // degrees
const float ROLL_MAX = 15.0;     // degrees

// -- BLE Configuration - REPLACE WITH YOUR IMU'S ACTUAL UUIDs
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");    // Update for your device
static BLEUUID CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");       // Update for your device

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
// IMU DATA STRUCTURES
// =============================================================================
struct IMUData {
  float roll = 0.0;
  float pitch = 0.0; 
  float yaw = 0.0;
  float ax = 0.0, ay = 0.0, az = 0.0;       // Acceleration (g)
  float wx = 0.0, wy = 0.0, wz = 0.0;       // Angular velocity (°/s)
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

// =============================================================================
// GLOBAL VARIABLES
// =============================================================================
// -- BLE Variables (using regular BLE instead of NimBLE)
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
bool bleConnected = false;
bool bleConnecting = false;
BLEAdvertisedDevice* targetDevice = nullptr;
bool deviceFound = false;
unsigned long connectionStartTime = 0;

// -- Actuator Position Tracking
float currentLength[3] = {minLength, minLength, minLength}; // Current lengths [A, B, C]
float targetLength[3] = {neutralLength[0], neutralLength[1], neutralLength[2]}; // Target lengths [A, B, C]
const float actuatorAngles[3] = {0, 120, 240}; // Actuator positions in degrees
bool systemHomed = false;

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
float filteredAngVelX = 0.0;
float filteredAngVelY = 0.0;
float previousTargets[3] = {neutralLength[0], neutralLength[1], neutralLength[2]};
unsigned long lastIMUTime = 0;
bool smoothingInitialized = false;

// -- Advanced Motion Control Variables
unsigned long lastIMUProcessTime = 0;     // For data rate limiting
bool targetsReached[3] = {true, true, true}; // Track if each actuator reached target


// -- IMU Data
IMUData imuData;
unsigned long lastPacketTime = 0;
int packetCount = 0;

// =============================================================================
// FUNCTION DECLARATIONS
// =============================================================================
// System initialization
void initializeSystem();
void performHomingSequence();

// BLE functions (using regular BLE instead of NimBLE)
bool initializeBLE();
void scanForIMU();
bool connectToIMU();
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

// IMU data processing
bool parseFullIMUPacket(uint8_t* data, size_t length);
void processIMUData();

// Lookup table functions
bool loadLookupTable();
bool validateWorkspaceConstraints(float pitch_deg, float roll_deg);
bool getActuatorLengthsFromLookup(float pitch_deg, float roll_deg, float& l1, float& l2, float& l3);
bool validateActuatorLengths(float l1, float l2, float l3);
float bilinearInterpolate(float x, float y, float x1, float y1, float x2, float y2, 
                         float f11, float f12, float f21, float f22);
void cleanupLookupTable();

// Smoothing functions
void applyIMUSmoothing(float& pitch, float& roll, float& ang_vel_x, float& ang_vel_y);
void applyTargetRateLimiting(float newTargets[3], float previousTargets[3], float maxRate_mm_per_s, float deltaTime_s);

// Advanced motion control functions
bool shouldProcessIMUData();          // Data rate limiting
bool allTargetsReached();             // Status display helper

void updateTargetReachedStatus();     // Update target reached flags

// Stewart platform kinematics
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg); // Mathematical fallback

// Motor control
void updateActuatorsSmoothly();


// =============================================================================
// BLE CALLBACK CLASSES (using regular BLE instead of NimBLE)
// =============================================================================
class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) override {
    debugPrint("BLE CLIENT CALLBACK: Connected to IMU device");
    bleConnected = true;
    bleConnecting = false;
  }

  void onDisconnect(BLEClient* pclient) override {
    debugPrint("BLE CLIENT CALLBACK: Disconnected from IMU device");
    bleConnected = false;
    bleConnecting = false;
    pRemoteCharacteristic = nullptr;  // Reset characteristic pointer
  }
};

class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    debugPrintf("Found device: %s", advertisedDevice.toString().c_str());
    debugPrintf("  - Name: %s", advertisedDevice.getName().c_str());
    debugPrintf("  - Address: %s", advertisedDevice.getAddress().toString().c_str());
    debugPrintf("  - RSSI: %d", advertisedDevice.getRSSI());
    
    // Show all service UUIDs this device advertises
    if (advertisedDevice.haveServiceUUID()) {
      debugPrint("  - Advertised Service UUIDs:");
      for (int i = 0; i < advertisedDevice.getServiceUUIDCount(); i++) {
        debugPrintf("    * %s", advertisedDevice.getServiceUUID(i).toString().c_str());
      }
    } else {
      debugPrint("  - No service UUIDs advertised");
    }
    
    // Check if this device has our service UUID
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      debugPrint("Found target IMU device! Stopping scan...");
      deviceFound = true;
      targetDevice = new BLEAdvertisedDevice(advertisedDevice);
      BLEDevice::getScan()->stop();
    } else {
      debugPrint("  - Does not match our target service UUID");
    }
    debugPrint("  ---");
  }
};

// =============================================================================
// SETUP
// =============================================================================
void setup() {
  setupDebugUART0();  // Use UART0 for debugging as requested
  delay(1000);
  
  debugPrint("\n==== Stewart Platform Real-time IMU Control (Final Version with Regular BLE) ====");
    
  // Initialize system components
    initializeSystem();
    
  // Initialize SPIFFS for lookup table
  if (!SPIFFS.begin(true)) {
    debugPrint("[!] SPIFFS mount failed");
    // Continue without lookup table - will use fallback
  } else {
    debugPrint("[✓] SPIFFS mounted");
    
    // Load lookup table
    if (loadLookupTable()) {
      debugPrint("[✓] Lookup table loaded successfully");
    } else {
      debugPrint("[!] Failed to load lookup table - using mathematical fallback");
    }
  }
  
  // Initialize regular BLE
  debugPrint("Initializing BLE...");
    if (initializeBLE()) {
    debugPrint("[✓] BLE initialized successfully");
      scanForIMU();
    } else {
    debugPrint("[!] BLE initialization failed!");
      return;
    }
  
  // Perform homing sequence
  performHomingSequence();
  
  debugPrint("[✓] System initialized and ready for IMU control");
  debugPrint("Waiting for IMU data packets via BLE...");
  debugPrint("---------------------------------------------------------------");
}

// =============================================================================
// MAIN LOOP
// =============================================================================
void loop() {
  // Debug: Show current BLE state
  static unsigned long lastStatusTime = 0;
  if (millis() - lastStatusTime > 5000) {
    debugPrintf("Status - BLE: %s | Homed: %s | Lookup: %s | Packets: %d", 
                bleConnected ? "CONN" : "DISC", 
                systemHomed ? "YES" : "NO",
                lookupTableLoaded ? "LOADED" : "MATH",
                packetCount);
    
    // Show advanced motion control status
    if (systemHomed) {
      debugPrintf("Motion - DataLimit: %s | Sequential: %s | AllReached: %s%s", 
                  ENABLE_DATA_RATE_LIMITING ? "ON" : "OFF",
                  ENABLE_SEQUENTIAL_TARGETING ? "ON" : "OFF", 
                  allTargetsReached() ? "YES" : "NO",
                  (ENABLE_DATA_RATE_LIMITING && ENABLE_SEQUENTIAL_TARGETING) ? " [CONFLICT!]" : "");
    }
    
    lastStatusTime = millis();
  }
  
  // Handle connection timeout
  if (bleConnecting && !bleConnected && millis() - connectionStartTime > 8000) {
    debugPrint("Connection timeout (8 seconds) - resetting connection state");
    bleConnecting = false;
    if (pClient != nullptr) {
      pClient->disconnect();
      delete pClient;
      pClient = nullptr;
    }
  }
  
  // Handle BLE connection state with retry delay
  static unsigned long lastScanTime = 0;
    if (!bleConnected && !bleConnecting && systemHomed) {
    if (millis() - lastScanTime > 10000) { // Wait 10 seconds between scan attempts
      debugPrint("Attempting to connect to IMU...");
      scanForIMU();
      lastScanTime = millis();
    }
  }
  
  // Try to establish service connection after BLE connection
  if (bleConnected && pRemoteCharacteristic == nullptr) {
    debugPrint(">>> BLE connected, establishing service connection...");
    if (connectToIMU()) {
      debugPrint(">>> Service connection established successfully");
  } else {
      debugPrint(">>> Failed to establish service connection");
      // If service connection fails, disconnect and retry
      bleConnected = false;
      if (pClient != nullptr) {
        pClient->disconnect();
      }
    }
  }
  
  // Update actuators at regular intervals (only after homing is complete)
  if (systemHomed) {
    unsigned long now = millis();
    if (now - lastUpdate >= updateInterval) {

      
      // Update target reached status
      updateTargetReachedStatus();
      
      // Update actuator control
      updateActuatorsSmoothly();
      lastUpdate = now;
    }
  }
  
  // Show status if no packets received for a while
  if (systemHomed && bleConnected && pRemoteCharacteristic != nullptr && 
      millis() - lastPacketTime > 5000 && packetCount == 0) {
    debugPrint("No IMU packets received yet. Check IMU configuration and UUIDs.");
    lastPacketTime = millis(); // Prevent spam
  }
  
  delay(10);
}

// =============================================================================
// SYSTEM INITIALIZATION
// =============================================================================
void initializeSystem() {
  // Initialize motor drivers and PWM from pin_definitions.cpp
  initializePins();
  setupPWM();
  stopAllMotors();
  debugPrint("[✓] Motor drivers initialized");
}

// =============================================================================
// HOMING SEQUENCE
// =============================================================================
void performHomingSequence() {
  debugPrint("[*] Starting homing sequence...");
  debugPrint("[*] Moving actuators down to shortest position (4 seconds at full speed)...");
  
  // Move all actuators down at full speed for reliable initialization
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
  
  // Set targets to neutral lengths and move there
  for (int i = 0; i < 3; i++) {
    targetLength[i] = neutralLength[i];
    currentLength[i] = minLength; // Set current position to minimum since we're at bottom
  }
  
  debugPrint("[*] Moving to neutral positions...");
  
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
  systemHomed = true;
  
  debugPrint("[✓] Homing complete - platform stable at neutral state");
}

// =============================================================================
// BLE INITIALIZATION AND CONNECTION (using regular BLE instead of NimBLE)
// =============================================================================
bool initializeBLE() {
  BLEDevice::init("Stewart_Platform_Control");
  return true;
}

void scanForIMU() {
  if (bleConnecting) return;
  
  // Reset flags
  deviceFound = false;
  if (targetDevice != nullptr) {
    delete targetDevice;
    targetDevice = nullptr;
  }
  
  bleConnecting = true;
  debugPrint("Scanning for IMU device...");
  debugPrintf("Looking for service UUID: %s", SERVICE_UUID.toString().c_str());
  debugPrint("Scan will run for 10 seconds...");
  
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  
  // Start scan 
  BLEScanResults foundDevices = pBLEScan->start(10, false); // Scan for 10 seconds
  
  // Scan completed
  debugPrintf("Scan completed. Found %d devices total", foundDevices.getCount());
  
  // Check if we found our target device
  if (deviceFound && targetDevice != nullptr) {
    debugPrint("Target device found, attempting connection...");
    
    // Clean up any existing client
    if (pClient != nullptr) {
      pClient->disconnect();
      delete pClient;
    }
    
    pClient = BLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    
    connectionStartTime = millis();
    debugPrint("Calling pClient->connect()...");
    
    bool connectResult = pClient->connect(targetDevice);
    debugPrintf("pClient->connect() returned: %s", connectResult ? "SUCCESS" : "FAILED");
    
    if (connectResult) {
      debugPrint("BLE connection established successfully");
      bleConnected = true;
      bleConnecting = false;
    } else {
      debugPrint("Failed to establish BLE connection");
      bleConnecting = false;
      // Clean up failed client
      delete pClient;
      pClient = nullptr;
    }
  } else {
    bleConnecting = false;
    debugPrint("No matching IMU device found during scan");
  }
}

bool connectToIMU() {
  debugPrint("*** connectToIMU() called");
  if (!bleConnected || pClient == nullptr) {
    debugPrintf("*** Early return: bleConnected=%s, pClient=%s", 
                bleConnected ? "true" : "false", 
                pClient ? "valid" : "null");
    return false;
  }
  
  debugPrint("*** Getting IMU service...");
  BLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  debugPrint("*** getService() call completed");
  
  if (pRemoteService == nullptr) {
    debugPrint("*** Failed to find IMU service");
    return false;
  }
  
  debugPrint("*** Getting IMU characteristic...");
  pRemoteCharacteristic = pRemoteService->getCharacteristic(CHAR_UUID);
  debugPrint("*** getCharacteristic() call completed");
  if (pRemoteCharacteristic == nullptr) {
    debugPrint("Failed to find IMU characteristic");
    
    // Try to get all characteristics and show what's available
    debugPrint("Available characteristics:");
    std::map<std::string, BLERemoteCharacteristic*>* charMap = pRemoteService->getCharacteristics();
    for (auto& pair : *charMap) {
      debugPrintf("  - %s", pair.first.c_str());
    }
    return false;
  }
  
  debugPrint("Found IMU characteristic, setting up notifications...");
  
  // Try to register for notifications without checking descriptors first
  try {
    pRemoteCharacteristic->registerForNotify(onNotify);
    debugPrint("BLE notifications registered successfully");
    
    // Try to subscribe to notifications by writing to CCCD
    uint8_t notificationOn[] = {0x1, 0x0};
    if (pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902)) != nullptr) {
      pRemoteCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      debugPrint("CCCD descriptor written");
    } else {
      debugPrint("No CCCD descriptor found, but notification callback registered");
    }
    
    return true;
  } catch (std::exception& e) {
    debugPrintf("Exception during notification setup: %s", e.what());
    return false;
  }
}

// =============================================================================
// IMU DATA PROCESSING  
// =============================================================================
// BLE notification callback
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("🔔 NOTIFICATION RECEIVED! %d bytes", length);
  
  if (parseFullIMUPacket(pData, length)) {
    packetCount++;
    lastPacketTime = millis();
    debugPrintf("Successfully parsed IMU packet #%d", packetCount);
    
    // Process the IMU data for platform control
    if (systemHomed) {
      processIMUData();
    }
    
  } else {
    debugPrint("Failed to parse IMU packet");
    // Print raw data for debugging
    debugPrint("Raw packet data:");
    for(int i = 0; i < length && i < 32; i++) {
      Serial.printf("0x%02X ", pData[i]);
      if((i + 1) % 8 == 0) Serial.println();
    }
    Serial.println();
  }
}

// Parse IMU packet from BLE data
bool parseFullIMUPacket(uint8_t* data, size_t length) {
  // Check for minimum packet size (header + flag + 18 data bytes)
  if (length < 20) return false;
  
  // Look for packet header (0x55 0x61)
  if (data[0] == 0x55 && data[1] == 0x61) {
    // Extract acceleration data (bytes 2-7) - not used but available for future
    int16_t axRaw = (data[3] << 8) | data[2];
    int16_t ayRaw = (data[5] << 8) | data[4];  
    int16_t azRaw = (data[7] << 8) | data[6];
    
    // Extract angular velocity data (bytes 8-13)
    int16_t wxRaw = (data[9] << 8) | data[8];
    int16_t wyRaw = (data[11] << 8) | data[10];
    int16_t wzRaw = (data[13] << 8) | data[12];
    
    // Extract angle data (bytes 14-19)
    int16_t rollRaw = (data[15] << 8) | data[14];
    int16_t pitchRaw = (data[17] << 8) | data[16]; 
    int16_t yawRaw = (data[19] << 8) | data[18];
    
    // Convert to physical units according to protocol
    imuData.ax = (float)axRaw / 32768.0 * 16.0;      // ±16g (not used - heave disabled)
    imuData.ay = (float)ayRaw / 32768.0 * 16.0;
    imuData.az = (float)azRaw / 32768.0 * 16.0;
    
    imuData.wx = (float)wxRaw / 32768.0 * 2000.0;    // ±2000°/s
    imuData.wy = (float)wyRaw / 32768.0 * 2000.0;
    imuData.wz = (float)wzRaw / 32768.0 * 2000.0;
    
    imuData.roll = (float)rollRaw / 32768.0 * 180.0;  // ±180°
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
      
      debugPrintf("BLE Time: %02d/%02d/%02d %02d:%02d:%02d.%03d", 
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

void processIMUData() {
  if (!imuData.dataValid) return;
  
  // 1. Data Rate Limiting - only process at maximum specified rate
  // NOTE: Incompatible with sequential targeting - either use one or the other
  if (ENABLE_DATA_RATE_LIMITING && !shouldProcessIMUData()) {
    debugPrint("[DEBUG] IMU processing skipped: Data rate limiting active");
    return; // Skip this data point
  }
  if (ENABLE_DATA_RATE_LIMITING) {
    lastIMUProcessTime = millis();
  }
  
  // 2. Sequential Targeting - process IMU but only update individual actuators that have reached targets
  // (No longer blocks entire IMU processing - handles per-actuator in the lookup/targeting logic)
  
  // Extract raw IMU values
  float pitch = imuData.pitch;
  float roll = imuData.roll;
  float ang_vel_x = imuData.wx;
  float ang_vel_y = imuData.wy;
  
  // Debug: Show raw IMU values
  debugPrintf("[DEBUG] Raw IMU: P:%.2f° R:%.2f° Wx:%.1f°/s Wy:%.1f°/s", 
              pitch, roll, ang_vel_x, ang_vel_y);
  
  // Apply IMU smoothing
  applyIMUSmoothing(pitch, roll, ang_vel_x, ang_vel_y);
  
  // Debug: Show smoothed values
  debugPrintf("[DEBUG] Smoothed IMU: P:%.2f° R:%.2f°", pitch, roll);
  
  // Motion cueing from angular velocity (no heave)
  float pitch_with_cue = pitch + (ang_vel_y * pitchCueFactor);
  float roll_with_cue = roll + (ang_vel_x * rollCueFactor);
  
  // Store current time for rate limiting
  unsigned long currentTime = millis();
  float deltaTime_s = (currentTime - lastIMUTime) / 1000.0;
  lastIMUTime = currentTime;
  
  // Calculate new target lengths using lookup table
  float newTargets[3];
  float lookup_l1, lookup_l2, lookup_l3;
  bool lookupSuccess = false;
  
  if (lookupTableLoaded) {
    // Use lookup table with automatic clamping for continuous movement
    lookupSuccess = getActuatorLengthsFromLookup(pitch_with_cue, roll_with_cue, 
                                                lookup_l1, lookup_l2, lookup_l3);
  }
  
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
    debugPrint("[!] Lookup table unavailable - using mathematical fallback");
    for (int i = 0; i < 3; i++) {
      newTargets[i] = computeActuatorLength(pitch_with_cue, roll_with_cue, actuatorAngles[i]);
    }
  }
  
  // Apply rate limiting for smoother movement
  if (deltaTime_s > 0 && deltaTime_s < 1.0) { // Valid time delta
    applyTargetRateLimiting(newTargets, previousTargets, TARGET_RATE_LIMIT, deltaTime_s);
  }
  
  // Debug: Show calculated targets before rate limiting
  debugPrintf("[DEBUG] Calculated targets: %.1f,%.1f,%.1f mm", 
              newTargets[0], newTargets[1], newTargets[2]);
  debugPrintf("[DEBUG] Previous targets: %.1f,%.1f,%.1f mm", 
              previousTargets[0], previousTargets[1], previousTargets[2]);
  
  // Update target lengths with smoothed values
  // With sequential targeting, only update individual actuators that have reached their current targets
  for (int i = 0; i < 3; i++) {
    if (!ENABLE_SEQUENTIAL_TARGETING || targetsReached[i]) {
      // Either sequential targeting is disabled, or this actuator has reached its target
      targetLength[i] = newTargets[i];
      previousTargets[i] = newTargets[i];
    } else {
      // Sequential targeting enabled and actuator hasn't reached target - keep current target
      debugPrintf("[DEBUG] Actuator %d still moving (%.1fmm->%.1fmm), keeping current target", 
                  i + 1, currentLength[i], targetLength[i]);
    }
  }
  
  // Debug: Show final targets after rate limiting
  debugPrintf("[DEBUG] Final targets: %.1f,%.1f,%.1f mm", 
              targetLength[0], targetLength[1], targetLength[2]);
  
  // Optional: Log received data to serial for debugging
  static unsigned long lastLogTime = 0;
  if (millis() - lastLogTime > 1000) { // Log once per second
    debugPrintf("IMU: P:%.1f° R:%.1f° | Targets: %.1f,%.1f,%.1f mm", 
                pitch, roll, targetLength[0], targetLength[1], targetLength[2]);
    lastLogTime = millis();
  }
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
  filteredAngVelX = IMU_SMOOTHING_FACTOR * ang_vel_x + (1.0 - IMU_SMOOTHING_FACTOR) * filteredAngVelX;
  filteredAngVelY = IMU_SMOOTHING_FACTOR * ang_vel_y + (1.0 - IMU_SMOOTHING_FACTOR) * filteredAngVelY;
  
  // Update input values with filtered values
  pitch = filteredPitch;
  roll = filteredRoll;
  ang_vel_x = filteredAngVelX;
  ang_vel_y = filteredAngVelY;
}

void applyTargetRateLimiting(float newTargets[3], float previousTargets[3], float maxRate_mm_per_s, float deltaTime_s) {
  // Limit how fast targets can change to prevent jerky movement
  float maxChange_mm = maxRate_mm_per_s * deltaTime_s;
  
  debugPrintf("[DEBUG] Rate limiting: maxChange=%.2f mm, deltaTime=%.3f s", maxChange_mm, deltaTime_s);
  
  for (int i = 0; i < 3; i++) {
    float targetChange = newTargets[i] - previousTargets[i];
    
    debugPrintf("[DEBUG] Motor %d: change=%.2f mm (threshold=%.1f mm)", 
                i+1, targetChange, DEADBAND_THRESHOLD);
    
    // Apply deadband - ignore very small changes to reduce noise
    if (abs(targetChange) < DEADBAND_THRESHOLD) {
      debugPrintf("[DEBUG] Motor %d: Change below deadband, ignored", i+1);
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

// =============================================================================
// ADVANCED MOTION CONTROL FUNCTIONS
// =============================================================================

bool shouldProcessIMUData() {
  // Data rate limiting - only process IMU data at maximum specified frequency
  unsigned long currentTime = millis();
  return (currentTime - lastIMUProcessTime >= IMU_DATA_INTERVAL_MS);
}

bool allTargetsReached() {
  // Check if all actuators have reached their targets within tolerance
  for (int i = 0; i < 3; i++) {
    if (!targetsReached[i]) {
      return false;
    }
  }
  return true;
}

void updateTargetReachedStatus() {
  // Update the target reached status for each actuator using the same logic as precision stop
  for (int i = 0; i < 3; i++) {
    targetsReached[i] = (abs(targetLength[i] - currentLength[i]) < 0.5); // Same as precision stop threshold
  }
  
  // Debug output when individual targets are reached
  static bool lastReached[3] = {false, false, false};
  for (int i = 0; i < 3; i++) {
    if (!lastReached[i] && targetsReached[i]) {
      debugPrintf("[*] Actuator %d reached target - ready for new commands", i + 1);
    } else if (lastReached[i] && !targetsReached[i]) {
      debugPrintf("[*] Actuator %d moving to new target...", i + 1);
    }
    lastReached[i] = targetsReached[i];
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
  
  // Use neutral length as baseline
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

