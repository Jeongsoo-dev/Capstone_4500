#include <Arduino.h>
#include <NimBLEDevice.h>
#include <vector>
#include "pin_definitions.h"

// =============================================================================
// IMU DATA MONITOR - Dual Mode (NimBLE + UART) - FIXED CONNECTION HANGING!
// =============================================================================
// This utility helps verify that IMU data is being received correctly
// Uses nimBLE instead of ESP32 BLE to solve connection hanging issues
// Supports both Bluetooth and UART communication modes
// Use Serial2 (UART2) for all debug output, keeps Serial available for IMU data
//
// NIMBLE ADVANTAGES:
// 1. ASYNC CONNECTIONS: No more hanging on pClient->connect()!
// 2. BETTER STABILITY: Improved BLE stack with better resource management
// 3. API COMPATIBLE: Minimal code changes from original BLE library
// 4. PROPER CLEANUP: Better client lifecycle management

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
// COMMUNICATION MODE CONFIGURATION
// =============================================================================
// Set to true for Bluetooth, false for UART/Serial communication
const bool USE_BLUETOOTH_MODE = true;

// DEBUG MODE - Set to true for additional debugging output
const bool DEBUG_MODE = true;

// BLE Configuration - Using the actual UUIDs found via LightBlue AND VENDOR PYTHON CODE
static NimBLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");        // Main service  
static NimBLEUUID NOTIFY_CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");    // For receiving notifications (FIXED!)
static NimBLEUUID WRITE_CHAR_UUID("0000ffe9-0000-1000-8000-00805f9a34fb");     // For sending commands (FIXED!)

// BLE Variables - nimBLE versions
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pNotifyCharacteristic = nullptr;  // For receiving data
NimBLERemoteCharacteristic* pWriteCharacteristic = nullptr;   // For sending commands
bool bleConnected = false;
bool bleConnecting = false;
NimBLEAdvertisedDevice* targetDevice = nullptr;
bool deviceFound = false;
unsigned long connectionStartTime = 0;
unsigned long lastDataRequestTime = 0;  // For periodic data requests
unsigned long lastScanAttempt = 0;       // For connection retry timing
int connectionAttempts = 0;              // Track connection attempts

IMUData imuData;
unsigned long lastPacketTime = 0;
int packetCount = 0;

// Function declarations
bool initializeNimBLE();
bool connectToIMU();
void scanForIMU();
bool parseFullIMUPacket(uint8_t* data, size_t length);
bool parseFullIMUPacketUART();  // For UART mode
void printIMUData();
void onNotify(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);
void sendDataRequest();  // Vendor protocol data request
std::vector<uint8_t> createReadCommand(uint8_t regAddr);  // Vendor protocol helper

// Debug functions are already declared in pin_definitions.h

// nimBLE Client Callbacks - ENHANCED with better error handling
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) {
    unsigned long connectTime = millis() - connectionStartTime;
    debugPrintf("🎉 NIMBLE CLIENT: Connected to IMU device! (took %lu ms)", connectTime);
    bleConnected = true;
    bleConnecting = false;
    
    // Reset connection attempts on successful connection
    connectionAttempts = 0;
    
    debugPrint("✅ Connection established successfully with nimBLE!");
    debugPrint("💡 If you see data after replug, it was buffered - this is now fixed!");
  }

  void onDisconnect(NimBLEClient* pclient, int reason) {
    debugPrintf("⚠️  NIMBLE CLIENT: Disconnected from IMU device (reason: %d)", reason);
    
    // Common disconnect reasons for debugging:
    switch(reason) {
      case 0x08: debugPrint("   Reason: Connection timeout"); break;
      case 0x13: debugPrint("   Reason: Remote user terminated connection"); break;
      case 0x16: debugPrint("   Reason: Connection terminated by local host"); break;
      case 0x22: debugPrint("   Reason: LMP response timeout"); break;
      case 0x28: debugPrint("   Reason: Instant passed"); break;
      default: debugPrintf("   Reason: Unknown (%d)", reason); break;
    }
    
    bleConnected = false;
    bleConnecting = false;
    
    // Clean up characteristics
    pNotifyCharacteristic = nullptr;
    pWriteCharacteristic = nullptr;
    
    debugPrint("Will retry connection in 15 seconds...");
  }
  
  void onConnectFail(NimBLEClient* pclient, int reason) {
    debugPrintf("❌ NIMBLE CLIENT: Connection failed (reason: %d)", reason);
    bleConnected = false;
    bleConnecting = false;
    debugPrint("Will retry connection after delay...");
  }
};

// nimBLE Advertisement Callback - ENHANCED
class MyAdvertisedDeviceCallbacks: public NimBLEAdvertisedDeviceCallbacks {
  void onResult(NimBLEAdvertisedDevice* advertisedDevice) {
    debugPrintf("📡 Found device: %s", advertisedDevice->toString().c_str());
    debugPrintf("  - Name: %s", advertisedDevice->getName().c_str());
    debugPrintf("  - Address: %s", advertisedDevice->getAddress().toString().c_str());
    debugPrintf("  - RSSI: %d dBm", advertisedDevice->getRSSI());
    
    // Show all service UUIDs this device advertises
    if (advertisedDevice->haveServiceUUID()) {
      debugPrint("  - Advertised Service UUIDs:");
      for (int i = 0; i < advertisedDevice->getServiceUUIDCount(); i++) {
        debugPrintf("    * %s", advertisedDevice->getServiceUUID(i).toString().c_str());
      }
    } else {
      debugPrint("  - No service UUIDs advertised");
    }
    
    // Check if this device has our service UUID
    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(SERVICE_UUID)) {
      debugPrint("🎯 Found target IMU device! Stopping scan...");
      deviceFound = true;
      targetDevice = advertisedDevice;
      NimBLEDevice::getScan()->stop();
    } else {
      debugPrint("  - Does not match our target service UUID");
    }
    debugPrint("  ---");
  }
};

void setup() {
  // Initialize debug interface first
  setupDebugUART2();
  delay(1000);
  
  if (USE_BLUETOOTH_MODE) {
    debugPrint("==== IMU Data Monitor (nimBLE) Starting ====");
    debugPrint("🚀 Using nimBLE - NO MORE CONNECTION HANGING!");
    debugPrint("Initializing nimBLE stack...");
    
    initializeNimBLE(); // Always succeeds  
    debugPrint("✅ nimBLE initialized successfully");
    scanForIMU();
  } else {
    debugPrint("==== IMU Data Monitor (UART) Starting ====");
    debugPrint("Monitoring for IMU packets (0x55 0x61)");
    debugPrint("Expected format: Header(0x55) + Flag(0x61) + 18 data bytes");
    
    // Initialize Serial for UART communication with IMU
    Serial.begin(115200);
    debugPrint("UART initialized for IMU communication at 115200 baud");
  }
  
  debugPrint("---------------------------------------------------------------");
}

void loop() {
  if (USE_BLUETOOTH_MODE) {
    // Debug: Show current nimBLE state
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 5000) {
      debugPrintf("📊 nimBLE Status - Connected: %s, Connecting: %s, Attempts: %d", 
                  bleConnected ? "YES" : "NO", 
                  bleConnecting ? "YES" : "NO",
                  connectionAttempts);
      lastStatusTime = millis();
    }
    
    // Handle connection timeout - if connecting too long without callback
    if (bleConnecting && !bleConnected) {
      if (millis() - connectionStartTime > 30000) { // 30 second timeout
        debugPrint("⏰ Connection timeout! Cleaning up and retrying...");
        bleConnecting = false;
        
        // Clean up failed connection attempt
        if (pClient != nullptr) {
          debugPrint("🧹 Cleaning up timed-out connection...");
          NimBLEDevice::deleteClient(pClient);
          pClient = nullptr;
        }
        
        // Reset for next attempt
        lastScanAttempt = millis() - 10000; // Allow retry in 5 seconds
      }
    }
    
    // Handle nimBLE connection state with retry delay  
    if (!bleConnected && !bleConnecting) {
      if (millis() - lastScanAttempt > 15000) { // Wait 15 seconds between attempts
        debugPrintf("🔄 Connection attempt #%d - Scanning for IMU...", ++connectionAttempts);
        scanForIMU();
        lastScanAttempt = millis();
      }
    }
    
    // Try to establish service connection after BLE connection
    if (bleConnected && pNotifyCharacteristic == nullptr) {
      debugPrint("🔗 nimBLE connected, establishing service connection...");
      if (connectToIMU()) {
        debugPrint("✅ Service connection established successfully");
        debugPrint("🚀 Ready to receive IMU data!");
        lastDataRequestTime = millis(); // Initialize request timer
      } else {
        debugPrint("❌ Failed to establish service connection - will retry in 5 seconds");
        // Don't immediately disconnect - give it a few tries first
        static int serviceRetryCount = 0;
        serviceRetryCount++;
        
        if (serviceRetryCount >= 3) {
          debugPrint("🔄 Service connection failed 3 times, reconnecting BLE...");
          serviceRetryCount = 0;
          if (pClient != nullptr) {
            pClient->disconnect();
          }
        } else {
          debugPrintf("⏳ Service retry attempt %d/3 in 5 seconds...", serviceRetryCount);
          delay(5000); // Wait before retry
        }
      }
    }
    
    // Send periodic data requests using vendor protocol (every 200ms)
    if (bleConnected && pWriteCharacteristic != nullptr && millis() - lastDataRequestTime > 200) {
      sendDataRequest();
      lastDataRequestTime = millis();
    }
    
    // Show status if no packets received for a while
    if (bleConnected && pNotifyCharacteristic != nullptr && millis() - lastPacketTime > 5000 && packetCount == 0) {
      debugPrint("📡 No IMU packets received yet. Trying vendor protocol data requests...");
      lastPacketTime = millis(); // Prevent spam
    }
  } else {
    // UART mode - check for incoming data
    if (parseFullIMUPacketUART()) {
      packetCount++;
      lastPacketTime = millis();
      debugPrintf("✅ Successfully parsed IMU packet #%d", packetCount);
      printIMUData();
    }
    
    // Show status if no packets received for a while  
    if (millis() - lastPacketTime > 5000 && packetCount == 0) {
      debugPrint("📡 No IMU packets received yet. Check connections and baud rate.");
      lastPacketTime = millis(); // Prevent spam
    }
  }
  
  delay(100);
}

bool initializeNimBLE() {
  NimBLEDevice::init("Stewart_Platform_Monitor");
  
  // Configure nimBLE for better performance and stability
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // Maximum power for better range
  
  debugPrint("📡 nimBLE device initialized with enhanced settings");
  return true;
}

void scanForIMU() {
  if (bleConnecting) return;
  
  // Reset flags
  deviceFound = false;
  targetDevice = nullptr;
  
  bleConnecting = true;
  debugPrint("🔍 Scanning for IMU device...");
  debugPrintf("Looking for service UUID: %s", SERVICE_UUID.toString().c_str());
  debugPrint("Scan will run for 10 seconds...");
  
  NimBLEScan* pBLEScan = NimBLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  
  // Start scan - nimBLE handles this more efficiently
  NimBLEScanResults foundDevices = pBLEScan->start(10, false); // Scan for 10 seconds
  
  // Scan completed
  debugPrintf("✅ Scan completed. Found %d devices total", foundDevices.getCount());
  
  // Check if we found our target device
  if (deviceFound && targetDevice != nullptr) {
    debugPrint("🎯 Target device found, attempting nimBLE connection...");
    
    // Clean up any existing client - nimBLE way
    if (pClient != nullptr) {
      debugPrint("🧹 Cleaning up existing client...");
      NimBLEDevice::deleteClient(pClient);
      pClient = nullptr;
    }
    
    // Create new nimBLE client
    debugPrint("🔨 Creating new nimBLE client...");
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    
    // ASYNC CONNECTION - This is the key fix for hanging!
    debugPrint("🚀 Attempting ASYNC connection (no more hanging!)");
    debugPrintf("Target: %s (RSSI: %d dBm)", targetDevice->getAddress().toString().c_str(), targetDevice->getRSSI());
    
    connectionStartTime = millis();
    
    // Connect to device using nimBLE
    bool connectResult = pClient->connect(targetDevice);  // nimBLE async by default
    
    if (connectResult) {
      debugPrint("✅ Async connection command sent successfully!");
      debugPrint("   Connection will complete asynchronously via callbacks");
      // bleConnecting remains true, will be set false in callbacks
    } else {
      debugPrint("⚠️  Connect command returned false, but trying anyway...");
      debugPrint("   nimBLE may still establish connection via callbacks");
      debugPrint("   Will timeout after 30 seconds if no connection");
      // DON'T clean up immediately - let callbacks handle it!
      // bleConnecting remains true, timeout will handle cleanup if needed
    }
  } else {
    bleConnecting = false;
    debugPrint("❌ No matching IMU device found during scan");
  }
}

bool connectToIMU() {
  debugPrint("🔗 connectToIMU() called");
  if (!bleConnected || pClient == nullptr) {
    debugPrintf("❌ Early return: bleConnected=%s, pClient=%s", 
                bleConnected ? "true" : "false", 
                pClient ? "valid" : "null");
    return false;
  }
  
  debugPrint("🔍 Getting IMU service...");
  NimBLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
  
  if (pRemoteService == nullptr) {
    debugPrint("❌ Failed to find IMU service");
    return false;
  }
  
  debugPrint("🔍 Getting IMU notify characteristic (FIXED UUID: ffe4)...");
  pNotifyCharacteristic = pRemoteService->getCharacteristic(NOTIFY_CHAR_UUID);
  if (pNotifyCharacteristic == nullptr) {
    debugPrint("❌ Failed to find IMU notify characteristic");
    
    // Try to get all characteristics and show what's available
    debugPrint("Available characteristics:");
    auto* charVector = pRemoteService->getCharacteristics(true); // Force refresh
    for (auto* pChar : *charVector) {
      debugPrintf("  - %s", pChar->getUUID().toString().c_str());
    }
    return false;
  }
  
  debugPrint("🔍 Getting IMU write characteristic (FIXED UUID: ffe9)...");
  pWriteCharacteristic = pRemoteService->getCharacteristic(WRITE_CHAR_UUID);
  if (pWriteCharacteristic == nullptr) {
    debugPrint("❌ Failed to find IMU write characteristic");
    return false;
  }
  
  debugPrint("✅ Found both IMU characteristics with CORRECTED UUIDs!");
  debugPrint("📡 Setting up notifications using vendor protocol...");
  
  // Subscribe to notifications using nimBLE method
  try {
    if (pNotifyCharacteristic->subscribe(true, onNotify)) {
      debugPrint("✅ nimBLE notifications registered successfully on FFE4 (receive) characteristic");
      
      // Clear any buffered data to prevent "data after replug" issue
      debugPrint("🧹 Flushing any buffered data from previous connections...");
      
      // Send a few immediate data requests to flush buffers
      delay(100); // Small delay to ensure subscription is active
      for (int i = 0; i < 3; i++) {
        sendDataRequest();
        delay(50);
      }
      
      debugPrint("🔧 Using VENDOR PROTOCOL: Device requires periodic register read commands");
      debugPrint("Will send register read commands every 200ms to get data");
      debugPrint("Commands: readReg(0x3A) for magnetic field, readReg(0x51) for quaternion");
      debugPrint("💡 Any buffered data should now be flushed and processed immediately");
      
      return true;
    } else {
      debugPrint("❌ Failed to subscribe to notifications");
      return false;
    }
  } catch (std::exception& e) {
    debugPrintf("❌ Exception during notification setup: %s", e.what());
    return false;
  }
}

// nimBLE notification callback
void onNotify(NimBLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("🔔 NIMBLE NOTIFICATION RECEIVED! %d bytes", length);
  
  // Always show raw packet data for analysis
  debugPrint("Raw packet data:");
  for(int i = 0; i < length && i < 32; i++) {
    Serial2.printf("0x%02X ", pData[i]);
    if((i + 1) % 8 == 0) Serial2.println();
  }
  if(length % 8 != 0) Serial2.println();  // Final newline if not divisible by 8
  
  // Look for common WT901 packet headers
  if (length >= 2) {
    if (pData[0] == 0x55) {
      debugPrintf("Found packet header 0x55, second byte: 0x%02X", pData[1]);
      
      // Common WT901 packet types (based on vendor Python code):
      if (pData[1] == 0x61) debugPrint("  -> Type: Combined Acceleration + Angular Velocity + Angle data");
      else if (pData[1] == 0x71) debugPrint("  -> Type: Register response (Magnetic field or Quaternion)");
      else if (pData[1] == 0x51) debugPrint("  -> Type: Acceleration data");
      else if (pData[1] == 0x52) debugPrint("  -> Type: Angular velocity data"); 
      else if (pData[1] == 0x53) debugPrint("  -> Type: Angle data");
      else if (pData[1] == 0x54) debugPrint("  -> Type: Magnetic field data");
      else if (pData[1] == 0x50) debugPrint("  -> Type: Time data");
      else debugPrintf("  -> Type: Unknown (0x%02X)", pData[1]);
    }
  }
  
  // Try to parse the packet
  if (parseFullIMUPacket(pData, length)) {
    packetCount++;
    lastPacketTime = millis();
    debugPrintf("✅ Successfully parsed IMU packet #%d", packetCount);
    printIMUData();
  } else {
    debugPrint("❌ Failed to parse IMU packet - analyzing structure:");
    
    // Detailed analysis for debugging
    if (length < 20) {
      debugPrintf("Packet too short (%d bytes) - need at least 20 bytes", length);
    } else if (pData[0] != 0x55) {
      debugPrintf("Wrong header byte 0x%02X - expected 0x55", pData[0]);
    } else if (pData[1] != 0x61) {
      debugPrintf("Wrong packet type 0x%02X - expected 0x61 for acceleration+angle data", pData[1]);
    } else {
      debugPrint("Header looks correct but parsing failed - check data format");
    }
  }
}

// Parse IMU packet from BLE data (unchanged - same logic)
bool parseFullIMUPacket(uint8_t* data, size_t length) {
  // Check for minimum packet size (header + type + data)
  if (length < 11) return false;
  
  // Look for packet header (0x55)
  if (data[0] != 0x55) return false;
  
  // Parse different packet types - length already checked >= 11 above
  switch (data[1]) {
    case 0x51: { // Acceleration data
      int16_t axRaw = (data[3] << 8) | data[2];
      int16_t ayRaw = (data[5] << 8) | data[4];  
      int16_t azRaw = (data[7] << 8) | data[6];
      
      imuData.ax = (float)axRaw / 32768.0 * 16.0;      // ±16g
      imuData.ay = (float)ayRaw / 32768.0 * 16.0;
      imuData.az = (float)azRaw / 32768.0 * 16.0;
      imuData.dataValid = true;
      return true;
    }
    
    case 0x52: { // Angular velocity data
      int16_t wxRaw = (data[3] << 8) | data[2];
      int16_t wyRaw = (data[5] << 8) | data[4];
      int16_t wzRaw = (data[7] << 8) | data[6];
      
      imuData.wx = (float)wxRaw / 32768.0 * 2000.0;    // ±2000°/s
      imuData.wy = (float)wyRaw / 32768.0 * 2000.0;
      imuData.wz = (float)wzRaw / 32768.0 * 2000.0;
      imuData.dataValid = true;
      return true;
    }
    
    case 0x53: { // Angle data
      int16_t rollRaw = (data[3] << 8) | data[2];
      int16_t pitchRaw = (data[5] << 8) | data[4]; 
      int16_t yawRaw = (data[7] << 8) | data[6];
      
      imuData.roll = (float)rollRaw / 32768.0 * 180.0;  // ±180°
      imuData.pitch = (float)pitchRaw / 32768.0 * 180.0;
      imuData.yaw = (float)yawRaw / 32768.0 * 180.0;
      imuData.dataValid = true;
      return true;
    }
    
    case 0x71: { // Register response data (magnetic field or quaternion) - VENDOR PROTOCOL
      if (length < 20) return false;
      
      uint8_t regAddr = data[2]; // Register address in byte 2
      debugPrintf("Register response for address 0x%02X", regAddr);
      
      if (regAddr == 0x3A) {
        // Magnetic field data response
        int16_t hxRaw = (data[5] << 8) | data[4];
        int16_t hyRaw = (data[7] << 8) | data[6];
        int16_t hzRaw = (data[9] << 8) | data[8];
        
        float hx = (float)hxRaw / 120.0;  // Based on vendor Python code
        float hy = (float)hyRaw / 120.0;
        float hz = (float)hzRaw / 120.0;
        
        debugPrintf("Magnetic field - X: %7.3f, Y: %7.3f, Z: %7.3f", hx, hy, hz);
        // Store in IMU data structure if needed
        imuData.dataValid = true;
        return true;
      }
      else if (regAddr == 0x51) {
        // Quaternion data response  
        int16_t q0Raw = (data[5] << 8) | data[4];
        int16_t q1Raw = (data[7] << 8) | data[6];
        int16_t q2Raw = (data[9] << 8) | data[8];
        int16_t q3Raw = (data[11] << 8) | data[10];
        
        float q0 = (float)q0Raw / 32768.0;  // Based on vendor Python code
        float q1 = (float)q1Raw / 32768.0;
        float q2 = (float)q2Raw / 32768.0;
        float q3 = (float)q3Raw / 32768.0;
        
        debugPrintf("Quaternion - Q0: %7.5f, Q1: %7.5f, Q2: %7.5f, Q3: %7.5f", q0, q1, q2, q3);
        // Store in IMU data structure if needed
        imuData.dataValid = true;
        return true;
      }
      else {
        debugPrintf("Unknown register response: 0x%02X", regAddr);
        return false;
      }
    }
    
    case 0x61: { // Combined acceleration + angle data (original format)
      if (length < 20) return false;
      
      // Extract acceleration data (bytes 2-7)
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
      imuData.ax = (float)axRaw / 32768.0 * 16.0;      // ±16g
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
    
    default:
      debugPrintf("Unknown packet type: 0x%02X", data[1]);
      return false;
  }
}

// Parse IMU packet from UART data (unchanged - same logic)  
bool parseFullIMUPacketUART() {
  static uint8_t buffer[32];
  static int bufferIndex = 0;
  
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
        // Extract acceleration data (bytes 2-7)
        int16_t axRaw = (buffer[3] << 8) | buffer[2];
        int16_t ayRaw = (buffer[5] << 8) | buffer[4];  
        int16_t azRaw = (buffer[7] << 8) | buffer[6];
        
        // Extract angular velocity data (bytes 8-13)
        int16_t wxRaw = (buffer[9] << 8) | buffer[8];
        int16_t wyRaw = (buffer[11] << 8) | buffer[10];
        int16_t wzRaw = (buffer[13] << 8) | buffer[12];
        
        // Extract angle data (bytes 14-19)
        int16_t rollRaw = (buffer[15] << 8) | buffer[14];
        int16_t pitchRaw = (buffer[17] << 8) | buffer[16]; 
        int16_t yawRaw = (buffer[19] << 8) | buffer[18];
        
        // Convert to physical units according to protocol
        imuData.ax = (float)axRaw / 32768.0 * 16.0;      // ±16g
        imuData.ay = (float)ayRaw / 32768.0 * 16.0;
        imuData.az = (float)azRaw / 32768.0 * 16.0;
        
        imuData.wx = (float)wxRaw / 32768.0 * 2000.0;    // ±2000°/s
        imuData.wy = (float)wyRaw / 32768.0 * 2000.0;
        imuData.wz = (float)wzRaw / 32768.0 * 2000.0;
        
        imuData.roll = (float)rollRaw / 32768.0 * 180.0;  // ±180°
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

void printIMUData() {
  debugPrintf("\n[PACKET #%d] ===================================", packetCount);
  
  // Show which data is available
  bool hasAccel = (imuData.ax != 0 || imuData.ay != 0 || imuData.az != 0);
  bool hasAngVel = (imuData.wx != 0 || imuData.wy != 0 || imuData.wz != 0);
  bool hasAngles = (imuData.roll != 0 || imuData.pitch != 0 || imuData.yaw != 0);
  
  debugPrint("Data Available: ");
  if (hasAccel) Serial2.print("✓ Acceleration ");
  if (hasAngVel) Serial2.print("✓ Angular Velocity ");  
  if (hasAngles) Serial2.print("✓ Orientation ");
  Serial2.println();
  
  if (hasAccel) {
    debugPrintf("Acceleration (g):     X: %7.3f  Y: %7.3f  Z: %7.3f", 
                  imuData.ax, imuData.ay, imuData.az);
  }
  
  if (hasAngVel) {
    debugPrintf("Angular Vel (°/s):    X: %7.2f  Y: %7.2f  Z: %7.2f", 
                  imuData.wx, imuData.wy, imuData.wz);
  }
  
  if (hasAngles) {
    debugPrintf("Orientation (°):   Roll: %7.2f  Pitch: %7.2f  Yaw: %7.2f", 
                  imuData.roll, imuData.pitch, imuData.yaw);
  }
  
  // Display timestamp if available
  if (imuData.timestamp.timeValid) {
    debugPrintf("Timestamp:         %02d/%02d/%02d %02d:%02d:%02d.%03d", 
                imuData.timestamp.year, imuData.timestamp.month, imuData.timestamp.day,
                imuData.timestamp.hour, imuData.timestamp.minute, imuData.timestamp.second,
                imuData.timestamp.millisecond);
  } else {
    debugPrint("Timestamp:         Not available");
  }
  
  // Show Stewart platform relevant info only if we have angle data
  if (hasAngles) {
    debugPrint("\n[STEWART PLATFORM MOTION]");
    debugPrintf("Primary Control Axes: Roll: %7.2f°, Pitch: %7.2f°", 
                  imuData.roll, imuData.pitch);
    
    // Calculate approximate actuator displacements (simplified)
    float armRadius = 200.0;
    float dz1 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(0)) + 
                             sin(radians(imuData.roll)) * sin(radians(0)));
    float dz2 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(120)) + 
                             sin(radians(imuData.roll)) * sin(radians(120)));  
    float dz3 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(240)) + 
                             sin(radians(imuData.roll)) * sin(radians(240)));
                             
    debugPrintf("Est. Actuator Δ (mm): A: %+6.1f  B: %+6.1f  C: %+6.1f", 
                  dz1, dz2, dz3);
  }
  
  debugPrint("=====================================================\n");
}

// =============================================================================
// VENDOR PROTOCOL IMPLEMENTATION - Based on Python sample code (unchanged)
// =============================================================================

// Create vendor protocol read command - matches Python get_readBytes()
std::vector<uint8_t> createReadCommand(uint8_t regAddr) {
  std::vector<uint8_t> cmd(5);
  cmd[0] = 0xFF;  // Header
  cmd[1] = 0xAA;  // Header  
  cmd[2] = 0x27;  // Read command
  cmd[3] = regAddr;
  cmd[4] = 0x00;
  return cmd;
}

// Send periodic data requests using vendor protocol
void sendDataRequest() {
  if (!bleConnected || pWriteCharacteristic == nullptr) return;
  
  static bool requestMagnetic = true; // Alternate between requests
  
  try {
    if (requestMagnetic) {
      // Request magnetic field data (register 0x3A)
      auto cmd = createReadCommand(0x3A);
      pWriteCharacteristic->writeValue(cmd.data(), cmd.size(), true);
      if (DEBUG_MODE) debugPrint("Sent magnetic field data request (0x3A)");
    } else {
      // Request quaternion data (register 0x51)  
      auto cmd = createReadCommand(0x51);
      pWriteCharacteristic->writeValue(cmd.data(), cmd.size(), true);
      if (DEBUG_MODE) debugPrint("Sent quaternion data request (0x51)");
    }
    
    requestMagnetic = !requestMagnetic; // Alternate requests
    
  } catch (std::exception& e) {
    debugPrintf("Exception sending data request: %s", e.what());
  }
} 