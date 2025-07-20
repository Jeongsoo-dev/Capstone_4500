#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEClient.h>
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include "pin_definitions.h"

// =============================================================================
// IMU DATA MONITOR - Dual Mode (Bluetooth 5.0 + UART)
// =============================================================================
// This utility helps verify that IMU data is being received correctly
// Supports both Bluetooth and UART communication modes
// Use Serial2 (UART2) for all debug output, keeps Serial available for IMU data

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
// If having BLE issues, try setting this to false first to test basic functionality
const bool USE_BLUETOOTH_MODE = true;

// DEBUG MODE - Set to true for additional debugging output
const bool DEBUG_MODE = true;

// BLE Configuration - Using the actual UUIDs found via LightBlue
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");        // Main service  
static BLEUUID NOTIFY_CHAR_UUID("0000ffe9-0000-1000-8000-00805f9a34fb");    // Notify-only characteristic
static BLEUUID WRITE_CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");     // Write-only characteristic

// BLE Variables
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pNotifyCharacteristic = nullptr;  // For receiving data
BLERemoteCharacteristic* pWriteCharacteristic = nullptr;   // For sending commands
bool bleConnected = false;
bool bleConnecting = false;
BLEAdvertisedDevice* targetDevice = nullptr;
bool deviceFound = false;
unsigned long connectionStartTime = 0;

IMUData imuData;
unsigned long lastPacketTime = 0;
int packetCount = 0;

// Function declarations
bool initializeBLE();
bool connectToIMU();
void scanForIMU();
bool parseFullIMUPacket(uint8_t* data, size_t length);
bool parseFullIMUPacketUART();  // For UART mode
void printIMUData();
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

// BLE Client Callbacks
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
  }
};

// BLE Advertisement Callback
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

void setup() {
  // Initialize debug interface first
  setupDebugUART2();
  delay(1000);
  
  if (USE_BLUETOOTH_MODE) {
    debugPrint("==== IMU Data Monitor (Bluetooth 5.0) Starting ====");
    debugPrint("Initializing Bluetooth Low Energy...");
    
    if (initializeBLE()) {
      debugPrint("BLE initialized successfully");
      scanForIMU();
    } else {
      debugPrint("BLE initialization failed!");
      return;
    }
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
    // Debug: Show current BLE state
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 3000) {
      debugPrintf("BLE Status - Connected: %s, Connecting: %s", 
                  bleConnected ? "YES" : "NO", 
                  bleConnecting ? "YES" : "NO");
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
    if (!bleConnected && !bleConnecting) {
      if (millis() - lastScanTime > 10000) { // Wait 10 seconds between scan attempts
        debugPrint("Attempting to connect to IMU...");
        scanForIMU();
        lastScanTime = millis();
      }
    }
    
    // Try to establish service connection after BLE connection
    if (bleConnected && pNotifyCharacteristic == nullptr) {
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
    
    // Show status if no packets received for a while
    if (bleConnected && pNotifyCharacteristic != nullptr && millis() - lastPacketTime > 5000 && packetCount == 0) {
      debugPrint("No IMU packets received yet. Check IMU configuration and UUIDs.");
      lastPacketTime = millis(); // Prevent spam
    }
  } else {
    // UART mode - check for incoming data
    if (parseFullIMUPacketUART()) {
      packetCount++;
      lastPacketTime = millis();
      debugPrintf("Successfully parsed IMU packet #%d", packetCount);
      printIMUData();
    }
    
    // Show status if no packets received for a while  
    if (millis() - lastPacketTime > 5000 && packetCount == 0) {
      debugPrint("No IMU packets received yet. Check connections and baud rate.");
      lastPacketTime = millis(); // Prevent spam
    }
  }
  
  delay(100);
}

bool initializeBLE() {
  BLEDevice::init("Stewart_Platform_Monitor");
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
  pNotifyCharacteristic = pRemoteService->getCharacteristic(NOTIFY_CHAR_UUID);
  debugPrint("*** getCharacteristic() call completed");
  if (pNotifyCharacteristic == nullptr) {
    debugPrint("Failed to find IMU notify characteristic");
    
    // Try to get all characteristics and show what's available
    debugPrint("Available characteristics:");
    std::map<std::string, BLERemoteCharacteristic*>* charMap = pRemoteService->getCharacteristics();
    for (auto& pair : *charMap) {
      debugPrintf("  - %s", pair.first.c_str());
    }
    return false;
  }
  
  debugPrint("*** Getting IMU write characteristic...");
  pWriteCharacteristic = pRemoteService->getCharacteristic(WRITE_CHAR_UUID);
  if (pWriteCharacteristic == nullptr) {
    debugPrint("Failed to find IMU write characteristic");
    return false;
  }
  
  debugPrint("Found both IMU characteristics, setting up notifications...");
  
  // Register for notifications on the notify-only characteristic
  try {
    pNotifyCharacteristic->registerForNotify(onNotify);
    debugPrint("BLE notifications registered successfully");
    
    // WT901 devices often don't need CCCD or streaming commands
    debugPrint("CCCD not needed for WT901 - checking if device streams automatically...");
    
    // Wait a moment to see if data comes automatically
    delay(2000);
    
    // If no data comes automatically, try different commands
    debugPrint("Testing different streaming commands...");
    
    // Try 1: No command (many WT901 devices stream automatically)
    debugPrint("Attempt 1: Waiting for automatic streaming (no command needed)...");
    delay(1000);
    
    // Try 2: Common WT901 command from documentation
    debugPrint("Attempt 2: Sending documented WT901 command...");
    uint8_t cmd1[] = {0xFF, 0xAA, 0x03, 0x08, 0x00, 0x0E};
    pWriteCharacteristic->writeValue(cmd1, sizeof(cmd1), true);
    delay(1000);
    
    // Try 3: Alternative command format
    debugPrint("Attempt 3: Sending alternative format...");
    uint8_t cmd2[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};  // Alternative start command
    pWriteCharacteristic->writeValue(cmd2, sizeof(cmd2), true);
    delay(1000);
    
    // Try 4: Simple wake up command
    debugPrint("Attempt 4: Sending wake-up command...");
    uint8_t cmd3[] = {0xFF, 0xAA, 0x01, 0x04, 0x00, 0x00};  // Wake up
    pWriteCharacteristic->writeValue(cmd3, sizeof(cmd3), true);
    delay(1000);
    
    // Try 5: Request data command
    debugPrint("Attempt 5: Sending data request command...");
    uint8_t cmd4[] = {0xFF, 0xAA, 0x01, 0x07, 0x00, 0x00};  // Request data
    pWriteCharacteristic->writeValue(cmd4, sizeof(cmd4), true);
    
    debugPrint("All streaming commands attempted. Monitor should receive data if IMU is working.");
    debugPrint("If no data appears, the issue may be with the IMU configuration or hardware.");
    
    return true;
  } catch (std::exception& e) {
    debugPrintf("Exception during notification setup: %s", e.what());
    return false;
  }
}

// BLE notification callback
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("🔔 NOTIFICATION RECEIVED! %d bytes", length);
  
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
      
      // Common WT901 packet types:
      if (pData[1] == 0x61) debugPrint("  -> Type: Acceleration + Angle data");
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

// Parse IMU packet from BLE data
bool parseFullIMUPacket(uint8_t* data, size_t length) {
  // Check for minimum packet size (header + type + data)
  if (length < 11) return false;
  
  // Look for packet header (0x55)
  if (data[0] != 0x55) return false;
  
  // Parse different packet types
  switch (data[1]) {
    case 0x51: { // Acceleration data
      if (length < 11) return false;
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
      if (length < 11) return false;
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
      if (length < 11) return false;
      int16_t rollRaw = (data[3] << 8) | data[2];
      int16_t pitchRaw = (data[5] << 8) | data[4]; 
      int16_t yawRaw = (data[7] << 8) | data[6];
      
      imuData.roll = (float)rollRaw / 32768.0 * 180.0;  // ±180°
      imuData.pitch = (float)pitchRaw / 32768.0 * 180.0;
      imuData.yaw = (float)yawRaw / 32768.0 * 180.0;
      imuData.dataValid = true;
      return true;
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

// Parse IMU packet from UART data  
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