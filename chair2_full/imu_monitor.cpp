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

// BLE Configuration - CORRECTED UUIDs based on protocol analysis
// TROUBLESHOOTING: If connection fails, try these alternative UUID sets:
// Option 1 (Your original code):
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");
static BLEUUID NOTIFY_CHAR_UUID("0000ffe9-0000-1000-8000-00805f9a34fb");
static BLEUUID WRITE_CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");
//
// Option 2 (From README):
// static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9b34fb");
// static BLEUUID NOTIFY_CHAR_UUID("0000ffe9-0000-1000-8000-00805f9b34fb");
// static BLEUUID WRITE_CHAR_UUID("0000ffe4-0000-1000-8000-00805f9b34fb");
//
// The code automatically tries these alternatives during connection

// BLE Variables
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pNotifyCharacteristic = nullptr;   // For receiving data
BLERemoteCharacteristic* pWriteCharacteristic = nullptr;    // For sending commands
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
    
    // Check if this device has our service UUID or is a likely IMU device
    bool isTargetDevice = false;
    
    if (advertisedDevice.haveServiceUUID() && advertisedDevice.isAdvertisingService(SERVICE_UUID)) {
      debugPrint("Found device with exact service UUID match!");
      isTargetDevice = true;
    } 
    // Also check for common IMU device names or alternative service UUIDs
    else if (advertisedDevice.haveName()) {
      String deviceName = String(advertisedDevice.getName().c_str());
      deviceName.toLowerCase();
      if (deviceName.indexOf("wt901") >= 0 || 
          deviceName.indexOf("wit") >= 0 || 
          deviceName.indexOf("imu") >= 0 ||
          deviceName.indexOf("mpu") >= 0 ||
          deviceName.indexOf("sensor") >= 0) {
        debugPrintf("Found potential IMU device by name: %s", deviceName.c_str());
        isTargetDevice = true;
      }
    }
    // Check alternative service UUIDs
    else if (advertisedDevice.haveServiceUUID()) {
      BLEUUID altService1("0000ffe5-0000-1000-8000-00805f9a34fb");
      BLEUUID altService2("0000ffe5-0000-1000-8000-00805f9b34fb");
      if (advertisedDevice.isAdvertisingService(altService1) || 
          advertisedDevice.isAdvertisingService(altService2)) {
        debugPrint("Found device with alternative service UUID!");
        isTargetDevice = true;
      }
    }
    
    if (isTargetDevice) {
      debugPrint("Target IMU device identified! Stopping scan...");
      deviceFound = true;
      targetDevice = new BLEAdvertisedDevice(advertisedDevice);
      BLEDevice::getScan()->stop();
    } else {
      debugPrint("  - Not identified as target IMU device");
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
      debugPrint("No IMU packets received yet. Trying direct read...");
      
      // Try reading directly from the characteristic
      if (pNotifyCharacteristic->canRead()) {
        debugPrint("Attempting direct characteristic read...");
        std::string value = pNotifyCharacteristic->readValue();
        if (value.length() > 0) {
          debugPrintf("Direct read got %d bytes:", value.length());
          for(int i = 0; i < value.length() && i < 32; i++) {
            Serial2.printf("0x%02X ", (uint8_t)value[i]);
            if((i + 1) % 8 == 0) Serial2.println();
          }
          Serial2.println();
        } else {
          debugPrint("Direct read returned no data");
        }
      } else {
        debugPrint("Characteristic is not readable");
      }
      
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
  debugPrint("*** Primary service getService() call completed");
  
  // If primary service not found, try alternative UUIDs
  if (pRemoteService == nullptr) {
    debugPrint("*** Primary service not found, trying alternatives...");
    BLEUUID altService1("0000ffe5-0000-1000-8000-00805f9a34fb");
    BLEUUID altService2("0000ffe5-0000-1000-8000-00805f9b34fb");
    
    pRemoteService = pClient->getService(altService1);
    if (pRemoteService == nullptr) {
      pRemoteService = pClient->getService(altService2);
    }
    
    if (pRemoteService != nullptr) {
      debugPrint("*** Found service using alternative UUID");
    }
  }
  
  if (pRemoteService == nullptr) {
    debugPrint("*** Failed to find any compatible IMU service");
    
    // List all available services for debugging
    debugPrint("*** Available services on device:");
    std::map<std::string, BLERemoteService*>* serviceMap = pClient->getServices();
    for (auto& pair : *serviceMap) {
      debugPrintf("    Service: %s", pair.first.c_str());
    }
    return false;
  }
  
  debugPrint("*** Getting notify characteristic...");
  pNotifyCharacteristic = pRemoteService->getCharacteristic(NOTIFY_CHAR_UUID);
  debugPrint("*** getNotifyCharacteristic() call completed");
  
  debugPrint("*** Getting write characteristic...");
  pWriteCharacteristic = pRemoteService->getCharacteristic(WRITE_CHAR_UUID);
  debugPrint("*** getWriteCharacteristic() call completed");
  
  // If characteristics not found, try alternatives
  if (pNotifyCharacteristic == nullptr || pWriteCharacteristic == nullptr) {
    debugPrint("*** Primary characteristics not found, trying alternatives...");
    
    // Alternative UUIDs for characteristics  
    BLEUUID altNotify1("0000ffe9-0000-1000-8000-00805f9a34fb");
    BLEUUID altNotify2("0000ffe9-0000-1000-8000-00805f9b34fb");
    BLEUUID altWrite1("0000ffe4-0000-1000-8000-00805f9a34fb");
    BLEUUID altWrite2("0000ffe4-0000-1000-8000-00805f9b34fb");
    
    if (pNotifyCharacteristic == nullptr) {
      pNotifyCharacteristic = pRemoteService->getCharacteristic(altNotify1);
      if (pNotifyCharacteristic == nullptr) {
        pNotifyCharacteristic = pRemoteService->getCharacteristic(altNotify2);
      }
    }
    
    if (pWriteCharacteristic == nullptr) {
      pWriteCharacteristic = pRemoteService->getCharacteristic(altWrite1);
      if (pWriteCharacteristic == nullptr) {
        pWriteCharacteristic = pRemoteService->getCharacteristic(altWrite2);
      }
    }
    
    // For some IMU devices, the same characteristic handles both read/write
    if (pNotifyCharacteristic != nullptr && pWriteCharacteristic == nullptr) {
      debugPrint("*** Using notify characteristic for both read and write");
      pWriteCharacteristic = pNotifyCharacteristic;
    }
  }
  
  if (pNotifyCharacteristic == nullptr || pWriteCharacteristic == nullptr) {
    debugPrintf("Failed to find characteristics - Notify: %s, Write: %s", 
                pNotifyCharacteristic ? "OK" : "MISSING",
                pWriteCharacteristic ? "OK" : "MISSING");
    
    // Try to get all characteristics and show what's available
    debugPrint("Available characteristics:");
    std::map<std::string, BLERemoteCharacteristic*>* charMap = pRemoteService->getCharacteristics();
    for (auto& pair : *charMap) {
      BLERemoteCharacteristic* pChar = pair.second;
      debugPrintf("  - %s (Properties: %s%s%s%s)", 
                  pair.first.c_str(),
                  pChar->canRead() ? "R" : "-",
                  pChar->canWrite() ? "W" : "-",
                  pChar->canNotify() ? "N" : "-",
                  pChar->canIndicate() ? "I" : "-");
    }
    return false;
  }
  
  debugPrint("Found both characteristics, setting up notifications...");
  
  // Try to register for notifications without checking descriptors first
  try {
    pNotifyCharacteristic->registerForNotify(onNotify);
    debugPrint("BLE notifications registered successfully");
    
    // Try to subscribe to notifications by writing to CCCD
    uint8_t notificationOn[] = {0x1, 0x0};
    if (pNotifyCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902)) != nullptr) {
      pNotifyCharacteristic->getDescriptor(BLEUUID((uint16_t)0x2902))->writeValue((uint8_t*)notificationOn, 2, true);
      debugPrint("CCCD descriptor written");
    } else {
      debugPrint("No CCCD descriptor found, but notification callback registered");
    }
    
    // Try to start data transmission using protocol-compliant commands
    delay(100);
    debugPrint("Sending IMU initialization commands through WRITE characteristic...");
    
    // Protocol-compliant WT901 commands based on documentation
    uint8_t unlockCmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5};   // Unlock register access
    uint8_t setRate[] = {0xFF, 0xAA, 0x03, 0x0A, 0x00};     // Set output rate (10Hz)
    uint8_t setBaud[] = {0xFF, 0xAA, 0x04, 0x06, 0x00};     // Set baud rate (115200)
    uint8_t saveConfig[] = {0xFF, 0xAA, 0x00, 0x00, 0x00};  // Save configuration
    uint8_t enableOutput[] = {0xFF, 0xAA, 0x02, 0x08, 0x00}; // Enable data output
    
    // Send commands with delays for processing
    pWriteCharacteristic->writeValue(unlockCmd, 5, true);
    delay(100);
    pWriteCharacteristic->writeValue(setRate, 5, true);
    delay(100);  
    pWriteCharacteristic->writeValue(setBaud, 5, true);
    delay(100);
    pWriteCharacteristic->writeValue(saveConfig, 5, true);
    delay(100);
    pWriteCharacteristic->writeValue(enableOutput, 5, true);
    delay(100);
    
    debugPrint("Protocol-compliant IMU initialization commands sent");
    debugPrint("Commands: Unlock -> Set Rate (10Hz) -> Set Baud -> Save -> Enable Output");
    
    return true;
  } catch (std::exception& e) {
    debugPrintf("Exception during notification setup: %s", e.what());
    return false;
  }
}

// BLE notification callback
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("*** NOTIFICATION RECEIVED: %d bytes", length);
  
  // Always print raw data first to see what we're getting
  debugPrint("Raw notification data:");
  for(int i = 0; i < length && i < 32; i++) {
    Serial2.printf("0x%02X ", pData[i]);
    if((i + 1) % 8 == 0) Serial2.println();
  }
  if(length > 0) Serial2.println();
  
  if (parseFullIMUPacket(pData, length)) {
    packetCount++;
    lastPacketTime = millis();
    debugPrintf("Successfully parsed IMU packet #%d", packetCount);
    printIMUData();
  } else {
    debugPrint("Data doesn't match expected IMU format (0x55 0x61)");
  }
}

// Parse IMU packet from BLE data
bool parseFullIMUPacket(uint8_t* data, size_t length) {
  // Check for minimum packet size (header + flag + 18 data bytes)
  if (length < 20) return false;
  
  // Look for packet header (0x55 0x61)
  if (data[0] == 0x55 && data[1] == 0x61) {
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
  
  return false;
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
  debugPrintf("Acceleration (g):     X: %7.3f  Y: %7.3f  Z: %7.3f", 
                imuData.ax, imuData.ay, imuData.az);
  debugPrintf("Angular Vel (°/s):    X: %7.2f  Y: %7.2f  Z: %7.2f", 
                imuData.wx, imuData.wy, imuData.wz);
  debugPrintf("Orientation (°):   Roll: %7.2f  Pitch: %7.2f  Yaw: %7.2f", 
                imuData.roll, imuData.pitch, imuData.yaw);
  
  // Display timestamp if available
  if (imuData.timestamp.timeValid) {
    debugPrintf("Timestamp:         %02d/%02d/%02d %02d:%02d:%02d.%03d", 
                imuData.timestamp.year, imuData.timestamp.month, imuData.timestamp.day,
                imuData.timestamp.hour, imuData.timestamp.minute, imuData.timestamp.second,
                imuData.timestamp.millisecond);
  } else {
    debugPrint("Timestamp:         Not available (20-byte packet)");
  }
  
  // Show Stewart platform relevant info
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
  debugPrint("=====================================================\n");
} 