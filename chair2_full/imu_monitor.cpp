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

// BLE Configuration - REPLACE WITH YOUR IMU'S ACTUAL UUIDs
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");    // Update for your device
static BLEUUID CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");       // Update for your device

// BLE Variables
BLEClient* pClient = nullptr;
BLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
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
    // Debug: Show current BLE state with more detail
    static unsigned long lastStatusTime = 0;
    if (millis() - lastStatusTime > 3000) {
      debugPrintf("LOOP-STATUS: Connected: %s, Connecting: %s, pClient: %s, pChar: %s", 
                  bleConnected ? "YES" : "NO", 
                  bleConnecting ? "YES" : "NO",
                  (pClient != nullptr) ? "OK" : "NULL",
                  (pRemoteCharacteristic != nullptr) ? "OK" : "NULL");
      lastStatusTime = millis();
    }
    
    // Handle connection timeout
    if (bleConnecting && !bleConnected && millis() - connectionStartTime > 8000) {
      debugPrint("Connection timeout (8 seconds) - resetting connection state");
      bleConnecting = false;
      if (pClient != nullptr) {
        // Client might be in a bad state, best to delete and recreate
        pClient->disconnect();
        delete pClient;
        pClient = nullptr;
      }
    }
    
    // Handle BLE connection state with retry delay
    static unsigned long lastScanTime = 0;
    if (!bleConnected && !bleConnecting) {
      if (millis() - lastScanTime > 10000) { // Wait 10 seconds between scan attempts
        debugPrint("LOOP: Not connected. Trying to scan and connect.");
        scanForIMU();
        lastScanTime = millis();
      }
    }
    
    // Try to establish service connection after BLE connection
    if (bleConnected && pRemoteCharacteristic == nullptr) {
      debugPrint("LOOP: BLE connected, characteristic is NULL. Trying to get service...");
      if (connectToIMU()) {
        debugPrint("LOOP: Service connection SUCCEEDED.");
        lastPacketTime = millis(); // Reset packet timer
      } else {
        debugPrint("LOOP: Service connection FAILED. Disconnecting to retry.");
        // If service connection fails, disconnect and retry
        if (pClient != nullptr) {
          pClient->disconnect(); // Callback will set bleConnected to false
        }
      }
    } 
    // This block represents the "fully connected and subscribed" state
    else if (bleConnected && pRemoteCharacteristic != nullptr) {
      if (millis() - lastPacketTime > 5000) {
        if (packetCount == 0) {
          debugPrint("LOOP: Connected, but no packets received yet. Check IMU/UUIDs.");
        } else {
          debugPrintf("LOOP: Connected, last packet was >5s ago. Total packets: %d", packetCount);
        }
        lastPacketTime = millis(); // Prevent spam
      }
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
  if (!bleConnected || pClient == nullptr || !pClient->isConnected()) {
    debugPrintf("*** Early return: bleConnected=%s, pClient=%s, pClient->isConnected=%s", 
                bleConnected ? "true" : "false", 
                pClient ? "valid" : "null",
                (pClient && pClient->isConnected()) ? "true" : "false");
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
    
    // Try to send a command to start data transmission
    // debugPrint("Sending start command to IMU...");
    // uint8_t startCmd[] = {0xFF, 0xAA, 0x69, 0x88, 0xB5}; // Common WT901 start command
    // pRemoteCharacteristic->writeValue(startCmd, sizeof(startCmd));
    
    delay(100); // Give IMU time to process command
    
    return true;
  } catch (std::exception& e) {
    debugPrintf("Exception during notification setup: %s", e.what());
    return false;
  }
}

// BLE notification callback
void onNotify(BLERemoteCharacteristic* pBLERemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
  debugPrintf("🔔 NOTIFICATION RECEIVED! %d bytes", length);
  
  if (parseFullIMUPacket(pData, length)) {
    packetCount++;
    lastPacketTime = millis();
    debugPrintf("Successfully parsed IMU packet #%d", packetCount);
    printIMUData();
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