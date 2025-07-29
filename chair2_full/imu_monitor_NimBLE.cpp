#include <Arduino.h>
#include <NimBLEDevice.h>
#include "pin_definitions.h"

// =============================================================================
// IMU DATA MONITOR - NimBLE 2.3 Version
// =============================================================================
// This utility helps verify that IMU data is being received correctly
// Uses NimBLE 2.3 for Bluetooth Low Energy communication
// Uses Serial (UART0) for all debug output

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
// CONFIGURATION
// =============================================================================
// DEBUG MODE - Set to true for additional debugging output
const bool DEBUG_MODE = true;

// BLE Configuration - REPLACE WITH YOUR IMU'S ACTUAL UUIDs
static NimBLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9a34fb");    // Update for your device
static NimBLEUUID CHAR_UUID("0000ffe4-0000-1000-8000-00805f9a34fb");       // Update for your device

// BLE Variables
NimBLEClient* pClient = nullptr;
NimBLERemoteCharacteristic* pRemoteCharacteristic = nullptr;
bool bleConnected = false;
bool bleConnecting = false;
const NimBLEAdvertisedDevice* targetDevice = nullptr;
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
void printIMUData();
void onNotify(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify);

// NimBLE Client Callbacks
class MyClientCallback : public NimBLEClientCallbacks {
  void onConnect(NimBLEClient* pclient) override {
    debugPrint("NIMBLE CLIENT CALLBACK: Connected to IMU device");
    bleConnected = true;
    bleConnecting = false;
  }

  void onDisconnect(NimBLEClient* pclient, int reason) override {
    debugPrintf("NIMBLE CLIENT CALLBACK: Disconnected from IMU device (reason: %d)", reason);
    bleConnected = false;
    bleConnecting = false;
    pRemoteCharacteristic = nullptr;  // Reset characteristic pointer
  }
};

// NimBLE Scan Callbacks
class MyScanCallbacks: public NimBLEScanCallbacks {
  void onResult(const NimBLEAdvertisedDevice* advertisedDevice) override {
    debugPrintf("Found device: %s", advertisedDevice->toString().c_str());
    debugPrintf("  - Name: %s", advertisedDevice->getName().c_str());
    debugPrintf("  - Address: %s", advertisedDevice->getAddress().toString().c_str());
    debugPrintf("  - RSSI: %d", advertisedDevice->getRSSI());
    
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
      debugPrint("Found target IMU device! Stopping scan...");
      deviceFound = true;
      targetDevice = advertisedDevice;
      NimBLEDevice::getScan()->stop();
    } else {
      debugPrint("  - Does not match our target service UUID");
    }
    debugPrint("  ---");
  }

  void onScanEnd(const NimBLEScanResults& results, int reason) override {
    debugPrintf("Scan ended. Found %d devices, reason: %d", results.getCount(), reason);
  }
};

void setup() {
  // Initialize debug interface (Serial UART0)
  setupDebugUART0();
  delay(1000);
  
  debugPrint("==== IMU Data Monitor (NimBLE 2.3 Version) Starting ====");
  debugPrint("Initializing NimBLE...");
  
  if (initializeBLE()) {
    debugPrint("NimBLE initialized successfully");
    scanForIMU();
  } else {
    debugPrint("NimBLE initialization failed!");
    return;
  }
  
  debugPrint("---------------------------------------------------------------");
}

void loop() {
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
      NimBLEDevice::deleteClient(pClient);
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
  
  // Show status if no packets received for a while
  if (bleConnected && pRemoteCharacteristic != nullptr && millis() - lastPacketTime > 5000 && packetCount == 0) {
    debugPrint("No IMU packets received yet. Check IMU configuration and UUIDs.");
    lastPacketTime = millis(); // Prevent spam
  }
  
  delay(100);
}

bool initializeBLE() {
  NimBLEDevice::init("Stewart_Platform_Monitor");
  // Optional: Set transmit power
  NimBLEDevice::setPower(3); // 3dBm
  return true;
}

void scanForIMU() {
  if (bleConnecting) return;
  
  // Reset flags
  deviceFound = false;
  targetDevice = nullptr;
  
  bleConnecting = true;
  debugPrint("Scanning for IMU device...");
  debugPrintf("Looking for service UUID: %s", SERVICE_UUID.toString().c_str());
  debugPrint("Scan will run for 10 seconds...");
  
  NimBLEScan* pScan = NimBLEDevice::getScan();
  pScan->setScanCallbacks(new MyScanCallbacks(), false);
  pScan->setInterval(1349);
  pScan->setWindow(449);
  pScan->setActiveScan(true);
  
  // Start scan 
  NimBLEScanResults results = pScan->start(10, false); // Scan for 10 seconds
  
  // Scan completed
  debugPrintf("Scan completed. Found %d devices total", results.getCount());
  
  // Check if we found our target device
  if (deviceFound && targetDevice != nullptr) {
    debugPrint("Target device found, attempting connection...");
    
    // Clean up any existing client
    if (pClient != nullptr) {
      pClient->disconnect();
      NimBLEDevice::deleteClient(pClient);
    }
    
    pClient = NimBLEDevice::createClient();
    pClient->setClientCallbacks(new MyClientCallback());
    
    // Set connection parameters for better stability
    pClient->setConnectionParams(12, 12, 0, 150);
    pClient->setConnectTimeout(5000); // 5 second timeout
    
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
      NimBLEDevice::deleteClient(pClient);
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
  NimBLERemoteService* pRemoteService = pClient->getService(SERVICE_UUID);
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
    std::vector<NimBLERemoteCharacteristic*>* pChars = pRemoteService->getCharacteristics(true);
    for (auto& pChar : *pChars) {
      debugPrintf("  - %s", pChar->getUUID().toString().c_str());
    }
    return false;
  }
  
  debugPrint("Found IMU characteristic, setting up notifications...");
  
  // Use NimBLE's subscribe method instead of registerForNotify
  if (pRemoteCharacteristic->canNotify()) {
    if (pRemoteCharacteristic->subscribe(true, onNotify)) {
      debugPrint("NimBLE notifications subscribed successfully");
      return true;
    } else {
      debugPrint("Failed to subscribe to notifications");
      return false;
    }
  } else if (pRemoteCharacteristic->canIndicate()) {
    if (pRemoteCharacteristic->subscribe(false, onNotify)) {
      debugPrint("NimBLE indications subscribed successfully");
      return true;
    } else {
      debugPrint("Failed to subscribe to indications");
      return false;
    }
  } else {
    debugPrint("Characteristic does not support notifications or indications");
    return false;
  }
}

// NimBLE notification callback
void onNotify(NimBLERemoteCharacteristic* pRemoteCharacteristic, uint8_t* pData, size_t length, bool isNotify) {
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