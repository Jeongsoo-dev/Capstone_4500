#include <Arduino.h>

// =============================================================================
// IMU DATA MONITOR - For Testing Communication
// =============================================================================
// This utility helps verify that IMU data is being received correctly
// Use this to debug before running the full Stewart platform control

struct IMUData {
  float roll = 0.0;
  float pitch = 0.0; 
  float yaw = 0.0;
  float ax = 0.0, ay = 0.0, az = 0.0;       // Acceleration (g)
  float wx = 0.0, wy = 0.0, wz = 0.0;       // Angular velocity (°/s)
  bool dataValid = false;
};

IMUData imuData;
unsigned long lastPacketTime = 0;
int packetCount = 0;

bool parseFullIMUPacket();
void printIMUData();

void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==== IMU Data Monitor Starting ====");
  Serial.println("[INFO] Monitoring for IMU packets (0x55 0x61)");
  Serial.println("[INFO] Expected format: Header(0x55) + Flag(0x61) + 18 data bytes");
  Serial.println("---------------------------------------------------------------");
}

void loop() {
  if (parseFullIMUPacket()) {
    packetCount++;
    lastPacketTime = millis();
    printIMUData();
  }
  
  // Show status if no packets received for a while
  if (millis() - lastPacketTime > 5000 && packetCount == 0) {
    Serial.println("[WARNING] No IMU packets received yet. Check connections and baud rate.");
    lastPacketTime = millis(); // Prevent spam
  }
}

bool parseFullIMUPacket() {
  static uint8_t buffer[32];
  static int bufferIndex = 0;
  
  while (Serial.available()) {
    uint8_t byte = Serial.read();
    
    // Debug: Print raw bytes (uncomment for debugging)
    // Serial.printf("Raw byte: 0x%02X\n", byte);
    
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

void printIMUData() {
  Serial.printf("\n[PACKET #%d] ===================================\n", packetCount);
  Serial.printf("Acceleration (g):     X: %7.3f  Y: %7.3f  Z: %7.3f\n", 
                imuData.ax, imuData.ay, imuData.az);
  Serial.printf("Angular Vel (°/s):    X: %7.2f  Y: %7.2f  Z: %7.2f\n", 
                imuData.wx, imuData.wy, imuData.wz);
  Serial.printf("Orientation (°):   Roll: %7.2f  Pitch: %7.2f  Yaw: %7.2f\n", 
                imuData.roll, imuData.pitch, imuData.yaw);
  
  // Show Stewart platform relevant info
  Serial.printf("\n[STEWART PLATFORM MOTION]\n");
  Serial.printf("Primary Control Axes: Roll: %7.2f°, Pitch: %7.2f°\n", 
                imuData.roll, imuData.pitch);
  
  // Calculate approximate actuator displacements (simplified)
  float armRadius = 200.0;
  float dz1 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(0)) + 
                           sin(radians(imuData.roll)) * sin(radians(0)));
  float dz2 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(120)) + 
                           sin(radians(imuData.roll)) * sin(radians(120)));  
  float dz3 = armRadius * (sin(radians(imuData.pitch)) * cos(radians(240)) + 
                           sin(radians(imuData.roll)) * sin(radians(240)));
                           
  Serial.printf("Est. Actuator Δ (mm): A: %+6.1f  B: %+6.1f  C: %+6.1f\n", 
                dz1, dz2, dz3);
  Serial.println("=====================================================\n");
} 