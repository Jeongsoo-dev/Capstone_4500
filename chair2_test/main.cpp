#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>
#include "pin_definitions.h"

// ===== Wi-Fi credentials =====
const char* ssid = "StewartPlatform";
const char* password = "esp32vrchair";

// ===== Motor pins =====
#define PWM_A 18
#define DIR_A 5
#define PWM_B 19
#define DIR_B 17
#define PWM_C 21
#define DIR_C 16

// ===== PWM channels =====
#define CH_A 0
#define CH_B 1
#define CH_C 2

// ===== Actuator configuration =====
const int minLength = 550;
const int maxLength = 850;
const float actuatorSpeed = 84.0;   // mm/s
const float armRadius = 200.0;      // mm from center to actuator

// ===== Length tracking =====
// Initialize to neutral state: Motor A(l1)=735mm, Motor B(l2)=735mm, Motor C(l3)=670mm
float currentLengthA = 735;  // Front Left = l1 = 735mm
float currentLengthB = 735;  // Front Right = l2 = 735mm  
float currentLengthC = 670;  // Rear = l3 = 670mm
float targetLengthA = 735;
float targetLengthB = 735;
float targetLengthC = 670;

// ===== Timing =====
unsigned long lastUpdate = 0;
const int updateInterval = 10; // ms

// ===== WebSocket Server =====
WebSocketsServer webSocket = WebSocketsServer(81);

// ===== Function declarations =====
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length);
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg);
void updateActuatorsSmoothly();
float stepTowards(float current, float target, float maxStep);
void driveMotor(float length, int dirPin, int pwmChannel);
void resetActuator(int dirPin, int pwmChannel);

// ===== Setup =====
void setup() {
  setupDebugUART0();
  delay(1000);
  debugPrint("\n==== ESP32 Stewart Platform Booting ====");

  // Motor direction pins
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  debugPrint("[✓] Motor DIR pins set");

  // PWM setup
  ledcSetup(CH_A, 20000, 8);
  ledcSetup(CH_B, 20000, 8);
  ledcSetup(CH_C, 20000, 8);
  ledcAttachPin(PWM_A, CH_A);
  ledcAttachPin(PWM_B, CH_B);
  ledcAttachPin(PWM_C, CH_C);
  debugPrint("[✓] PWM configured (20kHz)");

  // Start Wi-Fi Access Point
  if (WiFi.softAP(ssid, password)) {
    debugPrint("[✓] WiFi AP started");
    debugPrint("    IP: " + WiFi.softAPIP().toString());
  } else {
    debugPrint("[!] WiFi AP failed to start");
  }

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  debugPrint("[✓] WebSocket server running on port 81");
  
  // Move actuators to neutral state during initialization
  debugPrint("[*] Moving to neutral state...");
  
  // First move all actuators to minimum position for reference
  resetActuator(DIR_A, CH_A);
  resetActuator(DIR_B, CH_B);  
  resetActuator(DIR_C, CH_C);
  delay(4000); // Allow time to reach minimum
  
  // Then move to neutral positions
  debugPrint("Moving to neutral: A=735mm, B=735mm, C=670mm");
  driveMotor(735, DIR_A, CH_A);  // Motor A to 735mm
  driveMotor(735, DIR_B, CH_B);  // Motor B to 735mm
  driveMotor(670, DIR_C, CH_C);  // Motor C to 670mm
  
  // Wait for actuators to reach neutral positions
  delay(3000);
  debugPrint("[✓] All actuators at neutral state");
}

// ===== Main Loop =====
void loop() {
  webSocket.loop();

  unsigned long now = millis();
  if (now - lastUpdate >= updateInterval) {
    updateActuatorsSmoothly();
    lastUpdate = now;
  }
}

// ===== WebSocket Event Handler =====
void onWebSocketEvent(uint8_t client, WStype_t type, uint8_t *payload, size_t length) {
  switch (type) {
    case WStype_DISCONNECTED:
      debugPrintf("[WS] Client #%u disconnected", client);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(client);
      debugPrintf("[WS] Client #%u connected from %d.%d.%d.%d", 
                    client, ip[0], ip[1], ip[2], ip[3]);
      
      // Send welcome message
      webSocket.sendTXT(client, "{\"status\":\"connected\",\"message\":\"Stewart Platform Ready\"}");
      break;
    }
    
    case WStype_TEXT:
      // Validate message size
      if (length > 1024) {  // Reasonable limit for JSON payload
        debugPrintf("[WS] Message too large (%u bytes), ignoring", length);
        webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Message too large\"}");
        return;
      }
      
      debugPrintf("[WS] Received from client #%u: %s", client, payload);
      
      {
        StaticJsonDocument<512> doc;  // Increased size for larger JSON payload
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          debugPrintf("[WS] JSON parse error: %s", error.c_str());
          webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
          return;
        }

        // Extract pitch and roll from nested structure
        float pitch = doc["motion"]["orientation"]["pitch"] | 0.0;
        float roll  = doc["motion"]["orientation"]["roll"]  | 0.0;
        float yaw   = doc["motion"]["orientation"]["ayaw"]   | 0.0;
        
        // Extract timestamp for reference
        unsigned long timestamp = doc["timestamp"] | 0;
        
        // Extract control mode
        const char* mode = doc["control"]["mode"] | "realtime";
        const char* speed = doc["control"]["response_speed"] | "fast";

        debugPrintf("[WS] timestamp: %lu, pitch: %.2f°, roll: %.2f°, yaw: %.2f°", 
                      timestamp, pitch, roll, yaw);
        debugPrintf("[WS] mode: %s, speed: %s", mode, speed);

        // Use pitch and roll for actuator control (keeping existing logic)
        targetLengthA = computeActuatorLength(pitch, roll, 0);
        targetLengthB = computeActuatorLength(pitch, roll, 120);
        targetLengthC = computeActuatorLength(pitch, roll, 240);
        
        // Send acknowledgment
        webSocket.sendTXT(client, "{\"status\":\"ok\",\"message\":\"Motion data received\"}");
      }
      break;
      
    case WStype_BIN:
      debugPrintf("[WS] Binary data received (%u bytes), ignoring", length);
      break;
      
    case WStype_ERROR:
      debugPrintf("[WS] Error occurred with client #%u", client);
      break;
      
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      debugPrint("[WS] Fragmented message received, ignoring");
      break;
      
    default:
      debugPrintf("[WS] Unknown WebSocket event type: %d", type);
      break;
  }
}

// ===== Actuator Kinematics =====
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad  = radians(rollDeg);
  float angleRad = radians(angleDeg);

  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  float target = 550 + dz;

  return constrain(target, minLength, maxLength);
}

// ===== Motion Interpolation =====
void updateActuatorsSmoothly() {
  float maxStep = actuatorSpeed * updateInterval / 1000.0;

  currentLengthA = stepTowards(currentLengthA, targetLengthA, maxStep);
  currentLengthB = stepTowards(currentLengthB, targetLengthB, maxStep);
  currentLengthC = stepTowards(currentLengthC, targetLengthC, maxStep);

  driveMotor(currentLengthA, DIR_A, CH_A);
  driveMotor(currentLengthB, DIR_B, CH_B);
  driveMotor(currentLengthC, DIR_C, CH_C);
}

float stepTowards(float current, float target, float maxStep) {
  float delta = target - current;
  if (abs(delta) <= maxStep) return target;
  return current + (delta > 0 ? maxStep : -maxStep);
}

// ===== Motor Drive Logic =====
void driveMotor(float length, int dirPin, int pwmChannel) {
  static float lastLengthA = 550;
  static float lastLengthB = 550;
  static float lastLengthC = 550;

  float *last = nullptr;
  if (dirPin == DIR_A) last = &lastLengthA;
  else if (dirPin == DIR_B) last = &lastLengthB;
  else if (dirPin == DIR_C) last = &lastLengthC;

  float delta = length - *last;
  int dir = (delta >= 0) ? HIGH : LOW;
  digitalWrite(dirPin, dir);
  ledcWrite(pwmChannel, abs(delta) > 0.5 ? 200 : 0);
  *last = length;
}

void resetActuator(int dirPin, int pwmChannel) {
  digitalWrite(dirPin, LOW);
  ledcWrite(pwmChannel, 200);
  delay(5000);
  ledcWrite(pwmChannel, 0);
}
