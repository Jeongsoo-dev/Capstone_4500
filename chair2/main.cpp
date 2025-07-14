#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <WebSocketsServer.h>

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
float currentLengthA = 550;
float currentLengthB = 550;
float currentLengthC = 550;
float targetLengthA = 550;
float targetLengthB = 550;
float targetLengthC = 550;

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

// ===== Setup =====
void setup() {
  Serial.begin(115200);
  delay(1000);
  Serial.println("\n==== ESP32 Stewart Platform Booting ====");

  // Motor direction pins
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);
  Serial.println("[✓] Motor DIR pins set");

  // PWM setup
  ledcSetup(CH_A, 20000, 8);
  ledcSetup(CH_B, 20000, 8);
  ledcSetup(CH_C, 20000, 8);
  ledcAttachPin(PWM_A, CH_A);
  ledcAttachPin(PWM_B, CH_B);
  ledcAttachPin(PWM_C, CH_C);
  Serial.println("[✓] PWM configured (20kHz)");

  // Start Wi-Fi Access Point
  if (WiFi.softAP(ssid, password)) {
    Serial.println("[✓] WiFi AP started");
    Serial.print("    IP: ");
    Serial.println(WiFi.softAPIP());
  } else {
    Serial.println("[!] WiFi AP failed to start");
  }

  // Start WebSocket server
  webSocket.begin();
  webSocket.onEvent(onWebSocketEvent);
  Serial.println("[✓] WebSocket server running on port 81");
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
      Serial.printf("[WS] Client #%u disconnected\n", client);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(client);
      Serial.printf("[WS] Client #%u connected from %d.%d.%d.%d\n", 
                    client, ip[0], ip[1], ip[2], ip[3]);
      
      // Send welcome message
      webSocket.sendTXT(client, "{\"status\":\"connected\",\"message\":\"Stewart Platform Ready\"}");
      break;
    }
    
    case WStype_TEXT:
      // Validate message size
      if (length > 1024) {  // Reasonable limit for JSON payload
        Serial.printf("[WS] Message too large (%u bytes), ignoring\n", length);
        webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Message too large\"}");
        return;
      }
      
      Serial.printf("[WS] Received from client #%u: %s\n", client, payload);
      
      {
        StaticJsonDocument<512> doc;  // Increased size for larger JSON payload
        DeserializationError error = deserializeJson(doc, payload);
        if (error) {
          Serial.printf("[WS] JSON parse error: %s\n", error.c_str());
          webSocket.sendTXT(client, "{\"status\":\"error\",\"message\":\"Invalid JSON\"}");
          return;
        }

        // Extract pitch and roll from nested structure
        float pitch = doc["motion"]["orientation"]["pitch"] | 0.0;
        float roll  = doc["motion"]["orientation"]["roll"]  | 0.0;
        float yaw   = doc["motion"]["orientation"]["yaw"]   | 0.0;
        
        // Extract timestamp for reference
        unsigned long timestamp = doc["timestamp"] | 0;
        
        // Extract control mode
        const char* mode = doc["control"]["mode"] | "realtime";
        const char* speed = doc["control"]["response_speed"] | "fast";

        Serial.printf("[WS] timestamp: %lu, pitch: %.2f°, roll: %.2f°, yaw: %.2f°\n", 
                      timestamp, pitch, roll, yaw);
        Serial.printf("[WS] mode: %s, speed: %s\n", mode, speed);

        // Use pitch and roll for actuator control (keeping existing logic)
        targetLengthA = computeActuatorLength(pitch, roll, 0);
        targetLengthB = computeActuatorLength(pitch, roll, 120);
        targetLengthC = computeActuatorLength(pitch, roll, 240);
        
        // Send acknowledgment
        webSocket.sendTXT(client, "{\"status\":\"ok\",\"message\":\"Motion data received\"}");
      }
      break;
      
    case WStype_BIN:
      Serial.printf("[WS] Binary data received (%u bytes), ignoring\n", length);
      break;
      
    case WStype_ERROR:
      Serial.printf("[WS] Error occurred with client #%u\n", client);
      break;
      
    case WStype_FRAGMENT_TEXT_START:
    case WStype_FRAGMENT_BIN_START:
    case WStype_FRAGMENT:
    case WStype_FRAGMENT_FIN:
      Serial.printf("[WS] Fragmented message received, ignoring\n");
      break;
      
    default:
      Serial.printf("[WS] Unknown WebSocket event type: %d\n", type);
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
