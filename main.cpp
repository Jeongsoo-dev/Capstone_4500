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
  if (type == WStype_TEXT) {
    StaticJsonDocument<200> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      Serial.println("[!] Invalid JSON received.");
      return;
    }

    float pitch = doc["pitch"] | 0.0;
    float roll  = doc["roll"]  | 0.0;

    Serial.printf("[WS] pitch: %.2f°, roll: %.2f°\n", pitch, roll);

    targetLengthA = computeActuatorLength(pitch, roll, 0);
    targetLengthB = computeActuatorLength(pitch, roll, 120);
    targetLengthC = computeActuatorLength(pitch, roll, 240);
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
