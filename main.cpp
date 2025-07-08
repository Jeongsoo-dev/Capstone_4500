#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// Wi-Fi credentials
const char* ssid = "StewartPlatform";
const char* password = "esp32vrchair";

// Motor pins
#define PWM_A 18
#define DIR_A 5
#define PWM_B 19
#define DIR_B 17
#define PWM_C 21
#define DIR_C 16

// PWM channels
#define CH_A 0
#define CH_B 1
#define CH_C 2

// Actuator config
const int minLength = 550;
const int maxLength = 850;
const float actuatorSpeed = 84.0;  // mm/s
const float maxTiltDeg = 15.0;
const float armRadius = 200.0;     // mm from center to actuator

// Length tracking
float currentLengthA = 550;
float currentLengthB = 550;
float currentLengthC = 550;

// Web server
WebServer server(80);

// Function declarations
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg);
void moveActuator(float current, float target, int dirPin, int pwmChannel);
void handleMove();

void setup() {
  Serial.begin(115200);

  // Motor pin modes
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);

  // PWM setup
  ledcSetup(CH_A, 5000, 8);  // 5kHz, 8-bit
  ledcSetup(CH_B, 5000, 8);
  ledcSetup(CH_C, 5000, 8);
  ledcAttachPin(PWM_A, CH_A);
  ledcAttachPin(PWM_B, CH_B);
  ledcAttachPin(PWM_C, CH_C);

  // Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  // HTTP handler
  server.on("/move", HTTP_POST, handleMove);
  server.begin();
  Serial.println("HTTP server started.");
}

void loop() {
  server.handleClient();
}

// ------------------- HTTP POST Handler -------------------
void handleMove() {
  if (!server.hasArg("plain")) {
    server.send(400, "text/plain", "Missing body");
    return;
  }

  StaticJsonDocument<200> doc;
  DeserializationError error = deserializeJson(doc, server.arg("plain"));
  if (error) {
    server.send(400, "application/json", "{\"error\":\"Invalid JSON\"}");
    return;
  }

  float pitch = doc["pitch"];
  float roll  = doc["roll"];

  Serial.printf("Received pitch: %.2f°, roll: %.2f°\n", pitch, roll);

  float lA = computeActuatorLength(pitch, roll, 0);
  float lB = computeActuatorLength(pitch, roll, 120);
  float lC = computeActuatorLength(pitch, roll, 240);

  moveActuator(currentLengthA, lA, DIR_A, CH_A);
  moveActuator(currentLengthB, lB, DIR_B, CH_B);
  moveActuator(currentLengthC, lC, DIR_C, CH_C);

  currentLengthA = lA;
  currentLengthB = lB;
  currentLengthC = lC;

  server.send(200, "application/json", "{\"status\":\"moving\"}");
}

// ------------------- Kinematics -------------------
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad = radians(rollDeg);
  float angleRad = radians(angleDeg);

  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  float target = 550 + dz;

  return constrain(target, minLength, maxLength);
}

// ------------------- Motor Control -------------------
void moveActuator(float current, float target, int dirPin, int pwmChannel) {
  float delta = target - current;
  int dir = (delta >= 0) ? HIGH : LOW;
  float duration = abs(delta) / actuatorSpeed;  // seconds

  digitalWrite(dirPin, dir);
  ledcWrite(pwmChannel, 200); // moderate power

  delay((int)(duration * 1000)); // ms
  ledcWrite(pwmChannel, 0);
}
