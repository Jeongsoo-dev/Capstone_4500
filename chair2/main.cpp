/*
Enhanced Stewart Platform Chair2 - Real-time WebSocket Control
Provides smooth, high-frequency motion control with enhanced data processing

Required Arduino Libraries:
- WebSocketsServer by Markus Sattler (install via Library Manager)
- ArduinoJson by Benoit Blanchon

Features:
- WebSocket streaming for real-time communication
- Smooth actuator control with acceleration limiting
- 100Hz control loop for responsive motion
- Enhanced motion data format support
- Non-blocking motor control
*/

#include <WiFi.h>
#include <WebSocketsServer.h>
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

// Enhanced motion data structure
struct MotionData {
  uint32_t timestamp;
  float pitch;
  float roll;
  float yaw;
  float accelX, accelY, accelZ;
  float gyroX, gyroY, gyroZ;
  uint16_t flags;
};

// Actuator state for smooth control
struct ActuatorState {
  float currentLength;
  float targetLength;
  float velocity;
  unsigned long lastUpdate;
  bool moving;
};

ActuatorState actuators[3] = {
  {550, 550, 0, 0, false},  // A
  {550, 550, 0, 0, false},  // B
  {550, 550, 0, 0, false}   // C
};

// WebSocket server
WebSocketsServer webSocket = WebSocketsServer(81);

// Timing
unsigned long lastMotionUpdate = 0;
const unsigned long MOTION_UPDATE_INTERVAL = 10; // 100Hz control loop

// Function declarations
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg);
void updateActuatorSmooth(int actuatorIndex, int dirPin, int pwmChannel);
void processMotionData(MotionData& data);
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length);

void setup() {
  Serial.begin(115200);

  // Motor pin modes
  pinMode(DIR_A, OUTPUT);
  pinMode(DIR_B, OUTPUT);
  pinMode(DIR_C, OUTPUT);

  // PWM setup with higher frequency for smoother control
  ledcSetup(CH_A, 20000, 8);  // 20kHz, 8-bit
  ledcSetup(CH_B, 20000, 8);
  ledcSetup(CH_C, 20000, 8);
  ledcAttachPin(PWM_A, CH_A);
  ledcAttachPin(PWM_B, CH_B);
  ledcAttachPin(PWM_C, CH_C);

  // Initialize actuators to neutral position
  digitalWrite(DIR_A, LOW);
  digitalWrite(DIR_B, LOW);
  digitalWrite(DIR_C, LOW);
  ledcWrite(CH_A, 0);
  ledcWrite(CH_B, 0);
  ledcWrite(CH_C, 0);

  // Wi-Fi Access Point
  WiFi.softAP(ssid, password);
  Serial.print("WiFi AP started. IP: ");
  Serial.println(WiFi.softAPIP());

  // WebSocket setup
  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  
  Serial.println("WebSocket server started on port 81");
  Serial.println("System ready for real-time motion data");
}

void loop() {
  webSocket.loop();
  
  // High-frequency actuator control loop
  unsigned long currentTime = millis();
  if (currentTime - lastMotionUpdate >= MOTION_UPDATE_INTERVAL) {
    updateActuatorSmooth(0, DIR_A, CH_A);
    updateActuatorSmooth(1, DIR_B, CH_B);
    updateActuatorSmooth(2, DIR_C, CH_C);
    lastMotionUpdate = currentTime;
  }
}

// ------------------- WebSocket Event Handler -------------------
void webSocketEvent(uint8_t num, WStype_t type, uint8_t * payload, size_t length) {
  switch(type) {
    case WStype_DISCONNECTED:
      Serial.printf("Client %u disconnected\n", num);
      break;
      
    case WStype_CONNECTED: {
      IPAddress ip = webSocket.remoteIP(num);
      Serial.printf("Client %u connected from %d.%d.%d.%d\n", num, ip[0], ip[1], ip[2], ip[3]);
      break;
    }
    
    case WStype_TEXT: {
      // Handle JSON motion data
      StaticJsonDocument<512> doc;
      DeserializationError error = deserializeJson(doc, payload);
      
      if (error) {
        Serial.println("JSON parsing failed");
        return;
      }
      
      MotionData data;
      data.timestamp = doc["timestamp"] | millis();
      data.pitch = doc["motion"]["orientation"]["pitch"] | 0.0f;
      data.roll = doc["motion"]["orientation"]["roll"] | 0.0f;
      data.yaw = doc["motion"]["orientation"]["yaw"] | 0.0f;
      data.accelX = doc["motion"]["acceleration"]["x"] | 0.0f;
      data.accelY = doc["motion"]["acceleration"]["y"] | 0.0f;
      data.accelZ = doc["motion"]["acceleration"]["z"] | 9.81f;
      data.flags = doc["motion"]["motion_events"]["flags"] | 0;
      
      processMotionData(data);
      break;
    }
    
    case WStype_BIN: {
      // Handle binary motion data for maximum performance
      if (length >= sizeof(MotionData)) {
        MotionData* data = (MotionData*)payload;
        processMotionData(*data);
      }
      break;
    }
    
    default:
      break;
  }
}

// ------------------- Motion Processing -------------------
void processMotionData(MotionData& data) {
  // Constrain input angles
  float pitch = constrain(data.pitch, -maxTiltDeg, maxTiltDeg);
  float roll = constrain(data.roll, -maxTiltDeg, maxTiltDeg);
  
  // Calculate target actuator lengths
  float lA = computeActuatorLength(pitch, roll, 0);
  float lB = computeActuatorLength(pitch, roll, 120);
  float lC = computeActuatorLength(pitch, roll, 240);
  
  // Update target positions
  actuators[0].targetLength = lA;
  actuators[1].targetLength = lB;
  actuators[2].targetLength = lC;
  
  // Optional: Print debug info (remove for maximum performance)
  static unsigned long lastDebug = 0;
  if (millis() - lastDebug > 500) {  // Debug every 500ms
    Serial.printf("Motion: P=%.2f R=%.2f -> A=%.1f B=%.1f C=%.1f\n", 
                  pitch, roll, lA, lB, lC);
    lastDebug = millis();
  }
}

// ------------------- Enhanced Kinematics -------------------
float computeActuatorLength(float pitchDeg, float rollDeg, float angleDeg) {
  float pitchRad = radians(pitchDeg);
  float rollRad = radians(rollDeg);
  float angleRad = radians(angleDeg);

  // Calculate displacement for this actuator position
  float dz = armRadius * (sin(pitchRad) * cos(angleRad) + sin(rollRad) * sin(angleRad));
  
  // Base length + displacement
  float target = 550 + dz;

  return constrain(target, minLength, maxLength);
}

// ------------------- Smooth Actuator Control -------------------
void updateActuatorSmooth(int actuatorIndex, int dirPin, int pwmChannel) {
  ActuatorState& actuator = actuators[actuatorIndex];
  unsigned long currentTime = millis();
  float deltaTime = (currentTime - actuator.lastUpdate) / 1000.0; // seconds
  
  if (deltaTime < 0.001) return; // Skip if called too frequently
  
  float error = actuator.targetLength - actuator.currentLength;
  float absError = abs(error);
  
  // Deadband to prevent oscillation
  if (absError < 0.5) {
    if (actuator.moving) {
      ledcWrite(pwmChannel, 0);
      actuator.moving = false;
      actuator.velocity = 0;
    }
    actuator.lastUpdate = currentTime;
    return;
  }
  
  // Calculate desired velocity with acceleration limiting
  float maxVelocity = actuatorSpeed; // mm/s
  float desiredVelocity = (error > 0) ? maxVelocity : -maxVelocity;
  
  // Smooth acceleration/deceleration
  float maxAcceleration = 200.0; // mm/s²
  float maxVelChange = maxAcceleration * deltaTime;
  
  if (abs(desiredVelocity - actuator.velocity) > maxVelChange) {
    if (desiredVelocity > actuator.velocity) {
      actuator.velocity += maxVelChange;
    } else {
      actuator.velocity -= maxVelChange;
    }
  } else {
    actuator.velocity = desiredVelocity;
  }
  
  // Decelerate when approaching target
  float decelerationDistance = 20.0; // mm
  if (absError < decelerationDistance) {
    float scale = absError / decelerationDistance;
    actuator.velocity *= scale;
  }
  
  // Convert velocity to PWM (0-255)
  int pwmValue = map(abs(actuator.velocity), 0, maxVelocity, 0, 255);
  pwmValue = constrain(pwmValue, 0, 255);
  
  // Set direction and speed
  digitalWrite(dirPin, (actuator.velocity >= 0) ? HIGH : LOW);
  ledcWrite(pwmChannel, pwmValue);
  
  // Update position estimate
  actuator.currentLength += actuator.velocity * deltaTime;
  actuator.currentLength = constrain(actuator.currentLength, minLength, maxLength);
  
  actuator.moving = (pwmValue > 0);
  actuator.lastUpdate = currentTime;
}
