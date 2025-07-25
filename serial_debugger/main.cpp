/*
 * ESP32 USB-to-UART Serial Debugger
 * 
 * This program turns an ESP32 into a USB-to-UART bridge for debugging another ESP32
 * 
 * Connections:
 * Debugger ESP32 → Target ESP32
 * GPIO17 (TX2)   → RX pin (GPIO3 or any available RX pin)
 * GPIO16 (RX2)   → TX pin (GPIO1 or any available TX pin) 
 * GND            → GND
 * 
 * Usage:
 * 1. Upload this code to the debugger ESP32
 * 2. Connect the debugger ESP32 to your computer via USB
 * 3. Connect the UART pins between debugger and target ESP32
 * 4. Open Serial Monitor at 115200 baud
 * 5. You can now communicate with the target ESP32 through the debugger
 */

#include <Arduino.h>

#define LED_PIN 2          // Built-in LED for status indication
#define BAUD_RATE 115200   // Baud rate for both USB and UART communication

// UART2 pins (you can change these if needed)
#define UART_TX_PIN 17     // TX pin to target ESP32
#define UART_RX_PIN 16     // RX pin from target ESP32

unsigned long lastActivity = 0;
bool ledState = false;

void setup() {
  // Initialize built-in LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  
  // Initialize USB Serial (connection to computer)
  Serial.begin(BAUD_RATE);
  while (!Serial) {
    delay(10); // Wait for serial connection
  }
  
  Serial.println("\n=== ESP32 Serial Debugger Started ===");
  Serial.println("Debugger is connected to the computer");

  // Initialize UART2 (connection to target ESP32)
  Serial2.begin(BAUD_RATE, SERIAL_8N1, UART_RX_PIN, UART_TX_PIN);
  
  // Startup message
  Serial.println("Debugger ESP32 → Target ESP32 Bridge Active");
  Serial.printf("USB Serial: %d baud\n", BAUD_RATE);
  Serial.printf("UART2: TX=GPIO%d, RX=GPIO%d, %d baud\n", UART_TX_PIN, UART_RX_PIN, BAUD_RATE);
  Serial.println("Connections:");
  Serial.printf("  GPIO%d (TX2) → Target RX\n", UART_TX_PIN);
  Serial.printf("  GPIO%d (RX2) → Target TX\n", UART_RX_PIN);
  Serial.println("  GND → GND");
  Serial.println("========================================\n");
  
  // Blink LED to indicate ready
  for (int i = 0; i < 6; i++) {
    digitalWrite(LED_PIN, HIGH);
    delay(100);
    digitalWrite(LED_PIN, LOW);
    delay(100);
  }
}

void loop() {
  bool activity = false;
  
  // Forward data from USB Serial to UART2 (Computer → Target ESP32)
  if (Serial.available()) {
    String data = Serial.readString();
    Serial2.print(data);
    activity = true;
    
    // Optional: Echo back to confirm transmission (comment out if not needed)
    // Serial.print("[TX→] ");
    // Serial.print(data);
  }
  
  // Forward data from UART2 to USB Serial (Target ESP32 → Computer)
  if (Serial2.available()) {
    String data = Serial2.readString();
    Serial.print(data);
    activity = true;
  }
  
  // Activity indicator
  if (activity) {
    lastActivity = millis();
    digitalWrite(LED_PIN, HIGH);
  }
  
  // Turn off LED after 100ms of no activity
  if (millis() - lastActivity > 100) {
    digitalWrite(LED_PIN, LOW);
  }
  
  // Heartbeat every 5 seconds when no activity
  if (millis() - lastActivity > 5000) {
    if (millis() % 1000 < 50) {  // Brief blink every second
      if (!ledState) {
        digitalWrite(LED_PIN, HIGH);
        ledState = true;
      }
    } else if (millis() % 1000 > 100) {
      if (ledState) {
        digitalWrite(LED_PIN, LOW);
        ledState = false;
      }
    }
  }
  
  delay(1); // Small delay to prevent excessive CPU usage
} 