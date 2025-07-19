# Stewart Platform Chair - Dual Communication Mode (Bluetooth + UART Fallback)

This project controls a Stewart platform chair using IMU data received via **Bluetooth 5.0** with **UART fallback support** for maximum flexibility.

## 🔄 **Dual Communication Modes**

### **Mode 1: Bluetooth 5.0 (Primary)**
- Wireless IMU communication via BLE
- **UART0 (Serial)** remains available as fallback
- **UART2** used for all debug output

### **Mode 2: UART Fallback (Backup)**
- Wired IMU communication via Serial cable
- **UART0 (Serial)** used for IMU data
- **UART2** used for all debug output

## Hardware Requirements

- ESP32 Development Board (ESP32-DOIT-DEVKIT-V1 or compatible)  
- 3x Linear Actuators with motor drivers
- IMU sensor with Bluetooth 5.0 OR UART support
- USB-Serial adapter for UART2 debugging (optional)
- Power supply for actuators

## ⚡ **Quick Mode Selection**

### **Switch to Bluetooth Mode**
```cpp
// In main.cpp and imu_monitor.cpp
const bool USE_BLUETOOTH_MODE = true;
```

### **Switch to UART Mode** 
```cpp
// In main.cpp and imu_monitor.cpp  
const bool USE_BLUETOOTH_MODE = false;
```

## 🔧 **Hardware Setup**

### **Updated Pin Assignments (UART2 Support)**
```
Motor Driver 1: **CHANGED PINS**
- LPWM: GPIO4  (was GPIO16)
- RPWM: GPIO5  (was GPIO17)
- L_IS: GPIO34 (unchanged)
- R_IS: GPIO35 (unchanged)

Motor Driver 2 & 3: (Unchanged)
- See pin_definitions.cpp for details

UART Interfaces:
- UART0: GPIO1/GPIO3 (USB) - IMU data in UART mode
- UART2: GPIO17/GPIO16 - Debug output always
```

## 🖥️ **Debug Output Setup**

### **UART2 Debug Interface** 
All system output goes to UART2 (GPIO16/17):
- System status and error messages
- BLE connection events  
- IMU data parsing results
- Actuator calculations

### **Connect USB-Serial Adapter (Optional)**
```
ESP32 GPIO17 (TX) → USB-Serial RX
ESP32 GPIO16 (RX) → USB-Serial TX  
ESP32 GND        → USB-Serial GND
```

## 📱 **Bluetooth Configuration (Mode 1)**

### **Configure Your IMU's Bluetooth UUIDs**

Update these values in both `main.cpp` and `imu_monitor.cpp`:
```cpp
static BLEUUID SERVICE_UUID("0000ffe5-0000-1000-8000-00805f9b34fb");    
static BLEUUID CHAR_UUID("0000ffe9-0000-1000-8000-00805f9b34fb");       
```

### **Finding Your IMU's UUIDs**

1. **BLE Scanner Apps:**
   - Android: "nRF Connect for Mobile"
   - iOS: "LightBlue Explorer"
   
2. **Scan and Connect** to your IMU to find the UUIDs

## 🔌 **UART Configuration (Mode 2)**

### **IMU Connection for UART Mode**
```
IMU TX → ESP32 GPIO3 (RX)
IMU RX → ESP32 GPIO1 (TX)  
IMU GND → ESP32 GND
IMU VCC → ESP32 3.3V/5V
```

### **Expected Data Format (Both Modes)**
- Header: `0x55 0x61`
- 18 data bytes (acceleration, gyro, angles)
- Baud rate: 115200

## 🧪 **Testing Procedure**

### **1. Update Hardware First**
```bash
# Move Motor Driver 1 wires from GPIO16/17 to GPIO4/5
# Connect USB-Serial adapter to GPIO16/17 for debug output
```

### **2. Test Bluetooth Mode**
```cpp
const bool USE_BLUETOOTH_MODE = true;  // Enable BLE mode
```
```bash
pio run --target upload
# Monitor debug output via USB-Serial adapter on UART2
```

### **3. Test UART Fallback Mode**
```cpp
const bool USE_BLUETOOTH_MODE = false;  // Enable UART mode
```
```bash
pio run --target upload
# Connect IMU to USB pins, monitor debug via UART2
```

### **4. Switch Between Modes**
Just change `USE_BLUETOOTH_MODE` and re-upload!

## 📊 **Expected Debug Output (UART2)**

### **Bluetooth Mode**
```
==== Stewart Platform IMU Control (Bluetooth 5.0) Starting ====
Motor drivers initialized
Scanning for IMU device...  
Found device: YourIMU
BLE connection established with IMU device
BLE notifications registered successfully
IMU Data - Roll: 12.34°, Pitch: -5.67°, Yaw: 123.45°
Actuator Targets - A: 720.5mm, B: 685.2mm, C: 705.8mm
```

### **UART Mode**
```
==== Stewart Platform IMU Control (UART) Starting ====
UART initialized for IMU communication at 115200 baud
Motor drivers initialized  
IMU Data - Roll: 12.34°, Pitch: -5.67°, Yaw: 123.45°
Actuator Targets - A: 720.5mm, B: 685.2mm, C: 705.8mm
```

## 🛠️ **Debug Features**

### **Enable/Disable UART2 Debug**
```cpp
// In pin_definitions.cpp
const bool ENABLE_DEBUG_UART2 = true;  // Enable debugging
```

### **Debug Functions**
- `debugPrint(String)` - Simple string output
- `debugPrintf(format, ...)` - Printf-style formatting

## 🔍 **Troubleshooting**

### **Motor 1 Not Working**
- Check new GPIO4/GPIO5 connections  
- Verify PWM signals with multimeter
- Test with simple motor code

### **Bluetooth Issues**  
- Wrong UUIDs - check with BLE scanner app
- IMU not advertising - restart IMU
- Connection drops - check power/distance

### **UART Issues**
- Wrong baud rate (should be 115200)
- Crossed TX/RX connections 
- Missing packet headers (0x55 0x61)

### **No Debug Output**
- Check `ENABLE_DEBUG_UART2 = true`
- Verify USB-Serial adapter on GPIO16/17
- Confirm 115200 baud rate

## 💡 **Why This Setup?**

- **Flexibility**: Try Bluetooth first, fall back to UART if needed
- **No rewiring**: Switch modes with just code changes
- **Clean debugging**: UART2 keeps debug separate from data
- **Future-proof**: Easy to add new communication modes

## File Structure

- `main.cpp` - Main control with dual-mode support
- `imu_monitor.cpp` - Testing utility with dual-mode support
- `pin_definitions.cpp` - Pin assignments + UART2 debug functions
- `UART2_DEBUGGING_SETUP.md` - Hardware setup guide  