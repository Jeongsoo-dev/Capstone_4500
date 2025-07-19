# Stewart Platform IMU Control System

## Overview
This system reads IMU orientation data via USB UART and replicates the movement on a 3-actuator Stewart platform using BTS7960 motor drivers.

## Hardware Setup
- **ESP32**: DOIT DevKit v1
- **Motors**: 3x BTS7960 motor drivers controlling linear actuators  
- **IMU**: Any sensor supporting the WT901 communication protocol
- **Connections**: See `pin_definitions.cpp` for detailed wiring

## Quick Start

### 1. Test IMU Communication First
```bash
# Build and upload the IMU monitor
pio run --environment esp32doit-devkit-v1 --target upload
# Replace main.cpp with imu_monitor.cpp temporarily
```

### 2. Run Full Stewart Platform Control
```bash
# Use main.cpp for full control system
pio run --environment esp32doit-devkit-v1 --target upload
```

### 3. Monitor Serial Output
```bash
pio device monitor --baud 115200
```

## System Behavior

### IMU Data Processing
- Reads packets with header `0x55 0x61`
- Extracts roll, pitch, and yaw angles
- Converts to actuator target positions using Stewart platform kinematics

### Motor Control
- **3 Actuators** positioned at 0°, 120°, 240° around the platform
- **Automatic Homing**: System moves to neutral position (700mm) on startup
- **Smooth motion** with configurable speed limiting
- **Safety limits** prevent over-extension (550-850mm range)

### Control Loop
- **10ms update interval** for smooth motion
- **Real-time response** to IMU orientation changes
- **Gradual movement** prevents sudden jerks

## Configuration Parameters

| Parameter | Value | Description |
|-----------|-------|-------------|
| `actuatorSpeed` | 84 mm/s | Maximum movement speed |
| `armRadius` | 200 mm | Platform geometry |
| `minLength` | 550 mm | Minimum actuator length |
| `maxLength` | 850 mm | Maximum actuator length |
| `homeLength` | 700 mm | Neutral/level platform position |
| `updateInterval` | 10 ms | Control loop frequency |

## Troubleshooting

### No IMU Data Received
1. Check UART connections (TX/RX)
2. Verify baud rate (115200)
3. Confirm IMU is sending `0x55 0x61` packets
4. Use `imu_monitor.cpp` for debugging

### Motors Not Moving
1. Check motor power supply (12V-24V)
2. Verify PWM pin connections
3. Test individual motors with simple commands
4. Check enable pins are shorted (E-F on BTS7960)

### Erratic Movement
1. Reduce `actuatorSpeed` for smoother motion
2. Check for loose connections
3. Verify IMU mounting orientation
4. Adjust motion scaling if needed

## Safety Notes
- **Always** test with IMU monitor before full operation
- **Ensure** actuators cannot over-extend beyond limits
- **Use** appropriate power supply ratings for motors
- **Have** emergency stop capability during testing 