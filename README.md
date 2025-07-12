# ESP32-Based 2DOF VR Chair Control
## Hardware Overview
| Component      | Description                                    |
| -------------- | ---------------------------------------------- |
| **ESP32**      | Main microcontroller with Wi-Fi and PWM output |
| **BTS7960 x3** | Dual H-bridge motor drivers (PWM + DIR)        |
| **Actuators**  | 3 worm gear linear actuators (550–850 mm)      |
| **Power**      | 24V PSU (shared across all drivers)            |

Each actuator is individually controlled using PWM (speed) and a direction pin, enabling precise tilt movement in pitch and roll axes.

## Platform Geometry
- Structure: Triangular base and top frame

- Movement: Chair tilts about center via actuator height adjustments

- DOF: 2 (pitch and roll)

### Geometry Constants
| Parameter       | Value                                     |
| --------------- | ----------------------------------------- |
| Actuator Stroke | 300 mm (550 mm → 850 mm)                  |
| Max Speed       | 84 mm/s (full stroke ≈ 3.57 sec)          |
| Tilt Range      | ±15° (pitch & roll)                       |
| Arm Radius      | 200 mm (distance from center to actuator) |

## ESP32 Pin Assignments
| Actuator | PWM Pin | DIR Pin |
| -------- | ------- | ------- |
| A        | GPIO 18 | GPIO 5  |
| B        | GPIO 19 | GPIO 17 |
| C        | GPIO 21 | GPIO 16 |

Each pin controls one BTS7960 driver with 20kHz PWM, providing smooth and quiet operation.

## Wi-Fi Communication
### Access Point Mode:
- SSID: StewartPlatform
- Password: esp32vrchair
- ESP32 hosts a WebSocket server on port 81


### Data Format
{
  "pitch": 5.3,
  "roll": -3.0
}

### Motion Control Pipeline

#### Real-Time WebSocket Control

1. ESP32 receives pitch and roll angles via WebSocket.

2.Converts angles to target actuator lengths using inverse kinematics.

3. Smoothly moves each actuator using velocity-limited interpolation.

4. Motor direction is set based on delta sign; PWM duty is conditionally applied.

#### Kinematics Formula

dz = armRadius * (sin(pitch) * cos(angle) + sin(roll) * sin(angle));

length = 550 + dz;

length = constrain(length, 550, 850);

## Smooth Open-Loop Control
| Step | Description                                              |
| ---- | -------------------------------------------------------- |
| 1    | Compute target length per actuator                       |
| 2    | Use `stepTowards()` to incrementally move current length |
| 3    | If Δ > 0.5 mm, apply PWM (`ledcWrite`) and direction     |
| 4    | Repeat every 10 ms (100 Hz update loop)                  |

No position feedback is used (open-loop), but timing and control logic provide stable and fluid motion.

## Testing & Client Integration
import websocket, json, time

ws = websocket.WebSocket()

ws.connect("ws://192.168.4.1:81")

while True:
    ws.send(json.dumps({"pitch": 5.0, "roll": -3.0}))
    time.sleep(0.01)  # 100 Hz update rate
    
## System Summary
- ESP32 runs in AP mode with built-in WebSocket server

- Real-time motion control via WebSocket (sub-10ms latency)

- Acceleration-limited actuator motion using open-loop logic

-  Designed for integration with IMU-based Chair 1, CSV replay, or simulation streaming


## Future Extensions
1. Integration with ROS for IMU logging or visualization
2. Closed-loop position feedback with encoders or potentiometers
3. Vibration overlay channel (PWM superposition or pulse modulation)
