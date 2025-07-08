# ESP32-Based 2DOF VR Chair Control
## Hardware Overview
1. ESP32
   - Main controller with Wi-Fi capability
2. BTS7960
   - Motor driver for each actuator (PWM + DIR control)
3. Actuators
   - Worm gear linear actuators (550–850mm, 300mm stroke)
4. Power
   - 24V PSU powers actuators directly through BTS7960

## Platform Geometry
- The base and top are triangular frames
- 3 actuators are placed at triangle vertices
- The platform moves in pitch (forward/back) and roll (left/right) around the center

### Geometry Constants
- Actuator Min Length :	550 mm
- Actuator Max Length : 850 mm
- Actuator Stroke	: 300 mm
- Speed :	84 mm/s - full stroke in ~3.57 seconds
- Tilt Range :	±15° pitch and roll
- Arm Radius :	200 mm (distance from center to actuator mount)

## ESP32 Pin Assignments
| Actuator | PWM Pin | DIR Pin |
| -------- | ------- | ------- |
| A        | GPIO 18 | GPIO 5  |
| B        | GPIO 19 | GPIO 17 |
| C        | GPIO 21 | GPIO 16 |

## Wi-Fi Communication
### Mode:
ESP32 acts as a Wi-Fi Access Point (AP)

Device (PC or phone) connects directly to ESP32’s Wi-Fi

- SSID: StewartPlatform
- Password: esp32vrchair

### Server:
- ESP32 runs a WebServer on port 80
- Listens for POST /move requests

### Format
{ "pitch": 5.3, "roll": -3.0 }

## Movement is Calculated
1. Receive Command
ESP32 receives pitch and roll (in degrees) via HTTP POST.

2. Convert Pitch/Roll → Actuator Lengths
Uses geometry-based inverse kinematics:

- Assumes the platform tilts around center

- Computes how much up/down each actuator must move to achieve pitch/roll

- Uses sin() and cos() based on actuator angles: 0°, 120°, 240°

Formula:
dz = armRadius * (sin(pitch) * cos(angle) + sin(roll) * sin(angle));
target = 550 + dz;

3. Command Each Actuator (Open-loop Control)
- Compare current vs. target length
- Decide direction (increase/decrease)
- Calculate duration = distance / speed
- Run PWM for that duration
- No feedback — we assume actuator moves at constant speed

## Open-Loop Motion Control
| Step | Action                                   |
| ---- | ---------------------------------------- |
| 1    | Set direction pin (HIGH/LOW)             |
| 2    | Turn on PWM at fixed duty (e.g. 200/255) |
| 3    | Delay for time = (distance ÷ speed)      |
| 4    | Stop PWM                                 |

## Summary
- ESP32 receives real-time tilt data over Wi-Fi
- Converts pitch/roll to actuator lengths
- Sends signals to 3 BTS7960 drivers
- Actuators lift or lower to recreate tilt
- Fully self-contained system: no external PC needed (just Wi-Fi)



