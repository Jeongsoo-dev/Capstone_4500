# ESP32 Serial Debugger

This project turns one ESP32 into a USB-to-UART serial debugger/bridge for debugging another ESP32 when the target's USB port is occupied or unavailable.

## How It Works

The debugger ESP32 acts as a bridge between your computer and the target ESP32:
- **Computer ↔ Debugger ESP32**: USB Serial communication
- **Debugger ESP32 ↔ Target ESP32**: UART2 communication
- All data is forwarded bidirectionally in real-time

## Wiring Connections

| Debugger ESP32 | Target ESP32 | Notes |
|----------------|--------------|-------|
| GPIO17 (TX2)   | RX pin*      | Data from debugger to target |
| GPIO16 (RX2)   | TX pin*      | Data from target to debugger |
| GND            | GND          | Common ground (required) |

*For target ESP32 RX/TX pins, you can use:
- **Hardware Serial**: GPIO3 (RX0), GPIO1 (TX0) - default UART0
- **Hardware Serial 2**: GPIO16 (RX2), GPIO17 (TX2) - UART2
- **Any GPIO pins** if using SoftwareSerial in your target code

## Setup Instructions

1. **Upload the debugger code**:
   ```bash
   cd serial_debugger
   pio run --target upload
   ```

2. **Wire the connections** as shown in the table above

3. **Connect debugger ESP32 to computer** via USB

4. **Open Serial Monitor**:
   ```bash
   pio device monitor --baud 115200
   ```

5. **Power on target ESP32** and start debugging!

## Features

- **Real-time bidirectional communication**
- **LED activity indicator**:
  - LED turns ON during data transmission
  - Heartbeat blink when idle
  - 6 blinks on startup (ready indicator)
- **115200 baud rate** (configurable)
- **Clear startup message** with connection details

## Usage Tips

- Make sure both ESP32s share a common ground (GND connection)
- The target ESP32 can be powered separately or through the debugger
- You can change baud rates by modifying `BAUD_RATE` in the code
- If you need different UART pins, modify `UART_TX_PIN` and `UART_RX_PIN`

## Troubleshooting

- **No communication**: Check wiring, especially GND connection
- **Garbled text**: Verify baud rate matches on both sides
- **One-way communication**: Check TX/RX aren't swapped
- **No activity LED**: Ensure GPIO2 isn't used for other purposes on your board

## Example Target ESP32 Code

Your target ESP32 code should use the pins you connected to. For example:

```cpp
// If connected to default UART0 (GPIO1/GPIO3)
void setup() {
  Serial.begin(115200);
  Serial.println("Hello from target ESP32!");
}

// If connected to UART2 (GPIO16/GPIO17)
void setup() {
  Serial2.begin(115200);
  Serial2.println("Hello from target ESP32!");
}
``` 