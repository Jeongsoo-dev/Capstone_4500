# Connect to ESP wifi and run this scipt

import asyncio
import websockets
import json
import time

async def send_pitch_roll():
    uri = "ws://192.168.4.1:81"  # IP of ESP32
    try:
        async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as websocket:
            print("Connected to ESP32 WebSocket Server.")
            
            # Wait for welcome message
            try:
                welcome = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"Welcome message: {welcome}")
            except asyncio.TimeoutError:
                print("No welcome message received (older firmware?)")
            
            while True:
                try:
                    pitch = float(input("Enter pitch angle (°): "))
                    roll = float(input("Enter roll angle (°): "))
                    
                    # Create payload in new nested format
                    payload = {
                        "timestamp": int(time.time() * 1000),  # Current timestamp in milliseconds
                        "motion": {
                            "orientation": {
                                "pitch": pitch,
                                "roll": roll,
                                "yaw": 0.0  # Default yaw for testing
                            },
                            "acceleration": {
                                "x": 0.0,
                                "y": 0.0,
                                "z": 9.81  # Default gravity
                            },
                            "angular_velocity": {
                                "x": 0.0,
                                "y": 0.0,
                                "z": 0.0
                            },
                            "motion_events": {
                                "flags": 0
                            }
                        },
                        "control": {
                            "mode": "realtime",
                            "response_speed": "fast"
                        }
                    }
                    
                    payload_json = json.dumps(payload)
                    await websocket.send(payload_json)
                    print(f"Sent: pitch={pitch}°, roll={roll}°")
                    
                    # Wait for acknowledgment
                    try:
                        ack = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        ack_data = json.loads(ack)
                        if ack_data.get("status") == "ok":
                            print("✓ ESP32 acknowledged motion data")
                        else:
                            print(f"⚠ ESP32 response: {ack_data.get('message', 'Unknown')}")
                    except asyncio.TimeoutError:
                        print("⚠ No acknowledgment received")
                    except json.JSONDecodeError:
                        print(f"⚠ Invalid acknowledgment: {ack}")
                        
                except ValueError:
                    print("Invalid input. Use numeric values only.")
                except websockets.exceptions.ConnectionClosed:
                    print("Connection lost to ESP32")
                    break
    except Exception as e:
        print(f"Connection error: {e}")

if __name__ == "__main__":
    asyncio.run(send_pitch_roll())
