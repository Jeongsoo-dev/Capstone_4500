# Connect to ESP wifi and run this scipt

import asyncio
import websockets
import json

async def send_pitch_roll():
    uri = "ws://192.168.4.1:81"  # IP of ESP32
    try:
        async with websockets.connect(uri) as websocket:
            print("Connected to ESP32 WebSocket Server.")
            while True:
                try:
                    pitch = float(input("Enter pitch angle (°): "))
                    roll = float(input("Enter roll angle (°): "))
                    payload = json.dumps({"pitch": pitch, "roll": roll})
                    await websocket.send(payload)
                    print(f"Sent: {payload}")
                except ValueError:
                    print("Invalid input. Use numeric values only.")
    except Exception as e:
        print(f" Connection error: {e}")

if __name__ == "__main__":
    asyncio.run(send_pitch_roll())
