#!/usr/bin/env python3
"""
Enhanced Stewart Platform Chair2 Tester
Real-time WebSocket communication for smooth motion control

Dependencies:
    pip install websocket-client

Usage:
    python test_stewart.py

Features:
- Real-time WebSocket streaming (50Hz capable)
- Enhanced motion data format with acceleration
- Smooth sinusoidal motion demos
- Manual control mode
- Step-by-step test sequences
"""

import websocket
import json
import time
import threading
import math

# WebSocket connection details
WS_URL = "ws://192.168.4.1:81"

class StewartTester:
    def __init__(self):
        self.ws = None
        self.connected = False
        self.running = False
        
    def on_open(self, ws):
        print("WebSocket connection opened")
        self.connected = True
        
    def on_message(self, ws, message):
        print(f"Received: {message}")
        
    def on_error(self, ws, error):
        print(f"WebSocket error: {error}")
        
    def on_close(self, ws, close_status_code, close_msg):
        print("WebSocket connection closed")
        self.connected = False
        
    def connect(self):
        """Connect to the WebSocket server"""
        self.ws = websocket.WebSocketApp(
            WS_URL,
            on_open=self.on_open,
            on_message=self.on_message,
            on_error=self.on_error,
            on_close=self.on_close
        )
        
        # Start WebSocket in a separate thread
        self.ws_thread = threading.Thread(target=self.ws.run_forever)
        self.ws_thread.daemon = True
        self.ws_thread.start()
        
        # Wait for connection
        timeout = 10
        while not self.connected and timeout > 0:
            time.sleep(0.1)
            timeout -= 0.1
            
        if not self.connected:
            raise Exception("Failed to connect to WebSocket server")
            
        print("Connected to Stewart Platform Chair2")
        
    def send_motion_data(self, pitch, roll, yaw=0.0, accel_x=0.0, accel_y=0.0, accel_z=9.81):
        """Send enhanced motion data to chair2"""
        if not self.connected:
            print("Not connected to WebSocket")
            return False
            
        motion_data = {
            "timestamp": int(time.time() * 1000),  # milliseconds
            "motion": {
                "orientation": {
                    "pitch": pitch,
                    "roll": roll,
                    "yaw": yaw
                },
                "acceleration": {
                    "x": accel_x,
                    "y": accel_y,
                    "z": accel_z
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
        
        try:
            self.ws.send(json.dumps(motion_data))
            return True
        except Exception as e:
            print(f"Failed to send data: {e}")
            return False
            
    def run_test_sequence(self):
        """Run a sequence of test movements"""
        test_cases = [
            {"name": "Neutral", "pitch": 0.0, "roll": 0.0},
            {"name": "Pitch Forward", "pitch": 10.0, "roll": 0.0},
            {"name": "Pitch Back", "pitch": -10.0, "roll": 0.0},
            {"name": "Roll Right", "pitch": 0.0, "roll": 10.0},
            {"name": "Roll Left", "pitch": 0.0, "roll": -10.0},
            {"name": "Combined 1", "pitch": 8.0, "roll": 8.0},
            {"name": "Combined 2", "pitch": -8.0, "roll": -8.0},
            {"name": "Max Forward-Right", "pitch": 15.0, "roll": 15.0},
            {"name": "Max Back-Left", "pitch": -15.0, "roll": -15.0},
            {"name": "Return to Neutral", "pitch": 0.0, "roll": 0.0}
        ]
        
        print("\nStarting test sequence...")
        
        for i, test_case in enumerate(test_cases):
            print(f"\n[{i+1}/{len(test_cases)}] {test_case['name']}: "
                  f"pitch={test_case['pitch']}°, roll={test_case['roll']}°")
            
            if self.send_motion_data(test_case['pitch'], test_case['roll']):
                print("✓ Command sent successfully")
            else:
                print("✗ Failed to send command")
                
            # Wait between movements for smooth operation
            time.sleep(2.0)
            
        print("\nTest sequence completed!")
        
    def run_smooth_demo(self, duration=30):
        """Run a smooth continuous motion demo"""
        print(f"\nStarting smooth motion demo for {duration} seconds...")
        print("Press Ctrl+C to stop early")
        
        start_time = time.time()
        try:
            while time.time() - start_time < duration:
                current_time = time.time() - start_time
                
                # Generate smooth sinusoidal motion
                pitch = 12.0 * math.sin(current_time * 0.3)  # Slow pitch cycle
                roll = 8.0 * math.sin(current_time * 0.5)    # Faster roll cycle
                
                # Add some simulated acceleration (for future vibration features)
                accel_x = 0.5 * math.sin(current_time * 2.0)
                accel_y = 0.3 * math.cos(current_time * 1.5)
                
                if self.send_motion_data(pitch, roll, accel_x=accel_x, accel_y=accel_y):
                    print(f"Motion: P={pitch:6.2f}° R={roll:6.2f}° | "
                          f"Time: {current_time:5.1f}s", end='\r')
                else:
                    print("\nConnection lost during demo")
                    break
                    
                # High frequency updates (50Hz)
                time.sleep(0.02)
                
        except KeyboardInterrupt:
            print("\nDemo stopped by user")
            
        # Return to neutral
        print("\nReturning to neutral position...")
        self.send_motion_data(0.0, 0.0)
        
    def disconnect(self):
        """Close WebSocket connection"""
        if self.ws:
            self.ws.close()
            self.connected = False

def main():
    tester = StewartTester()
    
    try:
        # Connect to chair2
        print("Connecting to Stewart Platform Chair2...")
        tester.connect()
        
        # Run test sequence
        choice = input("\nChoose test mode:\n1. Step sequence\n2. Smooth demo\n3. Manual control\nChoice (1-3): ")
        
        if choice == "1":
            tester.run_test_sequence()
        elif choice == "2":
            duration = int(input("Demo duration in seconds (default 30): ") or "30")
            tester.run_smooth_demo(duration)
        elif choice == "3":
            print("\nManual control mode. Enter 'quit' to exit.")
            while True:
                try:
                    cmd = input("Enter 'pitch,roll' (e.g., '10.5,-5.2'): ").strip()
                    if cmd.lower() == 'quit':
                        break
                    
                    pitch, roll = map(float, cmd.split(','))
                    if tester.send_motion_data(pitch, roll):
                        print(f"Sent: pitch={pitch}°, roll={roll}°")
                    else:
                        print("Failed to send command")
                        
                except ValueError:
                    print("Invalid format. Use: pitch,roll (e.g., '10.5,-5.2')")
                except KeyboardInterrupt:
                    break
        else:
            print("Invalid choice")
            
    except Exception as e:
        print(f"Error: {e}")
        
    finally:
        # Clean shutdown
        tester.send_motion_data(0.0, 0.0)  # Return to neutral
        time.sleep(1)
        tester.disconnect()
        print("Disconnected from Stewart Platform")

if __name__ == "__main__":
    main()
