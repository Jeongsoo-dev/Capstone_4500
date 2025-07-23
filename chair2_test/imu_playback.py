#! /usr/bin/env python3

import asyncio
import websockets
import json
import time
import pandas as pd
import sys

# Configuration
WEBSOCKET_URI = "ws://192.168.4.1:81"
DATA_FILE = "playback_data/20250723005056.txt"
G_TO_MS2 = 9.80665 # Standard gravity to m/s^2

def load_imu_data(filepath):
    """Loads IMU data from a tab-separated file into a pandas DataFrame."""
    try:
        print(f"Loading data from {filepath}...")
        df = pd.read_csv(filepath, sep='\t')
        print("Data loaded successfully.")
        
        # Rename columns for easier access and consistency
        df.rename(columns={
            'AngleX(°)': 'roll',
            'AngleY(°)': 'pitch',
            'AngleZ(°)': 'yaw',
            'AccX(g)': 'acc_x',
            'AccY(g)': 'acc_y',
            'AccZ(g)': 'acc_z',
            'AsX(°/s)': 'ang_vel_x',
            'AsY(°/s)': 'ang_vel_y',
            'AsZ(°/s)': 'ang_vel_z',
            'time': 'timestamp'
        }, inplace=True)
        
        # Convert timestamps to datetime objects to calculate delays
        df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y-%m-%d %H:%M:%S.%f')
        
        # Convert acceleration from g to m/s^2
        for axis in ['x', 'y', 'z']:
            df[f'acc_{axis}'] = df[f'acc_{axis}'] * G_TO_MS2
            
        print("Data processed and ready for playback.")
        return df
    except FileNotFoundError:
        print(f"Error: Data file not found at '{filepath}'")
        sys.exit(1)
    except Exception as e:
        print(f"Error loading or processing data file: {e}")
        sys.exit(1)

async def imu_playback(data):
    """Connects to the WebSocket server and plays back IMU data."""
    uri = WEBSOCKET_URI
    try:
        async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as websocket:
            print(f"Connected to ESP32 WebSocket Server at {uri}.")
            
            # Wait for welcome message
            try:
                welcome = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"Server says: {welcome}")
            except asyncio.TimeoutError:
                print("No welcome message received (older firmware?)")

            print("\nStarting IMU data playback in 3 seconds...")
            await asyncio.sleep(3)
            
            last_timestamp = None

            for index, row in data.iterrows():
                # Calculate delay since last packet to simulate real-time playback
                if last_timestamp:
                    delay = (row['timestamp'] - last_timestamp).total_seconds()
                    await asyncio.sleep(max(0, delay))
                
                last_timestamp = row['timestamp']

                # Construct the JSON payload from the data row
                payload = {
                    "timestamp": int(time.time() * 1000),
                    "motion": {
                        "orientation": {
                            "pitch": row.get('pitch', 0.0),
                            "roll": row.get('roll', 0.0),
                            "yaw": row.get('yaw', 0.0)
                        },
                        "acceleration": {
                            "x": row.get('acc_x', 0.0),
                            "y": row.get('acc_y', 0.0),
                            "z": row.get('acc_z', G_TO_MS2)
                        },
                        "angular_velocity": {
                            "x": row.get('ang_vel_x', 0.0),
                            "y": row.get('ang_vel_y', 0.0),
                            "z": row.get('ang_vel_z', 0.0)
                        }
                    },
                    "control": {
                        "mode": "playback",
                        "response_speed": "fast"
                    }
                }
                
                payload_json = json.dumps(payload)
                await websocket.send(payload_json)
                
                # Print status without waiting for ACK to maintain playback speed
                # Truncate values for cleaner printing
                print(f"Sent [#{index+1}]: "
                      f"Pitch={payload['motion']['orientation']['pitch']:.1f}, "
                      f"Roll={payload['motion']['orientation']['roll']:.1f} | "
                      f"AccZ={payload['motion']['acceleration']['z']:.2f} | "
                      f"AngVelY={payload['motion']['angular_velocity']['y']:.1f}", end='\r')

            print("\n\nPlayback finished.")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"\nConnection lost to ESP32: {e}")
    except ConnectionRefusedError:
        print(f"\nConnection refused. Is the ESP32 running and on the network at {uri}?")
    except Exception as e:
        print(f"\nAn error occurred: {e}")

if __name__ == "__main__":
    try:
        import pandas
    except ImportError:
        print("Error: pandas library not found.")
        print("Please install it using: pip install pandas")
        sys.exit(1)
        
    imu_dataframe = load_imu_data(DATA_FILE)
    asyncio.run(imu_playback(imu_dataframe))
