#! /usr/bin/env python3

import asyncio
import websockets
import json
import time
import pandas as pd
import sys
import re
from datetime import datetime

# Configuration
WEBSOCKET_URI = "ws://192.168.4.1:81"
DATA_FILE = "playback_data/20250723005056.txt"
G_TO_MS2 = 9.80665 # Standard gravity to m/s^2
PLAYBACK_SPEED_MULTIPLIER = 1.0  # 1.0 = real-time, 0.5 = half speed, 2.0 = double speed

def normalize_timestamp(timestamp_str):
    """Normalize timestamps to ensure consistent millisecond formatting."""
    # Use regex to find and normalize the timestamp format
    pattern = r'(\d{4}-\d{1,2}-\d{1,2} \d{1,2}:\d{2}:\d{2})\.(\d+)'
    match = re.match(pattern, timestamp_str)
    
    if match:
        date_time_part = match.group(1)
        milliseconds = match.group(2)
        
        # Pad or truncate milliseconds to exactly 3 digits
        if len(milliseconds) < 3:
            milliseconds = milliseconds.ljust(3, '0')
        elif len(milliseconds) > 3:
            milliseconds = milliseconds[:3]
            
        return f"{date_time_part}.{milliseconds}"
    else:
        # Fallback: if no milliseconds, add .000
        return f"{timestamp_str}.000"

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
        
        # Normalize timestamps before parsing
        print("Normalizing timestamps...")
        df['timestamp'] = df['timestamp'].apply(normalize_timestamp)
        
        # Convert timestamps to datetime objects to calculate delays
        df['timestamp'] = pd.to_datetime(df['timestamp'], format='%Y-%m-%d %H:%M:%S.%f')
        
        # Convert acceleration from g to m/s^2
        for axis in ['x', 'y', 'z']:
            df[f'acc_{axis}'] = df[f'acc_{axis}'] * G_TO_MS2
            
        print("Data processed and ready for playback.")
        print(f"Playback speed: {PLAYBACK_SPEED_MULTIPLIER}x")
        if PLAYBACK_SPEED_MULTIPLIER != 1.0:
            print(f"(Original timing will be {'slowed down' if PLAYBACK_SPEED_MULTIPLIER < 1.0 else 'sped up'} by factor of {PLAYBACK_SPEED_MULTIPLIER})")
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
                    original_delay = (row['timestamp'] - last_timestamp).total_seconds()
                    # Apply speed multiplier (smaller multiplier = slower playback)
                    adjusted_delay = original_delay / PLAYBACK_SPEED_MULTIPLIER
                    await asyncio.sleep(max(0, adjusted_delay))
                
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
        
    # Allow command line argument for playback speed
    if len(sys.argv) > 1:
        try:
            PLAYBACK_SPEED_MULTIPLIER = float(sys.argv[1])
            if PLAYBACK_SPEED_MULTIPLIER <= 0:
                print("Error: Playback speed must be greater than 0")
                sys.exit(1)
        except ValueError:
            print("Error: Invalid playback speed. Please provide a number (e.g., 0.5 for half speed, 2.0 for double speed)")
            sys.exit(1)
        
    imu_dataframe = load_imu_data(DATA_FILE)
    asyncio.run(imu_playback(imu_dataframe))
