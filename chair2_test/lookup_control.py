#!/usr/bin/env python3

import asyncio
import websockets
import json
import time
import numpy as np
from scipy.interpolate import RegularGridInterpolator
import sys

# Configuration
WEBSOCKET_URI = "ws://192.168.4.1:81"
LOOKUP_TABLE_FILE = "lookup_table.txt"

def validate_workspace_constraints(pitch_deg, roll_deg):
    """
    Validate workspace constraints based on analysis
    
    Args:
        pitch_deg: Pitch angle in degrees
        roll_deg: Roll angle in degrees
        
    Returns:
        bool: True if within constraints, False otherwise
    """
    # Updated workspace constraints from analysis
    pitch_min, pitch_max = -10.0, 15.0
    roll_min, roll_max = -15.0, 15.0
    
    # Check basic ranges
    if not (pitch_min <= pitch_deg <= pitch_max):
        return False
    if not (roll_min <= roll_deg <= roll_max):
        return False
    
    # Check constraint pattern: pitch >= abs(roll) - 10
    if pitch_deg < abs(roll_deg) - 10:
        return False
    
    return True

class LookupTableController:
    def __init__(self, table_file):
        """Initialize the lookup table controller"""
        self.table_file = table_file
        self.interpolator = None
        self.load_lookup_table()
    
    def load_lookup_table(self):
        """Load the lookup table and create interpolator"""
        print(f"Loading lookup table from {self.table_file}...")
        
        try:
            # Read the lookup table file
            data = []
            with open(self.table_file, 'r', encoding='utf-8') as f:
                for line in f:
                    line = line.strip()
                    if line and not line.startswith('#'):
                        # Parse: pitch(deg) roll(deg) l1(mm) l2(mm) l3(mm)
                        parts = line.split()
                        if len(parts) == 5:
                            pitch, roll, l1, l2, l3 = map(float, parts)
                            data.append([pitch, roll, l1, l2, l3])
            
            if not data:
                raise ValueError("No valid data found in lookup table")
            
            # Convert to numpy array
            data = np.array(data)
            
            # Extract unique pitch and roll values
            pitches = np.unique(data[:, 0])
            rolls = np.unique(data[:, 1])
            
            print(f"Loaded {len(data)} entries")
            print(f"Pitch range: [{pitches.min():.1f}°, {pitches.max():.1f}°] with {len(pitches)} points")
            print(f"Roll range: [{rolls.min():.1f}°, {rolls.max():.1f}°] with {len(rolls)} points")
            
            # Reshape data into 3D grid (pitch, roll, actuator_lengths)
            # Create grids for each actuator length
            l1_grid = np.zeros((len(pitches), len(rolls)))
            l2_grid = np.zeros((len(pitches), len(rolls)))
            l3_grid = np.zeros((len(pitches), len(rolls)))
            
            for row in data:
                pitch, roll, l1, l2, l3 = row
                pitch_idx = np.where(pitches == pitch)[0][0]
                roll_idx = np.where(rolls == roll)[0][0]
                l1_grid[pitch_idx, roll_idx] = l1
                l2_grid[pitch_idx, roll_idx] = l2
                l3_grid[pitch_idx, roll_idx] = l3
            
            # Create interpolators for each actuator
            self.pitch_values = pitches
            self.roll_values = rolls
            self.l1_interpolator = RegularGridInterpolator((pitches, rolls), l1_grid, 
                                                          method='linear', bounds_error=False, fill_value=None)
            self.l2_interpolator = RegularGridInterpolator((pitches, rolls), l2_grid, 
                                                          method='linear', bounds_error=False, fill_value=None)
            self.l3_interpolator = RegularGridInterpolator((pitches, rolls), l3_grid, 
                                                          method='linear', bounds_error=False, fill_value=None)
            
            print("✓ Lookup table loaded successfully with interpolation support")
            
        except FileNotFoundError:
            print(f"Error: Lookup table file not found at '{self.table_file}'")
            sys.exit(1)
        except Exception as e:
            print(f"Error loading lookup table: {e}")
            sys.exit(1)
    
    def get_actuator_lengths(self, pitch_deg, roll_deg):
        """Get actuator lengths for given pitch and roll angles using interpolation
        
        Args:
            pitch_deg: Pitch angle in degrees
            roll_deg: Roll angle in degrees
            
        Returns:
            tuple: (l1, l2, l3) actuator lengths in mm, or (None, None, None) if out of range
        """
        # Check workspace constraints first
        if not validate_workspace_constraints(pitch_deg, roll_deg):
            print(f"Warning: Input ({pitch_deg:.1f}°, {roll_deg:.1f}°) violates workspace constraints")
            print(f"  Pitch range: [-10.0°, 15.0°]")
            print(f"  Roll range: [-15.0°, 15.0°]") 
            print(f"  Constraint: pitch >= abs(roll) - 10")
            return None, None, None
        
        # Check if values are within the lookup table range
        if (pitch_deg < self.pitch_values.min() or pitch_deg > self.pitch_values.max() or
            roll_deg < self.roll_values.min() or roll_deg > self.roll_values.max()):
            print(f"Warning: Input ({pitch_deg:.1f}°, {roll_deg:.1f}°) is outside lookup table range")
            print(f"  Pitch range: [{self.pitch_values.min():.1f}°, {self.pitch_values.max():.1f}°]")
            print(f"  Roll range: [{self.roll_values.min():.1f}°, {self.roll_values.max():.1f}°]")
            return None, None, None
        
        # Use interpolation to get actuator lengths
        try:
            point = np.array([[pitch_deg, roll_deg]])
            l1 = float(self.l1_interpolator(point)[0])
            l2 = float(self.l2_interpolator(point)[0])
            l3 = float(self.l3_interpolator(point)[0])
            
            return l1, l2, l3
            
        except Exception as e:
            print(f"Error during interpolation: {e}")
            return None, None, None
    
    def validate_lengths(self, l1, l2, l3):
        """Validate that actuator lengths are within safe operating range"""
        min_length = 550.0  # mm
        max_length = 850.0  # mm
        
        if l1 is None or l2 is None or l3 is None:
            return False
        
        if (min_length <= l1 <= max_length and 
            min_length <= l2 <= max_length and 
            min_length <= l3 <= max_length):
            return True
        else:
            print(f"Warning: Actuator lengths out of safe range [{min_length}-{max_length}mm]:")
            print(f"  L1: {l1:.1f}mm, L2: {l2:.1f}mm, L3: {l3:.1f}mm")
            return False

async def send_actuator_command(websocket, l1, l2, l3):
    """Send actuator length command to ESP32"""
    payload = {
        "timestamp": int(time.time() * 1000),
        "command": "set_actuator_lengths",
        "actuator_lengths": {
            "l1": l1,
            "l2": l2, 
            "l3": l3
        },
        "control": {
            "mode": "direct_control",
            "source": "lookup_table"
        }
    }
    
    payload_json = json.dumps(payload)
    await websocket.send(payload_json)
    
    print(f"✓ Sent command: L1={l1:.1f}mm, L2={l2:.1f}mm, L3={l3:.1f}mm")

def get_user_input():
    """Get pitch and roll input from user"""
    print("\n" + "="*50)
    print("Stewart Platform Lookup Table Controller")
    print("="*50)
    
    while True:
        try:
            pitch_input = input("Enter pitch angle (-10.0 to +15.0 degrees): ").strip()
            if pitch_input.lower() in ['q', 'quit', 'exit']:
                return None, None
                
            pitch = float(pitch_input)
            
            roll_input = input("Enter roll angle (-15.0 to +15.0 degrees): ").strip()
            if roll_input.lower() in ['q', 'quit', 'exit']:
                return None, None
                
            roll = float(roll_input)
            
            # Validate workspace constraints
            if not validate_workspace_constraints(pitch, roll):
                print("❌ Input violates workspace constraints:")
                print("  Pitch range: [-10.0°, 15.0°]")
                print("  Roll range: [-15.0°, 15.0°]")
                print("  Constraint: pitch >= abs(roll) - 10")
                continue
            
            return pitch, roll
            
        except ValueError:
            print("Error: Please enter valid numbers for pitch and roll angles")
        except KeyboardInterrupt:
            print("\nExiting...")
            return None, None

async def interactive_control():
    """Main interactive control loop"""
    # Initialize lookup table controller
    controller = LookupTableController(LOOKUP_TABLE_FILE)
    
    # Connect to ESP32
    uri = WEBSOCKET_URI
    try:
        async with websockets.connect(uri, ping_interval=20, ping_timeout=10) as websocket:
            print(f"✓ Connected to ESP32 WebSocket Server at {uri}")
            
            # Wait for welcome message
            try:
                welcome = await asyncio.wait_for(websocket.recv(), timeout=5.0)
                print(f"Server says: {welcome}")
            except asyncio.TimeoutError:
                print("No welcome message received")

            print("\nReady for commands. Type 'q' to quit.")
            
            while True:
                # Get user input
                pitch, roll = get_user_input()
                if pitch is None or roll is None:
                    break
                
                print(f"\nProcessing: Pitch={pitch:.1f}°, Roll={roll:.1f}°")
                
                # Look up actuator lengths
                l1, l2, l3 = controller.get_actuator_lengths(pitch, roll)
                
                if l1 is None:
                    print("❌ Could not determine actuator lengths (out of range)")
                    continue
                
                # Validate lengths
                if not controller.validate_lengths(l1, l2, l3):
                    print("❌ Actuator lengths are unsafe, command not sent")
                    continue
                
                print(f"Target lengths: L1={l1:.1f}mm, L2={l2:.1f}mm, L3={l3:.1f}mm")
                
                # Confirm before sending
                confirm = input("Send command to actuators? (y/N): ").strip().lower()
                if confirm in ['y', 'yes']:
                    await send_actuator_command(websocket, l1, l2, l3)
                    
                    # Optional: wait for acknowledgment
                    try:
                        response = await asyncio.wait_for(websocket.recv(), timeout=2.0)
                        print(f"ESP32 response: {response}")
                    except asyncio.TimeoutError:
                        print("No response from ESP32 (normal)")
                else:
                    print("Command cancelled")

    except websockets.exceptions.ConnectionClosed as e:
        print(f"\nConnection lost to ESP32: {e}")
    except ConnectionRefusedError:
        print(f"\nConnection refused. Is the ESP32 running and on the network at {uri}?")
    except Exception as e:
        print(f"\nAn error occurred: {e}")

def test_lookup_functionality():
    """Test the lookup table functionality without WebSocket"""
    print("Testing lookup table functionality...")
    
    controller = LookupTableController(LOOKUP_TABLE_FILE)
    
    # Test cases (updated for new workspace constraints)
    test_cases = [
        (0.0, 0.0),      # Center position
        (5.0, 3.0),      # Mild positive tilt
        (-5.0, -5.0),    # Negative tilt within constraints
        (15.0, 10.0),    # Near maximum positive (satisfies pitch >= abs(roll) - 10)
        (-10.0, 0.0),    # Maximum negative pitch
        (2.3, -7.8),     # Non-grid point (requires interpolation)
        (15.0, 15.0),    # This should fail constraint: pitch >= abs(roll) - 10
        (-10.0, -15.0),  # This should fail constraint
    ]
    
    print("\nTest Results:")
    print("-" * 80)
    print(f"{'Pitch (°)':<10} {'Roll (°)':<10} {'L1 (mm)':<10} {'L2 (mm)':<10} {'L3 (mm)':<10} {'Status'}")
    print("-" * 80)
    
    for pitch, roll in test_cases:
        l1, l2, l3 = controller.get_actuator_lengths(pitch, roll)
        if l1 is not None:
            valid = controller.validate_lengths(l1, l2, l3)
            status = "✓ Valid" if valid else "⚠ Invalid"
            print(f"{pitch:<10.1f} {roll:<10.1f} {l1:<10.1f} {l2:<10.1f} {l3:<10.1f} {status}")
        else:
            print(f"{pitch:<10.1f} {roll:<10.1f} {'N/A':<10} {'N/A':<10} {'N/A':<10} ❌ Out of range")

def main():
    """Main function"""
    if len(sys.argv) > 1 and sys.argv[1] == '--test':
        test_lookup_functionality()
    else:
        try:
            asyncio.run(interactive_control())
        except KeyboardInterrupt:
            print("\nExiting...")

if __name__ == "__main__":
    # Check dependencies
    try:
        import scipy
        import numpy
        import websockets
    except ImportError as e:
        print(f"Error: Required library not found: {e}")
        print("Please install required packages:")
        print("  pip install numpy scipy websockets")
        sys.exit(1)
    
    main()