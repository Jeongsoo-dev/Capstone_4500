# SPIFFS Setup for Lookup Table

## Overview
The updated `imu_playback.cpp` now uses a lookup table stored in SPIFFS for precise actuator control. You need to upload the `lookup_table.txt` file to the ESP32's SPIFFS filesystem.

## Setup Steps

### 1. Enable SPIFFS in PlatformIO
Make sure your `platformio.ini` includes SPIFFS configuration:

```ini
[env:esp32dev]
platform = espressif32
board = esp32dev
framework = arduino
monitor_speed = 115200
board_build.filesystem = spiffs
board_build.partitions = default.csv
```

### 2. Create SPIFFS Data Directory
In your project root, create a `data` folder:

```
chair2_test/
├── data/
│   └── lookup_table.txt  <- Copy your lookup table here
├── imu_playback.cpp
├── platformio.ini
└── ...
```

### 3. Copy Lookup Table
Copy your `lookup_table.txt` file into the `data/` directory. The file should have the CSV format:

```
roll_deg,pitch_deg,l1,l2,l3,height
-15.0,-15.0,817.7,613.3,760.0,684.8
-15.0,-14.5,812.1,612.1,759.4,682.3
...
```

### 4. Upload SPIFFS Data
Use PlatformIO to upload the filesystem:

```bash
# Upload SPIFFS data to ESP32
pio run --target uploadfs

# Then upload your code normally
pio run --target upload
```

### 5. Monitor Output
When the ESP32 starts, you should see:

```
[✓] SPIFFS mounted
[*] Loading lookup table from SPIFFS...
[✓] Loaded XXXX lookup table entries
[✓] Pitch range: [-15.0°, 15.0°] with XX points
[✓] Roll range: [-15.0°, 15.0°] with XX points
[✓] Lookup table loaded successfully
```

## Troubleshooting

### SPIFFS Mount Failed
- Check that `board_build.filesystem = spiffs` is in platformio.ini
- Verify ESP32 has SPIFFS partition space

### Lookup Table Not Found
- Ensure `lookup_table.txt` is in the `data/` directory
- Run `pio run --target uploadfs` to upload filesystem
- Check file permissions

### Invalid Data
- Verify CSV format matches: `roll_deg,pitch_deg,l1,l2,l3,height`
- Ensure no empty lines or malformed rows
- Check that values are within expected ranges

## Fallback Behavior
If lookup table loading fails, the system will:
1. Log an error message
2. Fall back to mathematical kinematics
3. Continue operating normally

This ensures the platform remains functional even without the lookup table.