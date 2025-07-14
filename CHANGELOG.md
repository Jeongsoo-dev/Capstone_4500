# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [2025/07/13 Yixuan Chen]

### Added
- Support for nested JSON data format from chair1 IMU sensors
- Timestamp logging and extraction from incoming data
- Yaw angle extraction and logging (not used in control yet)
- Control mode and response speed parameter extraction
- Enhanced WebSocket event logging with more motion data
- Comprehensive WebSocket event handling (connection, disconnection, errors)
- Client connection management and tracking
- Message size validation (1024 byte limit)
- WebSocket acknowledgment responses for client feedback
- Connection timeout and ping handling in test script

### Changed
- **chair2/main.cpp**: Updated JSON parsing to handle nested structure
  - Increased JSON buffer size from 200 to 512 bytes
  - Modified data extraction to read from `motion.orientation.pitch/roll` paths
  - Enhanced serial output to include timestamp, yaw, and control parameters
  - Maintained existing motor control logic and kinematics unchanged
- **chair2/test_stewart.py**: Updated test script to generate new data format
  - Now sends complete nested JSON structure matching IMU sensor output
  - Automatic timestamp generation using current system time
  - Added default values for acceleration, angular velocity, and motion events
  - Simplified user interaction while maintaining full data structure

### Technical Details
- JSON structure now supports:
  ```json
  {
    "timestamp": 1234567890123,
    "motion": {
      "orientation": {"pitch": -12.5, "roll": 8.3, "yaw": 45.2},
      "acceleration": {"x": 0.85, "y": -0.23, "z": 9.81},
      "angular_velocity": {"x": 2.3, "y": -1.8, "z": 0.1},
      "motion_events": {"flags": 0}
    },
    "control": {
      "mode": "realtime",
      "response_speed": "fast"
    }
  }
  ```
- Backward compatibility maintained for core Stewart platform functionality
- Ready for future expansion to use additional motion data (acceleration, angular velocity)

### Fixed
- **WebSocket Issues**: Resolved multiple potential problems with WebSocket implementation
  - Added proper handling for all WebSocket event types (previously only handled text messages)
  - Fixed missing client connection/disconnection management
  - Added message size validation to prevent buffer overflows
  - Improved error handling with detailed error messages and client feedback
  - Added acknowledgment system so clients know if messages were received successfully
  - Enhanced connection stability with ping/pong and timeout handling 
