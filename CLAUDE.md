# CLAUDE.md - LLM Integration Guide

## Project Overview

**AURA** (Advanced Utility for Reliable Acquisition) is embedded helmet firmware for the O.A.S.I.S. wearable computing platform. It collects and processes sensor data in real-time, including environmental monitoring, motion tracking, GPS positioning, and faceplate control.

---

## Architecture

### Version Overview

| Version | MCU | Status | Use Case |
|---------|-----|--------|----------|
| v2.5 | ESP32-S3 | **Current** | Full-featured, recommended for new projects |
| v2 | ESP32-C3 | Transitional | Testing newer hardware |
| v1 | Teensy 4.0 | Legacy | Original implementation |

### v2.5 Module Structure

```
aura-v2.5/
├── aura-v2.5.ino      # Main entry point
├── config.h           # Hardware/feature configuration
├── arduino_secrets.h  # WiFi/credentials (gitignored)
├── command_processor.* # Command handling
├── display_module.*   # TFT display driver
├── enviro_module.*    # Environmental sensors
├── espnow_module.*    # ESP-NOW communication
├── faceplate_module.* # Servo control
├── gps_module.*       # GPS processing
└── imu_module.*       # Motion/orientation
```

### Hardware (v2.5)

| Component | Hardware | Interface |
|-----------|----------|-----------|
| MCU | ESP32-S3 TFT Feather | - |
| IMU | SparkFun BNO086 | SPI |
| GPS | SparkFun u-blox GNSS | UART |
| Air Quality | ScioSense ENS160 | I2C |
| CO2/Temp/Humidity | Sensirion SCD41 | I2C |
| Display | ST7789 TFT (240x135) | SPI |
| Faceplate | Servo motors | GPIO/PWM |
| Eyes | NeoPixels/LEDs | GPIO |

---

## Build System

### Prerequisites

- Arduino IDE 2.x or PlatformIO
- Board support:
  - v2.5: ESP32-S3
  - v2: ESP32-C3
  - v1: Teensyduino
- Required libraries (see README.md for full list)

### Build Steps

```bash
# Arduino IDE
1. Open aura-v2.5/aura-v2.5.ino
2. Select board: ESP32-S3 Dev Module or Feather
3. Copy arduino_secrets.h template and configure
4. Review config.h for your hardware setup
5. Upload to device
```

### Configuration (v2.5)

Key options in `config.h`:

| Option | Purpose |
|--------|---------|
| `ENABLE_MQTT` / `ENABLE_SOCKET` | Network connectivity mode |
| `GPS_INTERFACE` | Serial interface for GPS |
| `SERVO_*` | Faceplate servo angles |
| `I2C_SDA` / `I2C_SCL` | I2C pin assignment |
| Display parameters | Resolution, orientation |

---

## Communication

### MQTT Topics

| Topic | Direction | Purpose |
|-------|-----------|---------|
| `helmet/sensors` | Publish | Environmental data |
| `helmet/imu` | Publish | Orientation/motion |
| `helmet/gps` | Publish | Location data |
| `helmet/faceplate` | Subscribe | Faceplate commands |
| `helmet/eyes` | Subscribe | LED control |

### ESP-NOW

Direct peer-to-peer communication with SPARK (hand modules) for low-latency coordination.

### Data Format

JSON via ArduinoJson for all sensor data and commands.

---

## Working with This Codebase

### When Modifying Firmware

1. **Test on target hardware** - Each version has specific hardware requirements
2. **Preserve real-time performance** - FreeRTOS tasks have timing requirements
3. **Thread safety** - Use proper synchronization for shared data
4. **Power management** - Consider battery life implications
5. **Keep credentials separate** - Never commit `arduino_secrets.h`

### Module Guidelines

| Module | Considerations |
|--------|----------------|
| `imu_module` | SPI timing critical, calibration data |
| `gps_module` | UART buffer handling, NMEA parsing |
| `enviro_module` | I2C sensor initialization order |
| `display_module` | Non-blocking updates, page state |
| `faceplate_module` | Servo limits, mechanical safety |
| `espnow_module` | MAC address pairing, packet size |

### Common Tasks

| Task | Approach |
|------|----------|
| Add new sensor | Create module, register with main loop/task |
| Add display page | Extend display_module page handling |
| New command | Add to command_processor switch |
| Change connectivity | Toggle ENABLE_MQTT/ENABLE_SOCKET in config.h |

---

## Integration Points

### O.A.S.I.S. Ecosystem

| Component | Relationship |
|-----------|--------------|
| **SPARK** | Receives hand sensor data via ESP-NOW |
| **DAWN** | Sends voice commands, receives sensor data |
| **MIRAGE** | Receives helmet data for HUD display |

### S.C.O.P.E. Coordination

- **Meta-repo**: [malcolmhoward/the-oasis-project-meta-repo](https://github.com/malcolmhoward/the-oasis-project-meta-repo)
- **Tracking issue**: See S.C.O.P.E. for cross-component coordination
- **Documentation**: Aggregated in S.C.O.P.E. coordination docs

---

## Commands

```bash
# Verify Arduino CLI (if using CLI)
arduino-cli board list

# Compile v2.5 (Arduino CLI)
arduino-cli compile --fqbn esp32:esp32:esp32s3 aura-v2.5/

# Upload v2.5 (Arduino CLI)
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 aura-v2.5/
```

---

## License

AURA is licensed under **GPLv3**. See LICENSE for details.

## Branch Naming Convention

**Critical**: Branch names must include the GitHub issue number being addressed.

### Format
```
feat/<component>/<issue#>-<short-description>
```

### Before Creating a Branch

1. **Identify the issue** you're working on (check GitHub Issues)
2. **Use that issue's number** in the branch name
3. **Verify** the issue number matches the work being done

### Examples
```bash
# Check available issues first
gh issue list --repo malcolmhoward/aura

# Create branch with correct issue number
git checkout -b feat/aura/<issue#>-description
```

### Common Mistake
❌ Using arbitrary numbers or the wrong issue number
✅ Always check `gh issue list` or GitHub Issues before creating a branch
