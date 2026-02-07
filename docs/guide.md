# A.U.R.A - Advanced Utility for Reliable Acquisition (Embedded Helmet Firmware)

<img src="https://www.oasisproject.net/assets/AURA_Logo_bg.png" alt="AURA Logo" width="350" align="right">

This code is a healthy mix of example code from various libraries with a special thanks to Adafruit for their awesome hardware and software.

## Overview

A.U.R.A. (Advanced Utility for Reliable Acquisition) is the embedded helmet firmware for the O.A.S.I.S. ecosystem. It collects environmental and motion sensor data from the helmet, relays it to MIRAGE for HUD display, and controls the motorized faceplate.

AURA runs on an ESP32-S3 microcontroller (v2.5) using FreeRTOS for concurrent sensor polling, display updates, and communication. It supports two communication modes: ESP-NOW for low-latency peer-to-peer links with SPARK gauntlets, and WiFi/MQTT for integration with the broader O.A.S.I.S. network.

Key capabilities:
- 9-DOF IMU (BNO086) for head tracking (roll, pitch, heading)
- GPS positioning via SparkFun u-blox GNSS
- Environmental monitoring: CO2, temperature, humidity, air quality (TVOC, eCO2)
- Motorized faceplate control with servo-driven open/close/toggle
- ESP-NOW encrypted peer-to-peer communication with SPARK devices
- FreeRTOS multi-task architecture across dual cores

**Note:** I will have a circuit diagram in the future. This is a pretty simple one though. All of the devices are I2C devices that can be daisy-chained and it's powered by USB.

## Hardware

### v2.5 (Current - ESP32-S3)

| Component | Description | Interface |
|-----------|-------------|-----------|
| ESP32-S3 TFT Feather | Main processor with WiFi and ESP-NOW | — |
| SparkFun BNO086 | 9-DOF IMU (accelerometer, gyroscope, magnetometer) | SPI |
| SparkFun u-blox GNSS | GPS module (latitude, longitude, altitude, speed) | UART or I2C |
| ScioSense ENS160 | Air quality sensor (eCO2, TVOC) | I2C (0x53) |
| Sensirion SCD41 | CO2, temperature, and humidity sensor | I2C (0x62) |
| ST7789 TFT Display | 240x135 pixel color display for local status | SPI |
| Servos (x2) | Faceplate open/close motors | GPIO 12, 13 |
| NeoPixel LEDs | Eye/status indicators | GPIO |

### v1 (Legacy - Teensy 4.0)

* [Teensy 4.0](https://www.pjrc.com/store/teensy40.html) - Main Processor
* [Adafruit LSM6SDSOX+LIS3MDL](http://adafru.it/4517) - 9-DOF Sensor
* [Adafruit Mini GPS](http://adafru.it/4415) - GPS
* [Adafruit HTS221 Humidity Sensor](http://adafru.it/4535) - Temperature and Humidity Sensor
* [128x64 OLED Display](http://adafru.it/326) - Optional: This was used to tune the system, especially the motorization that's in progress.

## Libraries

- [SPI](https://github.com/arduino-libraries/SPI)
- [Wire](https://github.com/arduino-libraries/Wire)
- [Adafruit GFX Library](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit Sensor Calibration](https://github.com/adafruit/Adafruit_Sensor_Calibration)
- [Adafruit GPS Library](https://github.com/adafruit/Adafruit_GPS)
- [Adafruit HTS221](https://github.com/adafruit/Adafruit_HTS221)
- [Servo (for PWMServo)](https://github.com/arduino-libraries/Servo)
- [Bounce2](https://github.com/thomasfredericks/Bounce2)
- [Adafruit Seesaw](https://github.com/adafruit/Adafruit_Seesaw)
- [ArduinoJson](https://github.com/bblanchon/ArduinoJson)
- [Adafruit AHRS](https://github.com/adafruit/Adafruit_AHRS)
- [Adafruit LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS)

## Building

This firmware is currently being built in the Arduino IDE using the libraries and target hardware mentioned above.

For v2.5 (ESP32-S3):
1. Open `aura-v2.5/aura-v2.5.ino` in Arduino IDE 2.x
2. Select board: ESP32-S3 Dev Module or Feather
3. Configure `arduino_secrets.h` with WiFi credentials (if using WiFi mode)
4. Review `config.h` for hardware pin assignments and sensor settings
5. Upload via USB

Alternatively, using Arduino CLI:
```bash
arduino-cli compile --fqbn esp32:esp32:esp32s3 aura-v2.5/
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32:esp32:esp32s3 aura-v2.5/
```

## Configuration

Configuration is managed through `aura-v2.5/config.h`:

### Communication Mode

Select one communication mode (not both simultaneously):
- `#define ESPNOW_MODE` — ESP-NOW peer-to-peer (recommended)
- `#define WIFI_MODE` — WiFi with MQTT/Socket

### Network Settings (WiFi mode)

- MQTT broker port: `1883`
- Socket port: `3000`
- WiFi credentials: Set in `arduino_secrets.h`

### ESP-NOW Settings

- Channel: 1 (configurable, must match SPARK devices)
- Shared PMK key: 16-byte encryption key (must match SPARK)
- Maximum peers: 20

### Sensor Settings

| Sensor | Key Settings |
|--------|-------------|
| BNO086 IMU | SPI pins: INT=6, RST=9, WAKE=10; Update interval: 100ms |
| GPS (u-blox) | UART pins: TX=39, RX=38; Baud: 38400 (factory) / 115200 (high speed) |
| Environmental (I2C) | SDA=3, SCL=4; CO2 thresholds: Excellent <600, Good <800, Fair <1000, Poor <1500 ppm |

### Faceplate Servo

- Servo pins: GPIO 12, GPIO 13
- Angles: Open=0°, Closed=77°, Backoff=1° (anti-jam protection)

## Usage

1. Flash the firmware (see Building section above)
2. Connect via USB serial at 115200 baud for debug output
3. AURA automatically initializes sensors and starts FreeRTOS tasks:
   - `gpsTask` — GPS polling (Core 1)
   - `imuTask` — IMU updates (Core 0, highest priority)
   - `enviroTask` — Environmental sensor readings (Core 1)
   - `displayTask` — TFT display updates (Core 0)
   - `messagingTask` — Communication dispatch (Core 1)
   - `servoTask` — Faceplate motor control (Core 0)
   - `espnowTask` — ESP-NOW communication (Core 1, ESP-NOW mode only)

### Serial Commands

Send JSON commands via USB serial for testing:
```json
{"action": "faceplate", "value": "open"}
{"action": "faceplate", "value": "close"}
{"action": "faceplate", "value": "toggle"}
```

## Communication

AURA supports two communication modes, selectable in `config.h`.

### ESP-NOW Mode (Recommended)

Direct peer-to-peer WiFi using encrypted ESP-NOW protocol. SPARK gauntlets register as peers during startup:

1. SPARK sends registration message with its topic identifier
2. AURA checks for duplicate topics and responds with ACK or REJECT
3. After ACK, encrypted communication begins

ESP-NOW message types:

| Type | Name | Purpose |
|------|------|---------|
| 1 | REGISTRATION | Device registration request |
| 2 | REG_ACK | Registration acknowledged |
| 3 | REG_REJECT | Registration rejected (duplicate topic) |
| 4 | DATA | Sensor data transfer |
| 5 | DATA_ACK | Data acknowledgment |
| 6 | PING | Connectivity check |
| 7 | PONG | Ping response |

Inactive peers are timed out after 30 seconds.

### WiFi/MQTT Mode (Alternative)

When in WiFi mode, AURA connects to an MQTT broker.

| Direction | Topic | Purpose |
|-----------|-------|---------|
| Subscribe | `helmet` | Receives commands (faceplate control, LED control) |
| Publish | Sensor data topics | Sends IMU, GPS, and environmental readings |

Incoming command format: `{"action": "<action>", "value": "<value>"}`.

### Data Published

Regardless of communication mode, AURA provides:

| Data | Source Sensor | Update Rate |
|------|--------------|-------------|
| Roll, Pitch, Heading | BNO086 IMU | 100ms |
| Latitude, Longitude, Altitude, Speed | u-blox GNSS | GPS fix rate |
| CO2 (ppm), Temperature, Humidity | SCD41 | Sensor polling interval |
| eCO2 (ppm), TVOC (ppb), Air Quality Index | ENS160 | Sensor polling interval |

## Troubleshooting

| Issue | Possible Cause | Solution |
|-------|---------------|----------|
| IMU not detected | SPI wiring or pin mismatch | Verify BNO086 SPI pins (INT=6, RST=9, WAKE=10) in `config.h` |
| GPS no fix | Antenna not connected or indoors | Move to open sky; check UART pins (TX=39, RX=38) |
| I2C sensors not found | Wrong bus or address conflict | Check SDA=3, SCL=4; verify ENS160 (0x53) and SCD41 (0x62) addresses |
| ESP-NOW peers not registering | Channel mismatch or wrong PMK | Ensure channel and PMK key match between AURA and SPARK |
| WiFi connection fails | Wrong credentials | Check `arduino_secrets.h` SSID and password |
| Faceplate servo jams | Angle calibration | Adjust OPEN/CLOSED angles in `config.h`; check backoff angle |
| Display not updating | SPI pin conflict | Verify ST7789 pins (CS=42, DC=40, RST=41) don't conflict with BNO086 |
| Build fails | Missing board support | Install ESP32 board support in Arduino IDE; verify board selection is ESP32-S3 |
| Serial garbled output | Wrong baud rate | Set serial monitor to 115200 baud |

## Related Components

- [M.I.R.A.G.E.](https://www.oasisproject.net/components/mirage/) - Receives helmet sensor data (IMU, GPS, environmental) for HUD display
- [D.A.W.N.](https://www.oasisproject.net/components/dawn/) - Sends faceplate commands to AURA via `helmet` MQTT topic
- [S.P.A.R.K.](https://www.oasisproject.net/components/spark/) - Registers as ESP-NOW peer; sends hand sensor data to AURA for relay
- [B.E.A.C.O.N.](https://www.oasisproject.net/components/beacon/) - CAD models for the helmet enclosure that houses AURA hardware
