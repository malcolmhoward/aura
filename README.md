# AURA - Advanced Utility for Reliable Acquisition

AURA is firmware for embedded helmet systems, designed to collect and process sensor data in real-time. This project serves as the foundation for instrumented helmet applications including environmental monitoring, motion tracking, and GPS positioning.

## Version Overview

### v2.5 (Current Recommended Version)

The latest version of AURA with significant improvements including multitasking, enhanced sensor drivers, and network capabilities. This version should be used for all new projects.

**Hardware Support:**
- **Microcontroller:** ESP32-S3 TFT Feather (or other ESP32-S3 variants)
- **IMU:** SparkFun BNO086 (SPI interface)
- **GPS:** SparkFun u-blox GNSS module (UART interface)
- **Environmental Sensors:** 
  - ScioSense ENS160 (Air quality, eCO2, TVOC, I2C interface)
  - Sensirion SCD41 (CO2, temperature, humidity, I2C interface)
- **Display:** ST7789 TFT color display (240x135)
- **Servo Motors:** For faceplate control
- **LEDs/NeoPixels:** For helmet eyes

**Libraries Required:**
- [Wire](https://www.arduino.cc/reference/en/language/functions/communication/wire/) - For I2C communication
- [SPI](https://www.arduino.cc/reference/en/language/functions/communication/SPI/) - For SPI devices
- [WiFi](https://www.arduino.cc/reference/en/libraries/wifi/) - For network connectivity
- [ArduinoJson](https://arduinojson.org/) - For structured data handling
- [FreeRTOS](https://freertos.org/) - For multitasking operations
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library) - Graphics library
- [Adafruit_ST7789](https://github.com/adafruit/Adafruit-ST7735-Library) - TFT display driver
- [Adafruit_NeoPixel](https://github.com/adafruit/Adafruit_NeoPixel) - For controlling NeoPixels
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo) - For servo control
- [SparkFun BNO08x Arduino Library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library) - For IMU
- [SparkFun u-blox GNSS Arduino Library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library) - For GPS
- [ScioSense_ENS160](https://github.com/sciosense/ENS160_driver) - For air quality sensor
- [SensirionI2cScd4x](https://github.com/Sensirion/arduino-i2c-scd4x) - For CO2 sensor
- Optional: [ArduinoMqttClient](https://github.com/arduino-libraries/ArduinoMqttClient) - For MQTT connectivity

**Key Features:**
- Multi-core operation with task scheduling
- Thread-safe data handling
- Network connectivity with MQTT or Socket support
- In-helmet environmental monitoring
- Servo-controlled faceplate mechanism
- Interactive display with multi-page data visualization for debugging
- JSON-based data reporting

### v2 (Transitional Version)

A transitional version used to test newer hardware.

**Hardware Support:**
- **Microcontroller:** ESP32-C3 (or similar ESP32 variants)
- **IMU:** SparkFun BNO08x
- **GPS:** SparkFun u-blox NEO-M9N
- **Environmental Sensor:** SparkFun BME688 (temperature, humidity, pressure, gas)
- **Display:** 128x64 OLED
- **Servo Control:** Basic faceplate mechanism (unused but available)

**Libraries Required:**
- [Wire](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [SPI](https://www.arduino.cc/reference/en/language/functions/communication/SPI/)
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [ESP32Servo](https://github.com/madhephaestus/ESP32Servo)
- [SparkFun BNO08x Arduino Library](https://github.com/sparkfun/SparkFun_BNO08x_Arduino_Library)
- [SparkFun u-blox GNSS Arduino Library](https://github.com/sparkfun/SparkFun_u-blox_GNSS_Arduino_Library)
- [MicroNMEA](https://github.com/stevemarple/MicroNMEA)
- [Adafruit_BME680](https://github.com/adafruit/Adafruit_BME680)
- [ArduinoJson](https://arduinojson.org/)
- [Bounce2](https://github.com/thomasfredericks/Bounce2) - For button debouncing

### v1 (Legacy Version)

The original version of AURA for Teensy microcontrollers.

**Hardware Support:**
- **Microcontroller:** Teensy 4.0
- **IMU:** Adafruit LSM6DSOX + LIS3MDL (9-DOF)
- **GPS:** Adafruit Mini GPS
- **Environmental Sensor:** Adafruit HTS221 (humidity and temperature)
- **Display:** 128x64 OLED Display (optional)
- **Servo Control:** Basic dual-servo setup (unused but basic functionality tested)

**Libraries Required:**
- [Wire](https://www.arduino.cc/reference/en/language/functions/communication/wire/)
- [SPI](https://www.arduino.cc/reference/en/language/functions/communication/spi/)
- [Adafruit_GFX](https://github.com/adafruit/Adafruit-GFX-Library)
- [Adafruit_SSD1306](https://github.com/adafruit/Adafruit_SSD1306)
- [Adafruit_Sensor](https://github.com/adafruit/Adafruit_Sensor)
- [Adafruit_LSM6DS](https://github.com/adafruit/Adafruit_LSM6DS)
- [Adafruit_LIS3MDL](https://github.com/adafruit/Adafruit_LIS3MDL)
- [Adafruit_GPS](https://github.com/adafruit/Adafruit_GPS)
- [Adafruit_HTS221](https://github.com/adafruit/Adafruit_HTS221)
- [Adafruit_AHRS](https://github.com/adafruit/Adafruit_AHRS)
- [PWMServo](https://github.com/PaulStoffregen/PWMServo) - For Teensy servo control
- [ArduinoJson](https://arduinojson.org/)
- [Bounce2](https://github.com/thomasfredericks/Bounce2) - For button debouncing

## Getting Started

1. Install the Arduino IDE
2. Install the required libraries for your targeted version using the Library Manager
3. Select the appropriate board in the Arduino IDE:
   - For v2.5: ESP32-S3 Dev Module or ESP32-S3 Feather
   - For v2: ESP32-C3 Dev Module
   - For v1: Teensy 4.0
4. Connect your hardware according to the pinout defined in the respective configuration files
5. Configure the `arduino_secrets.h` file with your WiFi credentials (for v2.5)
6. Adjust configuration in `config.h` as needed (for v2.5)
7. Upload the code to your device

## Hardware Setup

For v2.5, the current recommended setup includes:
- ESP32-S3 TFT Feather as the main controller
- BNO086 IMU connected via SPI
- u-blox GPS module connected via UART
- ENS160 and SCD41 sensors on the I2C bus
- Servos connected to GPIO pins for faceplate control
- Optional NeoPixels or standard LEDs for helmet eyes

Refer to `config.h` for the default pin configuration and adjust as needed for your specific hardware.

## Configuration

The v2.5 version is highly configurable through the `config.h` file, allowing you to:

- Enable/disable features like MQTT or Socket connectivity
- Configure GPS interface
- Set up display parameters
- Configure servo angles for faceplate control
- Set up I2C pins and addresses

## Documentation

For detailed documentation of the code:
1. Review the header files in the project to understand the structure
2. Check the comments in each module for function-specific details
3. Refer to the library documentation links provided above

## License

This program is free software: you can redistribute it and/or modify it under the terms of the GNU General Public License as published by the Free Software Foundation, either version 3 of the License, or any later version.

## Contributing

By contributing to this project, you agree to license your contributions under the GPLv3 (or any later version) or any future licenses chosen by the project author(s).
