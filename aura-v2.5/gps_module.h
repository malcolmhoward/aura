/*
 * This file is part of the OASIS Project.
 * https://github.com/orgs/The-OASIS-Project/
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 * By contributing to this project, you agree to license your contributions
 * under the GPLv3 (or any later version) or any future licenses chosen by
 * the project author(s). Contributions include any modifications,
 * enhancements, or additions to the project. These contributions become
 * part of the project and are adopted by the project author(s).
 */

#ifndef GPS_MODULE_H
#define GPS_MODULE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <SparkFun_u-blox_GNSS_Arduino_Library.h>

// Choose communication interface by uncommenting ONE of these lines:
#define GPS_USE_UART  // Use UART interface for GPS
//#define GPS_USE_I2C  // Use I2C interface for GPS

// Make sure exactly one interface is defined
#if defined(GPS_USE_UART) && defined(GPS_USE_I2C)
#error "Cannot define both GPS_USE_UART and GPS_USE_I2C. Choose one interface."
#elif !defined(GPS_USE_UART) && !defined(GPS_USE_I2C)
#error "Must define either GPS_USE_UART or GPS_USE_I2C for GPS communication."
#endif

// Configuration constants
#ifdef GPS_USE_UART
// UART configuration for Adafruit ESP32-S3 TFT Feather
#define GPS_UART_TX_PIN 1         // TX pin (GPIO1)
#define GPS_UART_RX_PIN 2         // RX pin (GPIO2)
#define GPS_RESET_PIN 5           // Reset pin on D5 (GPIO5)
#define GPS_BAUDRATE 38400        // Factory default u-blox baudrate
#define GPS_HIGH_BAUDRATE 115200  // High performance baudrate
#define GPS_SERIAL Serial1        // Use Serial1 for GPS communications
#endif

#ifdef GPS_USE_I2C
// I2C configuration
#define GPS_I2C_ADDR 0x42  // Default u-blox I2C address
// I2C pins are defined in the main sketch (SDA=42, SCL=41 for ESP32-S3 TFT Feather)
#endif

// Function prototypes
void setupGPS();
void attemptGPSReinitialization();
void gpsTask(void *pvParameters);

// External declarations
extern SFE_UBLOX_GNSS myGNSS;

#endif  // GPS_MODULE_H
