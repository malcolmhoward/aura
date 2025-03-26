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

#ifndef IMU_MODULE_H
#define IMU_MODULE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_Sensor.h>
#include <SPI.h>
#include <SparkFun_BNO08x_Arduino_Library.h>  // Using the SparkFun library
#include "config.h"


#ifdef ENABLE_MQTT
#include <ArduinoMqttClient.h>
#endif

// BNO086 SPI interface pins
#define BNO086_INT 6    // INT pin on D6 (GPIO6)
#define BNO086_RST 9    // RST pin on D9 (GPIO9)
#define BNO086_WAKE 10  // WAKE pin (PS0) on D10 (GPIO10)

// Build it myself secondary SPI
#define BNO086_SCK A0   // SCK pin on A0 (GPIO18)
#define BNO086_MISO A1  // MISO pin on A1 (GPIO17)
#define BNO086_MOSI A2  // MOSI pin on A2 (GPIO16)
#define BNO086_CS A3    // CS pin on A3 (GPIO15)

// Reset Holds
#define RESET_HOLD 15   // Time to hold reset line (10ms min)
#define RESET_WAIT 150  // Time to wait after reset  (90ms + 4ms min)

// BNO086 SPI configuration
#define BNO086_SPI_HIGH_SPEED 2000000  // 2MHz for dedicated SPI bus
#define BNO086_SPI_LOW_SPEED 500000    // 500KHz for dedicated SPI bus

// Function prototypes
void setupIMU();
void attemptIMUReinitialization();
void imuTask(void *pvParameters);

#endif  // IMU_MODULE_H
