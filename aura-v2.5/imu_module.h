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

// Uncomment only one of these to select your IMU sensor
//#define USE_ICM20948
#define USE_BNO086

// Compile-time check to ensure only one sensor is selected
#if defined(USE_ICM20948) && defined(USE_BNO086)
#error "Only one IMU sensor can be enabled at a time. Please choose either USE_ICM20948 or USE_BNO086, not both."
#endif

#if !defined(USE_ICM20948) && !defined(USE_BNO086)
#error "No IMU sensor selected. Please enable either USE_ICM20948 or USE_BNO086."
#endif

// Include the necessary headers based on selection
#ifdef USE_ICM20948
#include <Adafruit_ICM20948.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_AHRS.h>
#endif

#ifdef USE_BNO086
#include "SparkFun_BNO08x_Arduino_Library.h"  // Using the SparkFun library
#endif

#ifdef ENABLE_MQTT
#include <ArduinoMqttClient.h>
#endif

// IMU Pins for BNO086
#define BNO086_INT 2  // INT pin (RX pin on ESP32-S3)
#define BNO086_RST 1  // RST pin (TX pin on ESP32-S3)

// Filter update rate
#define FILTER_UPDATE_RATE_HZ 100

// Function prototypes
void setupIMU();
void imuTask(void *pvParameters);

#endif  // IMU_MODULE_H