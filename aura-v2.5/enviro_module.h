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

#ifndef ENVIRO_MODULE_H
#define ENVIRO_MODULE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <Adafruit_Sensor.h>

// Uncomment only one of these to select your sensor
#define USE_BME680
//#define USE_ENS160_AHT21

// Compile-time check to ensure only one sensor is selected
#if defined(USE_BME680) && defined(USE_ENS160_AHT21)
    #error "Only one environmental sensor can be enabled at a time. Please choose either USE_BME680 or USE_ENS160_AHT21, not both."
#endif

#if !defined(USE_BME680) && !defined(USE_ENS160_AHT21)
    #error "No environmental sensor selected. Please enable either USE_BME680 or USE_ENS160_AHT21."
#endif

#ifdef USE_BME680
#include <Adafruit_BME680.h>
#endif

#ifdef USE_ENS160_AHT21
#include <Adafruit_AHTX0.h> // Adafruit AHT20/AHT21 library
#include <ScioSense_ENS160.h> // ScioSense ENS160 library

// Calibration values for AHT21
#define AHT21_TEMP_OFFSET 0.0f  // Calibration offset in degrees C (adjust as needed)
#define AHT21_HUM_OFFSET 0.0f    // Calibration offset for humidity (adjust as needed)
#endif

#ifdef ENABLE_MQTT
#include <ArduinoMqttClient.h>
#endif

// Function prototypes
void setupEnvironmental();
void enviroTask(void *pvParameters);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

#ifdef USE_ENS160_AHT21
// Calibration functions for AHT21
void setAHT21TempOffset(float offset);
void setAHT21HumOffset(float offset);
float getAHT21TempOffset();
float getAHT21HumOffset();
#endif

#endif // ENVIRO_MODULE_H
