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
#include <ScioSense_ENS160.h>   // ScioSense ENS160 library
#include <SensirionI2cScd4x.h>  // Sensirion SCD4x library for SCD41 CO2 sensor
#include "config.h"

// CO2 thresholds (in ppm)
#define CO2_EXCELLENT 600
#define CO2_GOOD 800
#define CO2_FAIR 1000
#define CO2_POOR 1500
// Above 1500 ppm is considered "very poor"

#ifdef ENABLE_MQTT
#include <ArduinoMqttClient.h>
#endif

// Function prototypes
void setupEnvironmental();
void attemptEnviroReinitialization();
void enviroTask(void* pvParameters);
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max);

// Get a CO2 quality description based on ppm value
const char* getCO2QualityDescription(uint16_t co2_ppm);

#endif  // ENVIRO_MODULE_H
