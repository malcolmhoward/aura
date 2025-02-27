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

#ifndef OASIS_LOGGER_H
#define OASIS_LOGGER_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/queue.h>
#include <WiFi.h>

// Message structure for the queue
typedef struct {
  String deviceType;
  JsonDocument* doc;
} message_t;

// Initialize the logging system
void logger_init(void);

// Basic logging functions
void logger_print(const char* message);
void logger_print(const String& message);
void logger_print(const __FlashStringHelper* message);

void logger_println(const char* message);
void logger_println(const String& message);
void logger_println(const __FlashStringHelper* message);

// JSON serialization and sending
void logger_send_json(JsonDocument* doc, const char* device_type);

#ifdef ENABLE_MQTT
#include <ArduinoMqttClient.h>
void logger_send_mqtt_json(JsonDocument* doc, const char* device_type,
                           MqttClient* mqttClient, const char* topic);
#endif

// Network task for handling message queue
void networkTask(void* pvParameters);

// Convenience macros
#define LOG_PRINT(x) logger_print(x)
#define LOG_PRINTLN(x) logger_println(x)

#endif  // OASIS_LOGGER_H
