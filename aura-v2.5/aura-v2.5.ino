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

// I2C
#include <Wire.h>

// Common Includes
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "arduino_secrets.h"

// Logger
#include "logger.h"

// Component Headers
#include "gps_module.h"
#include "imu_module.h"
#include "enviro_module.h"
#include "network_utils.h"
#include "display_module.h"

// NeoPixel
#include <Adafruit_NeoPixel.h>

// I2C pins for ESP32-S3 TFT Feather
#define SDA_PIN 42
#define SCL_PIN 41

// I2C pins for ESP32-S3 Feather
//#define SDA_PIN 3
//#define SCL_PIN 4

// I2C pins for QT Py ESP32-S3
//#define SDA_PIN 41
//#define SCL_PIN 40

// NeoPixel
// PIN_NEOPIXEL and NEOPIXEL_POWER are defined
#define NUMPIXELS     1
Adafruit_NeoPixel pixels(NUMPIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

// WiFi
#define CONN_RETRY_ATTEMPTS  10

///////please enter your sensitive data in the Secret tab/arduino_secrets.h
char ssid[] = SECRET_SSID;    // your network SSID (name)
char pass[] = SECRET_PASS;    // your network password (use for WPA, or use as key for WEP)

WiFiClient wifiClient;

// Task handles
TaskHandle_t gpsTaskHandle;
TaskHandle_t imuTaskHandle;
TaskHandle_t enviroTaskHandle;
TaskHandle_t displayTaskHandle;
TaskHandle_t networkTaskHandle;

void setup() {
  unsigned long startTime = millis();
  int wifiRetries = 0;

  Serial.begin(115200);
  while (!Serial && (millis() - startTime < 2000)) { // Wait up to 2 seconds for Serial to connect
    delay(10);
  }

  // Initialize thread-safe logging
  logger_init();

  LOG_PRINTLN("AURA Initialized. Beginning power on sequence...");

  // NeoPixel
  pixels.begin();
  pixels.clear();
  
  // Initialize I2C with specified pins and speed
  if (!Wire.begin(SDA_PIN, SCL_PIN, 350000)) {
    LOG_PRINTLN("I2C initialization failed!");
  } else {
    LOG_PRINTLN("I2C initialization success!");
  }
  
  // Initialize the display
  setupDisplay();

  // Initialize components
  setupIMU();
  setupGPS();
  setupEnvironmental();

  pixels.setPixelColor(0, pixels.Color(255, 0, 0));
  pixels.show();

  // Initialize networking
  setupNetworking(ssid, pass, CONN_RETRY_ATTEMPTS, &wifiClient, &pixels);

#if 1
  // Create tasks
  if (xTaskCreatePinnedToCore(
      gpsTask,             // Task function
      "GPS Task",          // Name of the task
      10000,               // Stack size (in words)
      NULL,                // Task input parameter
      1,                   // Priority of the task
      &gpsTaskHandle,      // Task handle
      1                    // Core 1
  ) != pdPASS) {
    LOG_PRINTLN("Failed to create GPS Task");
  }
#endif
  if (xTaskCreatePinnedToCore(
      imuTask,             // Task function
      "IMU Task",          // Name of the task
      10000,               // Stack size (in words)
      NULL,                // Task input parameter
      1,                   // Priority of the task
      &imuTaskHandle,      // Task handle
      0                    // Core 0
  ) != pdPASS) {
    LOG_PRINTLN("Failed to create IMU Task");
  }

  if (xTaskCreatePinnedToCore(
      enviroTask,          // Task function
      "Environmental Task", // Name of the task
      10000,               // Stack size (in words)
      NULL,                // Task input parameter
      1,                   // Priority of the task
      &enviroTaskHandle,   // Task handle
      0                    // Core 0
  ) != pdPASS) {
    LOG_PRINTLN("Failed to create Environmental Task");
  }

  if (xTaskCreatePinnedToCore(
      displayTask,          // Task function
      "Display Task",       // Name of the task
      10000,                // Stack size (in words)
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &displayTaskHandle,   // Task handle
      0                     // Core 0
  ) != pdPASS) {
    LOG_PRINTLN("Failed to create Display Task");
  }

  if (xTaskCreatePinnedToCore(
      networkTask,          // Task function
      "Network Task",       // Name of the task
      10000,                // Stack size (in words)
      NULL,                 // Task input parameter
      1,                    // Priority of the task
      &networkTaskHandle,   // Task handle
      1                     // Core 1 - run on same core as environmental to separate from other tasks
  ) != pdPASS) {
      LOG_PRINTLN("Failed to create Network Task");
  }
}

void loop() {
    // Just monitor the network connection
    monitorConnection(&wifiClient, &pixels);
    
    // Use a small delay to prevent tight loop
    vTaskDelay(pdMS_TO_TICKS(100));
}
