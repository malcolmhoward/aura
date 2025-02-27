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

#ifndef DISPLAY_MODULE_H
#define DISPLAY_MODULE_H

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <WiFi.h>

// TFT Display pins for ESP32-S3 TFT Feather
#define TFT_CS 7
#define TFT_DC 39
#define TFT_RST 40
#define TFT_BL 45
#define TFT_PWR 21

// Display dimensions
#define TFT_WIDTH 240
#define TFT_HEIGHT 135

// Display configuration
#define PAGE_CHANGE_INTERVAL 8000  // 8 seconds per page

// Anti-flickering refresh rate (to reduce screen clearing frequency)
#define DISPLAY_REFRESH_RATE_MS 500

// Enum for display pages
typedef enum {
  PAGE_IMU = 0,
  PAGE_GPS,
  PAGE_ENVIRO,
  PAGE_WIFI,
  PAGE_COUNT  // Always keep this as the last item
} display_page_t;

// Function prototypes
void setupDisplay(void);
void displayTask(void *pvParameters);
void updateDisplayData(void);
void displayIMUPage(void);
void displayGPSPage(void);
void displayEnviroPage(void);
void displayWiFiPage(void);
void resetPageDrawFlags(void);

// Struct for holding sensor data for display
typedef struct {
  // IMU data
  float roll;
  float pitch;
  float heading;
  bool imu_available;

  // GPS data
  char time[10];
  char date[12];
  int fix;
  float latitude;
  float longitude;
  int speed;
  int altitude;
  int satellites;
  bool gps_available;

  // Environmental data
  float temperature;
  float humidity;
  float pressure;
  float air_quality;
  char air_quality_description[15];
  bool temp_available;
  bool humidity_available;
  bool pressure_available;
  bool air_quality_available;

  // ENS160 specific data
  int eco2;  // Equivalent CO2 in ppm
  int tvoc;  // Total Volatile Organic Compounds in ppb
  bool ens160_available;

  // SCD41 CO2 sensor data
  uint16_t co2;        // CO2 concentration in ppm
  bool co2_available;
  char co2_quality_description[15];

  // WiFi data
  char ssid[33];
  int rssi;
  char ip_address[16];
  bool is_connected;
} display_data_t;

// Global variables to be exposed
extern SemaphoreHandle_t displayMutex;
extern display_data_t display_data;
extern int current_page;

// First-draw flags
extern bool imu_page_first_draw;
extern bool gps_page_first_draw;
extern bool enviro_page_first_draw;
extern bool wifi_page_first_draw;

#endif  // DISPLAY_MODULE_H