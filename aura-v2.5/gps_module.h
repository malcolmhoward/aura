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
#include "config.h"

// Make sure exactly one interface is defined
#if defined(GPS_USE_UART) && defined(GPS_USE_I2C)
#error "Cannot define both GPS_USE_UART and GPS_USE_I2C. Choose one interface."
#elif !defined(GPS_USE_UART) && !defined(GPS_USE_I2C)
#error "Must define either GPS_USE_UART or GPS_USE_I2C for GPS communication."
#endif

// Function prototypes
void setupGPS();
void attemptGPSReinitialization();
void gpsTask(void *pvParameters);

// External declarations
extern SFE_UBLOX_GNSS myGNSS;

#endif  // GPS_MODULE_H
