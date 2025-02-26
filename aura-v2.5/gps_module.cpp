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

#include "gps_module.h"
#include "logger.h"
#include "display_module.h"

// External dependencies
extern WiFiClient wifiClient;

// GPS
SFE_UBLOX_GNSS myGNSS;

void setupGPS() {
  if (myGNSS.begin() == false) {
    LOG_PRINTLN(F("u-blox GNSS not detected at default I2C address."));
  } else {
    LOG_PRINTLN("GPS found!");
  }

  myGNSS.setI2COutput(COM_TYPE_UBX); //Set the I2C port to output only UBX messages
  LOG_PRINTLN("GPS initialized.");
}

// GPS task, pinned to Core 0, runs every 1 second
void gpsTask(void *pvParameters) {
  char time[9] = "00:00:00";
  char date[11] = "2024/01/01";
  int32_t altitude_mm = 0;
  int32_t speed_mmps = 0;
  float altitude_feet = 0.0f;
  float speed_mph = 0.0f;
  JsonDocument doc;

  LOG_PRINTLN(F("GPS task started."));

  while (true) {
    if (myGNSS.getTimeValid()) {
      snprintf(time, 9, "%02d:%02d:%02d", myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
    } else {
      LOG_PRINTLN("Time is not valid.");
    }
    if (myGNSS.getDateValid()) {
      snprintf(date, 11, "%04d/%02d/%02d", myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());
    } else {
      LOG_PRINTLN("Date is not valid.");
    }

    altitude_mm = myGNSS.getAltitudeMSL();
    altitude_feet = static_cast<float>(altitude_mm) / 304.8f;

    speed_mmps = myGNSS.getGroundSpeed();
    speed_mph = static_cast<float>(speed_mmps) * 0.00223694f;

    doc["device"] =     "GPS";
    doc["time"] =       time;
    doc["date"] =       date;
    doc["fix"] =        (int)myGNSS.getGnssFixOk();
    if (myGNSS.getGnssFixOk()) {
      doc["latitudeDegrees"] = round(myGNSS.getLatitude() / 10000000.0 * 1e6) / 1e6;
      doc["longitudeDegrees"] = round(myGNSS.getLongitude() / 10000000.0 * 1e6) / 1e6;
      doc["speed"] =      static_cast<int32_t>(speed_mph);
      doc["angle"] =      static_cast<int32_t>(round(myGNSS.getHeading() / 100000.0));
      doc["altitude"] =   static_cast<int32_t>(altitude_feet);
      doc["satellites"] = myGNSS.getSIV();
    }

    // Update display data
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      strncpy(display_data.time, time, sizeof(display_data.time));
      strncpy(display_data.date, date, sizeof(display_data.date));
      display_data.fix = myGNSS.getGnssFixOk();
      
      if (display_data.fix) {
        display_data.latitude = round(myGNSS.getLatitude() / 10000000.0 * 1e6) / 1e6;
        display_data.longitude = round(myGNSS.getLongitude() / 10000000.0 * 1e6) / 1e6;
        display_data.speed = static_cast<int32_t>(speed_mph);
        display_data.altitude = static_cast<int32_t>(altitude_feet);
        display_data.satellites = myGNSS.getSIV();
      }
      
      xSemaphoreGive(displayMutex);
    }

    // Thread-safe JSON sending
    #ifdef ENABLE_MQTT
      logger_send_mqtt_json(&doc, "GPS", &mqttClient, topic);
    #else
      logger_send_json(&doc, "GPS", &wifiClient);
    #endif

    doc.clear();

    vTaskDelay(pdMS_TO_TICKS(1000)); // Delay for 1 second
  }
}
