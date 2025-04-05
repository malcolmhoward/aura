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

// GPS
SFE_UBLOX_GNSS myGNSS;

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

void setupGPS() {
#ifdef GPS_USE_UART
  // Initialize reset pin
  pinMode(GPS_RESET_PIN, OUTPUT);
  digitalWrite(GPS_RESET_PIN, HIGH);  // Keep module out of reset

  // Reset the GPS module
  LOG_PRINTLN(F("Resetting GPS module..."));
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(100);  // Hold in reset for 100ms
  digitalWrite(GPS_RESET_PIN, HIGH);
  delay(500);  // Give it time to start up after reset

  // Initialize UART for GPS module with factory default baud rate
  LOG_PRINTLN(F("Initializing GPS with UART interface at 38400 baud..."));

  // Based on the Adafruit ESP32-S3 TFT Feather pinout, RX is GPIO2 and TX is GPIO1
  GPS_SERIAL.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);

  // Short delay to ensure serial port is ready
  delay(100);

  // Clear any pending data
  while (GPS_SERIAL.available()) {
    GPS_SERIAL.read();
  }

  // Try to connect with a longer timeout to account for the reset
  if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
    LOG_PRINTLN(F("u-blox GNSS not detected over UART - GPS functionality disabled"));

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.gps_available = false;
      xSemaphoreGive(displayMutex);
    }
  } else {
    LOG_PRINTLN(F("GPS found on UART!"));

    // Change the module's baud rate to the higher speed
    LOG_PRINTLN(F("Upgrading GPS baud rate to 115200..."));

    // First, configure the module to use the new baud rate
    // Note: setSerialRate returns void, so we just call it without checking return value
    myGNSS.setSerialRate(GPS_HIGH_BAUDRATE, COM_PORT_UART1);
    LOG_PRINTLN(F("GPS baud rate change command sent"));

    // Now we need to update our serial port to match
    GPS_SERIAL.end();
    delay(100);
    GPS_SERIAL.begin(GPS_HIGH_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    delay(100);

    // Clear any pending data
    while (GPS_SERIAL.available()) {
      GPS_SERIAL.read();
    }

    // Reconnect at new baudrate
    if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
      LOG_PRINTLN(F("Warning: Failed to reconnect at 115200 baud, reverting to 38400"));
      GPS_SERIAL.end();
      delay(100);
      GPS_SERIAL.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
      delay(100);

      // Try one more time with the original baud rate
      if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
        LOG_PRINTLN(F("GPS reconnection failed completely"));

        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          display_data.gps_available = false;
          xSemaphoreGive(displayMutex);
        }
        return;
      }
    } else {
      LOG_PRINTLN(F("Successfully connected at 115200 baud"));
    }

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.gps_available = true;
      xSemaphoreGive(displayMutex);
    }

    // Configure the module
    myGNSS.setUART1Output(COM_TYPE_UBX);  // Set the UART port to output only UBX messages (no NMEA)

    // Set navigation frequency to 5Hz - higher baudrate allows more frequent updates
    myGNSS.setNavigationFrequency(5);

    // Enable useful messages
    myGNSS.setAutoPVT(true);

    LOG_PRINTLN(F("GPS initialized over UART."));
  }
#endif

#ifdef GPS_USE_I2C
  // Initialize I2C for GPS module
  LOG_PRINTLN(F("Initializing GPS with I2C interface..."));
  if (myGNSS.begin() == false) {
    LOG_PRINTLN(F("u-blox GNSS not detected at default I2C address - GPS functionality disabled"));

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.gps_available = false;
      xSemaphoreGive(displayMutex);
    }
  } else {
    LOG_PRINTLN(F("GPS found on I2C!"));

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.gps_available = true;
      xSemaphoreGive(displayMutex);
    }

    myGNSS.setI2COutput(COM_TYPE_UBX);  // Set the I2C port to output only UBX messages
    myGNSS.setNavigationFrequency(5);   // Set navigation frequency to 5Hz for better responsiveness
    myGNSS.setAutoPVT(true);            // Enable automatic PVT messages

    LOG_PRINTLN(F("GPS initialized over I2C."));
  }
#endif
}

void attemptGPSReinitialization() {
  // Check if already available
  bool currently_available = false;

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currently_available = display_data.gps_available;
    xSemaphoreGive(displayMutex);
  }

  if (currently_available) return;  // Already working, no need to reinitialize

#ifdef GPS_USE_UART
  LOG_PRINTLN(F("Attempting to reinitialize GPS over UART..."));

  // Hard reset the module first
  digitalWrite(GPS_RESET_PIN, LOW);
  delay(100);
  digitalWrite(GPS_RESET_PIN, HIGH);
  delay(500);  // Give it time to start up after reset

  // Try reconnecting with the high baudrate first (which should be saved in flash)
  GPS_SERIAL.end();
  delay(100);
  GPS_SERIAL.begin(GPS_HIGH_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
  delay(100);

  // Clear any pending data
  while (GPS_SERIAL.available())
    GPS_SERIAL.read();

  LOG_PRINTLN(F("Trying to reconnect at 115200 baud..."));

  // Try to connect to the GPS module
  if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
    // If high baudrate fails, try the factory default
    LOG_PRINTLN(F("Failed at 115200, trying 38400 baud..."));

    GPS_SERIAL.end();
    delay(100);
    GPS_SERIAL.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
    delay(100);

    // Clear any pending data
    while (GPS_SERIAL.available())
      GPS_SERIAL.read();

    if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
      LOG_PRINTLN(F("GPS UART reinitialization failed at both baudrates"));
      return;
    } else {
      // If we successfully connect at 38400, try to upgrade to 115200 again
      LOG_PRINTLN(F("Connected at 38400, upgrading to 115200..."));

      // First, configure the module to use the new baud rate - setSerialRate() returns void
      myGNSS.setSerialRate(GPS_HIGH_BAUDRATE, COM_PORT_UART1);
      LOG_PRINTLN(F("GPS baud rate change command sent"));

      // Save the configuration
      if (myGNSS.saveConfiguration()) {
        LOG_PRINTLN(F("Configuration saved successfully"));
      } else {
        LOG_PRINTLN(F("Warning: Failed to save configuration"));
      }

      // Update our serial port to match
      GPS_SERIAL.end();
      delay(100);
      GPS_SERIAL.begin(GPS_HIGH_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
      delay(100);

      // Reconnect at new baudrate
      if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
        // If reconnection at high baudrate fails, fall back to default
        LOG_PRINTLN(F("Failed to reconnect at 115200, reverting to 38400"));
        GPS_SERIAL.end();
        delay(100);
        GPS_SERIAL.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_UART_RX_PIN, GPS_UART_TX_PIN);
        delay(100);

        if (myGNSS.begin(GPS_SERIAL, 2000) == false) {
          LOG_PRINTLN(F("GPS reconnection failed completely"));
          return;
        }
      } else {
        LOG_PRINTLN(F("Successfully reconnected at 115200 baud"));
      }
    }
  }

  // Configure the module
  myGNSS.setUART1Output(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(5);
  myGNSS.setAutoPVT(true);

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.gps_available = true;
    xSemaphoreGive(displayMutex);
  }

  LOG_PRINTLN(F("GPS successfully reinitialized over UART!"));
#endif

#ifdef GPS_USE_I2C
  LOG_PRINTLN(F("Attempting to reinitialize GPS over I2C..."));

  if (myGNSS.begin() == false) {
    LOG_PRINTLN(F("GPS I2C reinitialization failed"));
    return;
  }

  myGNSS.setI2COutput(COM_TYPE_UBX);
  myGNSS.setNavigationFrequency(5);
  myGNSS.setAutoPVT(true);

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.gps_available = true;
    xSemaphoreGive(displayMutex);
  }

  LOG_PRINTLN(F("GPS successfully reinitialized over I2C!"));
#endif
}

// GPS task, pinned to Core 0, runs every 1 second
void gpsTask(void *pvParameters) {
  char time[12] = "00:00:00";
  char date[14] = "2024/01/01";
  int32_t altitude_mm = 0;
  int32_t speed_mmps = 0;
  float altitude_feet = 0.0f;
  float speed_mph = 0.0f;
  JsonDocument doc;

  unsigned long last_reinit_attempt = 0;
  const unsigned long REINIT_INTERVAL = 60000;  // Try to reinitialize every minute
  bool is_available = false;

  // Start with longer duration between checks
  int checkDelay = 50;  // 50ms between checks initially (20Hz)

#ifdef GPS_USE_UART
  LOG_PRINTLN(F("GPS task started with UART communication."));
#endif

#ifdef GPS_USE_I2C
  LOG_PRINTLN(F("GPS task started with I2C communication."));
#endif

  while (true) {
    // Check if GPS is available
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      is_available = display_data.gps_available;
      xSemaphoreGive(displayMutex);
    }

    if (!is_available) {
      // Try to reinitialize periodically
      unsigned long now = millis();
      if (now - last_reinit_attempt > REINIT_INTERVAL) {
        attemptGPSReinitialization();
        last_reinit_attempt = now;
      }

      vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep longer when GPS not available
      continue;
    }

    // Check for new data from the GPS
    myGNSS.checkUblox();
    myGNSS.checkCallbacks();

    // Only proceed if we have a new PVT message
    if (myGNSS.getPVT()) {
      // Decrease delay for checks since we're receiving data
      checkDelay = 10;  // 10ms between checks (100Hz polling for 5Hz data)

      if (myGNSS.getTimeValid()) {
        snprintf(time, 12, "%02d:%02d:%02d", myGNSS.getHour(), myGNSS.getMinute(), myGNSS.getSecond());
      }

      if (myGNSS.getDateValid()) {
        snprintf(date, 14, "%04d/%02d/%02d", myGNSS.getYear(), myGNSS.getMonth(), myGNSS.getDay());
      }

      altitude_mm = myGNSS.getAltitudeMSL();
      altitude_feet = static_cast<float>(altitude_mm) / 304.8f;

      speed_mmps = myGNSS.getGroundSpeed();
      speed_mph = static_cast<float>(speed_mmps) * 0.00223694f;

      doc["device"] = "GPS";
      doc["time"] = time;
      doc["date"] = date;
      doc["fix"] = (int)myGNSS.getGnssFixOk();
      if (myGNSS.getGnssFixOk()) {
        doc["latitudeDegrees"] = round(myGNSS.getLatitude() / 10000000.0 * 1e6) / 1e6;
        doc["longitudeDegrees"] = round(myGNSS.getLongitude() / 10000000.0 * 1e6) / 1e6;
        doc["speed"] = static_cast<int32_t>(speed_mph);
        doc["angle"] = static_cast<int32_t>(round(myGNSS.getHeading() / 100000.0));
        doc["altitude"] = static_cast<int32_t>(altitude_feet);
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
      logger_send_json(&doc, "GPS");
#endif

      doc.clear();
    }

    vTaskDelay(pdMS_TO_TICKS(checkDelay));  // Adjust the polling rate based on data reception
  }
}
