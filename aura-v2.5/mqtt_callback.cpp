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

#include <ArduinoJson.h>
#include "network_utils.h"
#include "logger.h"
#include "faceplate_module.h"
#include "config.h"

#ifdef ENABLE_MQTT

// MQTT message callback function
void onMqttMessageReceived(int messageSize) {
  // Only process if the message isn't too large
  if (messageSize > 512) {
    LOG_PRINTLN(F("Received message is too large to process"));
    // Skip the message
    while (mqttClient.available()) {
      mqttClient.read();
    }
    return;
  }

  // Create a buffer for the message
  char message[messageSize + 1];
  int i = 0;

  // Read the message
  while (mqttClient.available() && i < messageSize) {
    message[i++] = (char)mqttClient.read();
  }
  message[i] = '\0';  // Null terminate

  // Log the received message
  LOG_PRINT(F("Received MQTT message: "));
  LOG_PRINTLN(message);

  // Parse the JSON
  JsonDocument doc;
  DeserializationError error = deserializeJson(doc, message);

  if (error) {
    LOG_PRINT(F("JSON parsing failed: "));
    LOG_PRINTLN(error.c_str());
    return;
  }

  // Check if this is a command for our device
  if (!doc.containsKey("device") || strcmp(doc["device"], "helmet") != 0) {
    // Not for us or missing device field
    return;
  }

  // Check for faceplate action
  if (doc.containsKey("action") && strcmp(doc["action"], "faceplate") == 0) {
    if (doc.containsKey("value")) {
      // Get the value
      const char* value = doc["value"];

      LOG_PRINT(F("Faceplate command received: "));
      LOG_PRINTLN(value);

      // Act based on the value
      if (strcmp(value, "open") == 0) {
        // Open faceplate command
        LOG_PRINTLN(F("MQTT Command: Opening faceplate"));
        openFaceplate();
      } else if (strcmp(value, "close") == 0) {
        // Close faceplate command
        LOG_PRINTLN(F("MQTT Command: Closing faceplate"));
        closeFaceplate();
      } else if (strcmp(value, "toggle") == 0) {
        // Toggle faceplate command - reuse existing function
        LOG_PRINTLN(F("MQTT Command: Toggling faceplate"));
        toggleFaceplateState();
      } else {
        LOG_PRINT(F("Unknown faceplate value: "));
        LOG_PRINTLN(value);
      }
    }
  }
}

#endif  // ENABLE_MQTT
