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

#include "logger.h"
#include <stdarg.h>

// Mutex for protecting serial output
static SemaphoreHandle_t serialMutex = NULL;

// Message queue for network communication
static QueueHandle_t messageQueue = NULL;

// External references
extern WiFiClient wifiClient;

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

// Initialize logging - call this before using any other logging functions
void logger_init(void) {
  // Create mutex for serial output
  serialMutex = xSemaphoreCreateMutex();
  if (serialMutex == NULL) {
    Serial.println("ERROR: Failed to create serialMutex");
  }

  // Create message queue for network communication
  messageQueue = xQueueCreate(20, sizeof(message_t*));  // Queue size increased to handle bursts
  if (messageQueue == NULL) {
    Serial.println("ERROR: Failed to create messageQueue");
  }
}

// Thread-safe version of Serial.print for char*
void logger_print(const char* message) {
  if (serialMutex == NULL) return;  // Safety check

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.print(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Thread-safe version of Serial.print for String
void logger_print(const String& message) {
  if (serialMutex == NULL) return;

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.print(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Thread-safe version of Serial.print for FlashStringHelper
void logger_print(const __FlashStringHelper* message) {
  if (serialMutex == NULL) return;

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.print(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Thread-safe version of Serial.println for char*
void logger_println(const char* message) {
  if (serialMutex == NULL) return;  // Safety check

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.println(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Thread-safe version of Serial.println for String
void logger_println(const String& message) {
  if (serialMutex == NULL) return;

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.println(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Thread-safe version of Serial.println for FlashStringHelper
void logger_println(const __FlashStringHelper* message) {
  if (serialMutex == NULL) return;

  if (xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      Serial.println(message);
    }
    xSemaphoreGive(serialMutex);
  }
}

// Queue a message for the network task to handle
static bool queueMessage(const char* device_type, JsonDocument* doc) {
  if (messageQueue == NULL) return false;

  // Create a new message with a copy of the document
  message_t* msg = new message_t;
  if (msg == NULL) return false;

  msg->deviceType = String(device_type);
  msg->doc = new JsonDocument(*doc);  // Create a deep copy of the document

  if (msg->doc == NULL) {
    delete msg;
    return false;
  }

  // Send to queue, wait up to 10ms if queue is full
  if (xQueueSend(messageQueue, &msg, pdMS_TO_TICKS(10)) != pdPASS) {
    // Queue is full or error occurred
    delete msg->doc;
    delete msg;
    return false;
  }

  return true;
}

// Thread-safe JSON serialization and sending
void logger_send_json(JsonDocument* doc, const char* device_type) {
  // Log to serial with mutex protection
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      serializeJson(*doc, Serial);
      Serial.println();
    }
    xSemaphoreGive(serialMutex);
  }

  // Queue the message for network transmission
  queueMessage(device_type, doc);
}

#ifdef ENABLE_MQTT
void logger_send_mqtt_json(JsonDocument* doc, const char* device_type,
                           MqttClient* mqttClient, const char* topic) {
  // Log to serial with mutex protection
  if (serialMutex != NULL && xSemaphoreTake(serialMutex, portMAX_DELAY) == pdTRUE) {
    if (Serial) {
      serializeJson(*doc, Serial);
      Serial.println();
    }
    xSemaphoreGive(serialMutex);
  }

  // Queue the message for network transmission
  queueMessage(device_type, doc);
}
#endif

// Network task that processes the message queue
void networkTask(void* pvParameters) {
  LOG_PRINTLN(F("Network task started."));

  while (true) {
    message_t* msg;

    // Wait for a message from the queue
    if (xQueueReceive(messageQueue, &msg, pdMS_TO_TICKS(100)) == pdPASS) {
      if (msg != NULL && msg->doc != NULL) {
        char output_json[512];
        size_t len = serializeJson(*(msg->doc), output_json, sizeof(output_json));

#ifdef ENABLE_MQTT
        // Send via MQTT if enabled
        if (mqttClient.connected()) {
          mqttClient.beginMessage(topic);
          mqttClient.print(output_json);
          mqttClient.endMessage();
        }
#else
        // Send via TCP socket if MQTT not enabled
        if (wifiClient.connected()) {
          wifiClient.write(output_json, len);
          wifiClient.write("\n", 1);  // Add newline for better readability
        }
#endif

        // Clean up
        delete msg->doc;
        delete msg;
      }
    }

    // Small delay to prevent CPU hogging
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}
