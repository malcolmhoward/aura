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

#include "imu_module.h"
#include "logger.h"
#include "display_module.h"

// External dependencies
extern WiFiClient wifiClient;

// IMU/AHRS
Adafruit_ICM20948 icm;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer, *temperature;
#define FILTER_UPDATE_RATE_HZ 100

// Pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;      // faster than NXP
//Adafruit_Mahony filter;        // fastest/smalleset

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

void setupIMU() {
  // IMU
  if (!icm.begin_I2C()) {
    LOG_PRINTLN("Failed to find ICM20948 chip");
    while (1) {
      delay(10);
    }
  }

  LOG_PRINTLN("ICM20948 Found!");
  temperature = icm.getTemperatureSensor();
  temperature->printSensorDetails();

  accelerometer = icm.getAccelerometerSensor();
  accelerometer->printSensorDetails();

  gyroscope = icm.getGyroSensor();
  gyroscope->printSensorDetails();

  magnetometer = icm.getMagnetometerSensor();
  magnetometer->printSensorDetails();

  filter.begin(FILTER_UPDATE_RATE_HZ);
}

// IMU task, pinned to Core 0, runs every 100 ms
void imuTask(void *pvParameters) {
  sensors_event_t accel, gyro, mag, temp;
  float roll, pitch, heading;
  float gx, gy, gz;
  JsonDocument doc;

  LOG_PRINTLN(F("IMU task started."));

  while (true) {
    /* Read the motion sensors */
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);
    temperature->getEvent(&temp);
#if defined(AHRS_DEBUG_OUTPUT)
    LOG_PRINT("I2C took "); LOG_PRINT(millis()-timestamp); LOG_PRINTLN(" ms");
#endif

    /* Calculate motion information */
#if 0
    cal.calibrate(mag);
    cal.calibrate(accel);
    cal.calibrate(gyro);
#endif
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz, 
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z, 
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);
#if defined(AHRS_DEBUG_OUTPUT)
    LOG_PRINT("Update took "); LOG_PRINT(millis()-timestamp); LOG_PRINTLN(" ms");
#endif

    // get the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();

    doc["device"] = "Motion";
    doc["format"] = "Orientation";
    // The following will remap the compass in case it's backwards.
    //doc["heading"] = map(heading, 0, 360, 360, 0);
    doc["heading"] = heading;
    doc["pitch"] = -1.0f * pitch;
    doc["roll"] = roll;

    // Update display data
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.roll = roll;
      display_data.pitch = pitch;
      display_data.heading = heading;
      
      xSemaphoreGive(displayMutex);
    }

    // Thread-safe JSON sending
    #ifdef ENABLE_MQTT
      logger_send_mqtt_json(&doc, "Motion", &mqttClient, topic);
    #else
      logger_send_json(&doc, "Motion", &wifiClient);
    #endif

    doc.clear();

    vTaskDelay(pdMS_TO_TICKS(1000 / FILTER_UPDATE_RATE_HZ)); // Delay for 100 ms
  }
}
