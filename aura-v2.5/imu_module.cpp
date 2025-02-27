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

#ifdef USE_ICM20948
// ICM20948 IMU/AHRS
Adafruit_ICM20948 icm;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer, *temperature;

// Pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter;  // slowest
//Adafruit_Madgwick filter;      // faster than NXP
//Adafruit_Mahony filter;        // fastest/smalleset
#endif

#ifdef USE_BNO086
// BNO086 IMU using SparkFun library
BNO08x myIMU;
#endif

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

void setupIMU() {
#ifdef USE_ICM20948
  // ICM20948
  if (!icm.begin_I2C()) {
    LOG_PRINTLN("Failed to find ICM20948 chip - IMU functionality disabled");

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = false;
      xSemaphoreGive(displayMutex);
    }
  } else {
    LOG_PRINTLN("ICM20948 Found!");

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = true;
      xSemaphoreGive(displayMutex);
    }

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
#endif

#ifdef USE_BNO086
  // Initialize BNO086 with the SparkFun library
  // This approach uses the hardware INT and RST pins for the most reliable operation
  if (myIMU.begin(0x4B, Wire, BNO086_INT, BNO086_RST) == false) {
    LOG_PRINTLN("Failed to find BNO08x chip - IMU functionality disabled");

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = false;
      xSemaphoreGive(displayMutex);
    }
  } else {
    LOG_PRINTLN("BNO08x Found!");

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = true;
      xSemaphoreGive(displayMutex);
    }

    // Set up optimal calibration for head tracking
    myIMU.setCalibrationConfig(SH2_CAL_ACCEL);

    // Use Gyro Integrated Rotation Vector instead of standard Rotation Vector
    // This provides lower latency and is better suited for AR/VR applications
    // The value 10 means a report interval of 10ms (100Hz)
    if (myIMU.enableGyroIntegratedRotationVector(10) == true) {
      LOG_PRINTLN("Gyro Integrated Rotation Vector enabled for AR headset tracking");
    } else {
      // Fallback to AR/VR stabilized rotation vector if Gyro Integrated isn't available
      if (myIMU.enableARVRStabilizedRotationVector(10) == true) {
        LOG_PRINTLN("AR/VR Stabilized Rotation Vector enabled for AR headset tracking");
      } else {
        // Final fallback to standard rotation vector
        if (myIMU.enableRotationVector(10) == true) {
          LOG_PRINTLN("Standard Rotation Vector enabled (AR optimized features unavailable)");
        } else {
          LOG_PRINTLN("Could not enable any rotation vector");
        }
      }
    }
  }
#endif
}

void attemptIMUReinitialization() {
  // Check if already available
  bool currently_available = false;

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currently_available = display_data.imu_available;
    xSemaphoreGive(displayMutex);
  }

  if (currently_available) return;  // Already working, no need to reinitialize

  LOG_PRINTLN("Attempting to reinitialize IMU...");

#ifdef USE_ICM20948
  if (!icm.begin_I2C()) {
    LOG_PRINTLN("IMU reinitialization failed");
    return;
  }

  temperature = icm.getTemperatureSensor();
  accelerometer = icm.getAccelerometerSensor();
  gyroscope = icm.getGyroSensor();
  magnetometer = icm.getMagnetometerSensor();
  filter.begin(FILTER_UPDATE_RATE_HZ);

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.imu_available = true;
    xSemaphoreGive(displayMutex);
  }

  LOG_PRINTLN("IMU successfully reinitialized!");
#endif

#ifdef USE_BNO086
  if (myIMU.begin(0x4B, Wire, BNO086_INT, BNO086_RST) == false) {
    LOG_PRINTLN("IMU reinitialization failed");
    return;
  }

  // Set up optimal calibration for head tracking
  myIMU.setCalibrationConfig(SH2_CAL_ACCEL);

  // Enable the optimal rotation vector for AR applications
  // Try each option in priority order
  if (!myIMU.enableGyroIntegratedRotationVector(10)) {
    if (!myIMU.enableARVRStabilizedRotationVector(10)) {
      myIMU.enableRotationVector(10);
    }
  }

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.imu_available = true;
    xSemaphoreGive(displayMutex);
  }

  LOG_PRINTLN("IMU successfully reinitialized!");
#endif
}

// IMU task, pinned to Core 0, runs at the filter update rate
void imuTask(void* pvParameters) {
#ifdef USE_ICM20948
  sensors_event_t accel, gyro, mag, temp;
  float roll, pitch, heading;
  float gx, gy, gz;
#endif

#ifdef USE_BNO086
  float roll = 0, pitch = 0, heading = 0;
  static float prevHeading = -999.0f;
  static float prevPitch = -999.0f;
  static float prevRoll = -999.0f;
  static int stuckValueCount = 0;
#endif

  JsonDocument doc;
  unsigned long last_reinit_attempt = 0;
  const unsigned long REINIT_INTERVAL = 60000;  // Try to reinitialize every minute
  bool is_available = false;

  LOG_PRINTLN(F("IMU task started."));

  while (true) {
    // Check if IMU is available
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      is_available = display_data.imu_available;
      xSemaphoreGive(displayMutex);
    }

    if (!is_available) {
      // Try to reinitialize periodically
      unsigned long now = millis();
      if (now - last_reinit_attempt > REINIT_INTERVAL) {
        attemptIMUReinitialization();
        last_reinit_attempt = now;
      }

      vTaskDelay(pdMS_TO_TICKS(1000));  // Sleep longer when IMU not available
      continue;
    }

#ifdef USE_ICM20948
    /* Read the motion sensors */
    accelerometer->getEvent(&accel);
    gyroscope->getEvent(&gyro);
    magnetometer->getEvent(&mag);
    temperature->getEvent(&temp);

    /* Calculate motion information */
    // Gyroscope needs to be converted from Rad/s to Degree/s
    // the rest are not unit-important
    gx = gyro.gyro.x * SENSORS_RADS_TO_DPS;
    gy = gyro.gyro.y * SENSORS_RADS_TO_DPS;
    gz = gyro.gyro.z * SENSORS_RADS_TO_DPS;

    // Update the SensorFusion filter
    filter.update(gx, gy, gz,
                  accel.acceleration.x, accel.acceleration.y, accel.acceleration.z,
                  mag.magnetic.x, mag.magnetic.y, mag.magnetic.z);

    // get the heading, pitch and roll
    roll = filter.getRoll();
    pitch = filter.getPitch();
    heading = filter.getYaw();
#endif

#ifdef USE_BNO086
    // Handle any resets
    if (myIMU.wasReset()) {
      LOG_PRINTLN("BNO08x was reset - re-enabling optimized rotation vector");
      // Try in priority order
      if (!myIMU.enableGyroIntegratedRotationVector(10)) {
        if (!myIMU.enableARVRStabilizedRotationVector(10)) {
          myIMU.enableRotationVector(10);
        }
      }
    }

    // Check if we have a new sensor event
    if (myIMU.getSensorEvent() == true) {
      // Make sure it's one of our desired sensor types
      uint8_t sensorId = myIMU.getSensorEventID();
      if (sensorId == SENSOR_REPORTID_ROTATION_VECTOR || sensorId == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR || sensorId == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR) {

        // Get orientation data - all rotation vector types use the same methods
        roll = (myIMU.getRoll()) * 180.0 / PI;    // Convert roll to degrees
        pitch = (myIMU.getPitch()) * 180.0 / PI;  // Convert pitch to degrees
        heading = (myIMU.getYaw()) * 180.0 / PI;  // Convert yaw / heading to degrees

        // Additional data available for Gyro Integrated RV (not used yet)
        /*
        if (sensorId == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR) {
          float angVelX = myIMU.getGyroIntegratedRVangVelX();
          float angVelY = myIMU.getGyroIntegratedRVangVelY();
          float angVelZ = myIMU.getGyroIntegratedRVangVelZ();
          
          // These could be used for motion prediction or effects
        }
        */

        // Check for stuck default values
        if (heading == 90.0f && pitch == 0.0f && roll == 90.0f) {
          stuckValueCount++;
          if (stuckValueCount > 5) {
            LOG_PRINTLN("BNO08x stuck at default values. Resetting...");
            myIMU.softReset();
            stuckValueCount = 0;

            // Set IMU as unavailable to trigger reinitialization
            if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
              display_data.imu_available = false;
              xSemaphoreGive(displayMutex);
            }
            continue;
          }
        } else {
          stuckValueCount = 0;
        }

        // Only process data if it's not stuck in default values or values have changed
        if (stuckValueCount < 5 && (heading != prevHeading || pitch != prevPitch || roll != prevRoll)) {
          // Prepare JSON data
          doc["device"] = "Motion";
          doc["format"] = "Orientation";
          doc["heading"] = heading;
          doc["pitch"] = -1.0f * pitch;  // Keep consistent with original sign convention
          doc["roll"] = roll;

          // Update display data
          if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display_data.roll = roll;
            display_data.pitch = pitch;
            display_data.heading = heading;

            xSemaphoreGive(displayMutex);
          }

// Send JSON data
#ifdef ENABLE_MQTT
          logger_send_mqtt_json(&doc, "Motion", &mqttClient, topic);
#else
          logger_send_json(&doc, "Motion");
#endif

          doc.clear();

          // Update previous values
          prevHeading = heading;
          prevPitch = pitch;
          prevRoll = roll;
        }
      }
    }

    vTaskDelay(pdMS_TO_TICKS(10));  // Short delay to allow other tasks to run
#endif

#ifdef USE_ICM20948
    // Prepare to send motion data
    doc["device"] = "Motion";
    doc["format"] = "Orientation";
    doc["heading"] = heading;
    doc["pitch"] = -1.0f * pitch;  // Keep consistent with the original sign inversion
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
    logger_send_json(&doc, "Motion");
#endif

    doc.clear();

    vTaskDelay(pdMS_TO_TICKS(1000 / FILTER_UPDATE_RATE_HZ));  // Delay at filter update rate
#endif
  }
}