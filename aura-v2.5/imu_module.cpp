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

// Let's build a secondary SPI bus
SPIClass mySPI;
// BNO086 IMU using SparkFun library
BNO08x myIMU;

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

void setupIMU() {
  LOG_PRINTLN(F("Setting up BNO086 with SPI interface"));

  // Configure SPI pins for BNO086
  pinMode(BNO086_CS, OUTPUT);
  digitalWrite(BNO086_CS, HIGH);  // Deselect by default

  pinMode(BNO086_INT, INPUT_PULLUP);  // INT pin as input with pull-up

  // Configure PS0/WAKE for SPI mode selection
  pinMode(BNO086_WAKE, OUTPUT);
  digitalWrite(BNO086_WAKE, HIGH);  // PS0=HIGH for SPI mode

  // Configure Reset pin
  pinMode(BNO086_RST, OUTPUT);
  digitalWrite(BNO086_RST, HIGH);  // Initial state (not in reset)

  LOG_PRINTLN(F("Performing BNO086 reset sequence for SPI mode"));

  // Reset sequence according to datasheet
  // 1. Set PS0 (WAKE) HIGH for SPI mode
  digitalWrite(BNO086_WAKE, HIGH);

  // 2. Set CS HIGH (deselected)
  digitalWrite(BNO086_CS, HIGH);

  // 3. Put BNO086 in reset
  digitalWrite(BNO086_RST, LOW);
  delay(RESET_HOLD);  // Hold in reset

  // 4. Release from reset
  digitalWrite(BNO086_RST, HIGH);

  // 5. Wait for BNO086 to initialize
  delay(RESET_WAIT);  // Generous delay

  LOG_PRINTLN(F("Initializing BNO086 SPI communication"));

  // Initialize BNO086 with SPI, using the shared SPI bus with the display
  mySPI.begin(BNO086_SCK, BNO086_MISO, BNO086_MOSI, BNO086_CS);

  if (myIMU.beginSPI(BNO086_CS, BNO086_INT, BNO086_RST, BNO086_SPI_HIGH_SPEED, mySPI) == false) {
    LOG_PRINTLN(F("Failed to find BNO08x chip with SPI - IMU functionality disabled"));
    LOG_PRINTLN(F("Verify SPI connections and mode settings"));

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = false;
      xSemaphoreGive(displayMutex);
    }

    // Try a lower SPI speed as fallback
    LOG_PRINTLN(F("Attempting with lower SPI speed..."));
    delay(100);
    if (myIMU.beginSPI(BNO086_CS, BNO086_INT, BNO086_RST, BNO086_SPI_LOW_SPEED, SPI) == false) {
      LOG_PRINTLN(F("SPI initialization still failed with lower speed"));
    } else {
      LOG_PRINTLN(F("SPI initialized with lower speed successfully!"));
      if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
        display_data.imu_available = true;
        xSemaphoreGive(displayMutex);
      }
    }
  } else {
    LOG_PRINTLN(F("BNO08x Found with SPI interface!"));

    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.imu_available = true;
      xSemaphoreGive(displayMutex);
    }

    // Set up calibration for head tracking
    myIMU.setCalibrationConfig(SH2_CAL_ACCEL);

    LOG_PRINTLN(F("Enabling rotation vector for tracking..."));

    // Try to enable the optimal rotation vector type
    if (myIMU.enableGyroIntegratedRotationVector(IMU_TIME_BETWEEN_REPORTS) == true) {
      LOG_PRINTLN(F("Gyro Integrated Rotation Vector enabled"));
    } else {
      // Fall back to standard options if needed
      if (myIMU.enableARVRStabilizedRotationVector(IMU_TIME_BETWEEN_REPORTS) == true) {
        LOG_PRINTLN(F("AR/VR Stabilized Rotation Vector enabled"));
      } else {
        if (myIMU.enableRotationVector(IMU_TIME_BETWEEN_REPORTS) == true) {
          LOG_PRINTLN(F("Standard Rotation Vector enabled"));
        } else {
          LOG_PRINTLN(F("Could not enable any rotation vector"));
        }
      }
    }
  }
}

void attemptIMUReinitialization() {
  // Check if already available
  bool currently_available = false;

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    currently_available = display_data.imu_available;
    xSemaphoreGive(displayMutex);
  }

  if (currently_available) return;  // Already working, no need to reinitialize

  LOG_PRINTLN(F("Attempting to reinitialize IMU..."));

  LOG_PRINTLN(F("Re-initializing BNO086 SPI interface"));

  // Make sure PS0/WAKE is HIGH for SPI mode
  digitalWrite(BNO086_WAKE, HIGH);

  // Ensure CS is HIGH (deselected)
  digitalWrite(BNO086_CS, HIGH);

  // Reset sequence
  digitalWrite(BNO086_RST, LOW);
  delay(RESET_HOLD);  // Hold in reset for 10ms
  digitalWrite(BNO086_RST, HIGH);
  delay(RESET_WAIT);  // Wait for BNO086 to initialize

  // Try to reinitialize with SPI
  if (myIMU.beginSPI(BNO086_CS, BNO086_INT, BNO086_RST, BNO086_SPI_HIGH_SPEED, SPI) == false) {
    LOG_PRINTLN(F("IMU SPI reinitialization failed"));

    // Try with a lower SPI speed
    LOG_PRINTLN(F("Attempting reinitialization with lower SPI speed..."));
    delay(100);
    if (myIMU.beginSPI(BNO086_CS, BNO086_INT, BNO086_RST, BNO086_SPI_LOW_SPEED, SPI) == false) {
      LOG_PRINTLN(F("IMU reinitialization failed with lower speed as well"));
      return;
    } else {
      LOG_PRINTLN(F("IMU reinitialized with lower SPI speed"));
    }
  }

  // Re-setup optimal calibration
  myIMU.setCalibrationConfig(SH2_CAL_ACCEL);

  // Re-enable the optimal rotation vector
  if (!myIMU.enableGyroIntegratedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
    if (!myIMU.enableARVRStabilizedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
      myIMU.enableRotationVector(IMU_TIME_BETWEEN_REPORTS);
    }
  }

  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.imu_available = true;
    xSemaphoreGive(displayMutex);
  }

  LOG_PRINTLN(F("IMU successfully reinitialized!"));
}

// IMU task, pinned to Core 0, runs at the filter update rate
void imuTask(void* pvParameters) {
  float roll = 0, pitch = 0, heading = 0;
  static float prevHeading = -999.0f;
  static float prevPitch = -999.0f;
  static float prevRoll = -999.0f;
  static int stuckValueCount = 0;
  static unsigned long lastDataTime = 0;
  const unsigned long DATA_TIMEOUT = 1000;  // 1 second timeout for data
  static int resetAttempts = 0;
  const int MAX_RESET_ATTEMPTS = 3;  // Maximum consecutive reset attempts before changing strategy

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

    // Check for data timeout
    unsigned long currentTime = millis();
    if (currentTime - lastDataTime > DATA_TIMEOUT) {
      LOG_PRINTLN(F("IMU data timeout - checking connection"));

      if (myIMU.wasReset()) {
        LOG_PRINTLN(F("BNO086 was reset during timeout - re-enabling features"));
        // Reset was detected - re-enable rotation vector
        if (!myIMU.enableGyroIntegratedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
          if (!myIMU.enableARVRStabilizedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
            myIMU.enableRotationVector(IMU_TIME_BETWEEN_REPORTS);
          }
        }
        resetAttempts = 0;           // Clear attempt counter
        lastDataTime = currentTime;  // Reset timeout
      } else {
        resetAttempts++;
        LOG_PRINT(F("No data from BNO086 - wake attempt #"));
        LOG_PRINTLN(String(resetAttempts));

        if (resetAttempts <= MAX_RESET_ATTEMPTS) {
          // Try wake strategy first
          // In SPI mode, pulsing WAKE/PS0 can help wake the device
          digitalWrite(BNO086_CS, HIGH);  // Ensure CS is HIGH (deselected)
          digitalWrite(BNO086_WAKE, LOW);
          delayMicroseconds(50);  // Short pulse
          digitalWrite(BNO086_WAKE, HIGH);
          delay(1);  // Give it a moment
        } else {
          // If multiple wake attempts failed, try a hard reset
          LOG_PRINTLN(F("Wake attempts failed, performing hard reset"));
          digitalWrite(BNO086_RST, LOW);
          delay(RESET_HOLD);  // Hold in reset for 10ms
          digitalWrite(BNO086_RST, HIGH);
          delay(RESET_WAIT);  // Wait for BNO086 to come out of reset

          // Set flag to trigger reinitialization
          if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display_data.imu_available = false;
            xSemaphoreGive(displayMutex);
          }

          resetAttempts = 0;  // Reset counter after hard reset
        }
      }
    }

    // Handle any resets
    if (myIMU.wasReset()) {
      LOG_PRINTLN(F("BNO08x was reset - performing initialization sequence"));

      // Set the calibration config first to ensure sensors are properly calibrated
      myIMU.setCalibrationConfig(SH2_CAL_ACCEL);

      // Wait a bit for the sensor to apply calibration settings
      delay(50);

      // Now try to enable sensors in priority order
      bool sensorEnabled = false;

      // Try in priority order with proper error checking
      if (myIMU.enableGyroIntegratedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
        LOG_PRINTLN(F("Re-enabled Gyro Integrated Rotation Vector"));
        sensorEnabled = true;
      } else if (myIMU.enableARVRStabilizedRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
        LOG_PRINTLN(F("Re-enabled AR/VR Stabilized Rotation Vector"));
        sensorEnabled = true;
      } else if (myIMU.enableRotationVector(IMU_TIME_BETWEEN_REPORTS)) {
        LOG_PRINTLN(F("Re-enabled Standard Rotation Vector"));
        sensorEnabled = true;
      }

      if (!sensorEnabled) {
        LOG_PRINTLN(F("Failed to enable any rotation vector sensors after reset"));

        // Set IMU as unavailable to trigger full reinitialization
        if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
          display_data.imu_available = false;
          xSemaphoreGive(displayMutex);
        }
      } else {
        // Update last data time to avoid timeout
        lastDataTime = currentTime;
        // Reset counters and variables
        stuckValueCount = 0;
        resetAttempts = 0;
      }
    }

    // Check if we have a new sensor event
    if (myIMU.getSensorEvent() == true) {
      // Update the data timeout timer
      lastDataTime = currentTime;

      // Make sure it's one of our desired sensor types
      uint8_t sensorId = myIMU.getSensorEventID();
      if (sensorId == SENSOR_REPORTID_ROTATION_VECTOR || sensorId == SENSOR_REPORTID_GYRO_INTEGRATED_ROTATION_VECTOR || sensorId == SENSOR_REPORTID_AR_VR_STABILIZED_ROTATION_VECTOR) {

        // Get orientation data - all rotation vector types use the same methods
        roll = (myIMU.getRoll()) * 180.0 / PI;    // Convert roll to degrees
        pitch = (myIMU.getPitch()) * 180.0 / PI;  // Convert pitch to degrees
        heading = (myIMU.getYaw()) * 180.0 / PI;  // Convert yaw / heading to degrees

        // Check for stuck default values
        if (heading == 90.0f && pitch == 0.0f && roll == 90.0f) {
          stuckValueCount++;
          if (stuckValueCount > 5) {
            LOG_PRINTLN(F("BNO08x stuck at default values. Resetting..."));

            // Hard reset the BNO086
            digitalWrite(BNO086_RST, LOW);
            delay(RESET_HOLD);
            digitalWrite(BNO086_RST, HIGH);
            delay(RESET_WAIT);

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
  }
}
