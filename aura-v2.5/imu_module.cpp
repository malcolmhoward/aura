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

#ifdef USE_ICM20948
// ICM20948 IMU/AHRS
Adafruit_ICM20948 icm;
Adafruit_Sensor *accelerometer, *gyroscope, *magnetometer, *temperature;

// Pick your filter! slower == better quality output
Adafruit_NXPSensorFusion filter; // slowest
//Adafruit_Madgwick filter;      // faster than NXP
//Adafruit_Mahony filter;        // fastest/smalleset
#endif

#ifdef USE_BNO086
// BNO086 IMU
Adafruit_BNO08x bno08x(BNO086_RST);
sh2_SensorValue_t sensorValue;

// Rotation vector report configuration
enum BNO08xReports {
  ROTATION_VECTOR = 0,
  GAME_ROTATION_VECTOR,
  GEOMAGNETIC_ROTATION_VECTOR,
  GYRO_INTEGRATED_ROTATION_VECTOR,
  ACCELEROMETER,
  GYROSCOPE,
  MAGNETOMETER,
  LINEAR_ACCELERATION,
  GRAVITY,
  TAP_DETECTOR,
  STEP_COUNTER,
  STABILITY_CLASSIFIER,
  ACTIVITY_CLASSIFIER,
  RAW_ACCELEROMETER,
  RAW_GYROSCOPE,
  RAW_MAGNETOMETER
};

// Store the quaternion data
float qr, qx, qy, qz;
// Store Euler angles (in degrees)
float yaw, pitch, roll;
// Store raw sensor data
float accelX, accelY, accelZ;
float gyroX, gyroY, gyroZ;
float magX, magY, magZ;
#endif

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

#ifdef USE_BNO086
// Function to convert quaternion to Euler angles
void quaternionToEuler(float qr, float qi, float qj, float qk, float& roll, float& pitch, float& yaw) {
  float sqr = sq(qr);
  float sqi = sq(qi);
  float sqj = sq(qj);
  float sqk = sq(qk);

  roll = atan2(2.0 * (qj * qk + qi * qr), sqr - sqi - sqj + sqk) * RAD_TO_DEG;
  pitch = asin(-2.0 * (qi * qk - qj * qr) / (sqr + sqi + sqj + sqk)) * RAD_TO_DEG;
  yaw = atan2(2.0 * (qi * qj + qk * qr), sqr + sqi - sqj - sqk) * RAD_TO_DEG;
  
  // Ensure yaw is between 0-360 for consistent heading
  if (yaw < 0) yaw += 360.0;
}

// Function to enable BNO086 reports
void enableBNO08xReports() {
  // Enable the quaternion rotation vector with high accuracy
  bno08x.enableReport(SH2_ROTATION_VECTOR, FILTER_UPDATE_RATE_HZ);
  
  // Enable accelerometer readings
  bno08x.enableReport(SH2_ACCELEROMETER, FILTER_UPDATE_RATE_HZ);
  
  // Enable gyroscope readings
  bno08x.enableReport(SH2_GYROSCOPE_CALIBRATED, FILTER_UPDATE_RATE_HZ);
  
  // Enable magnetometer readings
  bno08x.enableReport(SH2_MAGNETIC_FIELD_CALIBRATED, FILTER_UPDATE_RATE_HZ);
}
#endif

void setupIMU() {
#ifdef USE_ICM20948
  // ICM20948
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
#endif

#ifdef USE_BNO086
  // Set up the BNO086 interrupt and reset pins
  pinMode(BNO086_INT, INPUT);
  
  // Initialize BNO086
  if (!bno08x.begin_I2C(0x4B)) {
    LOG_PRINTLN("Failed to find BNO08x chip");
    while (1) {
      delay(10);
    }
  }
  
  LOG_PRINTLN("BNO08x Found!");
  
  // Enable the sensor reports we need
  enableBNO08xReports();
  
  LOG_PRINTLN("BNO08x reports enabled.");
  
  // Initialize Euler angles
  roll = pitch = yaw = 0.0f;
#endif
}

// IMU task, pinned to Core 0, runs at the filter update rate
void imuTask(void *pvParameters) {
#ifdef USE_ICM20948
  sensors_event_t accel, gyro, mag, temp;
  float roll, pitch, heading;
  float gx, gy, gz;
#endif

  JsonDocument doc;

  LOG_PRINTLN(F("IMU task started."));

  while (true) {
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
    /* Read data from BNO086 */
    if (bno08x.wasReset()) {
      LOG_PRINTLN("BNO08x was reset - re-enabling reports");
      enableBNO08xReports();
    }
    
    if (!bno08x.getSensorEvent(&sensorValue)) {
      continue; // No event available, check again next loop
    }
    
    // Process different sensor reports
    switch (sensorValue.sensorId) {
      case SH2_ROTATION_VECTOR:
        // Store quaternion data
        qr = sensorValue.un.rotationVector.real;
        qx = sensorValue.un.rotationVector.i;
        qy = sensorValue.un.rotationVector.j;
        qz = sensorValue.un.rotationVector.k;
        
        // Calculate Euler angles (in degrees)
        quaternionToEuler(qr, qx, qy, qz, roll, pitch, yaw);
        break;
        
      case SH2_ACCELEROMETER:
        accelX = sensorValue.un.accelerometer.x;
        accelY = sensorValue.un.accelerometer.y;
        accelZ = sensorValue.un.accelerometer.z;
        break;
        
      case SH2_GYROSCOPE_CALIBRATED:
        gyroX = sensorValue.un.gyroscope.x;
        gyroY = sensorValue.un.gyroscope.y;
        gyroZ = sensorValue.un.gyroscope.z;
        break;
        
      case SH2_MAGNETIC_FIELD_CALIBRATED:
        magX = sensorValue.un.magneticField.x;
        magY = sensorValue.un.magneticField.y;
        magZ = sensorValue.un.magneticField.z;
        break;
    }
    
    // Use yaw as heading to maintain consistent naming with ICM20948
    float heading = yaw;
#endif

    // Prepare to send motion data (common for both sensors)
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
      logger_send_json(&doc, "Motion", &wifiClient);
    #endif

    doc.clear();

    vTaskDelay(pdMS_TO_TICKS(1000 / FILTER_UPDATE_RATE_HZ)); // Delay at filter update rate
  }
}
