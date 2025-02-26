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

#include "enviro_module.h"
#include "logger.h"
#include "display_module.h"

// External dependencies
extern WiFiClient wifiClient;

// Environmental Sensors
#ifdef USE_BME680
Adafruit_BME680 bme;
#define SEALEVELPRESSURE_HPA (1018)
#endif

#ifdef USE_ENS160_AHT21
Adafruit_AHTX0 aht; // AHT20/AHT21 sensor
ScioSense_ENS160 ens160(0x53); // ENS160 sensor with I2C address 0x53

// Runtime calibration values (initialized from defines)
float aht21_temp_offset = AHT21_TEMP_OFFSET;
float aht21_hum_offset = AHT21_HUM_OFFSET;

// Calibration functions
void setAHT21TempOffset(float offset) {
  aht21_temp_offset = offset;
  LOG_PRINT("AHT21 temperature offset set to: ");
  LOG_PRINTLN(String(offset));
}

void setAHT21HumOffset(float offset) {
  aht21_hum_offset = offset;
  LOG_PRINT("AHT21 humidity offset set to: ");
  LOG_PRINTLN(String(offset));
}

float getAHT21TempOffset() {
  return aht21_temp_offset;
}

float getAHT21HumOffset() {
  return aht21_hum_offset;
}
#endif

#ifdef USE_SCD41
SensirionI2cScd4x scd4x; // SCD41 CO2 sensor
bool scd41_available = false;

// Return descriptive quality string based on CO2 reading
const char* getCO2QualityDescription(uint16_t co2_ppm) {
  if (co2_ppm < CO2_EXCELLENT) return "Excellent";
  else if (co2_ppm < CO2_GOOD) return "Good";
  else if (co2_ppm < CO2_FAIR) return "Fair";
  else if (co2_ppm < CO2_POOR) return "Poor";
  else return "Very Poor";
}
#endif

#ifdef ENABLE_MQTT
extern MqttClient mqttClient;
extern const char topic[];
#endif

// Helper function for floating-point mapping (Arduino's map() only works with integers)
float mapFloat(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

void setupEnvironmental() {
#ifdef USE_BME680
  // BME680 setup
  if (!bme.begin(0x76, true)) {
    LOG_PRINTLN("Could not find a valid BME688 sensor, check wiring!");
  } else {
    LOG_PRINTLN("BME688 found!");

    // Set up oversampling and filter initialization
    bme.setTemperatureOversampling(BME680_OS_8X);
    bme.setHumidityOversampling(BME680_OS_2X);
    bme.setPressureOversampling(BME680_OS_4X);
    bme.setIIRFilterSize(BME680_FILTER_SIZE_3);
    bme.setGasHeater(320, 150); // 320*C for 150 ms
  }
#endif

#ifdef USE_ENS160_AHT21
  // AHT21 setup
  if (!aht.begin()) {
    LOG_PRINTLN("Could not find a valid AHT20/AHT21 sensor, check wiring!");
  } else {
    LOG_PRINTLN("AHT20/AHT21 found!");
  }

  // ENS160 setup
  if (!ens160.begin()) {
    LOG_PRINTLN("Could not find a valid ENS160 sensor, check wiring!");
  } else {
    LOG_PRINTLN("ENS160 found!");
    
    // Set mode to standard operation
    ens160.setMode(ENS160_OPMODE_STD);
    
    // Give some time to warm up (sensor needs up to 1 hour for full warm-up,
    // but we'll allow a few seconds for basic operation)
    delay(2000);
  }
#endif

// Revised SCD41 initialization for setupEnvironmental()

#ifdef USE_SCD41
  // SCD41 setup following manufacturer's recommended initialization sequence
  LOG_PRINTLN("Initializing SCD41...");
  scd4x.begin(Wire, SCD41_I2C_ADDR_62);
  
  // Ensure sensor is in clean state by proper initialization sequence
  delay(30); // Short delay after begin
  
  uint16_t error = scd4x.wakeUp();
  if (error != 0) {
    LOG_PRINT("Error during wakeUp(): ");
    LOG_PRINTLN(String(error));
  }
  
  error = scd4x.stopPeriodicMeasurement();
  if (error != 0) {
    LOG_PRINT("Error during stopPeriodicMeasurement(): ");
    LOG_PRINTLN(String(error));
  }
  
  error = scd4x.reinit();
  if (error != 0) {
    LOG_PRINT("Error during reinit(): ");
    LOG_PRINTLN(String(error));
  }
  
  // Read serial number to verify communication
  uint64_t serialNumber = 0;
  error = scd4x.getSerialNumber(serialNumber);
  
  if (error != 0) {
    LOG_PRINTLN("Could not read SCD41 serial number, sensor might not be present");
    scd41_available = false;
  } else {
    LOG_PRINT("SCD41 found! Serial: ");
    // Print serial number in hex format
    char serialStr[20];
    sprintf(serialStr, "0x%08X%08X", 
            (uint32_t)(serialNumber >> 32), 
            (uint32_t)(serialNumber & 0xFFFFFFFF));
    LOG_PRINTLN(serialStr);
    
    // Set additional configuration if needed
    // For example, scd4x.setAmbientPressure() if needed
    
    // Start periodic measurements
    error = scd4x.startPeriodicMeasurement();
    if (error != 0) {
      LOG_PRINT("Error starting periodic measurement: ");
      LOG_PRINTLN(String(error));
      scd41_available = false;
    } else {
      LOG_PRINTLN("SCD41 periodic measurement started");
      scd41_available = true;
    }
  }
 
  // Initialize the CO2 availability in display data
  if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    display_data.co2_available = scd41_available;
    
    // Default values if sensor not available
    if (!scd41_available) {
      display_data.co2 = 0;
      strcpy(display_data.co2_quality_description, "Unavailable");
    }
    
    xSemaphoreGive(displayMutex);
  }
#endif
}

// Environmental sensor task runs every 5 seconds
void enviroTask(void *pvParameters) {
  float temperature = 0.0;
  float pressure = 0.0;
  float humidity = 0.0;
  float air_quality_score = 0.0;
  float gas_resistance = 0.0;
  float altitude = 0.0;
  
  // Variables for IAQ (Indoor Air Quality) calculation
  float hum_reference = 40.0; // Reference humidity for IAQ calculation
  float gas_reference = 250000.0; // Reference gas resistance for IAQ calculation
  float hum_score, gas_score;
  
  uint32_t timestamp = 0;
  
#ifdef USE_BME680
  unsigned long bmeEndTime = 0L;
#endif

#ifdef USE_SCD41
  uint16_t co2_ppm = 0;
  float temp_scd41 = 0.0;
  float humidity_scd41 = 0.0;
  bool dataReady = false;
  uint16_t error = 0;
#endif

  JsonDocument doc;

  LOG_PRINTLN(F("Environmental task started."));

  while (true) {
    timestamp = millis();

#ifdef USE_BME680
    /* Read BME680 sensor */
    if (bmeEndTime == 0) {
      bmeEndTime = bme.beginReading();
      if (bmeEndTime == 0) {
        LOG_PRINTLN(F("Failed to begin reading :("));
      } else {
        bmeEndTime += 5000; // Add padding so we don't block on the reading
      }
    }

    if (bmeEndTime != 0 && timestamp > bmeEndTime) {
      LOG_PRINTLN("Reading BME");
      if (bme.endReading()) {
        temperature = bme.temperature; // C
        pressure = bme.pressure / 100.0; // hPa
        humidity = bme.humidity; // %
        gas_resistance = bme.gas_resistance / 1000.0; // KOhms
        altitude = bme.readAltitude(SEALEVELPRESSURE_HPA); // m
        
        // Calculate air quality score (simplified IAQ calculation)
        // Humidity score (optimal: 40%)
        if (humidity >= 38 && humidity <= 42)
          hum_score = 100.0;
        else { // Humidity is sub-optimal
          if (humidity < 38)
            hum_score = mapFloat(humidity, 0, 38, 0, 100);
          else
            hum_score = mapFloat(humidity, 42, 100, 100, 0);
        }
        
        // Gas resistance score (higher resistance = better air quality)
        gas_score = mapFloat(gas_resistance, 0, 500, 0, 100);
        if (gas_score > 100) gas_score = 100;
        if (gas_score < 0) gas_score = 0;
        
        // Calculate IAQ index (weighted average)
        air_quality_score = (hum_score * 0.25) + (gas_score * 0.75);

        bmeEndTime = 0L;
      }
    }
#endif

#ifdef USE_ENS160_AHT21
    /* Read AHT21 + ENS160 sensors */
    LOG_PRINTLN("Reading AHT21+ENS160");
    
    // Read temperature and humidity from AHT20/AHT21
    sensors_event_t humidity_event, temp_event;
    if (aht.getEvent(&humidity_event, &temp_event)) {
      // Apply calibration offsets
      temperature = temp_event.temperature + aht21_temp_offset; // C
      humidity = humidity_event.relative_humidity + aht21_hum_offset; // %
      
      // Constrain humidity to valid range (0-100%)
      if (humidity > 100.0f) humidity = 100.0f;
      if (humidity < 0.0f) humidity = 0.0f;
      
      // Set temperature and humidity compensation for ENS160
      ens160.set_envdata(temperature, humidity);
    } else {
      LOG_PRINTLN("Failed to read from AHT20/AHT21 sensor");
    }
    
    // Read air quality data from ENS160
    if (ens160.measure()) {
      // Read Air Quality Index (AQI) - 1 (good) to 5 (poor)
      uint8_t aqi = ens160.getAQI();
      
      // Read eCO2 level
      uint16_t eco2 = ens160.geteCO2();
      
      // Read TVOC level
      uint16_t tvoc = ens160.getTVOC();
      
      // Map AQI to our 0-100 scale (where 100 is excellent)
      // ENS160 AQI: 1=excellent, 2=good, 3=moderate, 4=poor, 5=unhealthy
      // Inverse mapping: 1->100, 2->80, 3->60, 4->40, 5->20
      air_quality_score = 120 - (aqi * 20);
      
      // Store TVOC as our gas_resistance equivalent (for compatibility)
      gas_resistance = tvoc;
      
      // No pressure sensor in this combo, so use a default or previous value
      pressure = 1013.25; // Standard sea level pressure in hPa
      
      // No altitude data available with this sensor
      altitude = 0.0;
    } else {
      LOG_PRINTLN("Failed to read from ENS160 sensor");
    }
#endif

#ifdef USE_SCD41
    /* Read SCD41 CO2 sensor (if available) */
    if (scd41_available) {
      // Check if new data is available
      bool dataReady = false;
      uint16_t error = scd4x.getDataReadyStatus(dataReady);
      
      if (error != 0) {
        LOG_PRINT("Error checking data ready status: ");
        LOG_PRINTLN(String(error));
      } else if (dataReady) {
        // Read measurement
        uint16_t co2_ppm = 0;
        float temp_scd41 = 0.0;
        float humidity_scd41 = 0.0;
        
        error = scd4x.readMeasurement(co2_ppm, temp_scd41, humidity_scd41);
        
        if (error != 0) {
          LOG_PRINT("Error reading measurement: ");
          LOG_PRINTLN(String(error));
        } else if (co2_ppm != 0) {  // CO2 of 0ppm indicates an invalid reading
          // Print results in proper format
          LOG_PRINT("CO2 concentration [ppm]: ");
          LOG_PRINTLN(String(co2_ppm));
          LOG_PRINT("Temperature [°C]: ");
          LOG_PRINTLN(String(temp_scd41));
          LOG_PRINT("Relative Humidity [%RH]: ");
          LOG_PRINTLN(String(humidity_scd41));
          
          // We now have valid CO2 data
          // Update display data
          if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
            display_data.co2 = co2_ppm;
            strncpy(display_data.co2_quality_description, 
                   getCO2QualityDescription(co2_ppm), 
                   sizeof(display_data.co2_quality_description));
            xSemaphoreGive(displayMutex);
          }
          
          // Option: If using SCD41 temperature and humidity is preferred
          // over the AHT21 values, you could use those instead:
          /*
          temperature = temp_scd41;
          humidity = humidity_scd41;
          */
        }
      }
    }
#endif

    /* Prepare data for sending */
    doc["device"] = "Enviro";
    doc["temp_c"] = temperature;
    doc["temp_f"] = (temperature * 9.0/5.0) + 32.0; // Convert to Fahrenheit
    doc["humidity"] = humidity;
    
#ifdef USE_BME680
    doc["pressure_hpa"] = pressure;
    doc["altitude_m"] = altitude;
    doc["altitude_ft"] = altitude * 3.28084; // Convert to feet
    doc["gas_kohms"] = gas_resistance;
#endif

#ifdef USE_ENS160_AHT21
    doc["pressure_hpa"] = pressure;
    doc["tvoc_ppb"] = gas_resistance; // Using the TVOC value
    doc["eco2_ppm"] = ens160.geteCO2();
#endif

#ifdef USE_SCD41
    // Add SCD41 data if available
    if (scd41_available && co2_ppm != 0) {
      doc["co2_ppm"] = co2_ppm;
      doc["co2_quality"] = getCO2QualityDescription(co2_ppm);
      
      // If we have both eCO2 and real CO2, add the difference
      #ifdef USE_ENS160_AHT21
      doc["co2_eco2_diff"] = (int)co2_ppm - ens160.geteCO2();
      #endif
    }
#endif

    doc["air_quality"] = air_quality_score;
    
    // Add air quality description
    if (air_quality_score > 80)
      doc["air_quality_description"] = "Excellent";
    else if (air_quality_score > 60)
      doc["air_quality_description"] = "Good";
    else if (air_quality_score > 40)
      doc["air_quality_description"] = "Average";
    else if (air_quality_score > 20)
      doc["air_quality_description"] = "Poor";
    else
      doc["air_quality_description"] = "Very Poor";
    
    // Add heat index calculation (Fahrenheit)
    float temp_f = (temperature * 9.0/5.0) + 32.0;
    if (temp_f >= 80.0 && humidity >= 40) {
      // Simplified heat index calculation
      float heat_index_f = -42.379 + 
                         2.04901523 * temp_f + 
                         10.14333127 * humidity - 
                         0.22475541 * temp_f * humidity - 
                         0.00683783 * temp_f * temp_f - 
                         0.05481717 * humidity * humidity + 
                         0.00122874 * temp_f * temp_f * humidity + 
                         0.00085282 * temp_f * humidity * humidity - 
                         0.00000199 * temp_f * temp_f * humidity * humidity;
      doc["heat_index_f"] = heat_index_f;
      doc["heat_index_c"] = (heat_index_f - 32.0) * 5.0/9.0;
    }
    
    // Add dew point calculation
    float a = 17.27;
    float b = 237.7;
    float alpha = ((a * temperature) / (b + temperature)) + log(humidity/100.0);
    float dew_point = (b * alpha) / (a - alpha);
    doc["dew_point_c"] = dew_point;
    doc["dew_point_f"] = (dew_point * 9.0/5.0) + 32.0;

    // Update display data
    if (xSemaphoreTake(displayMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      display_data.temperature = temperature;
      display_data.humidity = humidity;
      display_data.pressure = pressure;
      display_data.air_quality = air_quality_score;
      
      if (air_quality_score > 80)
        strncpy(display_data.air_quality_description, "Excellent", sizeof(display_data.air_quality_description));
      else if (air_quality_score > 60)
        strncpy(display_data.air_quality_description, "Good", sizeof(display_data.air_quality_description));
      else if (air_quality_score > 40)
        strncpy(display_data.air_quality_description, "Average", sizeof(display_data.air_quality_description));
      else if (air_quality_score > 20)
        strncpy(display_data.air_quality_description, "Poor", sizeof(display_data.air_quality_description));
      else
        strncpy(display_data.air_quality_description, "Very Poor", sizeof(display_data.air_quality_description));
      
#ifdef USE_ENS160_AHT21
      // Update ENS160 specific data for display
      display_data.eco2 = ens160.geteCO2();
      display_data.tvoc = ens160.getTVOC();
#endif

#ifdef USE_SCD41
      // Update SCD41 data if available
      if (scd41_available && co2_ppm != 0) {
        display_data.co2 = co2_ppm;
        // Update CO2 quality description
        strncpy(display_data.co2_quality_description, 
                getCO2QualityDescription(co2_ppm), 
                sizeof(display_data.co2_quality_description));
      }
#endif
      
      xSemaphoreGive(displayMutex);
    }

    // Thread-safe JSON sending
    #ifdef ENABLE_MQTT
      logger_send_mqtt_json(&doc, "Enviro", &mqttClient, topic);
    #else
      logger_send_json(&doc, "Enviro", &wifiClient);
    #endif

    doc.clear();
    
    vTaskDelay(pdMS_TO_TICKS(5000)); // Delay for 5 seconds
  }
}
