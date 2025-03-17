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

#include "faceplate_module.h"
#include "logger.h"
#include "display_module.h"

// Global variables
Servo servo1, servo2;
servo_data_t servo_data;
SemaphoreHandle_t servoMutex = NULL;

#ifdef USE_NEOPIXELS
// NeoPixel objects - one per LED pin
Adafruit_NeoPixel pixel1(1, LED1_PIN, NEO_GRB + NEO_KHZ800);
Adafruit_NeoPixel pixel2(1, LED2_PIN, NEO_GRB + NEO_KHZ800);
#endif

// Button debounce variables
volatile unsigned long lastInterruptTime = 0;

// Minimum time between faceplate toggles to prevent accidental double-presses
// This doubles as the button debounce time since we only want one per toggle
unsigned long faceplateMoveTime = 500;

// This is our interrupt handler - must be IRAM_ATTR for ESP32
void IRAM_ATTR buttonInterruptHandler() {
  unsigned long interruptTime = millis();

  // Debounce in the ISR
  if (interruptTime - lastInterruptTime > faceplateMoveTime) {
    // Notify the servo task that a button press has occurred
    if (servoTaskHandle != NULL) {
      // High priority notification from interrupt context
      BaseType_t xHigherPriorityTaskWoken = pdFALSE;
      vTaskNotifyGiveFromISR(servoTaskHandle, &xHigherPriorityTaskWoken);
      if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
      }
    }
    lastInterruptTime = interruptTime;
  }
}

// Update the LEDs based on faceplate state
void updateLEDs(bool isClosed) {
#ifdef USE_NEOPIXELS
  // NeoPixel handling
  if (isClosed) {
    // Turn on NeoPixels with specified color when faceplate is closed
    pixel1.setBrightness(NEOPIXEL_BRIGHTNESS);
    pixel2.setBrightness(NEOPIXEL_BRIGHTNESS);

    pixel1.setPixelColor(0, pixel1.Color(NEOPIXEL_COLOR_R, NEOPIXEL_COLOR_G, NEOPIXEL_COLOR_B));
    pixel2.setPixelColor(0, pixel2.Color(NEOPIXEL_COLOR_R, NEOPIXEL_COLOR_G, NEOPIXEL_COLOR_B));

    pixel1.show();
    pixel2.show();
  } else {
    // Turn off NeoPixels when faceplate is open
    pixel1.setPixelColor(0, 0);  // Set to 0 (off)
    pixel2.setPixelColor(0, 0);

    pixel1.show();
    pixel2.show();
  }
#else
  // Regular LED handling
  digitalWrite(LED1_PIN, isClosed ? HIGH : LOW);
  digitalWrite(LED2_PIN, isClosed ? HIGH : LOW);
#endif
}

void setupFaceplate() {
  LOG_PRINTLN(F("Setting up faceplate module..."));

  // Create mutex for thread-safe faceplate data access
  servoMutex = xSemaphoreCreateMutex();
  if (servoMutex == NULL) {
    LOG_PRINTLN(F("Failed to create servoMutex"));
  }

  // Initialize servo data with defaults
  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
    servo_data.faceplate_state = true;  // Start with faceplate closed
    servo_data.last_toggle_time = 0;
    xSemaphoreGive(servoMutex);
  }

  // Calculate movement time based on angle change
  // Most servos move at ~60 degrees per 100ms
  int angleDifference = abs(OPEN_ANGLE_DEFAULT - CLOSED_ANGLE_DEFAULT);
  faceplateMoveTime = max(500, (angleDifference * 100) / 60);  // Minimum 500ms, scale up for larger angles
  LOG_PRINT(F("Faceplate Move Time Calculation: "));
  LOG_PRINTLN(String(faceplateMoveTime));

  // Initialize LEDs or NeoPixels
#ifdef USE_NEOPIXELS
  // Initialize NeoPixels
  pixel1.begin();
  pixel2.begin();

  pixel1.setBrightness(NEOPIXEL_BRIGHTNESS);
  pixel2.setBrightness(NEOPIXEL_BRIGHTNESS);

  pixel1.clear();
  pixel2.clear();

  pixel1.show();
  pixel2.show();

  LOG_PRINTLN(F("NeoPixels initialized"));
#else
  // Setup regular LEDs
  pinMode(LED1_PIN, OUTPUT);
  pinMode(LED2_PIN, OUTPUT);

  // Initial state off
  digitalWrite(LED1_PIN, LOW);
  digitalWrite(LED2_PIN, LOW);

  LOG_PRINTLN(F("LEDs initialized"));
#endif

  // Allocation of timers for servo control
  // Skipped timer 0 since attach() returns 0 on failure and the allocated timer number
  // It makes errors confusing
  //ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  //ESP32PWM::allocateTimer(3);

  LOG_PRINTLN(F("Timers allocated for servos"));

  // Set servo parameters
  servo1.setPeriodHertz(50);  // Standard 50hz servo
  delay(50);                  // Small delay between configurations
  servo2.setPeriodHertz(50);  // Standard 50hz servo
  delay(50);

  // Attach servos to pins with default min/max pulse width
  // Using wider pulse width range for better compatibility
  if (!servo1.attach(SERVO1_PIN, 500, 2500)) {
    LOG_PRINTLN(F("WARNING: Failed to attach servo1"));
  } else {
    LOG_PRINTLN(F("Servo1 attached successfully"));
  }
  delay(50);  // Allow time between servo attachments
  if (!servo2.attach(SERVO2_PIN, 500, 2500)) {
    LOG_PRINTLN(F("WARNING: Failed to attach servo2"));
  } else {
    LOG_PRINTLN(F("Servo2 attached successfully"));
  }
  delay(50);

  // Move servos to initial closed position with verification
  int closed_angle = CLOSED_ANGLE_DEFAULT;

  // Move and verify first servo
  LOG_PRINT(F("Moving servo1 to closed position: "));
  LOG_PRINTLN(String(closed_angle));
  servo1.write(closed_angle);

  // For the second servo, use simple inversion (180-angle)
  int servo2_angle = 180 - closed_angle;

  LOG_PRINT(F("Moving servo2 to closed position: "));
  LOG_PRINTLN(String(servo2_angle));
  servo2.write(servo2_angle);

  // Initial state is closed, so turn on LEDs
  updateLEDs(true);

  // Set up the button with interrupt
  pinMode(BUTTON_PIN, INPUT_PULLUP);

  // Wait a bit for capacitors to charge, pins to stabilize
  delay(100);

  // Attach the interrupt to the button pin - trigger on FALLING edge (button pressed)
  attachInterrupt(digitalPinToInterrupt(BUTTON_PIN), buttonInterruptHandler, FALLING);

  LOG_PRINTLN(F("Button interrupt configured"));
  LOG_PRINTLN(F("Servo module initialized"));
}

// Function to toggle faceplate state - can be called externally
void toggleFaceplateState() {
  unsigned long currentTime = millis();

  if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(100)) == pdTRUE) {
    // Check if enough time has passed since last toggle
    if (currentTime - servo_data.last_toggle_time > faceplateMoveTime) {
      servo_data.faceplate_state = !servo_data.faceplate_state;
      servo_data.last_toggle_time = currentTime;

      // Log the state change
      LOG_PRINT(F("Faceplate state toggled to: "));
      LOG_PRINTLN(servo_data.faceplate_state ? "Closed" : "Open");
    }
    xSemaphoreGive(servoMutex);
  }
}

// Servo control task - waits for notifications rather than polling
void servoTask(void *pvParameters) {
  bool currentFaceplateState = true;  // Local tracking of faceplate state

  LOG_PRINTLN(F("Servo task started."));

  // Allow the servos to settle in initial position
  vTaskDelay(pdMS_TO_TICKS(faceplateMoveTime));

  // Create JSON document for status reporting
  JsonDocument doc;

  while (true) {
    // Wait for notification from the interrupt handler
    // Will wait indefinitely until a notification is received
    // We don't need the notification value itself, just the fact that we were notified
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

    // If we're here, the button was pressed
    LOG_PRINTLN(F("Button interrupt received, toggling faceplate state"));
    toggleFaceplateState();

    // Get current servo data with mutex protection
    bool newFaceplateState = currentFaceplateState;

    if (xSemaphoreTake(servoMutex, pdMS_TO_TICKS(10)) == pdTRUE) {
      newFaceplateState = servo_data.faceplate_state;
      xSemaphoreGive(servoMutex);
    }

    // If faceplate state changed, move servos and update LEDs
    if (newFaceplateState != currentFaceplateState) {
      int targetAngle;
      int servo2_angle;

      // Set appropriate servo angles based on faceplate state
      if (newFaceplateState) {
        // Close faceplate - move to closed position
        targetAngle = CLOSED_ANGLE_DEFAULT;
        LOG_PRINT(F("Moving servos to closed position: "));
        LOG_PRINTLN(String(targetAngle));
      } else {
        // Open faceplate - move to open position
        targetAngle = OPEN_ANGLE_DEFAULT;
        LOG_PRINT(F("Moving servos to open position: "));
        LOG_PRINTLN(String(targetAngle));
      }

      if (currentFaceplateState) {
        // Wait for servos to complete their movement
        // Typical servo movement takes ~300-500ms for a full sweep
        vTaskDelay(pdMS_TO_TICKS(faceplateMoveTime));
      }

      // Move servo1
      servo1.write(targetAngle);

      // Move servo2 with inverted angle
      servo2_angle = 180 - targetAngle;
      LOG_PRINT(F("Servo2 angle: "));
      LOG_PRINTLN(String(servo2_angle));
      servo2.write(servo2_angle);

      if (!currentFaceplateState) {
        // Wait for servos to complete their movement
        // Typical servo movement takes ~300-500ms for a full sweep
        vTaskDelay(pdMS_TO_TICKS(faceplateMoveTime));
      }

      // Update LEDs based on new state
      updateLEDs(newFaceplateState);

      // Update local state
      currentFaceplateState = newFaceplateState;

      // Send status update to logger
      doc["device"] = "Servo";
      doc["state"] = currentFaceplateState ? "Closed" : "Open";
      doc["openAngle"] = OPEN_ANGLE_DEFAULT;
      doc["closedAngle"] = CLOSED_ANGLE_DEFAULT;
      logger_send_json(&doc, "Servo");
      doc.clear();
    }

    // This is a "reactive" task now - it doesn't need to poll
    // It will only run when triggered by a notification
  }
}