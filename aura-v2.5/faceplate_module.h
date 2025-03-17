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

#ifndef FACEPLATE_MODULE_H
#define FACEPLATE_MODULE_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <freertos/task.h>
#include <ArduinoJson.h>
#include <Adafruit_NeoPixel.h>

// Servo pins for ESP32-S3 TFT Feather
#define SERVO1_PIN 12
#define SERVO2_PIN 13

// LED pins
#define LED1_PIN A4
#define LED2_PIN A5

// Button pins
#define BUTTON_PIN 11

// Define servo angle limits
#define OPEN_ANGLE_DEFAULT 20
#define CLOSED_ANGLE_DEFAULT 80

// LED/NeoPixel configuration
//#define USE_NEOPIXELS // define for NeoPixels vs. regular LEDs

// NeoPixel settings
#define NEOPIXEL_BRIGHTNESS 64  // 0-255
#define NEOPIXEL_COLOR_R 255    // Red component (0-255)
#define NEOPIXEL_COLOR_G 255    // Green component (0-255)
#define NEOPIXEL_COLOR_B 255    // Blue component (0-255)

// Create a mutex for servo data
extern SemaphoreHandle_t servoMutex;

// Servo task handle is defined in main .ino file
extern TaskHandle_t servoTaskHandle;

// Struct to hold servo data
typedef struct {
  bool faceplate_state;            // Current faceplate state (true = closed, false = open)
  unsigned long last_toggle_time;  // Time of last toggle to prevent bouncing
} servo_data_t;

// Global data accessible to other modules
extern servo_data_t servo_data;

// Function prototypes
void setupFaceplate();
void buttonInterruptHandler();  // ISR handler
void servoTask(void *pvParameters);
void toggleMaskState();
void updateLEDs(bool isClosed);

#endif  // FACEPLATE_MODULE_H