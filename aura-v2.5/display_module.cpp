#include "display_module.h"
#include "logger.h"
#include <cstring>  // for strcpy, strcmp

// External dependencies
extern WiFiClient wifiClient;
extern char ssid[];

// Create semaphore for thread-safe display access
SemaphoreHandle_t displayMutex = NULL;

// First-draw flags for each page (to reduce flicker)
bool imu_page_first_draw = true;
bool gps_page_first_draw = true; 
bool enviro_page_first_draw = true;
bool wifi_page_first_draw = true;

// Global variables
display_data_t display_data;
int current_page = PAGE_IMU;
unsigned long last_page_change = 0;

// Initialize display
Adafruit_ST7789 tft = Adafruit_ST7789(TFT_CS, TFT_DC, TFT_RST);

// Color definitions
#define COLOR_BG        ST77XX_BLACK
#define COLOR_TITLE     ST77XX_WHITE
#define COLOR_IMU       ST77XX_MAGENTA
#define COLOR_GPS       ST77XX_BLUE
#define COLOR_ENVIRO    ST77XX_GREEN
#define COLOR_WIFI      ST77XX_CYAN
#define COLOR_ALERT     ST77XX_RED
#define COLOR_GOOD      ST77XX_GREEN
#define COLOR_TEXT      ST77XX_WHITE
#define COLOR_VALUE     ST77XX_YELLOW

// Setup TFT display
void setupDisplay(void) {
    // Enable power for TFT
    pinMode(TFT_PWR, OUTPUT);
    digitalWrite(TFT_PWR, HIGH);

    // Enable backlight
    pinMode(TFT_BL, OUTPUT);
    digitalWrite(TFT_BL, HIGH);

    // Initialize the display
    tft.init(TFT_HEIGHT, TFT_WIDTH);  // Note: We're initializing with swapped height and width due to rotation
    tft.setRotation(3);  // Rotate to landscape
    tft.fillScreen(COLOR_BG);
    
    // Create mutex for display access
    displayMutex = xSemaphoreCreateMutex();
    if (displayMutex == NULL) {
        LOG_PRINTLN("Failed to create displayMutex");
    }
    
    // Initialize display data structure
    memset(&display_data, 0, sizeof(display_data));
    strcpy(display_data.air_quality_description, "Unknown");
    strcpy(display_data.time, "00:00:00");
    strcpy(display_data.date, "0000/00/00");
    strcpy(display_data.ssid, ssid);
    strcpy(display_data.ip_address, "0.0.0.0");
    
    // Reset page change timer
    last_page_change = millis();
    
    // Draw initial welcome screen
    tft.fillScreen(COLOR_BG);
    tft.setCursor(20, 30);
    tft.setTextSize(3);
    tft.setTextColor(COLOR_TITLE);
    tft.println("OASIS AURA");
    tft.setCursor(20, 70);
    tft.setTextSize(2);
    tft.setTextColor(COLOR_TEXT);
    tft.println("Sensor Display");
    tft.setCursor(20, 100);
    tft.setTextSize(1);
    tft.println("Initializing...");
    
    delay(2000);  // Show welcome screen for 2 seconds

    LOG_PRINTLN("Display initialized");
}

// Function to reset all page first-draw flags when changing pages
void resetPageDrawFlags(void) {
    imu_page_first_draw = true;
    gps_page_first_draw = true;
    enviro_page_first_draw = true;
    wifi_page_first_draw = true;
}

// Display IMU page with minimal flicker
// Display IMU page with minimal flicker - Fixed overlapping graphics
void displayIMUPage(void) {
    // Static variables to track previous values
    static float prev_heading = -999;
    static float prev_pitch = -999;
    static float prev_roll = -999;
    
    // On first draw or page change, draw the static elements
    if (imu_page_first_draw) {
        // Clear the screen on first draw
        tft.fillScreen(COLOR_BG);
        
        // Page title
        tft.setTextSize(2);
        tft.setTextColor(COLOR_IMU);
        tft.setCursor(10, 10);
        tft.print("Motion Sensor");
        
        // Draw divider line
        tft.drawLine(0, 30, TFT_WIDTH, 30, COLOR_IMU);
        
        // UPDATED: Define fixed locations for our elements
        int compass_x = 180;  // Moved further right
        int compass_y = 55;
        int indicator_size = 35; // Size for compass
        
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        
        // Static labels - moved text position further left to avoid overlap
        tft.setCursor(10, 50);
        tft.print("Heading:");
        
        tft.setCursor(10, 70);
        tft.print("Pitch:");
        
        tft.setCursor(10, 90);
        tft.print("Roll:");
        
        // Mark cardinal directions
        tft.setCursor(compass_x - 3, compass_y - indicator_size - 7);
        tft.print("N");
        tft.setCursor(compass_x + indicator_size + 2, compass_y - 3);
        tft.print("E");
        tft.setCursor(compass_x - 3, compass_y + indicator_size + 2);
        tft.print("S");
        tft.setCursor(compass_x - indicator_size - 7, compass_y - 3);
        tft.print("W");
        
        // Page indicator
        tft.setCursor(TFT_WIDTH - 60, TFT_HEIGHT - 15);
        tft.setTextColor(COLOR_TEXT);
        tft.print("Page ");
        tft.print(current_page + 1);
        tft.print("/");
        tft.print(PAGE_COUNT);
        
        imu_page_first_draw = false;
        
        // Set initial values to force first update
        prev_heading = -999;
        prev_pitch = -999;
        prev_roll = -999;
    }
    
    // Use the same size for both indicators
    int indicator_size = 30; // Slightly reduced size 
    
    // Only update values if they've changed
    if (prev_heading != display_data.heading || prev_pitch != display_data.pitch || prev_roll != display_data.roll) {
        // Draw a simple orientation indicator in top-right quadrant
        int compass_x = 180;  // Moved further right
        int compass_y = 55;
        
        // Update heading display (erase old value first) - reduced width for data display area
        tft.fillRect(70, 50, 50, 10, COLOR_BG);
        tft.setCursor(70, 50);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.heading, 1);
        tft.print("°");
        
        // Update pitch display
        tft.fillRect(70, 70, 50, 10, COLOR_BG);
        tft.setCursor(70, 70);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.pitch, 1);
        tft.print("°");
        
        // Update roll display
        tft.fillRect(70, 90, 50, 10, COLOR_BG);
        tft.setCursor(70, 90);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.roll, 1);
        tft.print("°");
        
        // Redraw compass (erase old content first)
        tft.fillCircle(compass_x, compass_y, indicator_size, COLOR_BG);
        tft.drawCircle(compass_x, compass_y, indicator_size, COLOR_TEXT);
        
        // Draw heading line (convert to radians)
        float heading_rad = display_data.heading * PI / 180.0;
        int x2 = compass_x + sin(heading_rad) * indicator_size;
        int y2 = compass_y - cos(heading_rad) * indicator_size;
        tft.drawLine(compass_x, compass_y, x2, y2, COLOR_IMU);
        
        // Mark cardinal directions again (they might have been erased)
        tft.setTextColor(COLOR_TEXT);
        tft.setCursor(compass_x - 3, compass_y - indicator_size - 7);
        tft.print("N");
        tft.setCursor(compass_x + indicator_size + 2, compass_y - 3);
        tft.print("E");
        tft.setCursor(compass_x - 3, compass_y + indicator_size + 2);
        tft.print("S");
        tft.setCursor(compass_x - indicator_size - 7, compass_y - 3);
        tft.print("W");
        
        // Draw Roll/Pitch indicator in bottom-right quadrant
        // UPDATED: Moved the level indicator
        int level_x = 180; // Keep aligned with compass on x-axis
        int level_y = 115; // Moved down more
        
        // Erase old content
        tft.fillRect(level_x - indicator_size, level_y - indicator_size/2,
                     indicator_size * 2, indicator_size, COLOR_BG);
        
        // Draw level border (same size as compass)
        tft.drawRect(level_x - indicator_size, level_y - indicator_size/2, 
                     indicator_size * 2, indicator_size, COLOR_TEXT);
        
        // Calculate position based on roll and pitch (scale down to keep in bounds)
        int ball_x = level_x + (display_data.roll / 45.0) * (indicator_size - 5);
        int ball_y = level_y + (display_data.pitch / 45.0) * (indicator_size/2 - 5);
        
        // Constrain to box
        if (ball_x < level_x - indicator_size + 5) ball_x = level_x - indicator_size + 5;
        if (ball_x > level_x + indicator_size - 5) ball_x = level_x + indicator_size - 5;
        if (ball_y < level_y - indicator_size/2 + 5) ball_y = level_y - indicator_size/2 + 5;
        if (ball_y > level_y + indicator_size/2 - 5) ball_y = level_y + indicator_size/2 - 5;
        
        // Draw center crosshair
        tft.drawLine(level_x - 8, level_y, level_x + 8, level_y, COLOR_TEXT);
        tft.drawLine(level_x, level_y - 8, level_x, level_y + 8, COLOR_TEXT);
        
        // Draw level bubble
        tft.fillCircle(ball_x, ball_y, 5, COLOR_IMU);
        
        // Update previous values
        prev_heading = display_data.heading;
        prev_pitch = display_data.pitch;
        prev_roll = display_data.roll;
    }
}

// Display GPS page with minimal flicker
void displayGPSPage(void) {
    // Static variables to track previous values
    static char prev_time[10] = "";
    static char prev_date[12] = "";
    static int prev_fix = -1;
    static float prev_lat = -999;
    static float prev_lon = -999;
    static int prev_alt = -999;
    static int prev_satellites = -1;
    static int prev_speed = -1;
    
    // On first draw, display the static elements
    if (gps_page_first_draw) {
        // Clear the screen on first draw
        tft.fillScreen(COLOR_BG);
        
        // Page title
        tft.setTextSize(2);
        tft.setTextColor(COLOR_GPS);
        tft.setCursor(10, 10);
        tft.print("GPS Status");
        
        // Draw divider line
        tft.drawLine(0, 30, TFT_WIDTH, 30, COLOR_GPS);
        
        // Display static labels
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        
        // Time and date labels
        tft.setCursor(10, 40);
        tft.print("Time:");
        
        tft.setCursor(120, 40);
        tft.print("Date:");
        
        // Fix status label
        tft.setCursor(10, 55);
        tft.print("Fix:");
        
        // Satellites label
        tft.setCursor(120, 55);
        tft.print("Sats:");
        
        // Position labels
        tft.setCursor(10, 70);
        tft.print("Lat:");
        
        tft.setCursor(10, 85);
        tft.print("Lon:");
        
        tft.setCursor(10, 100);
        tft.print("Alt:");
        
        // Speed label - moved from 120 to 120 to ensure it's fully visible
        tft.setCursor(120, 70);
        tft.print("Speed:");
        
        // Speed gauge labels
        tft.setCursor(120, 95);
        tft.print("0");
        
        tft.setCursor(205, 95);
        tft.print("100");
        
        // Page indicator
        tft.setCursor(TFT_WIDTH - 60, TFT_HEIGHT - 15);
        tft.print("Page ");
        tft.print(current_page + 1);
        tft.print("/");
        tft.print(PAGE_COUNT);
        
        gps_page_first_draw = false;
        
        // Force initial update
        strcpy(prev_time, "");
        strcpy(prev_date, "");
        prev_fix = -1;
        prev_speed = -1;  // Force speed update on first draw
    }
    
    // Only update time if it's changed
    if (strcmp(prev_time, display_data.time) != 0) {
        tft.fillRect(50, 40, 60, 10, COLOR_BG);
        tft.setCursor(50, 40);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.time);
        strcpy(prev_time, display_data.time);
    }
    
    // Only update date if it's changed
    if (strcmp(prev_date, display_data.date) != 0) {
        tft.fillRect(160, 40, 80, 10, COLOR_BG);
        tft.setCursor(160, 40);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.date);
        strcpy(prev_date, display_data.date);
    }
    
    // Only update fix status if it's changed
    if (prev_fix != display_data.fix) {
        tft.fillRect(50, 55, 70, 10, COLOR_BG);
        tft.setCursor(50, 55);
        
        // Show fix status with appropriate color
        if (display_data.fix) {
            tft.setTextColor(COLOR_GOOD);
            tft.print("Valid");
            
            // Draw a small indicator
            tft.fillCircle(85, 57, 3, COLOR_GOOD);
        } else {
            tft.setTextColor(COLOR_ALERT);
            tft.print("No Fix");
            
            // Draw a small indicator
            tft.fillCircle(100, 57, 3, COLOR_ALERT);
        }
        
        prev_fix = display_data.fix;
    }
    
    // Only update satellites if changed
    if (prev_satellites != display_data.satellites) {
        tft.fillRect(160, 55, 30, 10, COLOR_BG);
        tft.setCursor(160, 55);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.satellites);
        
        prev_satellites = display_data.satellites;
    }
    
    // Only show location data if we have a fix
    if (display_data.fix) {
        // Update latitude if changed
        if (prev_lat != display_data.latitude) {
            tft.fillRect(50, 70, 80, 10, COLOR_BG);
            tft.setCursor(50, 70);
            tft.setTextColor(COLOR_VALUE);
            tft.print(display_data.latitude, 6);
            
            prev_lat = display_data.latitude;
        }
        
        // Update longitude if changed
        if (prev_lon != display_data.longitude) {
            tft.fillRect(50, 85, 80, 10, COLOR_BG);
            tft.setCursor(50, 85);
            tft.setTextColor(COLOR_VALUE);
            tft.print(display_data.longitude, 6);
            
            prev_lon = display_data.longitude;
        }
        
        // Update altitude if changed
        if (prev_alt != display_data.altitude) {
            tft.fillRect(50, 100, 60, 10, COLOR_BG);
            tft.setCursor(50, 100);
            tft.setTextColor(COLOR_VALUE);
            tft.print(display_data.altitude);
            tft.print(" ft");
            
            prev_alt = display_data.altitude;
        }
        
        // UPDATE: Always redraw the Speed label to ensure it's visible
        // Then update the speed value and gauge
        {
            // Make sure the Speed label is visible
            tft.setCursor(120, 70);
            tft.setTextColor(COLOR_TEXT);
            tft.print("Speed:");
            
            // Clear just the speed value area
            tft.fillRect(160, 70, 70, 10, COLOR_BG);
            tft.setCursor(160, 70);
            tft.setTextColor(COLOR_VALUE);
            tft.print(display_data.speed);
            tft.print(" mph");
            
            // Speed gauge (simple horizontal bar)
            int max_speed = 100; // Maximum speed for display scale
            int gauge_width = 100;
            int gauge_height = 8;
            int gauge_x = 120;
            int gauge_y = 85;
            
            // Always clear and redraw the entire gauge area to prevent artifacts
            tft.fillRect(gauge_x, gauge_y - 2, gauge_width + 2, gauge_height + 4, COLOR_BG);
            
            // Draw gauge outline
            tft.drawRect(gauge_x, gauge_y, gauge_width, gauge_height, COLOR_TEXT);
            
            // Fill based on speed (with limit)
            int speed_fill = (display_data.speed > max_speed) ? gauge_width : 
                            (display_data.speed * gauge_width / max_speed);
            
            // Choose color based on speed
            uint16_t speed_color;
            if (display_data.speed < 30) {
                speed_color = ST77XX_GREEN;
            } else if (display_data.speed < 60) {
                speed_color = ST77XX_YELLOW;
            } else {
                speed_color = ST77XX_RED;
            }
            
            // Always draw filled portion, even if speed is 0 (will draw a 0-width rectangle)
            if (speed_fill > 0) {
                tft.fillRect(gauge_x, gauge_y, speed_fill, gauge_height, speed_color);
            }
            
            // Update the speed scale indicators
            tft.setCursor(gauge_x, gauge_y + gauge_height + 2);
            tft.setTextColor(COLOR_TEXT);
            tft.print("0");
            
            tft.setCursor(gauge_x + gauge_width - 15, gauge_y + gauge_height + 2);
            tft.print("100");
            
            prev_speed = display_data.speed;
        }
    } else if (prev_fix != display_data.fix) {
        // If we just lost fix, clear the data area
        tft.fillRect(10, 70, TFT_WIDTH - 20, 60, COLOR_BG);
        
        // If no fix, show message
        tft.setCursor(10, 85);
        tft.setTextColor(COLOR_ALERT);
        tft.print("Waiting for satellite signal...");
    }
}

// Display environmental sensor page with minimal flicker
void displayEnviroPage(void) {
    // Static variables to track previous values
    static float prev_temp = -999;
    static float prev_humidity = -999;
    static float prev_pressure = -999;
    static float prev_air_quality = -999;
    static char prev_aq_desc[15] = "";
    
    // Additional values for ENS160 sensor
    #ifdef USE_ENS160_AHT21
    static int prev_eco2 = -1;
    static int prev_tvoc = -1;
    #endif
    
    // Draw static elements on first load
    if (enviro_page_first_draw) {
        // Clear screen
        tft.fillScreen(COLOR_BG);
        
        // Page title
        tft.setTextSize(2);
        tft.setTextColor(COLOR_ENVIRO);
        tft.setCursor(10, 10);
        tft.print("Environment");
        
        // Draw divider line
        tft.drawLine(0, 30, TFT_WIDTH, 30, COLOR_ENVIRO);
        
        // Static labels
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        
        tft.setCursor(10, 40);
        tft.print("Temp:");
        
        tft.setCursor(10, 55);
        tft.print("Humidity:");
        
        #ifndef USE_ENS160_AHT21
        // Only show pressure for BME680
        tft.setCursor(10, 70);
        tft.print("Pressure:");
        #else
        // Show eCO2 and TVOC for ENS160
        tft.setCursor(10, 70);
        tft.print("eCO2:");
        
        tft.setCursor(120, 70);
        tft.print("TVOC:");
        #endif
        
        tft.setCursor(10, 85);
        tft.print("Air Quality:");
        
        // Page indicator
        tft.setCursor(TFT_WIDTH - 60, TFT_HEIGHT - 15);
        tft.print("Page ");
        tft.print(current_page + 1);
        tft.print("/");
        tft.print(PAGE_COUNT);
        
        enviro_page_first_draw = false;
        
        // Force initial update
        prev_temp = -999;
        prev_humidity = -999;
        prev_pressure = -999;
        prev_air_quality = -999;
        strcpy(prev_aq_desc, "");
        
        #ifdef USE_ENS160_AHT21
        prev_eco2 = -1;
        prev_tvoc = -1;
        #endif
    }
    
    // Only update temperature if changed
    if (prev_temp != display_data.temperature) {
        tft.fillRect(120, 40, 120, 10, COLOR_BG);
        tft.setCursor(120, 40);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.temperature, 1);
        tft.print("C / ");
        tft.print(display_data.temperature * 9.0/5.0 + 32.0, 1);
        tft.print("F");
        
        prev_temp = display_data.temperature;
    }
    
    // Only update humidity if changed
    if (prev_humidity != display_data.humidity) {
        tft.fillRect(120, 55, 70, 10, COLOR_BG);
        tft.setCursor(120, 55);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.humidity, 1);
        tft.print("%");
        
        prev_humidity = display_data.humidity;
    }
    
    #ifndef USE_ENS160_AHT21
    // Only update pressure if changed (BME680 only)
    if (prev_pressure != display_data.pressure) {
        tft.fillRect(120, 70, 70, 10, COLOR_BG);
        tft.setCursor(120, 70);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.pressure, 1);
        tft.print(" hPa");
        
        prev_pressure = display_data.pressure;
    }
    #else
    // Update eCO2 and TVOC for ENS160
    if (prev_eco2 != display_data.eco2) {
        tft.fillRect(50, 70, 60, 10, COLOR_BG);
        tft.setCursor(50, 70);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.eco2);
        tft.print(" ppm");
        
        prev_eco2 = display_data.eco2;
    }
    
    if (prev_tvoc != display_data.tvoc) {
        tft.fillRect(160, 70, 70, 10, COLOR_BG);
        tft.setCursor(160, 70);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.tvoc);
        tft.print(" ppb");
        
        prev_tvoc = display_data.tvoc;
    }
    #endif
    
    // Only update air quality if changed
    if (prev_air_quality != display_data.air_quality || 
        strcmp(prev_aq_desc, display_data.air_quality_description) != 0) {
        
        tft.fillRect(120, 85, 120, 10, COLOR_BG);
        tft.setCursor(120, 85);
        
        // Set color based on air quality
        uint16_t aq_color;
        if (display_data.air_quality > 80) {
            aq_color = COLOR_GOOD;
        } else if (display_data.air_quality > 60) {
            aq_color = ST77XX_CYAN;
        } else if (display_data.air_quality > 40) {
            aq_color = ST77XX_YELLOW;
        } else if (display_data.air_quality > 20) {
            aq_color = ST77XX_ORANGE;
        } else {
            aq_color = COLOR_ALERT;
        }
        
        tft.setTextColor(aq_color);
        tft.print(display_data.air_quality, 1);
        tft.print(" (");
        tft.print(display_data.air_quality_description);
        tft.print(")");
        
        // Redraw air quality bar
        int bar_width = 200;
        int bar_height = 10;
        int bar_x = 20;
        int bar_y = 100;
        
        // Clear old bar
        tft.fillRect(bar_x, bar_y, bar_width, bar_height, COLOR_BG);
        
        // Draw outline
        tft.drawRect(bar_x, bar_y, bar_width, bar_height, COLOR_TEXT);
        
        // Fill based on air quality (0-100)
        int fill_width = (display_data.air_quality / 100.0) * bar_width;
        tft.fillRect(bar_x, bar_y, fill_width, bar_height, aq_color);
        
        prev_air_quality = display_data.air_quality;
        strcpy(prev_aq_desc, display_data.air_quality_description);
    }
}

// Display WiFi status page with minimal flicker
void displayWiFiPage(void) {
    // Static variables to track previous values
    static char prev_ssid[33] = "";
    static char prev_ip[16] = "";
    static int prev_rssi = 999;
    static bool prev_connected = false;
    
    // Draw static elements on first load
    if (wifi_page_first_draw) {
        // Clear screen
        tft.fillScreen(COLOR_BG);
        
        // Page title
        tft.setTextSize(2);
        tft.setTextColor(COLOR_WIFI);
        tft.setCursor(10, 10);
        tft.print("WiFi Status");
        
        // Draw divider line
        tft.drawLine(0, 30, TFT_WIDTH, 30, COLOR_WIFI);
        
        // Static labels
        tft.setTextSize(1);
        tft.setTextColor(COLOR_TEXT);
        
        tft.setCursor(10, 40);
        tft.print("SSID:");
        
        tft.setCursor(10, 55);
        tft.print("Status:");
        
        tft.setCursor(10, 70);
        tft.print("IP:");
        
        tft.setCursor(10, 85);
        tft.print("RSSI:");
        
        // Page indicator
        tft.setCursor(TFT_WIDTH - 60, TFT_HEIGHT - 15);
        tft.print("Page ");
        tft.print(current_page + 1);
        tft.print("/");
        tft.print(PAGE_COUNT);
        
        wifi_page_first_draw = false;
        
        // Force initial update
        strcpy(prev_ssid, "");
        strcpy(prev_ip, "");
        prev_rssi = 999;
        prev_connected = !display_data.is_connected;
    }
    
    // Only update SSID if changed
    if (strcmp(prev_ssid, display_data.ssid) != 0) {
        tft.fillRect(120, 40, 120, 10, COLOR_BG);
        tft.setCursor(120, 40);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.ssid);
        strcpy(prev_ssid, display_data.ssid);
    }
    
    // Only update connection status if changed
    if (prev_connected != display_data.is_connected) {
        tft.fillRect(120, 55, 100, 10, COLOR_BG);
        tft.setCursor(120, 55);
        
        // Show connection status with appropriate color
        if (display_data.is_connected) {
            tft.setTextColor(COLOR_GOOD);
            tft.print("Connected");
        } else {
            tft.setTextColor(COLOR_ALERT);
            tft.print("Disconnected");
        }
        
        prev_connected = display_data.is_connected;
    }
    
    // Only update IP address if changed
    if (strcmp(prev_ip, display_data.ip_address) != 0) {
        tft.fillRect(120, 70, 120, 10, COLOR_BG);
        tft.setCursor(120, 70);
        tft.setTextColor(COLOR_VALUE);
        tft.print(display_data.ip_address);
        strcpy(prev_ip, display_data.ip_address);
    }
    
    // Only update RSSI if changed
    if (prev_rssi != display_data.rssi) {
        tft.fillRect(120, 85, 60, 10, COLOR_BG);
        tft.setCursor(120, 85);
        
        // Color code signal strength
        int signal_strength = display_data.rssi;
        uint16_t signal_color;
        
        if (signal_strength > -50) {
            signal_color = COLOR_GOOD;
        } else if (signal_strength > -70) {
            signal_color = ST77XX_YELLOW;
        } else {
            signal_color = COLOR_ALERT;
        }
        
        tft.setTextColor(signal_color);
        tft.print(signal_strength);
        tft.print(" dBm");
        
        // UPDATED: Draw WiFi signal strength bars - moved up to avoid overlap
        int base_x = 180;  // Moved more to the right
        int base_y = 105;   // Moved up to align with RSSI text
        int bar_width = 10; // Slightly smaller bars
        int bar_spacing = 3; // Closer together
        int max_height = 15; // Slightly shorter
        
        // Clear previous bars
        tft.fillRect(base_x, base_y - max_height, (bar_width + bar_spacing) * 4, max_height + 5, COLOR_BG);
        
        // Draw 4 bars with varying heights
        for (int i = 0; i < 4; i++) {
            int height = (i + 1) * (max_height / 4);
            int x = base_x + i * (bar_width + bar_spacing);
            int y = base_y + (max_height - height) - max_height; // Position above the text line
            
            // Determine if this bar should be filled based on signal strength
            if (display_data.is_connected && ((signal_strength > -90 && i == 0) || 
               (signal_strength > -70 && i == 1) || 
               (signal_strength > -60 && i == 2) || 
               (signal_strength > -50 && i == 3))) {
                tft.fillRect(x, y, bar_width, height, signal_color);
            } else {
                // Draw outline only for inactive bars
                tft.drawRect(x, y, bar_width, height, COLOR_TEXT);
            }
        }
        
        prev_rssi = display_data.rssi;
    }
}

// Main display task - handles page rotation and refresh
void displayTask(void *pvParameters) {
    LOG_PRINTLN(F("Display task started."));
    
    int current_display_page = -1;  // Force initial draw
    unsigned long last_refresh = 0;
    
    while (true) {
        unsigned long current_time = millis();
        
        // Take the display mutex
        if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
            // Check if it's time to change the page
            if (current_time - last_page_change >= PAGE_CHANGE_INTERVAL) {
                // Move to next page
                current_page = (current_page + 1) % PAGE_COUNT;
                last_page_change = current_time;
                
                // Reset the first-draw flags to force redraw of new page
                resetPageDrawFlags();
                
                LOG_PRINT("Changing to page: ");
                LOG_PRINTLN(String(current_page));
            }
            
            // Determine if we should refresh the display now
            bool should_refresh = (current_display_page != current_page) ||
                                  (current_time - last_refresh >= DISPLAY_REFRESH_RATE_MS);
            
            if (should_refresh) {
                current_display_page = current_page;
                
                // Draw the current page - no need to clear as each page handles its own drawing
                switch (current_page) {
                    case PAGE_IMU:
                        displayIMUPage();
                        break;
                    case PAGE_GPS:
                        displayGPSPage();
                        break;
                    case PAGE_ENVIRO:
                        displayEnviroPage();
                        break;
                    case PAGE_WIFI:
                        displayWiFiPage();
                        break;
                    default:
                        // Should never get here
                        current_page = PAGE_IMU;
                        break;
                }
                
                last_refresh = current_time;
            }
            
            // Release the mutex
            xSemaphoreGive(displayMutex);
        }
        
        // Sleep for a short period
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// Updates display_data with the latest information
// This should be called from each sensor task or loop after new readings
void updateDisplayData(void) {
    // This function should be called by other tasks to update the display data
    // It must acquire the mutex before modifying display_data
    
    if (xSemaphoreTake(displayMutex, portMAX_DELAY) == pdTRUE) {
        // Here, the calling function would update specific fields of display_data
        // but we don't do that in this function itself
        
        // Release the mutex
        xSemaphoreGive(displayMutex);
    }
}