/*********************************************
 *                IMP PROJECT                *
 *                Tachometer                 *
 *               Michal Belovec              *
 *                                           *
 *********************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "nvs.h"

#include "ssd1306.h"

// DISPLAY PIN SETUP
#define SCLK 18  //D0
#define MOSI 23  //D1
#define CS 5     //CS
#define DC 27    //DC
#define RESET 17 //RES

// BUTTON PIN SETUP
#define BUT1 26
#define BUT2 14

// DISPLAY MODES
#define ALL 0
#define SPEED 1
#define DIST 2
#define AVGSPEED 3

// BICYCLE WHEEL CIRCUMFERENCE - METRES
#define CIRCUMFERENCE 2.07

// FUNCTION DECLARATIONS
void ButtonSetup();
void NVS();
void UpdateBuffers();
void UpdateNVSData();
void ResetData();
void RewriteDisplayData();
void SwitchDisplayMode();

// BUTTON N.1 
int lastSteadyState1 = 0;
int lastFlickerableState1 = 0;
int currentState1;
unsigned long lastDebounceTime1 = 0;
unsigned long holdTime = 0;

// BUTTON N.2
int lastSteadyState2 = 0;
int lastFlickerableState2 = 0;
int currentState2;
unsigned long lastDebounceTime2 = 0;

// BICYCLE STATS
double SpeedMS = 0.0;
double SpeedKMH = 0.0;
double AvgSpeed = 0.0;
double Distance = 0.0;
int RPS = 0; // ROTATION PER SECOND
double lastSpeed1 = 0.0;
unsigned long periodTime;
int32_t totalWorkTime;

// BICYCLE STAT CHAR ARRAYS
char SpdBuffer[10];
char SpdBufferAll[20];
char AvgBuffer[10];
char AvgBufferAll[20];
char DistBuffer[10];
char DistBufferAll[20];
char TimeBufferAll[30];

// NVS VARS
esp_err_t NVS_err;
nvs_handle_t NVSHandle;
size_t sizeDst;
int32_t IntDistance = 0;

// SCREEN MODE
int ScreenMode = 0;

// SCREEN VARIABLE
SSD1306_t dev;

// LAST DISPLAY REWRITE RECORD
unsigned long lastRewrite = 0;

void app_main() {
    
    totalWorkTime = 0;

    // GPIO BUTTON SETTINGS
    ButtonSetup();

    // LOAD DATA FROM NVS
    NVS();

    // WRITE DATA ACQUIRED FROM NVS
    Distance = IntDistance / 1000.0;

    // INITITALIZE DISPLAY
    spi_master_init(&dev, MOSI, SCLK, CS, DC, RESET);
    ssd1306_init(&dev, 128, 64);

    // INITIAL SCREEN TEXT
    ssd1306_clear_screen(&dev, false);
	ssd1306_contrast(&dev, 0xff);
	ssd1306_display_text(&dev, 2, "IMP - Tachometer", 17, false);
    ssd1306_display_text(&dev, 5, "Author: xbelov04", 17, false);
	vTaskDelay(3000 / portTICK_PERIOD_MS);

    // INITIALIZE PERIOD TIMER
    periodTime = esp_timer_get_time() / 1000;

    while (1) {
        // CHECK BUTTON LEVELS
        currentState1 = gpio_get_level(BUT1);
        currentState2 = gpio_get_level(BUT2);

        // Clicks per minute counter
        if(((esp_timer_get_time() / 1000) - periodTime) > 1000) {
            // GET PROGRAM RUN TIME
            totalWorkTime++;

            // SHIFTING SPEED RECORDS BY 1 SECOND
            lastSpeed1 = SpeedMS;
            SpeedMS = ((RPS*CIRCUMFERENCE + lastSpeed1) / 2);

            // UPDATE TRAVELED DISTANCE 
            Distance += SpeedMS / 1000;

            // CONVERT M/S TO KM/H
            SpeedKMH = SpeedMS*3.6;

            // UPDATE AVERAGE SPEED
            AvgSpeed = Distance / (totalWorkTime/3600.0);

            // UPDATE BUFFERS
            UpdateBuffers();

            // RESET ROTATION COUNTER
            RPS = 0;

            // UPDATE NVS RECORDS
            IntDistance = (int32_t) (Distance*1000);
            UpdateNVSData();

            // UPDATE PERIOD TIME
            periodTime = esp_timer_get_time() / 1000;
        }

        if (currentState1 != lastFlickerableState1) {
            lastDebounceTime1 = esp_timer_get_time() / 1000;
            holdTime = esp_timer_get_time() / 1000;
            
            // save the the last flickerable state
            lastFlickerableState1 = currentState1;
        }

        // RESET BUTTON DETECTION
        if(lastSteadyState1 == 0 && currentState1 == 0 && ((esp_timer_get_time() / 1000) - holdTime) > 2000) {
            ResetData();
        }

        // REACTION FOR BUTTON N.1 PRESS
        if (((esp_timer_get_time() / 1000) - lastDebounceTime1) > 10) {
            if(lastSteadyState1 == 1 && currentState1 == 0) {
                SwitchDisplayMode();
            } 
            lastSteadyState1 = currentState1;
        }

        if (currentState2 != lastFlickerableState2) {
            lastDebounceTime2 = esp_timer_get_time() / 1000;
            
            // save the the last flickerable state
            lastFlickerableState2 = currentState2;
        }

        // REACTION FOR BUTTON N.2 PRESS
        if (((esp_timer_get_time() / 1000) - lastDebounceTime2) > 10) {
            if(lastSteadyState2 == 1 && currentState2 == 0) {
                RPS++;
            } 
            lastSteadyState2 = currentState2;
        }

        // ACTUALIZING DISPLAY DATA
        if(((esp_timer_get_time() / 1000) - lastRewrite) > 500) {
            RewriteDisplayData();
        }

        vTaskDelay(1);
    }
}

// Set up button pin modes.
void ButtonSetup() {

    gpio_set_direction(BUT1, GPIO_MODE_INPUT);
    gpio_set_direction(BUT2, GPIO_MODE_INPUT);
    gpio_set_pull_mode(BUT1, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode(BUT2, GPIO_PULLUP_ONLY);

}

// Initialize, open and load/save data to NVS.
// Saves distance and time in NVS.
void NVS() {

    // INITIALIZE NVS
    NVS_err = nvs_flash_init();
    if (NVS_err == ESP_ERR_NVS_NO_FREE_PAGES || NVS_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        NVS_err = nvs_flash_init();
    }

    // OPEN NVS
    NVS_err = nvs_open("tachometer", NVS_READWRITE, &NVSHandle);
    if(NVS_err == ESP_ERR_NVS_NOT_INITIALIZED) {
        NVS_err = nvs_flash_init();
        if(NVS_err == ESP_ERR_NVS_NO_FREE_PAGES || NVS_err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            NVS_err = nvs_flash_init();
        }
        NVS_err = nvs_open("tachometer", NVS_READWRITE, &NVSHandle);
        ESP_ERROR_CHECK(NVS_err);
    }

    // GET/SET DISTANCE DATA
    NVS_err = nvs_get_i32(NVSHandle, "Distance", &IntDistance);
    if(NVS_err == ESP_ERR_NVS_NOT_FOUND) {
        // WRITE DISTANCE INTO NVS
        NVS_err = nvs_set_i32(NVSHandle, "Distance", IntDistance);
        if(NVS_err != ESP_OK){
            printf("nvs_set_i32 failed\n");
        }
    }

    // GET/SET TIME DATA
    NVS_err = nvs_get_i32(NVSHandle, "Time", &totalWorkTime);
    if(NVS_err == ESP_ERR_NVS_NOT_FOUND) {
        // WRITE TIME INTO NVS
        NVS_err = nvs_set_i32(NVSHandle, "Time", totalWorkTime);
        if(NVS_err != ESP_OK){
            printf("nvs_set_i32 failed\n");
        }
    }

    // COMMIT CHANGES
    NVS_err = nvs_commit(NVSHandle);

    // Close NVS handle
    nvs_close(NVSHandle);
}

// Updates buffer content
void UpdateBuffers() {
    sprintf(SpdBuffer, "%.2f", SpeedKMH);
    sprintf(SpdBufferAll, " SPD: %.2fkm/h", SpeedKMH);
    sprintf(DistBuffer, "%.2f", Distance);
    sprintf(DistBufferAll, "DIST: %.2fkm", Distance);
    sprintf(AvgBuffer, "%.2f", AvgSpeed);
    sprintf(AvgBufferAll, " AVG: %.2fkm/h", AvgSpeed);
    sprintf(TimeBufferAll, "TIME: %02lu:%02lu:%02lu", totalWorkTime/3600, totalWorkTime/60, totalWorkTime%60);
}

// Updates data stored in the NVS
void UpdateNVSData() {
    NVS_err = nvs_open("tachometer", NVS_READWRITE, &NVSHandle);
    NVS_err = nvs_set_i32(NVSHandle, "Time", totalWorkTime);
    if(NVS_err != ESP_OK) {
        printf("nvs_set_i32 failed\n");
    }
    NVS_err = nvs_set_i32(NVSHandle, "Distance", IntDistance);
    if(NVS_err != ESP_OK) {
        printf("nvs_set_i32 failed\n");
    }
    NVS_err = nvs_commit(NVSHandle);
    if(NVS_err != ESP_OK) {
        printf("nvs_commit failed\n");
    }
    nvs_close(NVSHandle);
}

// Reaction for holding button longer than 2 seconds.
void ResetData() {
    holdTime = esp_timer_get_time() / 1000;
    Distance = 0.0;
    totalWorkTime = 0;
    AvgSpeed = 0.0;
    SpeedMS = 0.0;
    SpeedKMH = 0.0;
    lastSpeed1 = 0.0;
    ScreenMode = ALL;
}

// Actualize screen data.
void RewriteDisplayData() {
    if(ScreenMode == ALL) {
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 1, SpdBufferAll, 20, false);
        ssd1306_display_text(&dev, 3, AvgBufferAll, 20, false);
        ssd1306_display_text(&dev, 5, DistBufferAll, 20, false);
        ssd1306_display_text(&dev, 7, TimeBufferAll, 20, false);
        lastRewrite = esp_timer_get_time() / 1000;
    }
    else if(ScreenMode == SPEED) {
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 1, "SPEED:", 6, false);
        ssd1306_display_text_x3(&dev, 3, SpdBuffer, 10, false);
        ssd1306_display_text(&dev, 6, "km/h", 4, false);
        lastRewrite = esp_timer_get_time() / 1000;
    }
    else if(ScreenMode == DIST) {
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 1, "DISTANCE:", 10, false);
        ssd1306_display_text_x3(&dev, 3, DistBuffer, 10, false);
        ssd1306_display_text(&dev, 6, "kilometres", 11, false);
        lastRewrite = esp_timer_get_time() / 1000;
    }
    else if(ScreenMode == AVGSPEED) {
        ssd1306_clear_screen(&dev, false);
        ssd1306_display_text(&dev, 1, "AVERAGE SPEED:", 15, false);
        ssd1306_display_text_x3(&dev, 3, AvgBuffer, 10, false);
        ssd1306_display_text(&dev, 6, "km/h", 4, false);
        lastRewrite = esp_timer_get_time() / 1000;
    }
}

void SwitchDisplayMode() {
    switch(ScreenMode) {
        case ALL:
            ScreenMode = SPEED;
            break;
        case SPEED:
            ScreenMode = DIST;
            break;
        case DIST:
            ScreenMode = AVGSPEED;
            break;
        case AVGSPEED:
            ScreenMode = ALL;
            break;
        default:
            break;
    }
}

