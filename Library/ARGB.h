/**
 *******************************************
 * @file    ARGB.h
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @link    https://crazygeeks.ru
 * @version 1.33
 * @date	17-May-2022
 * @brief   Header file for ARGB Driver (Addressable RGB)
 *******************************************
 *
 * @note Repo: https://github.com/Crazy-Geeks/STM32-ARGB-DMA
 * @note RU article: https://crazygeeks.ru/stm32-argb-lib
 */

/**
 * Significant history (date, user, job code, action):
 * - 2022-10-18, Dylan Arrabito  <darrabito@carngeigerobotics.com>, IRAD.8015.1, Greatly modified to support ChibiOS.
 */

#pragma once

#include "hal.h"
#include "board.h"

/**
 * @addtogroup ARGB_Driver
 * @brief Addressable RGB LED Driver
 * @{
 * @addtogroup User_settings
 * @brief LED & Timer's settings
 * @{
 */


#define NUM_PIXELS NUM_LEDS ///< Pixel quantity

#define USE_GAMMA_CORRECTION 1 ///< Gamma-correction should fix red&green, try for yourself

#define TIM_CHANNEL_1 0
#define TIM_CHANNEL_2 1
#define TIM_CHANNEL_3 2
#define TIM_CHANNEL_4 3

#define TIM_HANDLE LED_TIMER

#define LED_SIGNAL_RISE_DELAY_US LED_PWM_RISE_DELAY_US

/// @}

/**
 * @addtogroup Global_entities
 * @brief All driver's methods
 * @{
 * @enum argb_state
 * @brief Driver's status enum
 */
typedef enum argb_state {
    ARGB_BUSY = 0,  ///< DMA Transfer in progress
    ARGB_READY = 1, ///< DMA Ready to transfer
    ARGB_OK = 2,    ///< Function execution success
    ARGB_PARAM_ERR = 3, ///< Error in input parameters
} argb_state;

// stolen from https://github.com/FastLED/FastLED
typedef struct {
	union {
		struct {
            union {
                uint8_t r;
                uint8_t red;
            };
            union {
                uint8_t g;
                uint8_t green;
            };
            union {
                uint8_t b;
                uint8_t blue;
            };
        };
		uint8_t raw[3];
	};
} rgb_t;

typedef struct {
    union {
		struct {
		    union {
		        uint8_t hue;
		        uint8_t h; };
		    union {
		        uint8_t saturation;
		        uint8_t sat;
		        uint8_t s; };
		    union {
		        uint8_t value;
		        uint8_t val;
		        uint8_t v; };
		};
		uint8_t raw[3];
	};
} hsv_t;

typedef enum 
{
    HUE_RED    = 0,
    HUE_YELLOW = 42,
    HUE_GREEN  = 85,
    HUE_AQUA   = 128,
    HUE_BLUE   = 171,
    HUE_PURPLE = 213
    // HUE_RED = 0,
    // HUE_ORANGE = 32,
    // HUE_YELLOW = 64,
    // HUE_GREEN = 96,
    // HUE_AQUA = 128,
    // HUE_BLUE = 160,
    // HUE_PURPLE = 192,
    // HUE_PINK = 224
} hsv_hue;

void argb_init(void);   // Initialization
void argb_clear(void);  // Clear strip

void argb_set_brightness(uint8_t br); // Set global brightness

void argb_set_rgb(uint16_t i, uint8_t r, uint8_t g, uint8_t b);  // Set single LED by RGB
void argb_set_hsv(uint16_t i, hsv_hue hue, uint8_t sat, uint8_t val); // Set single LED by HSV
void argb_set_white(uint16_t i, uint8_t w); // Set white component in LED (RGBW)

void argb_fill_rgb_range(uint16_t start, uint16_t end, uint8_t r, uint8_t g, uint8_t b); 
void argb_fill_rgb(uint8_t r, uint8_t g, uint8_t b); // Fill all strip with RGB color
void argb_fill_hsv_range(uint16_t start, uint16_t end, uint8_t hue, uint8_t sat, uint8_t val); 
void argb_fill_hsv(hsv_hue hue, uint8_t sat, uint8_t val); // Fill all strip with HSV color
void argb_fill_white_range(uint16_t start, uint16_t end, uint8_t w);
void argb_fill_white(uint8_t w); // Fill all strip's white component (RGBW)

void hsv2rgb_spectrum( const hsv_t hsv, rgb_t * rgb);

hsv_t argb_get_hue(uint16_t i);

argb_state argb_ready(void); // Get DMA Ready state
argb_state argb_show(void); // Push data to the strip

/// @} @}
