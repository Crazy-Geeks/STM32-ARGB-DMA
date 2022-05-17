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

#ifndef ARGB_H_
#define ARGB_H_

#include "libs.h"

/**
 * @addtogroup ARGB_Driver
 * @brief Addressable RGB LED Driver
 * @{
 * @addtogroup User_settings
 * @brief LED & Timer's settings
 * @{
 */

#define WS2812       ///< Family: {WS2811S, WS2811F, WS2812, SK6812}
// WS2811S — RGB, 400kHz;
// WS2811F — RGB, 800kHz;
// WS2812  — GRB, 800kHz;
// SK6812  — RGBW, 800kHz

#define NUM_PIXELS 5 ///< Pixel quantity

#define USE_GAMMA_CORRECTION 1 ///< Gamma-correction should fix red&green, try for yourself

#define TIM_NUM	   2  ///< Timer number
#define TIM_CH	   TIM_CHANNEL_2  ///< Timer's PWM channel
#define DMA_HANDLE hdma_tim2_ch2_ch4  ///< DMA Channel
#define DMA_SIZE_WORD     ///< DMA Memory Data Width: {.._BYTE, .._HWORD, .._WORD}
// DMA channel can be found in main.c / tim.c

/// @}

/**
 * @addtogroup Global_entities
 * @brief All driver's methods
 * @{
 * @enum ARGB_STATE
 * @brief Driver's status enum
 */
typedef enum ARGB_STATE {
    ARGB_BUSY = 0,  ///< DMA Transfer in progress
    ARGB_READY = 1, ///< DMA Ready to transfer
    ARGB_OK = 2,    ///< Function execution success
    ARGB_PARAM_ERR = 3, ///< Error in input parameters
} ARGB_STATE;

void ARGB_Init(void);   // Initialization
void ARGB_Clear(void);  // Clear strip

void ARGB_SetBrightness(u8_t br); // Set global brightness

void ARGB_SetRGB(u16_t i, u8_t r, u8_t g, u8_t b);  // Set single LED by RGB
void ARGB_SetHSV(u16_t i, u8_t hue, u8_t sat, u8_t val); // Set single LED by HSV
void ARGB_SetWhite(u16_t i, u8_t w); // Set white component in LED (RGBW)

void ARGB_FillRGB(u8_t r, u8_t g, u8_t b); // Fill all strip with RGB color
void ARGB_FillHSV(u8_t hue, u8_t sat, u8_t val); // Fill all strip with HSV color
void ARGB_FillWhite(u8_t w); // Fill all strip's white component (RGBW)

ARGB_STATE ARGB_Ready(void); // Get DMA Ready state
ARGB_STATE ARGB_Show(void); // Push data to the strip

/// @} @}
#endif /* ARGB_H_ */