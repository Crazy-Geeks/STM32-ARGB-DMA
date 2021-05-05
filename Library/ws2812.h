/*
 * ws2812.h
 *
 *  Created on: May 04, 2021
 *      Author: Dmitriy Semenov
 *      Github: https://github.com/Crazy-Geeks/STM32-WS2812B-DMA
 *      Main idea: Frank VFD
 *		Main repo: https://github.com/hey-frnk/STM32_HAL_NeoPixel
 *		License: MIT
 *		(c) #Crazy_Geeks
 */

#ifndef SRC_WS2812_H_
#define SRC_WS2812_H_

//--------------------------------------------------
/*INCLUDES*/
#include "stm32f1xx_hal.h"
#include "libs.h"

#define NUM_PIXELS 24			// length of your strip
#define RGB 					// define RGB or RGBW
#define USE_GAMMA_CORRECTION 0	// corrects red & green color, try for your strip

/* Will be fixed in future versions */
//#define USE_HSV_FLOAT 0			// uses float model for HSV
/* NOTE! Float HSV model is smoother, but it perfomance only if you have FPU */

/* TIMER */
#define TIM_HANDLE  htim2				// Select your timer
#define TIM_NUM		2
#define TIM_POINTER TIM2
/* TIMER CHANNEL */
#define TIM_CH	   TIM_CHANNEL_2
/* DMA CHANNEL */
#define DMA_HANDLE  hdma_tim2_ch2_ch4 	// search for it in main.c


void led_init(void);	// Initalization
void led_clear(void);	// Fill strip with black
void led_setRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b); 	// Set one pixel with RGB color
void led_setHSV(uint16_t index, uint8_t hue, uint8_t sat, uint8_t val);	// Set one pixel with HSV color
void led_setRGBW(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w); // Set one pixel with RGB color for RGBW strip
void led_fillRGB(uint8_t r, uint8_t g, uint8_t b);	// Fill strip with RGB color
void led_fillHSV(uint8_t hue, uint8_t sat, uint8_t val);	// Fill strip with HSV color
void led_fillRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);	// Fill strip with RGB color for RGBW strip
bool led_show(void);	// Send color buffer to the strip - use with while!



//#define TIM_HANDLE ((TIM_HandleTypeDef *)TIM_NUM);

#endif /* SRC_WS2812_H_ */
