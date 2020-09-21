/*
 * ws2812.h
 *
 *  Created on: September 21, 2020
 *      Author: Dmitriy Semenov
 *		Github: https://github.com/Crazy-Geeks/STM32-WS2812B-DMA
 *		License: MIT
 */

#ifndef SRC_WS2812_H_
#define SRC_WS2812_H_

//--------------------------------------------------
/*INCLUDES*/
#include "stm32f1xx_hal.h"
#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
//--------------------------------------------------
/*SYS DEFINES*/
#define DELAY_LEN 48						// Delay period
#define ARRAY_LEN DELAY_LEN + LED_COUNT*24	// Length of DMA Buffer
#define HIGH 65								// HIGH Byte period
#define LOW 26								// LOW Byte period
//--------------------------------------------------
/*USR SETTINGS*/
#define LED_COUNT 12						// Count of led in your strip (LED_COUNT%12 = 0)
#define BRIGHT 10 							// 0-255
//--------------------------------------------------
#define BitIsSet(reg, bit) ((reg & (1<<bit)) != 0)	// Search Bit in integer
//--------------------------------------------------

void led_init(void);		// Initalization
void led_set(uint8_t Rpixel , uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);  // Draw pixel in X position by RGB
void led_fill (uint8_t Rpix, uint8_t Gpix, uint8_t Bpix);	// Fill strip with color
void led_clear(void);	// Fill strip with black
void led_show(void);	// Recieve DMA Buffer to strip
#endif /* SRC_WS2812_H_ */
