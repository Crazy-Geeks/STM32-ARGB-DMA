/*
 * ws2812.c
 *
 *  Created on: September 21, 2020
 *      Author: Dmitriy Semenov
 *		Github: https://github.com/Crazy-Geeks/STM32-WS2812B-DMA
 *		License: MIT
 *		(c) #Crazy_Geeks
 */

#include "ws2812.h"  // include header file
//----------------------------------------------------------------------------

extern TIM_HandleTypeDef (TIM_HANDLE);
extern DMA_HandleTypeDef (DMA_HANDLE);

volatile uint8_t PWM_HI;
volatile uint8_t PWM_LO;

// LED parameters
#ifdef RGB
#define NUM_BPP (3) // WS2812B
#endif
#ifdef RGBW
#define NUM_BPP (4) // SK6812
#endif
#define NUM_BYTES (NUM_BPP * NUM_PIXELS)

// LED color buffer
uint8_t rgb_arr[NUM_BYTES] = { 0 };

// LED write buffer
#define WR_BUF_LEN (NUM_BPP * 8 * 2)
uint8_t wr_buf[WR_BUF_LEN] = { 0 };
uint_fast16_t wr_buf_p = 0;

// Private function for gamma correction
static inline uint8_t scale8(uint8_t x, uint8_t scale) {
	return ((uint16_t) x * scale) >> 8;
}

// Private function for HSV-RGB conversion
static void HSV2RGB(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r,
		uint8_t *_g, uint8_t *_b) {
	/* Source:
	 * https://stackoverflow.com/questions/3018313/algorithm-to-convert-rgb-to-hsv-and-hsv-to-rgb-in-range-0-255-for-both
	 */

	if (sat == 0) { // if white color
		_r[0] = _g[0] = _b[0] = val;
		return;
	}

#if USE_HSV_FLOAT  /* DOESN'T WORK NOW */
	// Float is smoother but check for FPU (Floating point unit) in your MCU
	// Otherwise it will take longer time in the code
	// FPU is in: F3/L3 and greater
	float _hue = (float) hue * (360.0f / 255.0f);
	float _sat = (float) sat * (1.0f / 255.0f);
	float _val = (float) val * (1.0f / 255.0f);

	//float s = sat / 100;
	//float v = val / 100;
	float C = _sat * _val;
	float X = C * (1 - abs(fmod(_hue / 60.0, 2) - 1));
	float m = _val - C;
	float r, g, b;
	if (_hue >= 0 && _hue < 60) {
		r = C, g = X, b = 0;
	} else if (_hue >= 60 && _hue < 120) {
		r = X, g = C, b = 0;
	} else if (_hue >= 120 && _hue < 180) {
		r = 0, g = C, b = X;
	} else if (_hue >= 180 && _hue < 240) {
		r = 0, g = X, b = C;
	} else if (_hue >= 240 && _hue < 300) {
		r = X, g = 0, b = C;
	} else {
		r = C, g = 0, b = X;
	}
	_r[0] = (r + m) * 255;
	_g[0] = (g + m) * 255;
	_b[0] = (b + m) * 255;
#endif

//#if !USE_HSV_FLOAT
	uint8_t reg = hue / 43;
	uint8_t rem = (hue - (reg * 43)) * 6;

	uint8_t p = (val * (255 - sat)) >> 8;
	uint8_t q = (val * (255 - ((sat * rem) >> 8))) >> 8;
	uint8_t t = (val * (255 - ((sat * (255 - rem)) >> 8))) >> 8;

	switch (reg) {
	case 0:
		_r[0] = val;
		_g[0] = t;
		_b[0] = p;
		break;
	case 1:
		_r[0] = q;
		_g[0] = val;
		_b[0] = p;
		break;
	case 2:
		_r[0] = p;
		_g[0] = val;
		_b[0] = t;
		break;
	case 3:
		_r[0] = p;
		_g[0] = q;
		_b[0] = val;
		break;
	case 4:
		_r[0] = t;
		_g[0] = p;
		_b[0] = val;
		break;
	default:
		_r[0] = val;
		_g[0] = p;
		_b[0] = q;
		break;
	}
//#endif

}

// Inits Timers clock prescalers from timer's frequency
void led_init(void) {
	/* Auto-calculation! */
	uint32_t APBfq;								 // clock frequencies
	RCC_ClkInitTypeDef clk_obj; 				 // clocks structure
	uint32_t latency = FLASH_ACR_LATENCY_2; 	 // dummy hardcode
	HAL_RCC_GetClockConfig(&clk_obj, &latency);  // get clocks table

#if TIM_NUM == 1 || TIM_NUM == 8 || TIM_NUM == 9 || TIM_NUM == 10 ||  TIM_NUM == 11 // APB2

	APBfq = HAL_RCC_GetPCLK2Freq(); // get fq
	if (clk_obj.APB2CLKDivider != RCC_HCLK_DIV1)  // if not divided to 1
		APBfq *= 2;							  	  // multiple to 2
#else  // APB1
	APBfq = HAL_RCC_GetPCLK1Freq(); // get fq
	if (clk_obj.APB1CLKDivider != RCC_HCLK_DIV1)  // if not divided to 1
		APBfq *= 2;							  	  // multiple to 2
#endif
	APBfq /= (uint32_t) (800 * 1000);  // get prescaler

	TIM_POINTER->PSC = 0;					   	// dummy hardcode now
	TIM_POINTER->ARR = (uint16_t) (APBfq - 1); 	// set timer prescaler
	TIM_POINTER->EGR = 1;						// update timer registers

	PWM_HI = (uint8_t) (APBfq * 2 / 3) - 1;	 // set HIGH period to 2/3
	PWM_LO = (uint8_t) (APBfq / 3) - 1;		 // set LOW period to 1/3

//	PWM_HI = (uint8_t) (APBfq * 0.85);		 // set HIGH period to 2/3
//	PWM_LO = (uint8_t) (APBfq * 0.25);		 // set LOW period to 1/3
}

// Set a single color (RGB) to index
void led_setRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b) {
	// GRB order!
#if USE_GAMMA_CORRECTION
	rgb_arr[NUM_BPP * index] = scale8(g, 0xB0); 	// g;
	rgb_arr[NUM_BPP * index + 1] = r;				// r;
	rgb_arr[NUM_BPP * index + 2] = scale8(b, 0xF0); // b;
#endif
#if !USE_GAMMA_CORRECTION
	rgb_arr[NUM_BPP * index] = g; 		// g;
	rgb_arr[NUM_BPP * index + 1] = r;	// r;
	rgb_arr[NUM_BPP * index + 2] = b; 	// b;
#endif

#ifdef RGBW
	rgb_arr[4 * index + 3] = 0;
#endif
}

// Set a single color (HSV) to index
void led_setHSV(uint16_t index, uint8_t hue, uint8_t sat, uint8_t val) {
	uint8_t _r, _g, _b; 					// init buffer color
	HSV2RGB(hue, sat, val, &_r, &_g, &_b);	// get color
	led_setRGB(index, _r, _g, _b);			// set color
}

// Set a single color (RGBW) to index
void led_setRGBW(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
	led_setRGB(index, r, g, b);				// set rgb part
#if (NUM_BPP == 4)
	rgb_arr[4 * index + 3] = w;				// set white part
#endif
}

// Set all colors to RGB
void led_fillRGB(uint8_t r, uint8_t g, uint8_t b) {
	for (uint_fast16_t i = 0; i < NUM_PIXELS; ++i)
		led_setRGB(i, r, g, b);
}

// Set all colors to HSV
void led_fillHSV(uint8_t hue, uint8_t sat, uint8_t val) {
	uint8_t _r, _g, _b;						// init buffer color
	HSV2RGB(hue, sat, val, &_r, &_g, &_b);	// get color once (!)
	led_fillRGB(_r, _g, _b);				// set color
}

// Set all colors to RGBW
void led_fillRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w) {
	for (uint_fast16_t i = 0; i < NUM_PIXELS; ++i)
		led_setRGBW(i, r, g, b, w);
}

// Shuttle the data to the LEDs!
bool led_show(void) {
	if (wr_buf_p != 0 || DMA_HANDLE.State != HAL_DMA_STATE_READY) {
		return 0;
	} else {
		// Ooh boi the first data buffer half (and the second!)

		for (uint_fast8_t i = 0; i < 8; ++i) {
			wr_buf[i] = PWM_LO << (((rgb_arr[0] << i) & 0x80) > 0);
			wr_buf[i + 8] = PWM_LO << (((rgb_arr[1] << i) & 0x80) > 0);
			wr_buf[i + 16] = PWM_LO << (((rgb_arr[2] << i) & 0x80) > 0);
			wr_buf[i + 24] = PWM_LO << (((rgb_arr[3] << i) & 0x80) > 0);
			wr_buf[i + 32] = PWM_LO << (((rgb_arr[4] << i) & 0x80) > 0);
			wr_buf[i + 40] = PWM_LO << (((rgb_arr[5] << i) & 0x80) > 0);
#ifdef RGBW
			wr_buf[i + 48] = PWM_LO << (((rgb_arr[6] << i) & 0x80) > 0);
			wr_buf[i + 56] = PWM_LO << (((rgb_arr[7] << i) & 0x80) > 0);
#endif
		}

		HAL_TIM_PWM_Start_DMA(&TIM_HANDLE, TIM_CH, (uint32_t*) wr_buf,
		WR_BUF_LEN);
		wr_buf_p = 2; // Since we're ready for the next buffer
		return 1;
	}
}

void led_clear(void) {
	led_fillRGB(0, 0, 0);	// fill with black
	while (!led_show());	// wait for color changes
}

/* DMA CALLBACKS */

void HAL_TIM_PWM_PulseFinishedHalfCpltCallback(TIM_HandleTypeDef *htim) {
	// DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
	if (wr_buf_p < NUM_PIXELS) {
		// We're in. Fill the even buffer
		for (uint_fast8_t i = 0; i < 8; ++i) {
			wr_buf[i] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p] << i) & 0x80) > 0);
			wr_buf[i + 8] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p + 1] << i) & 0x80) > 0);
			wr_buf[i + 16] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p + 2] << i) & 0x80) > 0);
#ifdef RGW
			wr_buf[i + 24] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 3] << i) & 0x80) > 0);
#endif
		}

		wr_buf_p++;
	} else if (wr_buf_p < NUM_PIXELS + 2) {
		// Last two transfers are resets. SK6812: 64 * 1.25 us = 80 us == good enough reset
		//                               WS2812B: 48 * 1.25 us = 60 us == good enough reset
		// First half reset zero fill
		for (uint8_t i = 0; i < WR_BUF_LEN / 2; ++i)
			wr_buf[i] = 0;
		wr_buf_p++;
	}
}

void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim) {
	// DMA buffer set from LED(wr_buf_p) to LED(wr_buf_p + 1)
	if (wr_buf_p < NUM_PIXELS) {
		// We're in. Fill the odd buffer
		for (uint_fast8_t i = 0; i < 8; ++i) {
			wr_buf[i + 24] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p] << i) & 0x80) > 0);
			wr_buf[i + 32] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p + 1] << i) & 0x80) > 0);
			wr_buf[i + 40] = PWM_LO
					<< (((rgb_arr[NUM_BPP * wr_buf_p + 2] << i) & 0x80) > 0);
#ifdef RGBW
			wr_buf[i + 48] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 2] << i) & 0x80) > 0);
			wr_buf[i + 56] = PWM_LO << (((rgb_arr[4 * wr_buf_p + 3] << i) & 0x80) > 0);
			#endif
		}

		wr_buf_p++;
	} else if (wr_buf_p < NUM_PIXELS + 2) {
		// Second half reset zero fill
		for (uint8_t i = WR_BUF_LEN / 2; i < WR_BUF_LEN; ++i)
			wr_buf[i] = 0;
		++wr_buf_p;
	} else {
		// We're done. Lean back and until next time!
		wr_buf_p = 0;
		HAL_TIM_PWM_Stop_DMA(&TIM_HANDLE, TIM_CH);
	}
}
