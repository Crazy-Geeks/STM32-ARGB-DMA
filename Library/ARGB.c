/**
 *******************************************
 * @file    ARGB.c
 * @author  Dmitriy Semenov / Crazy_Geeks
 * @link    https://crazygeeks.ru
 * @version 1.33
 * @date	17-May-2022
 * @brief   Source file for ARGB Driver (Addressable RGB)
 *******************************************
 *
 * @note Repo: https://github.com/Crazy-Geeks/STM32-ARGB-DMA
 * @note RU article: https://crazygeeks.ru/stm32-argb-lib
 */

/**
 * Significant history (date, user, job code, action):
 * - 2022-10-18, Dylan Arrabito  <darrabito@carngeigerobotics.com>, IRAD.8015.1, Greatly modified to support ChibiOS.
 */

/* WS2811 Timings
 * Tolerance: +/- 150ns <-> +/- 0.15us
 * RES: >50us
 *
 * Slow mode:
 * Period: 2.5us <-> 400 KHz
 * T0H: 0.5us
 * T1H: 1.2us
 * T0L: 2.0us
 * T1L: 1.3us
 *
 * Fast mode:
 * Period: 1.25us <-> 800 KHz
 * T0H: 0.25us - 20%
 * T1H: 0.6us  - 48%
 * T0L: 1.0us
 * T1H: 0.65us
 *
 */

/* WS2811 Timings
 * Tolerance: +/- 150ns <-> +/- 0.15us
 * RES: >50us

 * Period: 1.25us <-> 800 KHz
 * T0H: 0.35us - 20%
 * T1H: 0.7us  - 48%
 * T0L: 0.8us
 * T1H: 0.6us
 *
 */

#include "ARGB.h"
#include "stm32_dma.h"
#include "pwm.h"
#include "math.h"
#include <string.h>

/**
 * @addtogroup ARGB_Driver
 * @{
 */

/**
 * @addtogroup Private_entities
 * @brief Private methods and variables
 * @{
*/

/// Timer handler
#if (TIM_HANDLE == PWMD2) || (TIM_HANDLE == PWMD3) || (TIM_HANDLE == PWMD4) || \
    (TIM_HANDLE == PWMD5) || (TIM_HANDLE == PWMD5) || (TIM_HANDLE == PWMD12)
#define APB_FREQ  STM32_TIMCLK1
#else
#define APB_FREQ  STM32_TIMCLK2
#endif

/// DMA Size
#if defined(DMA_SIZE_BYTE)
typedef uint8_t dma_siz;
#elif defined(DMA_SIZE_HWORD)
typedef uint16_t dma_siz;
#elif defined(DMA_SIZE_WORD)
typedef uint32_t dma_siz;
#endif

#ifdef APB1
#define APB_FREQ STM32_TIMCLK1
#elif defined(APB2)
#define APB_FREQ STM32_TIMCLK2
#endif

#ifdef WS2811S
#define ARR_VAL (APB_FREQ / (400*1000)) // 400 KHz - 2.5us
#else
#define ARR_VAL (APB_FREQ / (800*1000)) // 800 KHz - 1.25us
#endif

#if defined(MIXED_RGB_GRB)
#define WS2812_PWM_HI (uint8_t) (ARR_VAL * (0.583 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 56% - 0.70us
#define WS2812_PWM_LO (uint8_t) (ARR_VAL * (0.2916 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 28% - 0.35us

#define SK6812_PWM_HI (uint8_t) (ARR_VAL * (0.5 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us
#define SK6812_PWM_LO (uint8_t) (ARR_VAL * (0.25 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 24% - 0.30us

#elif defined(WS2811F) || defined(WS2811S)
#define PWM_HI (uint8_t) (ARR_VAL * (0.48 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us/1.2us
#define PWM_LO (uint8_t) (ARR_VAL * (0.20 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 20% - 0.25us/0.5us

#elif defined(WS2812)
#define PWM_HI (uint8_t) (ARR_VAL * (0.583 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 56% - 0.70us
#define PWM_LO (uint8_t) (ARR_VAL * (0.2916 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 28% - 0.35us

#elif defined(SK6812)
#define PWM_HI (uint8_t) (ARR_VAL * (0.5 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us
#define PWM_LO (uint8_t) (ARR_VAL * (0.25 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 24% - 0.30us
#endif

#if defined(MIXED_RGB_RGBW)
#define NUM_BYTES ((4 * (RGBW_END - RGBW_START)) + (3 * (RGB_END - RGB_START))) ///< Strip size in bytes
#define PWM_BUF_LEN (4 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs
#elif defined(RGBW)
#define NUM_BYTES (4 * NUM_PIXELS) ///< Strip size in bytes
#define PWM_BUF_LEN (4 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs
#else
#define NUM_BYTES (3 * NUM_PIXELS) ///< Strip size in bytes
#define PWM_BUF_LEN (3 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs
#endif

#define DMA_MODE (STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CIRC | \
                  STM32_DMA_CR_HTIE | STM32_DMA_CR_TCIE  | STM32_DMA_CR_MINC | \
                  STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_CHSEL(3))

static const PWMConfig pwm2_conf = 
{
    APB_FREQ,
    ARR_VAL - 1,
    NULL,
    {
        {PWM_OUTPUT_DISABLED,   NULL},
        {PWM_OUTPUT_DISABLED,   NULL},
        {PWM_OUTPUT_DISABLED,   NULL},
        {LED_PWM_ACTIVE_EDGE, NULL}
    },
    0,
    0  
};

/// Static LED buffer
volatile uint8_t rgb_buf[NUM_BYTES] = {0,};

/// Timer PWM value buffer
volatile dma_siz pwm_buf[PWM_BUF_LEN] = {0,};
/// PWM buffer iterator
volatile uint16_t buf_counter = 0;

volatile uint8_t argb_brightness = 255;     ///< LED Global brightness
volatile argb_state argb_lock_state; ///< Buffer send status

// get around -Werror=type-limits
#if defined(MIXED_RGB_GRB)
static const uint16_t ws2812_start = WS2812_START;
static const uint16_t ws2812_end = WS2812_END;
static const uint16_t sk6812_start = SK6812_START;
static const uint16_t sk6812_end = SK6812_END;
#elif defined(MIXED_RGB_RGBW)
static const uint16_t rgbw_start = RGBW_START;
static const uint16_t rgbw_end = RGBW_END;
static const uint16_t rgb_start = RGB_START;
static const uint16_t rgb_end = RGB_END;
#endif


static inline uint8_t scale8(uint8_t x, uint8_t scale); // Gamma correction
static void hsv_to_rgb(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r, uint8_t *_g, uint8_t *_b);

static void argb_tim_dma_delay_pulse(void *param, uint32_t flags);
/// @} //Private

/**
 * @brief Init timer & prescalers
 * @param none
 */
void argb_init(void) 
{
    // initialize PWM with config
    pwmStart(&TIM_HANDLE, &pwm2_conf);

    argb_lock_state = ARGB_READY; // Set Ready Flag

    // initialize DMA stream with callback
    dmaStreamAllocate(DMA_HANDLE, 10, (stm32_dmaisr_t) argb_tim_dma_delay_pulse, NULL);

    // set up DMA properties
    dmaStreamSetPeripheral(DMA_HANDLE, &TIM_HANDLE.tim->CCR[TIM_CH]);
    dmaStreamSetMemory0(DMA_HANDLE, &pwm_buf[0]);
    dmaStreamSetTransactionSize(DMA_HANDLE, PWM_BUF_LEN);
    dmaStreamSetMode(DMA_HANDLE, DMA_MODE);
}

/**
 * @brief Fill ALL LEDs with (0,0,0)
 * @param none
 * @note Update strip after that
 */
void argb_clear(void) 
{
    argb_fill_rgb(0, 0, 0);
#ifdef RGBW
    argb_fill_white(0);
#endif
}

/**
 * @brief Set GLOBAL LED brightness
 * @param[in] br Brightness [0..255]
 */
void argb_set_brightness(uint8_t br) 
{
    argb_brightness = br;
}

/**
 * @brief Set LED with RGB color by index
 * @param[in] i LED position
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void argb_set_rgb(uint16_t i, uint8_t r, uint8_t g, uint8_t b) 
{
    // overflow protection
    if (i >= NUM_PIXELS) {
        uint16_t _i = i / NUM_PIXELS;
        i -= _i * NUM_PIXELS;
    }
    // set brightness
    r /= 256 / ((uint16_t) argb_brightness + 1);
    g /= 256 / ((uint16_t) argb_brightness + 1);
    b /= 256 / ((uint16_t) argb_brightness + 1);
#if USE_GAMMA_CORRECTION
    g = scale8(g, 0xB0);
    b = scale8(b, 0xF0);
#endif

// support multiple different strips with white channel
#if defined(MIXED_RGB_GRB)
    // Subpixel chain order
    // RGBW or GRBW
    if ((i >= sk6812_start) && (i <= sk6812_end))
    {
        rgb_buf[4 * i] = r;
        rgb_buf[4 * i + 1] = g;
        rgb_buf[4 * i + 2] = b;
    }
    else if ((i >= ws2812_start) && (i <= ws2812_end))
    {
        rgb_buf[4 * i] = g;
        rgb_buf[4 * i + 1] = r;
        rgb_buf[4 * i + 2] = b;
    }
#elif defined(MIXED_RGB_RGBW)
    if ((i >= rgbw_start) && (i <= rgbw_end))
    {
        rgb_buf[4 * i] = g;
        rgb_buf[4 * i + 1] = r;
        rgb_buf[4 * i + 2] = b;
    }
    else if ((i >= rgb_start) && (i <= rgb_end))
    {
        rgb_buf[3 * i] = g;
        rgb_buf[3 * i + 1] = r;
        rgb_buf[3 * i + 2] = b;
    }
// one type of strip
// RGBW, GRB, or RGB
#elif defined(SK6812) && defined(RGBW)
    rgb_buf[4 * i] = r;
    rgb_buf[4 * i + 1] = g;
    rgb_buf[4 * i + 2] = b;
#elif defined(WS2812) && defined(RGBW)
    rgb_buf[4 * i] = g;
    rgb_buf[4 * i + 1] = r;
    rgb_buf[4 * i + 2] = b;
#elif defined(WS2812)
    rgb_buf[3 * i] = g;
    rgb_buf[3 * i + 1] = r;
    rgb_buf[3 * i + 2] = b;
#else
    rgb_buf[3 * i] = r;
    rgb_buf[3 * i + 1] = g;
    rgb_buf[3 * i + 2] = b;
#endif
}

/**
 * @brief Set LED with HSV color by index
 * @param[in] i LED position
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void argb_set_hsv(uint16_t i, uint8_t hue, uint8_t sat, uint8_t val) 
{
    uint8_t _r, _g, _b;                    // init buffer color
    hsv_to_rgb(hue, sat, val, &_r, &_g, &_b); // get RGB color
    argb_set_rgb(i, _r, _g, _b);     // set color
}

/**
 * @brief Set White component in strip by index
 * @param[in] i LED position
 * @param[in] w White component [0..255]
 */
void argb_set_white(uint16_t i, uint8_t w) 
{
#ifdef RGB
    return;
#endif
    w /= 256 / ((uint16_t) argb_brightness + 1); // set brightness
    rgb_buf[4 * i + 3] = w;                // set white part
}

void argb_fill_rgb_range(uint16_t start, uint16_t end, uint8_t r, uint8_t g, uint8_t b) 
{
    for (volatile uint16_t i = start; i <= end; i++)
        argb_set_rgb(i, r, g, b);
}

/**
 * @brief Fill ALL LEDs with RGB color
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void argb_fill_rgb(uint8_t r, uint8_t g, uint8_t b) 
{
    argb_fill_rgb_range(0, NUM_LEDS-1, r, g, b);
}

void argb_fill_hsv_range(uint16_t start, uint16_t end, uint8_t hue, uint8_t sat, uint8_t val) 
{
    uint8_t _r, _g, _b;                    // init buffer color
    hsv_to_rgb(hue, sat, val, &_r, &_g, &_b); // get color once (!)
    argb_fill_rgb_range(start, end, _r, _g, _b);       // set color
}

/**
 * @brief Fill ALL LEDs with HSV color
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void argb_fill_hsv(uint8_t hue, uint8_t sat, uint8_t val) 
{
    argb_fill_hsv_range(0, NUM_LEDS-1, hue, sat, val);
}

void argb_fill_white_range(uint16_t start, uint16_t end, uint8_t w) 
{
    for (volatile uint16_t i = start; i <= end; i++)
        argb_set_white(i, w);
}

/**
 * @brief Set ALL White components in strip
 * @param[in] w White component [0..255]
 */
void argb_fill_white(uint8_t w) 
{
    argb_fill_white_range(0, NUM_LEDS-1, w);
}

/**
 * @brief Get current DMA status
 * @param none
 * @return #argb_state enum
 */
argb_state argb_ready(void) 
{
    return argb_lock_state;
}

/**
 * @brief Update strip
 * @param none
 * @return #argb_state enum
 */
argb_state argb_show(void) 
{
    argb_lock_state = ARGB_BUSY;

    // if nothing to do or DMA busy
    if ((buf_counter != 0) || (DMA_HANDLE->stream->CR & STM32_DMA_CR_EN))
    {
        return ARGB_BUSY;
    } 
    else 
    {
        for (volatile uint8_t i = 0; i < 8; i++) 
        {
#if defined(MIXED_RGB_GRB)
            // if the first LEDs are SK6812
            if (sk6812_start == 0)
            {
                // set first transfer from first values
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
            }
            else if (ws2812_start == 0)
            {
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
            }
#elif defined(MIXED_RGB_RGBW)
            // if the first LEDs are SK6812
            if (rgb_start == 0)
            {
                // set first transfer from first values
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            }
            else if (rgbw_start == 0)
            {
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            }
#else
            // set first transfer from first values
            pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#ifdef RGBW
            pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
#endif
        }

        // wait for PWM to be ready
        while (pwmIsChannelEnabledI(&TIM_HANDLE, (TIM_CH)));  

        // enable half and full transfer interrupt along with stream
        DMA_HANDLE->stream->CR |= STM32_DMA_CR_TCIE | STM32_DMA_CR_HTIE;
        dmaStreamEnable(DMA_HANDLE);

        // enable TIM DMA requests
        TIM_HANDLE.tim->DIER |= STM32_TIM_DIER_CC4DE;
        TIM_HANDLE.tim->CNT = 0;
        TIM_HANDLE.tim->CR1 |= STM32_TIM_CR1_CEN;
        pwmEnableChannel(&TIM_HANDLE, TIM_CH, 0);

        buf_counter = 2;
        return ARGB_OK;
    }
}

/**
 * @addtogroup Private_entities
 * @{ */

/**
 * @brief Private method for gamma correction
 * @param[in] x Param to scale
 * @param[in] scale Scale coefficient
 * @return Scaled value
 */
static inline uint8_t scale8(uint8_t x, uint8_t scale) 
{
    return ((uint16_t) x * scale) >> 8;
}

/**
 * @brief Convert color in HSV to RGB
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 * @param[out] _r Pointer to RED component value
 * @param[out] _g Pointer to GREEN component value
 * @param[out] _b Pointer to BLUE component value
 */
static void hsv_to_rgb(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r, uint8_t *_g, uint8_t *_b) 
{
    if (sat == 0) 
    { // if white color
        *_r = *_g = *_b = val;
        return;
    }
    // Float is smoother but check for FPU (Floating point unit) in your MCU
    // Otherwise it will take longer time in the code
    // FPU is in: F3/L3 and greater
    // Src: https://github.com/Inseckto/HSV-to-RGB
    float h = (float)hue / 255;
    float s = (float)sat / 255;
    float v = (float)val / 255;

    int i = (int)floorf(h * 6);
    float f = h * 6 - (float)i;
    uint8_t p = (uint8_t)(v * (1 - s) * 255.0);
    uint8_t q = (uint8_t)(v * (1 - f * s) * 255.0);
    uint8_t t = (uint8_t)(v * (1 - (1 - f) * s)*255.0);

    switch (i % 6) 
    {
// Src: https://stackoverflow.com/questions/3018313
//    uint8_t reg = hue / 43;
//    uint8_t rem = (hue - (reg * 43)) * 6;
//    uint8_t p = (val * (255 - sat)) >> 8;
//    uint8_t q = (val * (255 - ((sat * rem) >> 8))) >> 8;
//    uint8_t t = (val * (255 - ((sat * (255 - rem)) >> 8))) >> 8;
//    switch (reg) {
        case 0: *_r = val, *_g = t, *_b = p; break;
        case 1: *_r = q, *_g = val, *_b = p; break;
        case 2: *_r = p, *_g = val, *_b = t; break;
        case 3: *_r = p, *_g = q, *_b = val; break;
        case 4: *_r = t, *_g = p, *_b = val; break;
        default: *_r = val, *_g = p, *_b = q; break;
    }
}

/**
  * @brief  TIM DMA Delay Pulse callback.
  * @param  dummy param, null ptr
  * @retval None
  */
void argb_tim_dma_delay_pulse(void *param, uint32_t flags) 
{
    (void) param;

    if (buf_counter == 0) return; // if no data to transmit - return
    
    if (flags & STM32_DMA_ISR_HTIF)
    {
        if (!(flags & STM32_DMA_ISR_TCIF))
        {
            dmaStreamClearInterrupt(DMA_HANDLE);
        }

        if (buf_counter < NUM_PIXELS) 
        {
            // fill first part of buffer
            for (volatile uint8_t i = 0; i < 8; i++) 
            {
#if defined(MIXED_RGB_GRB)
                if ((buf_counter >= sk6812_start) && (buf_counter <= sk6812_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)?  SK6812_PWM_HI : SK6812_PWM_LO;
                }
                else if ((buf_counter >= ws2812_start) && (buf_counter <= ws2812_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)?  WS2812_PWM_HI : WS2812_PWM_LO;
                }
#elif defined(MIXED_RGB_RGBW)
                if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          PWM_HI : PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  PWM_HI : PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                }
                else if ((buf_counter >= rgbw_start) && (buf_counter <= rgbw_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          PWM_HI : PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  PWM_HI : PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)?  PWM_HI : PWM_LO;
                }
#else
#ifdef RGBW
                pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)? PWM_HI : PWM_LO;
#else
                pwm_buf[i] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
#endif
            }
            buf_counter++;
        } 
        else if (buf_counter < NUM_PIXELS + 2) // if RET transfer
        {
            memset((dma_siz *) &pwm_buf[0], 0, (PWM_BUF_LEN / 2)*sizeof(dma_siz)); // first part
            buf_counter++;
        }
    }
    if (flags & STM32_DMA_ISR_TCIF)
    {
        // if data transfer
        if (buf_counter < NUM_PIXELS) 
        {
            // fill second part of buffer
            for (volatile uint8_t i = 0; i < 8; i++) 
            {
#if defined(MIXED_RGB_GRB)
                if ((buf_counter >= sk6812_start) && (buf_counter <= sk6812_end))
                {
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                    pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? SK6812_PWM_HI : SK6812_PWM_LO;
                }
                else if ((buf_counter >= ws2812_start) && (buf_counter <= ws2812_end))
                {
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                    pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? WS2812_PWM_HI : WS2812_PWM_LO;
                }
#elif defined(MIXED_RGB_RGBW)
                if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     PWM_HI : PWM_LO;
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     PWM_HI : PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                }
                else if ((buf_counter >= rgbw_start) && (buf_counter <= rgbw_end))
                {
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     PWM_HI : PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                    pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                    pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                }
#else
#ifdef RGBW
                pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#else
                pwm_buf[i + 24] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
#endif
            }
            buf_counter++;
        } 
        else if (buf_counter < NUM_PIXELS + 2) // if RET transfer
        {
            memset((dma_siz *) &pwm_buf[PWM_BUF_LEN / 2], 0, (PWM_BUF_LEN / 2)*sizeof(dma_siz)); // second part
            buf_counter++;
        } 
        else 
        { // if END of transfer
            buf_counter = 0;

            // STOP DMA
            dmaStreamDisable(DMA_HANDLE);
            
            /* Disable the Peripheral */
            TIM_HANDLE.tim->DIER &= ~STM32_TIM_DIER_CC4DE;
            pwmDisableChannelI(&TIM_HANDLE, TIM_CH);
            TIM_HANDLE.tim->CR1 &= ~STM32_TIM_CR1_CEN;

            argb_lock_state = ARGB_READY;
        }
    }
}

/** @} */ // Private

/** @} */ // Driver

// Check strip type
#if !(defined(SK6812) || defined(WS2811F) || defined(WS2811S) || defined(WS2812))
#error INCORRECT LED TYPE
#warning Set it from list in ARGB.h string 29
#endif

// Check channel
#if !(TIM_CH == TIM_CHANNEL_1 || TIM_CH == TIM_CHANNEL_2 || TIM_CH == TIM_CHANNEL_3 || TIM_CH == TIM_CHANNEL_4)
#error Wrong channel! Fix it in ARGB.h string 40
#warning If you shure, search and set TIM_CHANNEL by yourself
#endif

// Check DMA Size
#if !(defined(DMA_SIZE_BYTE) | defined(DMA_SIZE_HWORD) | defined(DMA_SIZE_WORD))
#error Wrong DMA Size! Fix it in ARGB.h string 42
#endif
