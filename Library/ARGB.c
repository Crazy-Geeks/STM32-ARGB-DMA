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

#include "ARGB.h"  // include header file
#include "board.h"
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

#define LED_SIGNAL_RISE_DELAY_US 0.125

#if defined(WS2811F) || defined(WS2811S)
#define PWM_HI (uint8_t) (ARR_VAL * (0.48 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us/1.2us
#define PWM_LO (uint8_t) (ARR_VAL * (0.20 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 20% - 0.25us/0.5us

#elif defined(WS2812)
#define PWM_HI (uint8_t) (ARR_VAL * (0.56 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 56% - 0.70us
#define PWM_LO (uint8_t) (ARR_VAL * (0.28 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 28% - 0.35us

#elif defined(SK6812)
#define PWM_HI (uint8_t) (ARR_VAL * (0.48 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us
#define PWM_LO (uint8_t) (ARR_VAL * (0.24 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 24% - 0.30us
#endif

#ifdef SK6812
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
    STM32_TIMCLK1,
    ARR_VAL - 1,
    NULL,
    {
        {PWM_OUTPUT_DISABLED,    NULL},
        {PWM_OUTPUT_DISABLED, /*ACTIVE_HIGH,*/ NULL},
        {PWM_OUTPUT_DISABLED,    NULL},
        {PWM_OUTPUT_ACTIVE_LOW,    NULL}
    },
    0, //STM32_TIM_CR2_MMS(0b101),
    0  //TIM_DIER_UDE
};

/// Static LED buffer
volatile uint8_t RGB_BUF[NUM_BYTES] = {0,};

/// Timer PWM value buffer
volatile dma_siz PWM_BUF[PWM_BUF_LEN] = {0,};
/// PWM buffer iterator
volatile uint16_t BUF_COUNTER = 0;

volatile uint8_t ARGB_BR = 255;     ///< LED Global brightness
volatile ARGB_STATE ARGB_LOC_ST; ///< Buffer send status

static inline uint8_t scale8(uint8_t x, uint8_t scale); // Gamma correction
static void HSV2RGB(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r, uint8_t *_g, uint8_t *_b);

static void ARGB_TIM_DMADelayPulseCplt(void *param, uint32_t flags);
/// @} //Private

/**
 * @brief Init timer & prescalers
 * @param none
 */
void ARGB_Init(void) 
{
    // initialize PWM with config
    pwmStart(&TIM_HANDLE, &pwm2_conf);

    ARGB_LOC_ST = ARGB_READY; // Set Ready Flag

    // initialize DMA stream with callback
    dmaStreamAllocate(DMA_HANDLE, 10, (stm32_dmaisr_t) ARGB_TIM_DMADelayPulseCplt, NULL);

    // set up DMA properties
    dmaStreamSetPeripheral(DMA_HANDLE, &TIM_HANDLE.tim->CCR[TIM_CH]);
    dmaStreamSetMemory0(DMA_HANDLE, &PWM_BUF[0]);
    dmaStreamSetTransactionSize(DMA_HANDLE, PWM_BUF_LEN);
    dmaStreamSetMode(DMA_HANDLE, DMA_MODE);
}

/**
 * @brief Fill ALL LEDs with (0,0,0)
 * @param none
 * @note Update strip after that
 */
void ARGB_Clear(void) 
{
    ARGB_FillRGB(0, 0, 0);
#ifdef SK6812
    ARGB_FillWhite(0);
#endif
}

/**
 * @brief Set GLOBAL LED brightness
 * @param[in] br Brightness [0..255]
 */
void ARGB_SetBrightness(uint8_t br) 
{
    ARGB_BR = br;
}

/**
 * @brief Set LED with RGB color by index
 * @param[in] i LED position
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void ARGB_SetRGB(uint16_t i, uint8_t r, uint8_t g, uint8_t b) 
{
    // overflow protection
    if (i >= NUM_PIXELS) {
        uint16_t _i = i / NUM_PIXELS;
        i -= _i * NUM_PIXELS;
    }
    // set brightness
    r /= 256 / ((uint16_t) ARGB_BR + 1);
    g /= 256 / ((uint16_t) ARGB_BR + 1);
    b /= 256 / ((uint16_t) ARGB_BR + 1);
#if USE_GAMMA_CORRECTION
    g = scale8(g, 0xB0);
    b = scale8(b, 0xF0);
#endif

// support multiple different strips with white channel
#if defined(NEXT_LED_STRIP_START)
    // Subpixel chain order
    // RGBW or GRBW
    if (i >= NEXT_LED_STRIP_START)
    {
#if NEXT_LED_STRIP == SK6812_LEDS
        RGB_BUF[4 * i] = r;
        RGB_BUF[4 * i + 1] = g;
        RGB_BUF[4 * i + 2] = b;
#elif NEXT_LED_STRIP == WS2812_LEDS
        RGB_BUF[4 * i] = g;
        RGB_BUF[4 * i + 1] = r;
        RGB_BUF[4 * i + 2] = b;
#endif
    }
    else    // not next strip
    {
#if NEXT_LED_STRIP == SK6812_LEDS
        RGB_BUF[4 * i] = g;
        RGB_BUF[4 * i + 1] = r;
        RGB_BUF[4 * i + 2] = b;
#elif NEXT_LED_STRIP == WS2812_LEDS
        RGB_BUF[4 * i] = r;
        RGB_BUF[4 * i + 1] = g;
        RGB_BUF[4 * i + 2] = b;
#endif
    }
// one type of strip
// RGBW, GRB, or RGB
#elif defined(SK6812)
    RGB_BUF[4 * i] = r;
    RGB_BUF[4 * i + 1] = g;
    RGB_BUF[4 * i + 2] = b;
#elif defined(WS2812)
    RGB_BUF[3 * i] = g;
    RGB_BUF[3 * i + 1] = r;
    RGB_BUF[3 * i + 2] = b;
#else
    RGB_BUF[3 * i] = r;
    RGB_BUF[3 * i + 1] = g;
    RGB_BUF[3 * i + 2] = b;
#endif
}

/**
 * @brief Set LED with HSV color by index
 * @param[in] i LED position
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void ARGB_SetHSV(uint16_t i, uint8_t hue, uint8_t sat, uint8_t val) 
{
    uint8_t _r, _g, _b;                    // init buffer color
    HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get RGB color
    ARGB_SetRGB(i, _r, _g, _b);     // set color
}

/**
 * @brief Set White component in strip by index
 * @param[in] i LED position
 * @param[in] w White component [0..255]
 */
void ARGB_SetWhite(uint16_t i, uint8_t w) 
{
#ifdef RGB
    return;
#endif
    w /= 256 / ((uint16_t) ARGB_BR + 1); // set brightness
    RGB_BUF[4 * i + 3] = w;                // set white part
}

/**
 * @brief Fill ALL LEDs with RGB color
 * @param[in] r Red component   [0..255]
 * @param[in] g Green component [0..255]
 * @param[in] b Blue component  [0..255]
 */
void ARGB_FillRGB(uint8_t r, uint8_t g, uint8_t b) 
{
    for (volatile uint16_t i = 0; i < NUM_PIXELS; i++)
        ARGB_SetRGB(i, r, g, b);
}

/**
 * @brief Fill ALL LEDs with HSV color
 * @param[in] hue HUE (color) [0..255]
 * @param[in] sat Saturation  [0..255]
 * @param[in] val Value (brightness) [0..255]
 */
void ARGB_FillHSV(uint8_t hue, uint8_t sat, uint8_t val) 
{
    uint8_t _r, _g, _b;                    // init buffer color
    HSV2RGB(hue, sat, val, &_r, &_g, &_b); // get color once (!)
    ARGB_FillRGB(_r, _g, _b);       // set color
}

/**
 * @brief Set ALL White components in strip
 * @param[in] w White component [0..255]
 */
void ARGB_FillWhite(uint8_t w) 
{
    for (volatile uint16_t i = 0; i < NUM_PIXELS; i++)
        ARGB_SetWhite(i, w);
}

/**
 * @brief Get current DMA status
 * @param none
 * @return #ARGB_STATE enum
 */
ARGB_STATE ARGB_Ready(void) 
{
    return ARGB_LOC_ST;
}

/**
 * @brief Update strip
 * @param none
 * @return #ARGB_STATE enum
 */
ARGB_STATE ARGB_Show(void) 
{
    ARGB_LOC_ST = ARGB_BUSY;

    // if nothing to do or DMA busy
    if ((BUF_COUNTER != 0) || (DMA_HANDLE->stream->CR & STM32_DMA_CR_EN))
    {
        return ARGB_BUSY;
    } 
    else 
    {
        for (volatile uint8_t i = 0; i < 8; i++) 
        {
            // set first transfer from first values
            PWM_BUF[i] = (((RGB_BUF[0] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 8] = (((RGB_BUF[1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 16] = (((RGB_BUF[2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 24] = (((RGB_BUF[3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 32] = (((RGB_BUF[4] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 40] = (((RGB_BUF[5] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#ifdef SK6812
            PWM_BUF[i + 48] = (((RGB_BUF[6] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
            PWM_BUF[i + 56] = (((RGB_BUF[7] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
        }

        // wait for PWM to be ready
        while (pwmIsChannelEnabledI(&TIM_HANDLE, (TIM_CH)));  

        // enable half and full transfer interrupt along with stream
        DMA_HANDLE->stream->CR |= STM32_DMA_CR_TCIE | STM32_DMA_CR_HTIE;
        dmaStreamEnable(DMA_HANDLE);

        // enable TIM DMA requests
        TIM_HANDLE.tim->DIER |= STM32_TIM_DIER_CC4DE;
        pwmEnableChannel(&TIM_HANDLE, TIM_CH, 0);

        BUF_COUNTER = 2;
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
static void HSV2RGB(uint8_t hue, uint8_t sat, uint8_t val, uint8_t *_r, uint8_t *_g, uint8_t *_b) 
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
void ARGB_TIM_DMADelayPulseCplt(void *param, uint32_t flags) 
{
    (void) param;

    if (BUF_COUNTER == 0) return; // if no data to transmit - return
    
    if (flags & STM32_DMA_ISR_HTIF)
    {
        if (!(flags & STM32_DMA_ISR_TCIF))
        {
            dmaStreamClearInterrupt(DMA_HANDLE);
        }

        if (BUF_COUNTER < NUM_PIXELS) 
        {
            // fill first part of buffer
            for (volatile uint8_t i = 0; i < 8; i++) 
            {
#ifdef SK6812
                PWM_BUF[i] = (((RGB_BUF[4 * BUF_COUNTER] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 8] = (((RGB_BUF[4 * BUF_COUNTER + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 16] = (((RGB_BUF[4 * BUF_COUNTER + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 24] = (((RGB_BUF[4 * BUF_COUNTER + 3] << i) & 0x80) > 0)? PWM_HI : PWM_LO;
#else
                PWM_BUF[i] = (((RGB_BUF[3 * BUF_COUNTER] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 8] = (((RGB_BUF[3 * BUF_COUNTER + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 16] = (((RGB_BUF[3 * BUF_COUNTER + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
            }
            BUF_COUNTER++;
        } 
        else if (BUF_COUNTER < NUM_PIXELS + 2) // if RET transfer
        {
            memset((dma_siz *) &PWM_BUF[0], 0, (PWM_BUF_LEN / 2)*sizeof(dma_siz)); // first part
            BUF_COUNTER++;
        }
    }
    if (flags & STM32_DMA_ISR_TCIF)
    {
        // if data transfer
        if (BUF_COUNTER < NUM_PIXELS) 
        {
            // fill second part of buffer
            for (volatile uint8_t i = 0; i < 8; i++) 
            {
#ifdef SK6812
                PWM_BUF[i + 32] = (((RGB_BUF[4 * BUF_COUNTER] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 40] = (((RGB_BUF[4 * BUF_COUNTER + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 48] = (((RGB_BUF[4 * BUF_COUNTER + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 56] = (((RGB_BUF[4 * BUF_COUNTER + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#else
                PWM_BUF[i + 24] = (((RGB_BUF[3 * BUF_COUNTER] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 32] = (((RGB_BUF[3 * BUF_COUNTER + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                PWM_BUF[i + 40] = (((RGB_BUF[3 * BUF_COUNTER + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#endif
            }
            BUF_COUNTER++;
        } 
        else if (BUF_COUNTER < NUM_PIXELS + 2) // if RET transfer
        {
            memset((dma_siz *) &PWM_BUF[PWM_BUF_LEN / 2], 0, (PWM_BUF_LEN / 2)*sizeof(dma_siz)); // second part
            BUF_COUNTER++;
        } 
        else 
        { // if END of transfer
            BUF_COUNTER = 0;

            // STOP DMA
            dmaStreamDisable(DMA_HANDLE);
            
            /* Disable the Peripheral */
            TIM_HANDLE.tim->DIER &= ~STM32_TIM_DIER_CC4DE;
            pwmDisableChannelI(&TIM_HANDLE, TIM_CH);

            ARGB_LOC_ST = ARGB_READY;
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
