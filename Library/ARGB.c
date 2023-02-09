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
#include "fast_math.h"
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

#if defined(WS2811F) || defined(WS2811S)
#define WS2811_PWM_HI (uint8_t) (ARR_VAL * (0.48 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us/1.2us
#define WS2811_PWM_LO (uint8_t) (ARR_VAL * (0.20 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 20% - 0.25us/0.5us

#if defined(MIXED_RGB_GRB)
#define RGB_PWM_HI WS2811_PWM_HI
#define RGB_PWM_LO WS2811_PWM_LO
#else
#define PWM_HI WS2811_PWM_HI
#define PWM_LO WS2811_PWM_LO
#endif
#endif

#if defined(WS2812)
#define WS2812_PWM_HI (uint8_t) (ARR_VAL * (0.583 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 56% - 0.70us
#define WS2812_PWM_LO (uint8_t) (ARR_VAL * (0.2916 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 28% - 0.35us

#if defined(MIXED_RGB_GRB)
#define GRB_PWM_HI WS2811_PWM_HI
#define GRB_PWM_LO WS2811_PWM_LO
#else
#define PWM_HI WS2811_PWM_HI
#define PWM_LO WS2811_PWM_LO
#endif
#endif

#if defined(SK6812)
#define SK6812_PWM_HI (uint8_t) (ARR_VAL * (0.5 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.1 - 48% - 0.60us
#define SK6812_PWM_LO (uint8_t) (ARR_VAL * (0.25 + LED_SIGNAL_RISE_DELAY_US)) - 1     // Log.0 - 24% - 0.30us

#if defined(MIXED_RGB_GRB)
#define GRB_PWM_HI SK6812_PWM_HI
#define GRB_PWM_LO SK6812_PWM_LO
#else
#define PWM_HI SK6812_PWM_HI
#define PWM_LO SK6812_PWM_LO
#endif
#endif

#if defined(RGBW)
#define NUM_BYTES (4 * NUM_PIXELS) ///< Strip size in bytes
#define PWM_BUF_LEN (4 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs
#else
#define NUM_BYTES (3 * NUM_PIXELS) ///< Strip size in bytes
#define PWM_BUF_LEN (3 * 8 * 2)    ///< Pack len * 8 bit * 2 LEDs
#endif

#define DMA_MODE (STM32_DMA_CR_DIR_M2P | STM32_DMA_CR_CIRC | \
                  STM32_DMA_CR_HTIE | STM32_DMA_CR_TCIE  | STM32_DMA_CR_MINC | \
                  STM32_DMA_CR_PSIZE_WORD | STM32_DMA_CR_MSIZE_WORD | STM32_DMA_CR_CHSEL(3))

#define APPLY_DIMMING(X) (X)
#define HSV_SECTION_6 (0x20)
#define HSV_SECTION_3 (0x40)

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
static const uint16_t rgb_start = RGB_START;
static const uint16_t rgb_end = RGB_END;
static const uint16_t grb_start = GRB_START;
static const uint16_t grb_end = GRB_END;
#endif


static inline uint8_t scale8(uint8_t x, uint8_t scale); // Gamma correction

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
#if defined(MIXED_RGB_GRB) && defined(RGBW)
    // Subpixel chain order
    // RGBW or GRBW
    if ((i >= rgb_start) && (i <= rgb_end))
    {
        rgb_buf[4 * i] = r;
        rgb_buf[4 * i + 1] = g;
        rgb_buf[4 * i + 2] = b;
    }
    else if ((i >= grb_start) && (i <= grb_end))
    {
        rgb_buf[4 * i] = g;
        rgb_buf[4 * i + 1] = r;
        rgb_buf[4 * i + 2] = b;
    }
#elif defined(MIXED_RGB_GRB) && !defined(RGBW)
    // Subpixel chain order
    // RGBW or GRBW
    if ((i >= rgb_start) && (i <= rgb_end))
    {
        rgb_buf[3 * i] = r;
        rgb_buf[3 * i + 1] = g;
        rgb_buf[3 * i + 2] = b;
    }
    else if ((i >= grb_start) && (i <= grb_end))
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
    rgb_t rgb = {.r=0, .g=0, .b=0};
    hsv_t hsv = {.h=hue, .s=sat, .v=val};
    hsv2rgb_spectrum(hsv, &rgb); // get RGB color
    argb_set_rgb(i, rgb.r, rgb.g, rgb.b);     // set color
}

/**
 * @brief Set White component in strip by index
 * @param[in] i LED position
 * @param[in] w White component [0..255]
 */
void argb_set_white(uint16_t i, uint8_t w) 
{
#if !defined(RGBW)
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
    rgb_t rgb = {.r=0, .g=0, .b=0};
    hsv_t hsv = {.h=hue, .s=sat, .v=val};
    hsv2rgb_spectrum(hsv, &rgb); // get color once (!)
    argb_fill_rgb_range(start, end, rgb.r, rgb.g, rgb.b);       // set color
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
            if (grb_start == 0)
            {
                // set first transfer from first values
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
#if defined(RGBW)
                pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
#endif
            }
            else if (rgb_start == 0)
            {
                pwm_buf[i] = (((rgb_buf[0] << i) & 0x80) > 0) ?      RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[1] << i) & 0x80) > 0) ?  RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[2] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[3] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[4] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[5] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
#if defined(RGBW)
                pwm_buf[i + 48] = (((rgb_buf[6] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[7] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
#endif
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

void hsv2rgb_raw(const hsv_t hsv, rgb_t * rgb)
{
    // Convert hue, saturation and brightness ( HSV/HSB ) to RGB
    // "Dimming" is used on saturation and brightness to make
    // the output more visually linear.

    // Apply dimming curves
    uint8_t value = APPLY_DIMMING( hsv.val);
    uint8_t saturation = hsv.sat;

    // The brightness floor is minimum number that all of
    // R, G, and B will be set to.
    uint8_t invsat = APPLY_DIMMING( 255 - saturation);
    uint8_t brightness_floor = (value * invsat) / 256;

    // The color amplitude is the maximum amount of R, G, and B
    // that will be added on top of the brightness_floor to
    // create the specific hue desired.
    uint8_t color_amplitude = value - brightness_floor;

    // Figure out which section of the hue wheel we're in,
    // and how far offset we are withing that section
    uint8_t section = hsv.hue / HSV_SECTION_3; // 0..2
    uint8_t offset = hsv.hue % HSV_SECTION_3;  // 0..63

    uint8_t rampup = offset; // 0..63
    uint8_t rampdown = (HSV_SECTION_3 - 1) - offset; // 63..0

    // We now scale rampup and rampdown to a 0-255 range -- at least
    // in theory, but here's where architecture-specific decsions
    // come in to play:
    // To scale them up to 0-255, we'd want to multiply by 4.
    // But in the very next step, we multiply the ramps by other
    // values and then divide the resulting product by 256.
    // So which is faster?
    //   ((ramp * 4) * othervalue) / 256
    // or
    //   ((ramp    ) * othervalue) /  64
    // It depends on your processor architecture.
    // On 8-bit AVR, the "/ 256" is just a one-cycle register move,
    // but the "/ 64" might be a multicycle shift process. So on AVR
    // it's faster do multiply the ramp values by four, and then
    // divide by 256.
    // On ARM, the "/ 256" and "/ 64" are one cycle each, so it's
    // faster to NOT multiply the ramp values by four, and just to
    // divide the resulting product by 64 (instead of 256).
    // Moral of the story: trust your profiler, not your insticts.

    // Since there's an AVR assembly version elsewhere, we'll
    // assume what we're on an architecture where any number of
    // bit shifts has roughly the same cost, and we'll remove the
    // redundant math at the source level:

    //  // scale up to 255 range
    //  //rampup *= 4; // 0..252
    //  //rampdown *= 4; // 0..252

    // compute color-amplitude-scaled-down versions of rampup and rampdown
    uint8_t rampup_amp_adj   = (rampup   * color_amplitude) / (256 / 4);
    uint8_t rampdown_amp_adj = (rampdown * color_amplitude) / (256 / 4);

    // add brightness_floor offset to everything
    uint8_t rampup_adj_with_floor   = rampup_amp_adj   + brightness_floor;
    uint8_t rampdown_adj_with_floor = rampdown_amp_adj + brightness_floor;


    if( section ) {
        if( section == 1) {
            // section 1: 0x40..0x7F
            rgb->r = brightness_floor;
            rgb->g = rampdown_adj_with_floor;
            rgb->b = rampup_adj_with_floor;
        } else {
            // section 2; 0x80..0xBF
            rgb->r = rampup_adj_with_floor;
            rgb->g = brightness_floor;
            rgb->b = rampdown_adj_with_floor;
        }
    } else {
        // section 0: 0x00..0x3F
        rgb->r = rampdown_adj_with_floor;
        rgb->g = rampup_adj_with_floor;
        rgb->b = brightness_floor;
    }
}

void hsv2rgb_spectrum( const hsv_t hsv, rgb_t * rgb)
{
    hsv_t hsv2 = hsv;
    hsv2.hue = scale8( hsv2.hue, 191);
    hsv2rgb_raw(hsv2, rgb);
}

// This function is only an approximation, and it is not
// nearly as fast as the normal HSV-to-RGB conversion.
// See extended notes in the .h file.
hsv_t rgb2hsv_approximate(const rgb_t rgb)
{
    uint8_t r = rgb.r;
    uint8_t g = rgb.g;
    uint8_t b = rgb.b;
    uint8_t h, s, v;
    
    // find desaturation
    uint8_t desat = 255;
    if( r < desat) desat = r;
    if( g < desat) desat = g;
    if( b < desat) desat = b;
    
    // remove saturation from all channels
    r -= desat;
    g -= desat;
    b -= desat;
    
    //uint8_t orig_desat = sqrt16( desat * 256);
    
    // saturation is opposite of desaturation
    s = 255 - desat;
    
    if( s != 255 ) {
        // undo 'dimming' of saturation
        s = 255 - sqrt16( (255-s) * 256);
    }
    // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
    // if( s != 255 ) s = (255 - (256.0 * sqrt( (float)(255-s) / 256.0)));
    
    
    // at least one channel is now zero
    // if all three channels are zero, we had a
    // shade of gray.
    if( (r + g + b) == 0) {
        // we pick hue zero for no special reason
        hsv_t hsv = {.h=0, .s=0, .v=255-s};
        return hsv;
    }
    
    // scale all channels up to compensate for desaturation
    if( s < 255) {
        if( s == 0) s = 1;
        uint32_t scaleup = 65535 / (s);
        r = ((uint32_t)(r) * scaleup) / 256;
        g = ((uint32_t)(g) * scaleup) / 256;
        b = ((uint32_t)(b) * scaleup) / 256;
    }

    uint16_t total = r + g + b;
    
    // scale all channels up to compensate for low values
    if( total < 255) {
        if( total == 0) total = 1;
        uint32_t scaleup = 65535 / (total);
        r = ((uint32_t)(r) * scaleup) / 256;
        g = ((uint32_t)(g) * scaleup) / 256;
        b = ((uint32_t)(b) * scaleup) / 256;
    }
    
    if( total > 255 ) {
        v = 255;
    } else {
        v = qadd8(desat,total);
        // undo 'dimming' of brightness
        if( v != 255) v = sqrt16( v * 256);
        // without lib8tion: float ... ew ... sqrt... double ew, or rather, ew ^ 0.5
        // if( v != 255) v = (256.0 * sqrt( (float)(v) / 256.0));
        
    }
    
    // since this wasn't a pure shade of gray,
    // the interesting question is what hue is it    
    
    // start with which channel is highest
    // (ties don't matter)
    uint8_t highest = r;
    if( g > highest) highest = g;
    if( b > highest) highest = b;
    
    if( highest == r ) {
        // Red is highest.
        // Hue could be Purple/Pink-Red,Red-Orange,Orange-Yellow
        if( g == 0 ) {
            // if green is zero, we're in Purple/Pink-Red
            h = (HUE_PURPLE + HUE_PINK) / 2;
            h += scale8( qsub8(r, 128), FIXFRAC8(48,128));
        } else if ( (r - g) > g) {
            // if R-G > G then we're in Red-Orange
            h = HUE_RED;
            h += scale8( g, FIXFRAC8(32,85));
        } else {
            // R-G < G, we're in Orange-Yellow
            h = HUE_ORANGE;
            h += scale8( qsub8((g - 85) + (171 - r), 4), FIXFRAC8(32,85)); //221
        }
        
    } else if ( highest == g) {
        // Green is highest
        // Hue could be Yellow-Green, Green-Aqua
        if( b == 0) {
            // if Blue is zero, we're in Yellow-Green
            //   G = 171..255
            //   R = 171..  0
            h = HUE_YELLOW;
            uint8_t radj = scale8( qsub8(171,r),   47); //171..0 -> 0..171 -> 0..31
            uint8_t gadj = scale8( qsub8(g,171),   96); //171..255 -> 0..84 -> 0..31;
            uint8_t rgadj = radj + gadj;
            uint8_t hueadv = rgadj / 2;
            h += hueadv;
            //h += scale8( qadd8( 4, qadd8((g - 128), (128 - r))),
            //             FIXFRAC8(32,255)); //
        } else {
            // if Blue is nonzero we're in Green-Aqua
            if( (g-b) > b) {
                h = HUE_GREEN;
                h += scale8( b, FIXFRAC8(32,85));
            } else {
                h = HUE_AQUA;
                h += scale8( qsub8(b, 85), FIXFRAC8(8,42));
            }
        }
        
    } else /* highest == b */ {
        // Blue is highest
        // Hue could be Aqua/Blue-Blue, Blue-Purple, Purple-Pink
        if( r == 0) {
            // if red is zero, we're in Aqua/Blue-Blue
            h = HUE_AQUA + ((HUE_BLUE - HUE_AQUA) / 4);
            h += scale8( qsub8(b, 128), FIXFRAC8(24,128));
        } else if ( (b-r) > r) {
            // B-R > R, we're in Blue-Purple
            h = HUE_BLUE;
            h += scale8( r, FIXFRAC8(32,85));
        } else {
            // B-R < R, we're in Purple-Pink
            h = HUE_PURPLE;
            h += scale8( qsub8(r, 85), FIXFRAC8(32,85));
        }
    }
    
    h += 1;
    hsv_t hsv = {.h=h, .s=s, .v=v};
    return hsv;
}

hsv_t argb_get_hue(uint16_t i)
{
    rgb_t rgb = {.r=rgb_buf[i], .g=rgb_buf[i+1], .b=rgb_buf[i+2]};
    return rgb2hsv_approximate(rgb);
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
#if defined(MIXED_RGB_GRB) && defined(RGBW)
                if ((buf_counter >= grb_start) && (buf_counter <= grb_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)?  GRB_PWM_HI : GRB_PWM_LO;
                }
                else if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?          RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ?  RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)?  RGB_PWM_HI : RGB_PWM_LO;
                }
#elif defined(MIXED_RGB_GRB) && !defined(RGBW)
                if ((buf_counter >= grb_start) && (buf_counter <= grb_end))
                {
                    pwm_buf[i] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ?          GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ?  GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                }
                else if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ?          RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 8] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ?  RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 16] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                }
#elif defined(RGBW)
                pwm_buf[i] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 24] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0)? PWM_HI : PWM_LO;
#else
                pwm_buf[i] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 8] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 16] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
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
#if defined(MIXED_RGB_GRB) && defined(RGBW)
                if ((buf_counter >= grb_start) && (buf_counter <= grb_end))
                {
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                }
                else if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ?     RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                }
#elif defined(MIXED_RGB_GRB) && !defined(RGBW)
                if ((buf_counter >= grb_start) && (buf_counter <= grb_end))
                {
                    pwm_buf[i + 24] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ?     GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 32] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? GRB_PWM_HI : GRB_PWM_LO;
                }
                else if ((buf_counter >= rgb_start) && (buf_counter <= rgb_end))
                {
                    pwm_buf[i + 24] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ?     RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 32] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                    pwm_buf[i + 40] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? RGB_PWM_HI : RGB_PWM_LO;
                }
#elif defined(RGBW)
                pwm_buf[i + 32] = (((rgb_buf[4 * buf_counter] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[4 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 48] = (((rgb_buf[4 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 56] = (((rgb_buf[4 * buf_counter + 3] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
#else
                pwm_buf[i + 24] = (((rgb_buf[3 * buf_counter] << i) & 0x80) > 0) ?     PWM_HI : PWM_LO;
                pwm_buf[i + 32] = (((rgb_buf[3 * buf_counter + 1] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
                pwm_buf[i + 40] = (((rgb_buf[3 * buf_counter + 2] << i) & 0x80) > 0) ? PWM_HI : PWM_LO;
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
