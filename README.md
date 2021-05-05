## STM32-WS2812B-DMA
 Fastest and simpliest library for *WS28xx* and *SK68xx* Series *RGB* or *RGBW* for *STM32* Series. Uses *DMA Interrupts* and *PWM* to control LED Strip
 
### Features:
- Can be used for addresable RGB and RGBW LED strips
- Uses double-buffer and half-ready DMA interrupts, so RAM consumption is small
- Uses standard neopixel's 800 KHz protocol
- Supports *RGB* and *HSV* color models
- Timer frequency auto-calculation *(later)*
  
### Function reference (from .h file):
```c
void led_init(void);	// Initalization
void led_clear(void);	// Fill strip with black
void led_setRGB(uint16_t index, uint8_t r, uint8_t g, uint8_t b); 	// Set one pixel with RGB color
void led_setHSV(uint16_t index, uint8_t hue, uint8_t sat, uint8_t val);	// Set one pixel with HSV color
void led_setRGBW(uint16_t index, uint8_t r, uint8_t g, uint8_t b, uint8_t w); // Set one pixel with RGB color for RGBW strip
void led_fillRGB(uint8_t r, uint8_t g, uint8_t b);	// Fill strip with RGB color
void led_fillHSV(uint8_t hue, uint8_t sat, uint8_t val);	// Fill strip with HSV color
void led_fillRGBW(uint8_t r, uint8_t g, uint8_t b, uint8_t w);	// Fill strip with RGB color for RGBW strip
bool led_show(void);	// Send color buffer to the strip - use with while!
```

### Instructions for use: 
> Also available in PDF (RU/EN)
- Use *CubeMX* to configure clocks and peripheral. You need to get **72 MHz** for timer clock (ABPx)
> In this version you can use only 72 MHz (check issues)
> In future versions it will be auto-calculation
- Enable *PWM Generation* for your preferred timer channel.
- *PWM Mode 1*, *Pulse: 0*, *OC Preload: Enable*, *Fast Mode: Disable*, *CH Polarity: High*
- Enable **DMA** for your timer channel with **"Memory To Peripheral"** direction.
- Set *DMA* mode to **Circular**, *Data Width* to **Word**/**Byte**, *Increment Address* checkbox only for **Memory**.
- Save CubeMX .ioc file and generate code.
- Add library to your source destination and add #include in your code. 
- In **main.c** file search for your DMA Handler
```c
/* Private variables */
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;  <-- THIS
```
- Add this handler in **ws2812.h** file in **DMA HANDLE** define.
- Set other defines by your settings.
- Now we're ready to go!    

### NOTE (if you have problems)
- Check datasheet for your LEDs, there is a voltage level for logic supply. For example, for WS2812b it is 0.7\*VDD Min. So 5.1\*0.7 = 3.57V, that is greater than STM32's 3.3V logic supply. The strip won't light at all.
- If you have noise on your strip, or confetti effect, check signal length with logic analyzer, or just play with values. Change timer values in .c file in 154-155 strings.     
    
**Special thanks** to: 
[**NarodStream***](https://narodstream.ru/stm-urok-119-ws2812b-lenta-na-umnyx-svetodiodax-rgb-chast-2/ )    
[**VFD***](https://www.thevfdcollective.com/blog/stm32-and-sk6812-rgbw-led )    


**Donate:** [PayPal](https://paypal.me/yasnosos ) / [DonationAlerts](https://www.donationalerts.com/r/yasnosos )
