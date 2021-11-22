## STM32-ARGB-DMA
 **Fastest** and **simplest** library for **ARGB LEDs**: *WS28xx* and *SK68xx* Series *RGB* or *RGBW* for *STM32* Series. 
<br> Uses ***DMA Interrupts*** and ***PWM*** to control LED Strip

> ### [RU Description](https://crazygeeks.ru/stm32-argb-lib )
 
### Features:
- Can be used for **addressable RGB** and **RGBW LED** strips
- Uses double-buffer and half-ready **DMA interrupts**, so RAM **consumption is small**
- Uses standard neopixel's **800/400 KHz** protocol
- Supports ***RGB*** and ***HSV*** color models
- Timer frequency **auto-calculation**
  
### Lib settings
```c
#define WS2811F  // Family: {WS2811S, WS2811F, WS2812, SK6812}
// WS2811S — RGB, 400kHz;
// WS2811F — RGB, 800kHz;
// WS2812  — GRB, 800kHz;
// SK6812  — RGBW, 800kHz

#define NUM_PIXELS 4  // Pixel quantity

// Gamma-correction should fix red & green colors, try for yourself
#define USE_GAMMA_CORRECTION 1 
```

### Function reference (from .h file):
```c
// API enum status
typedef enum ARGB_STATE {
    ARGB_BUSY = 0,      // DMA Transfer in progress
    ARGB_READY = 1,     // DMA Ready to transfer
    ARGB_OK = 2,        // Function execution success
    ARGB_PARAM_ERR = 3, // Error in input parameters
} ARGB_STATE;

ARGB_STATE ARGB_Init(void);   // Initialization
ARGB_STATE ARGB_Clear(void);  // Clear strip

ARGB_STATE ARGB_SetBrightness(u8_t br); // Set global brightness

ARGB_STATE ARGB_SetRGB(u16_t i, u8_t r, u8_t g, u8_t b);  // Set single LED by RGB
ARGB_STATE ARGB_SetHSV(u16_t i, u8_t hue, u8_t sat, u8_t val); // Set single LED by HSV
ARGB_STATE ARGB_SetWhite(u16_t i, u8_t w); // Set white component in LED (RGBW)

ARGB_STATE ARGB_FillRGB(u8_t r, u8_t g, u8_t b); // Fill all strip with RGB color
ARGB_STATE ARGB_FillHSV(u8_t hue, u8_t sat, u8_t val); // Fill all strip with HSV color
ARGB_STATE ARGB_FillWhite(u8_t w); // Fill all strip's white component (RGBW)

ARGB_STATE ARGB_Ready(void); // Get DMA Ready state
ARGB_STATE ARGB_Show(void); // Push data to the strip
```

### Instructions for use: 
> #### Also available in PDF (RU/EN)
- Use *CubeMX* to configure clocks and peripheral.
- Enable *PWM Generation* for your preferred timer channel.
- ***PWM Mode 1***,  ***OC Preload**: Enable*, ***Fast Mode**: Disable*, ***CH Polarity**: High*
- Enable **DMA** for your timer channel with **"Memory To Peripheral"** direction.
- Set *DMA* mode to **Circular**, *Data Width* to **Word**/**Byte**, *Increment Address* checkbox only for **Memory**.
- Set *GPIO Speed* to the **Maximum**, use **Open Drain** or **Push Pull** Mode - details in **Troubleshooting**.
- Save CubeMX .ioc file and generate code.
- Add library to your source destination and add #include in your code. 
- In **main.c** file search for your DMA Handler
```c
/* Private variables */
TIM_HandleTypeDef htim2;
DMA_HandleTypeDef hdma_tim2_ch2_ch4;  <-- THIS
```
- Add this handler in **ARGB.h** file in **DMA HANDLE** define.
- Set other defines with your settings.
- Now we're ready to go!

### TROUBLESHOOTING
- **IF STRIP DOESN'T WORK**
    - You should **convert logic levels**
        - In **Push Pull** GPIO Mode use **SN74LVC** Translator
        - In **Open Drain** GPIO Mode use **1K PullUp** Resistor 
- **COLOR NOISE**
    - Use _Logic Analyzer_ or _Oscilloscope_ to **measure** the signal, or just play with values
        - Correct timer values in **.c** file **152 string**
- **ANY OTHER** 
  - Write an [**issue**](https://github.com/Crazy-Geeks/STM32-ARGB-DMA/issues )!

### Suggestions
- Write an [**issue**](https://github.com/Crazy-Geeks/STM32-ARGB-DMA/issues ) or use [**pull request**](https://github.com/Crazy-Geeks/STM32-ARGB-DMA/pulls )
    
### Special thanks
[**NarodStream**](https://narodstream.ru/stm-urok-119-ws2812b-lenta-na-umnyx-svetodiodax-rgb-chast-2 ), [**VFD**](https://www.thevfdcollective.com/blog/stm32-and-sk6812-rgbw-led )

### Donate options
- [My Site (RU)](https://crazygeeks.ru/donate/ ) 
- [PayPal](https://paypal.me/yasnosos )
- [DonationAlerts](https://www.donationalerts.com/r/yasnosos )
