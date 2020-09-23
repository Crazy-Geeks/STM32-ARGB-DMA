## STM32-WS2812B-DMA
 Fastest and simpliest library for *WS2812b* for *STM32* Series. Uses *DMA* and *PWM* to control LED Strip
 
### Function reference:
```c
void led_init(void);		// Initalization
void led_set(uint8_t Rpixel, uint8_t Gpixel, uint8_t Bpixel, uint16_t posX);  // Draw pixel in X position by RGB
void led_fill (uint8_t Rpix, uint8_t Gpix, uint8_t Bpix);	// Fill all the strip with color
void led_clear(void);	// Fill strip with black
void led_show(void);	// Recieve DMA Buffer to strip
```

### Instructions for use: 
> Also available in PDF (RU/EN)
- Use *CubeMX* to configure clocks and peripheral. You need to get **72 MHz** for timer clock (ABPx)    
**TIM1**, **TIM8**-**TIM11** uses **ABP2**    
**TIM2**-**TIM7**, **TIM12**-**TIM14** uses **ABP1**
- Next you need to enable *PWM Generation* on your preferred timer channel. *Counter Period* must be set to **89**.
> How can you get this value? Easy. Your clock freq is 72 MHz i.e *72x10^6 Hz*. Divide it to LED freq (8x10^5 Hz), and you will get **90**. Then you need to subtract 1, *because variable starts from **0***, and then you'll get **89**.
- Next enable **DMA** for your timer channel with **"Memory To Peripheral"** mode.
- Next save CubeMX file and generate code.
- Add library to your source destination. 
- Open **"stm32xxxx_it"** file, and include the library. This needs to include custom handlers:    
```c
/* USER CODE BEGIN Includes */
#include "ws2812.h"
/* USER CODE END Includes */
```
- Search for your *DMA IRQ Handler* and add string to **stop DMA**. This will stop DMA broadcast: 
```c
void DMA1_Channel7_IRQHandler(void){
  /* USER CODE BEGIN DMA1_Channel7_IRQn 0 */
  /* USER CODE END DMA1_Channel7_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_tim2_ch2_ch4);
  /* USER CODE BEGIN DMA1_Channel7_IRQn 1 */
  HAL_TIM_PWM_Stop_DMA(&TIM_HANDLE,TIM_CH); // <--- This string
  /* USER CODE END DMA1_Channel7_IRQn 1 */
}
```
- Then include the library to your **main.c** file. 
```c
/* USER CODE BEGIN Includes */
#include "ws2812.h"
/* USER CODE END Includes */
```
- Open **ws2812.h** file and edit your parameters in **22 string**.
- Now it is ready to work!    
    
    
**Special thanks** to: *NarodStream*    
[**Original code**](https://narodstream.ru/stm-urok-119-ws2812b-lenta-na-umnyx-svetodiodax-rgb-chast-2/)    

**Donate:** [PayPal](paypal.me/yasnosos) / [DonationAlerts](https://www.donationalerts.com/r/yasnosos)
