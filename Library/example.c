#include "ws2812.h"


void main_func(void){
	led_init();
	led_clear();
	
	uint8_t hue = 0;
	while (1){
		led_fillHSV(hue++, 255, 20); // 255 - sat; 20 - value (brightness)
		while (!led_show) ; // just wait for showing
		HAL_Delay(1000);
	}
}