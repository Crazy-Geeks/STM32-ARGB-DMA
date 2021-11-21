#include "ARGB.h"

void main(void){
    ARGB_Init();  // Initialization

    ARGB_Clear(); // Clear stirp
    while (ARGB_Show() != ARGB_OK); // Update - Option 1

    ARGB_SetBrightness(100);  // Set global brightness to 40%

    ARGB_SetRGB(2, 0, 255, 0); // Set LED №3 with 255 Green
    while (!ARGB_Show());  // Update - Option 2

    ARGB_SetHSV(0, 0, 255, 255); // Set LED №1 with Red
    while (!ARGB_Ready()); // Update - Option 3
    ARGB_Show();

    ARGB_FillWhite(230); // Fill all white component with 230
    while (ARGB_Ready() == ARGB_BUSY); // Update - Option 4
    ARGB_Show();

    ARGB_FillRGB(200, 0, 0); // Fill all the strip with Red
    while (!ARGB_Show());
}
