Littlevgl on STM32F103C BluePill and ILI9341 in 8-Bit Parallel with XPT2046 Touch-pad on SPI2 port.
Which able to dim an LED attached to PB6 Pin (thru 100to330 ohm series resistor from GND) by adjusting the slider.
Library dependencies:
1. TFT_eSPI https://github.com/Bodmer/TFT_eSPI
  and choosing the right setup inside User_Setup_Select.h and also set right pins on selected user file.
2.  TFT_eTouch.h    https://github.com/achillhasler/TFT_eTouch
  and set the right pins inside TFT_eTouchUser.h and also run calibrate.ino to get the calibration value and store it inside
  TFT_eTouchUser.h like ` #define TOUCH_DEFAULT_CALIBRATION { 294, 3913, 339, 3869, 2 }`
  Note that TFT_eSPI touch include extension does nether support second SPI port nor touch in parallel mode. 
3. Installing the last master Arduino_Core_STM32 on https://github.com/stm32duino/Arduino_Core_STM32
  because the Hardwaretimer definitions changed a bit in 1.9.0 version which about to release.
4. lv_arduino https://github.com/littlevgl/lv_arduino
  and config the lv_conf.h eg: 
  ```C++
  #define LV_HOR_RES_MAX          (320)
  #define LV_VER_RES_MAX          (240)
  #define LV_USE_THEME_NIGHT      1   /*Dark elegant theme*/
  #define LV_MEM_CUSTOM           1  // otherwise occupied 32k buffer of RAM
  ```
![Example](https://github.com/HamidSaffari/lv_arduino/blob/master/examples/STM32_TFT_eSPI_Slider/photo_2020-04-02_01-55-36.jpg)

EDIT: Since lvgl updated to 7.0.1 the program no longer fits inside STM32F103CB so you have to go to larger like STM32F103RC.

  Created by Hamid Saffari @ Apr 2020. https://github.com/HamidSaffari/
  Released into the public domain.
