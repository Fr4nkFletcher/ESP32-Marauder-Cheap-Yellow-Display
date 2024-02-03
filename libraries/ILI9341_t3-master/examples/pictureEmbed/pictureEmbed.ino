// Simple example to embed pictures in your sketch
// and draw on the ILI9341 display with writeRect()
//
// By Frank Bösing
//
// https://forum.pjrc.com/threads/32601-SPI-Library-Issue-w-ILI9341-TFT-amp-PN532-NFC-Module-on-Teensy-3-2?p=94534&viewfull=1#post94534

#include "SPI.h"
#include "ILI9341_t3.h"

// Converted to code with:
// http://www.rinkydinkelectronics.com/t_imageconverter565.php
//
#include "picture.c" //the picture

/* GIMP (https://www.gimp.org/) can also be used to export the image using the following steps:

    1. File -> Export As
    2. In the Export Image dialog, use 'C source code (*.c)' as filetype.
    3. Press export to get the export options dialog.
    4. Type the desired variable name into the 'prefixed name' box.
    5. Uncheck 'GLIB types (guint8*)'
    6. Check 'Save as RGB565 (16-bit)'
    7. Press export to save your image.

  Assuming 'image_name' was typed in the 'prefixed name' box of step 4, you can have to include the c file as above,
  using the image can be done with:

    tft.writeRect(0, 0, image_name.width, image_name.height, (uint16_t*)(image_name.pixel_data));

  See also https://forum.pjrc.com/threads/35575-Export-for-ILI9341_t3-with-GIMP 
*/


// Normal Connections
#define TFT_DC       9
#define TFT_CS      10
#define TFT_RST    255  // 255 = unused, connect to 3.3V
#define TFT_MOSI    11
#define TFT_SCLK    13
#define TFT_MISO    12

// Alternate Connections with Teensy Audio Shield
//#define TFT_DC      20
//#define TFT_CS      21
//#define TFT_RST    255  // 255 = unused, connect to 3.3V
//#define TFT_MOSI     7
//#define TFT_SCLK    14
//#define TFT_MISO    12

ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC, TFT_RST, TFT_MOSI, TFT_SCLK, TFT_MISO);

void setup() {
  tft.begin();
  tft.setRotation(3);
  tft.fillScreen(ILI9341_BLACK);
  tft.writeRect(32, 33, 256, 174, (uint16_t*)picture);
}

void loop(void) {
}

