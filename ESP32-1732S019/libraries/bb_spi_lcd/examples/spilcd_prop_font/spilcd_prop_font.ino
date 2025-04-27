//
// Simple demo of custom font drawing with the bb_spi_lcd library
//

#include <bb_spi_lcd.h>
#include "FreeSerif12pt7b.h"

SPILCD lcd;

// ILI9341 LCD info for my custom ESP32 rig with an ILI9341
#define LED_PIN 16
#define DC_PIN 12
#define CS_PIN 4
#define SCK_PIN 18
#define MISO_PIN 19
#define MOSI_PIN 23

void setup()
{
//  spilcdInit(&lcd, LCD_ILI9341, FLAGS_NONE, 32000000, CS_PIN, DC_PIN, -1, LED_PIN, MISO_PIN, MOSI_PIN, SCK_PIN); // custom ESP32 rig
 spilcdInit(&lcd, LCD_ST7789_135, FLAGS_NONE, 32000000, 5, 16, -1, 4, -1, 19, 18); // TTGO T-Display pin numbering

//   spilcdSetOrientation(&lcd, LCD_ORIENTATION_90); // 90 degrees rotated
   spilcdFill(&lcd, 0, DRAW_TO_LCD);
} /* setup() */

void loop()
{
int iFG, y;
char szTemp[32];
uint16_t usPal[] = {0xf800, 0x07e0, 0x001f, 0x7ff, 0xf81f, 0xffe0, 0xffff, 0x73ef};

  for (y=0; y<2000; y++)
  {
    sprintf(szTemp, "%04d", y);
    spilcdWriteStringCustom(&lcd, (GFXfont *)&FreeSerif12pt7b, 0, 30, szTemp, 0xffff, 0, 1, DRAW_TO_LCD);   
  }
  delay(4000);
  for (iFG=0; iFG<8; iFG++)
  {
    for (y=20; y<240; y+=20)
    {
      spilcdWriteStringCustom(&lcd, (GFXfont *)&FreeSerif12pt7b, 0, y, (char *)"Hello World!", usPal[iFG], 0, 0, DRAW_TO_LCD);
    }
  }
} // loop
