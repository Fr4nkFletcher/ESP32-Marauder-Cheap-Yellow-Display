//
// Demo sketch to show how to use
// the Kuman 3.5" UNO LCD Shield
// (8-bit parallel ILI9486)
// with bb_spi_lcd on the RP2040
// written by Larry Bank 7/22/2022
// bitbank@pobox.com
//
#include <bb_spi_lcd.h>
#include <AnimatedGIF.h>
#include "homer_bushes.h"

#define WIDTH 480
#define HEIGHT 320

BB_SPI_LCD lcd;
AnimatedGIF gif;
int x_offset, y_offset;
uint16_t *pImage;

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t c, *s, ucTransparent;
    uint16_t *d, *usPalette;
    int x, iWidth;

    if (pDraw->y == 0) { // first line, allocate a backing buffer
       pImage = (uint16_t *)malloc(pDraw->iWidth * pDraw->iHeight * 2);
       lcd.setAddrWindow(x_offset+pDraw->iX, y_offset+pDraw->iY, pDraw->iWidth, pDraw->iHeight);
    }
    iWidth = pDraw->iWidth;
    if (iWidth > lcd.width())
       iWidth = lcd.width();
    usPalette = pDraw->pPalette;

    s = pDraw->pPixels;
    d = &pImage[pDraw->y * pDraw->iWidth];
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
      memset(d, 0, pDraw->iWidth * 2); // set background color to black
    }
    ucTransparent = pDraw->ucTransparent;
    for (x=0; x<iWidth; x++)
    {
      c = *s++;
      if (c != ucTransparent)
        d[x] = usPalette[c];
    } // for x
    lcd.pushPixels(d, pDraw->iWidth);
    if (pDraw->y == pDraw->iHeight-1) { // last line
      free(pImage); // free backing buffer
    }
} /* GIFDraw() */

void playGIF(void)
{
  if (gif.open((uint8_t *)homer_bushes, sizeof(homer_bushes), GIFDraw))
  {
    x_offset = (lcd.width() - gif.getCanvasWidth())/2;
    if (x_offset < 0) x_offset = 0;
    y_offset = (lcd.height() - gif.getCanvasHeight())/2;
    if (y_offset < 0) y_offset = 0;
    while (gif.playFrame(false, NULL))
    {
    }
    gif.close();
  } // if GIF opened
} /* playGIF() */

void setup() {
   lcd.begin(DISPLAY_KUMAN_35);
   lcd.fillScreen(0);
   lcd.setFont(FONT_12x16);
   lcd.setTextColor(0x7e0,0x0000);
   lcd.println("Starting GIF Demo...");
   gif.begin(GIF_PALETTE_RGB565_BE); // big endian pixels
   delay(3000);
   lcd.fillScreen(0);
} /* setup() */

void loop() {
  playGIF();
} /* loop() */
