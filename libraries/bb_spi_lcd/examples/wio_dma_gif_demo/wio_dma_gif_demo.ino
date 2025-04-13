//
// Demo to display an animated GIF
// sequence on the 320x240 LCD 
// of the Seeed Studio Wio Termina
// To get a faster framerate, DMA is used
// on the SPI writes so that the CPU can
// continue decoding GIF data while pixels
// are written to the display
//
#include <bb_spi_lcd.h>
#include <AnimatedGIF.h>
#include "badger.h"

#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240

// Wio Terminal Arduino pin definitions
#define DC_PIN 70
#define RESET_PIN 71
#define LED_PIN 72
#define CS_PIN PIN_SPI3_SS
// No need to define these since the Adafruit_ZeroDMA lib manages them
#define MOSI_PIN -1
#define MISO_PIN -1
#define CLK_PIN -1

AnimatedGIF gif;
static SPILCD lcd;
static uint8_t ucTXBuf[1024];

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    uint16_t *d, *usPalette, usTemp[320];
    int x, y, iWidth;

    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    iWidth = pDraw->iWidth;
    if (iWidth > DISPLAY_WIDTH)
       iWidth = DISPLAY_WIDTH;
    s = pDraw->pPixels;
    if (pDraw->ucDisposalMethod == 2) // restore to background color
    {
      for (x=0; x<iWidth; x++)
      {
        if (s[x] == pDraw->ucTransparent)
           s[x] = pDraw->ucBackground;
      }
      pDraw->ucHasTransparency = 0;
    }
    // Apply the new pixels to the main image
    if (pDraw->ucHasTransparency) // if transparency used
    {
      uint8_t *pEnd, c, ucTransparent = pDraw->ucTransparent;
      int x, iCount;
      pEnd = s + iWidth;
      x = 0;
      iCount = 0; // count non-transparent pixels
      while(x < iWidth)
      {
        c = ucTransparent-1;
        d = usTemp;
        while (c != ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent) // done, stop
          {
            s--; // back up to treat it like transparent
          }
          else // opaque
          {
             *d++ = usPalette[c];
             iCount++;
          }
        } // while looking for opaque pixels
        if (iCount) // any opaque pixels?
        {
          spilcdSetPosition(&lcd, pDraw->iX+x, y, iCount, 1, DRAW_TO_LCD);
          spilcdWriteDataBlock(&lcd, (uint8_t *)usTemp, iCount*2, DRAW_TO_LCD | DRAW_WITH_DMA);
          x += iCount;
          iCount = 0;
        }
        // no, look for a run of transparent pixels
        c = ucTransparent;
        while (c == ucTransparent && s < pEnd)
        {
          c = *s++;
          if (c == ucTransparent)
             iCount++;
          else
             s--; 
        }
        if (iCount)
        {
          x += iCount; // skip these
          iCount = 0;
        }
      }
    }
    else
    {
      s = pDraw->pPixels;
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<iWidth; x++)
        usTemp[x] = usPalette[*s++];
      spilcdSetPosition(&lcd, pDraw->iX, y, iWidth, 1, DRAW_TO_LCD);
      spilcdWriteDataBlock(&lcd, (uint8_t *)usTemp, iWidth*2, DRAW_TO_LCD | DRAW_WITH_DMA);
    }
} /* GIFDraw() */

void setup() {

// A TX buffer needs to be defined to use DMA for writing
  spilcdSetTXBuffer(ucTXBuf, sizeof(ucTXBuf));
  spilcdInit(&lcd, LCD_ILI9341, FLAGS_NONE, 30000000, CS_PIN, DC_PIN, RESET_PIN, LED_PIN, MISO_PIN, MOSI_PIN, CLK_PIN);
  spilcdSetOrientation(&lcd, LCD_ORIENTATION_270);
  spilcdFill(&lcd, 0, DRAW_TO_LCD);

  gif.begin(BIG_ENDIAN_PIXELS);
} /* setup() */

void loop() {

  if (gif.open((uint8_t *)badger, sizeof(badger), GIFDraw))
  {
    Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
    while (gif.playFrame(true, NULL))
    {      
    }
    gif.close();
  }
} /* loop() */
