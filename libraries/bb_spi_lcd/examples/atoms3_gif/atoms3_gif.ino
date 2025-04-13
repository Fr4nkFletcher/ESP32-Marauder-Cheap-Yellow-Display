#include <bb_spi_lcd.h>
#include <AnimatedGIF.h>
#include "thisisfine_128x128.h"

#define WIDTH 128
#define HEIGHT 128

#define BUTTON_USER 41
#define SDA_PIN 38
#define SCL_PIN 39

BB_SPI_LCD lcd;
AnimatedGIF gif;

static uint16_t *pImage; // buffered GIF image
int x_offset, y_offset;

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t c, *s, ucTransparent;
    uint16_t *d, *usPalette;
    int x, iWidth;

    if (pDraw->y == 0) { // first line, allocate a backing buffer
       pImage = (uint16_t *)malloc(gif.getCanvasWidth() * gif.getCanvasHeight() * 2);
       lcd.setAddrWindow(x_offset+pDraw->iX, y_offset+pDraw->iY, pDraw->iWidth, pDraw->iHeight);
    }
    iWidth = pDraw->iWidth;
    if (iWidth > lcd.width())
       iWidth = lcd.width();
    usPalette = pDraw->pPalette;

    s = pDraw->pPixels;
    d = &pImage[pDraw->iX + (pDraw->y + pDraw->iY) * pDraw->iWidth];
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

void setup() {
//   Serial.begin(115200);
//   delay(3000);
//   Serial.println("Starting...");
   pinMode(BUTTON_USER, INPUT);
   lcd.begin(DISPLAY_M5STACK_ATOMS3);
   lcd.allocBuffer();
   lcd.fillScreen(0);
   lcd.setFont(FONT_12x16);
   lcd.setTextColor(0x7e0,0x0000);
   lcd.println("Starting...");
   gif.begin(GIF_PALETTE_RGB565_BE); // big endian pixels
   delay(3000);
} /* setup() */

void playGIF(void)
{
  long lTime = millis();
  int iFrames = 0;
  if (gif.open((uint8_t *)thisisfine_128x128, sizeof(thisisfine_128x128), GIFDraw))
  {
    x_offset = (lcd.width() - gif.getCanvasWidth())/2;
    if (x_offset < 0) x_offset = 0;
    y_offset = (lcd.height() - gif.getCanvasHeight())/2;
    if (y_offset < 0) y_offset = 0;
//    Serial.printf("Successfully opened GIF; Canvas size = %d x %d\n", gif.getCanvasWidth(), gif.getCanvasHeight());
//    Serial.flush();
    while (gif.playFrame(true, NULL))
    {
      iFrames++;
    }
    gif.close();
    lTime = millis() - lTime;
    //Serial.printf("%d frames in %d ms\n", iFrames, (int)lTime);
  } // if GIF opened
} /* playGIF() */

void loop() {
  playGIF();
} /* loop() */
