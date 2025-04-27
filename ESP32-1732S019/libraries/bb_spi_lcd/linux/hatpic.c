//
// HatPic - display a picture on a TFT 'HAT'
// written by Larry Bank
// 11/1/2021
//
// select the hat type here (defaults to WaveShare 1.3" ST7789 240x240
//#define HAT_ADAFRUIT_PITFT

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>
#include <unistd.h>
#include "bb_spi_lcd.h"
#include <armbianio.h>
#include "../../AnimatedGIF/src/AnimatedGIF.h"
#include "../../AnimatedGIF/src/gif.inl"
#ifdef HAT_ADAFRUIT_PITFT
#define DISPLAY_WIDTH 320
#define DISPLAY_HEIGHT 240
#define LCD_TYPE LCD_ILI9341

#else // WaveShare 1.3" 240x240

#define DISPLAY_WIDTH 240
#define DISPLAY_HEIGHT 240
#define LCD_TYPE LCD_ST7789_240

#endif

// Common pins
// bb_spi_lcd library uses the header pin number, not the BCM GPIO number
// to reduce confusion

// GPIO 25 = Pin 22
#define DC_PIN 22
// GPIO 27 = Pin 13
#define RESET_PIN 13
// GPIO 8 = Pin 24
#define CS_PIN 24
// GPIO 24 = Pin 18
#define LED_PIN 18

SPILCD lcd;
static uint8_t ucBuffer[4096], ucBuffer2[4096];
GIFIMAGE gif;
uint8_t *pGIFBuf;

// Draw a line of image directly on the LCD
void GIFDraw(GIFDRAW *pDraw)
{
    uint8_t *s;
    uint16_t *usPalette, *d;
    int x, y, iCanvasWidth, iWidth;
    GIFIMAGE *pGIF = (GIFIMAGE *)pDraw->pUser; // access to full gif frame info

    iCanvasWidth = pGIF->iCanvasWidth; // full width of the image
    usPalette = pDraw->pPalette;
    y = pDraw->iY + pDraw->y; // current line
    iWidth = pDraw->iWidth;
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
        d = (uint16_t *)&pGIFBuf[y*iCanvasWidth*2 + (x + pDraw->iX) * 2];
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
      d = (uint16_t *)&pGIFBuf[(y * iCanvasWidth * 2) + (pDraw->iX * 2)];
      // Translate the 8-bit pixels through the RGB565 palette (already byte reversed)
      for (x=0; x<iWidth; x++) {
        d[x] = usPalette[*s++];
      }
    }
} /* GIFDraw() */

//
// Read a file into memory
//
uint8_t * ReadFile(char *fname, int *pFileSize)
{
FILE *infile;
unsigned long ulSize;
uint8_t *pData;
   
    infile = fopen(fname, "rb");
    if (infile != NULL) {
      // get the file size
        fseek(infile, 0L, SEEK_END);
        ulSize = ftell(infile);
        fseek(infile, 0L, SEEK_SET);
        pData = (uint8_t *)malloc(ulSize);
        if (pData) {
            *pFileSize = (int)ulSize;
            fread(pData, 1, ulSize, infile);
        }
        fclose(infile);
        return pData;
    }
    return NULL; // failure to open
} /* ReadFile() */

int main(int argc, char **argv)
{
uint8_t *pData;
int i, iLen;

   i = AIOInitBoard("Raspberry Pi");
   if (i == 0) // problem
   {
       printf("Error in AIOInit(); check if this board is supported\n");
       return 0;
   }

   if (argc != 2) {
       printf("Usage: hatpic <image file>\n");
       return -1;
   }
  pData = ReadFile(argv[1], &iLen);
  if (pData == NULL) {
     printf("Something went wrong :(\n");
     return -1;
  }
  printf("Successfully read file; size = %d bytes\n", iLen);
  spilcdSetTXBuffer(ucBuffer, 4096);
// Both LCD hats can handle the max SPI speed of 62.5Mhz
  i = spilcdInit(&lcd, LCD_TYPE, FLAGS_NONE, 62500000, CS_PIN, DC_PIN, RESET_PIN, LED_PIN, -1,-1,-1);
  if (i == 0)
  {
      spilcdSetOrientation(&lcd, LCD_ORIENTATION_90);
      spilcdFill(&lcd, 0, DRAW_TO_LCD);
      if (pData[0] == 'B' && pData[1] == 'M') {
                spilcdDrawBMP(&lcd, pData, 0, 0, 0, -1, DRAW_TO_LCD);
		} else if (pData[0] == 0xff && pData[1] == 0xd8) {
                // JPEG
		} else if (pData[1] == 'P' && pData[2] == 'N') {
		// PNG
		} else if (pData[0] == 'G' && pData[1] == 'I')
		{ // GIF
		       int iDelay, rc, y, ty, dy, cx, cy;
		       pGIFBuf = (uint8_t *)malloc(640 * 240);
		       GIF_begin(&gif, GIF_PALETTE_RGB565_BE);
                       rc = GIF_openRAM(&gif, pData, iLen, GIFDraw);
                       if (rc) {
                          printf("GIF dimensions: %d x %d\n", gif.iCanvasWidth, gif.iCanvasHeight);
			  cx = (DISPLAY_WIDTH - gif.iCanvasWidth) /2;
			  cy = (DISPLAY_HEIGHT - gif.iCanvasHeight) / 2;
			  while (1) {
                          while (GIF_playFrame(&gif, &iDelay, (void *)&gif)) {
		            spilcdSetPosition(&lcd, cx+ gif.iX, cy + gif.iY, gif.iWidth, gif.iHeight, DRAW_TO_LCD); // only draw the delta frame size
			    ty = 2048 / gif.iWidth; // max lines per 4k write
			    //printf("number of lines = %d\n", ty);
			    for (y=0; y<gif.iHeight; y+=ty) {
			       dy = gif.iHeight - y;
			       if (dy > ty) dy = ty; // last block?
                               // Gather the data into a contiguous block
			       // to write in one shot
			       for (int i=0; i<dy; i++) {
				   uint8_t *s = &pGIFBuf[((gif.iY + y + i) * gif.iCanvasWidth * 2) + (gif.iX * 2)];
				   uint8_t *d = &ucBuffer2[i * gif.iWidth * 2];
				   memcpy (d, s, gif.iWidth * 2);
			       }
			       spilcdWriteDataBlock(&lcd, ucBuffer2, gif.iWidth*2*dy, DRAW_TO_LCD);
			    } // for y
                            // usleep(iDelay * 1000);
                       } // while GIF_PlayFrame
			  } // while (1)
		   }
		}
	} else {
	   printf("Error initialized spilcd library\n");
	}
} /* main() */

