#include <bb_spi_lcd.h>
//
// callback_demo
//
// An example sketch for using the callback functions of bb_spi_lcd to allow connection to displays
// other than SPI. This allows you to write 2 simple functions specific to your target CPU and display
// while making use of all of the features of bb_spi_lcd
//
// The particular display used for this code is a Kuman 3.5" 320x480 LCD + touch shield for Uno and ATmega2560
// (ILI9486)
//
// Port C bits
#define LCD_RST 0x10
#define LCD_CS 0x8
#define LCD_RS 0x4
#define LCD_WR 0x2
#define LCD_RD 0x1
// Data Lines
// D0-D1 -> PORTB 0, 1
// D2-D7 -> PORTD 2-7

SPILCD lcd;
//
// Called once during spilcdInit()
// Use this function to set up the GPIO ports
// and reset the display (if needed)
//
void ResetCallback(void)
{
int i;

  for (i=2; i<10; i++)
     pinMode(i, OUTPUT);
  for (i=A0; i<A5; i++)
     pinMode(i, OUTPUT);
  digitalWrite(A0, HIGH); // LCD_RD high (disabled)
  PORTC |= LCD_RST;
  delay(100);
  PORTC &= ~LCD_RST; // assert reset low
  delay(100);
  PORTC |= LCD_RST;
  delay(100);
  
} /* ResetCallback() */

//
// Called to send data to the display
// iMode can be 2 possible values: MODE_COMMAND or MODE_DATA
//
void DataCallback(uint8_t *pData, int iLen, int iMode)
{
uint8_t uc, ucWRL, ucWRH;

// CS low
  uc = PORTC;
  uc &= ~LCD_CS;
  if (iMode == MODE_DATA)
     uc |= LCD_RS;
  else
     uc &= ~LCD_RS;
  PORTC = uc;
  ucWRH = uc | LCD_WR;
  ucWRL = uc & ~LCD_WR;
  uc = 255 - pData[0]; // make sure first byte isn't misinterpreted as a repeat
  while (iLen)
  {
    uc = *pData++;
    PORTB = uc; // lower 2 bits
    PORTD = uc; // upper 6 bits
    PORTC = ucWRL; // LCD_WR low (active)
    iLen--; // this is enough delay on a 16Mhz AVR
    PORTC = ucWRH; // data latched on rising edge
  }
  PORTC |= LCD_CS; // CS high to end trasaction
} /* DataCallback() */

// the setup function runs once when you press reset or power the board
void setup()
{
  spilcdSetCallbacks(&lcd, ResetCallback, DataCallback);
// int spilcdInit(int iLCDType, int bFlipRGB, int bInvert, int bFlipped, int32_t iSPIFreq, int iCSPin, int iDCPin, int iResetPin, int iLEDPin, int iMISOPin, int iMOSIPin, int iCLKPin);
  spilcdInit(&lcd, LCD_ILI9486, FLAGS_NONE, 0, -1, -1, -1, -1,  -1, -1, -1);
} /* setup() */

void loop()
{
int iFG, y;
uint16_t usPal[] = {0xf800, 0x07e0, 0x001f, 0x7ff, 0xf81f, 0xffe0, 0xffff, 0x73ef};

  for (iFG=0; iFG<8; iFG++)
  {
    for (y=0; y<480; y+=16)
    { // draw simple colored text down the length of the display
      spilcdWriteString(&lcd, 0, y, (char *)"Hello World!", usPal[iFG], 0, FONT_16x16, DRAW_TO_LCD);
    }
  }
} /* loop() */
