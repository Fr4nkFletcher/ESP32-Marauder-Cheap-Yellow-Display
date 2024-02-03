// https://github.com/PaulStoffregen/ILI9341_t3
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit  ILI9341 Breakout and Shield
  ----> http://www.adafruit.com/products/1651

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

// <SoftEgg>

//Additional graphics routines by Tim Trzepacz, SoftEgg LLC added December 2015
//(And then accidentally deleted and rewritten March 2016. Oops!)
//Gradient support
//----------------
//		fillRectVGradient	- fills area with vertical gradient
//		fillRectHGradient	- fills area with horizontal gradient
//		fillScreenVGradient - fills screen with vertical gradient
// 	fillScreenHGradient - fills screen with horizontal gradient

//Additional Color Support
//------------------------
//		color565toRGB		- converts 565 format 16 bit color to RGB
//		color565toRGB14		- converts 16 bit 565 format color to 14 bit RGB (2 bits clear for math and sign)
//		RGB14tocolor565		- converts 14 bit RGB back to 16 bit 565 format color

//Low Memory Bitmap Support
//-------------------------
// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
// writeRect1BPP - 	write 1 bit per pixel paletted bitmap

//String Pixel Length support
//---------------------------
//		strPixelLen			- gets pixel length of given ASCII string

// <\SoftEgg>

#ifndef _ILI9341_t3H_
#define _ILI9341_t3H_

#ifdef __cplusplus
#include "Arduino.h"
#endif

#if defined(__MKL26Z64__)
#error "Sorry, ILI9341_t3 does not work with Teensy LC.  Use Adafruit_ILI9341."
#elif defined(__AVR__)
#error "Sorry, ILI9341_t3 does not work with Teensy 2.0 or Teensy++ 2.0.  Use Adafruit_ILI9341."
#endif

#define ILI9341_TFTWIDTH  240
#define ILI9341_TFTHEIGHT 320

#define ILI9341_NOP     0x00
#define ILI9341_SWRESET 0x01
#define ILI9341_RDDID   0x04
#define ILI9341_RDDST   0x09

#define ILI9341_SLPIN   0x10
#define ILI9341_SLPOUT  0x11
#define ILI9341_PTLON   0x12
#define ILI9341_NORON   0x13

#define ILI9341_RDMODE  0x0A
#define ILI9341_RDMADCTL  0x0B
#define ILI9341_RDPIXFMT  0x0C
#define ILI9341_RDIMGFMT  0x0D
#define ILI9341_RDSELFDIAG  0x0F

#define ILI9341_INVOFF  0x20
#define ILI9341_INVON   0x21
#define ILI9341_GAMMASET 0x26
#define ILI9341_DISPOFF 0x28
#define ILI9341_DISPON  0x29

#define ILI9341_CASET   0x2A
#define ILI9341_PASET   0x2B
#define ILI9341_RAMWR   0x2C
#define ILI9341_RAMRD   0x2E

#define ILI9341_PTLAR    0x30
#define ILI9341_MADCTL   0x36
#define ILI9341_VSCRSADD 0x37
#define ILI9341_PIXFMT   0x3A

#define ILI9341_FRMCTR1 0xB1
#define ILI9341_FRMCTR2 0xB2
#define ILI9341_FRMCTR3 0xB3
#define ILI9341_INVCTR  0xB4
#define ILI9341_DFUNCTR 0xB6

#define ILI9341_PWCTR1  0xC0
#define ILI9341_PWCTR2  0xC1
#define ILI9341_PWCTR3  0xC2
#define ILI9341_PWCTR4  0xC3
#define ILI9341_PWCTR5  0xC4
#define ILI9341_VMCTR1  0xC5
#define ILI9341_VMCTR2  0xC7

#define ILI9341_RDID1   0xDA
#define ILI9341_RDID2   0xDB
#define ILI9341_RDID3   0xDC
#define ILI9341_RDID4   0xDD

#define ILI9341_GMCTRP1 0xE0
#define ILI9341_GMCTRN1 0xE1
/*
#define ILI9341_PWCTR6  0xFC

*/

// Color definitions
#define ILI9341_BLACK       0x0000      /*   0,   0,   0 */
#define ILI9341_NAVY        0x000F      /*   0,   0, 128 */
#define ILI9341_DARKGREEN   0x03E0      /*   0, 128,   0 */
#define ILI9341_DARKCYAN    0x03EF      /*   0, 128, 128 */
#define ILI9341_MAROON      0x7800      /* 128,   0,   0 */
#define ILI9341_PURPLE      0x780F      /* 128,   0, 128 */
#define ILI9341_OLIVE       0x7BE0      /* 128, 128,   0 */
#define ILI9341_LIGHTGREY   0xC618      /* 192, 192, 192 */
#define ILI9341_DARKGREY    0x7BEF      /* 128, 128, 128 */
#define ILI9341_BLUE        0x001F      /*   0,   0, 255 */
#define ILI9341_GREEN       0x07E0      /*   0, 255,   0 */
#define ILI9341_CYAN        0x07FF      /*   0, 255, 255 */
#define ILI9341_RED         0xF800      /* 255,   0,   0 */
#define ILI9341_MAGENTA     0xF81F      /* 255,   0, 255 */
#define ILI9341_YELLOW      0xFFE0      /* 255, 255,   0 */
#define ILI9341_WHITE       0xFFFF      /* 255, 255, 255 */
#define ILI9341_ORANGE      0xFD20      /* 255, 165,   0 */
#define ILI9341_GREENYELLOW 0xAFE5      /* 173, 255,  47 */
#define ILI9341_PINK        0xF81F

#define CL(_r,_g,_b) ((((_r)&0xF8)<<8)|(((_g)&0xFC)<<3)|((_b)>>3))

#define sint16_t int16_t


// Documentation on the ILI9341_t3 font data format:
// https://forum.pjrc.com/threads/54316-ILI9341_t-font-structure-format

typedef struct {
	const unsigned char *index;
	const unsigned char *unicode;
	const unsigned char *data;
	unsigned char version;
	unsigned char reserved;
	unsigned char index1_first;
	unsigned char index1_last;
	unsigned char index2_first;
	unsigned char index2_last;
	unsigned char bits_index;
	unsigned char bits_width;
	unsigned char bits_height;
	unsigned char bits_xoffset;
	unsigned char bits_yoffset;
	unsigned char bits_delta;
	unsigned char line_space;
	unsigned char cap_height;
} ILI9341_t3_font_t;


#ifdef __cplusplus
// At all other speeds, ILI9241_KINETISK__pspi->beginTransaction() will use the fastest available clock
#include <SPI.h>
#define ILI9341_SPICLOCK 30000000
#define ILI9341_SPICLOCK_READ 6500000

class ILI9341_t3 : public Print
{
  public:
	ILI9341_t3(uint8_t _CS, uint8_t _DC, uint8_t _RST = 255, uint8_t _MOSI=11, uint8_t _SCLK=13, uint8_t _MISO=12);
	void begin(void);
  	void sleep(bool enable);		
        void setClock(unsigned long clk) { _clock = clk;}
	void pushColor(uint16_t color);
	void fillScreen(uint16_t color);
	void drawPixel(int16_t x, int16_t y, uint16_t color);
	void drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color);
	void drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color);
	void fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
			
	void fillRectHGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2);
	void fillRectVGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2);
	void fillScreenVGradient(uint16_t color1, uint16_t color2);
	void fillScreenHGradient(uint16_t color1, uint16_t color2);

	void setRotation(uint8_t r);
	void setScroll(uint16_t offset);
	void invertDisplay(boolean i);
	void setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1);
	// Pass 8-bit (each) R,G,B, get back 16-bit packed color
	static uint16_t color565(uint8_t r, uint8_t g, uint8_t b) {
		return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
	}

	//color565toRGB		- converts 565 format 16 bit color to RGB
	static void color565toRGB(uint16_t color, uint8_t &r, uint8_t &g, uint8_t &b) {
		r = (color>>8)&0x00F8;
		g = (color>>3)&0x00FC;
		b = (color<<3)&0x00F8;
	}
	
	//color565toRGB14		- converts 16 bit 565 format color to 14 bit RGB (2 bits clear for math and sign)
	//returns 00rrrrr000000000,00gggggg00000000,00bbbbb000000000
	//thus not overloading sign, and allowing up to double for additions for fixed point delta
	static void color565toRGB14(uint16_t color, int16_t &r, int16_t &g, int16_t &b) {
		r = (color>>2)&0x3E00;
		g = (color<<3)&0x3F00;
		b = (color<<9)&0x3E00;
	}
	
	//RGB14tocolor565		- converts 14 bit RGB back to 16 bit 565 format color
	static uint16_t RGB14tocolor565(int16_t r, int16_t g, int16_t b)
	{
		return (((r & 0x3E00) << 2) | ((g & 0x3F00) >>3) | ((b & 0x3E00) >> 9));
	}
	
	//uint8_t readdata(void);
	uint8_t readcommand8(uint8_t reg, uint8_t index = 0);
	uint16_t readScanLine();

	// Added functions to read pixel data...
	uint16_t readPixel(int16_t x, int16_t y);
	void readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors);
	void writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors);

	// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
	//					bitmap data in array at pixels, one byte per pixel
	//					color palette data in array at palette
	void writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );

	// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
	//					bitmap data in array at pixels, 4 bits per pixel
	//					color palette data in array at palette
	//					width must be at least 2 pixels
	void writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );
	
	// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
	//					bitmap data in array at pixels, 4 bits per pixel
	//					color palette data in array at palette
	//					width must be at least 4 pixels
	void writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );
	
	// writeRect1BPP - 	write 1 bit per pixel paletted bitmap
	//					bitmap data in array at pixels, 4 bits per pixel
	//					color palette data in array at palette
	//					width must be at least 8 pixels
	void writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette );

	// from Adafruit_GFX.h
	void drawCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void drawCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, uint16_t color);
	void fillCircle(int16_t x0, int16_t y0, int16_t r, uint16_t color);
	void fillCircleHelper(int16_t x0, int16_t y0, int16_t r, uint8_t cornername, int16_t delta, uint16_t color);
	void drawTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void fillTriangle(int16_t x0, int16_t y0, int16_t x1, int16_t y1, int16_t x2, int16_t y2, uint16_t color);
	void drawRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void fillRoundRect(int16_t x0, int16_t y0, int16_t w, int16_t h, int16_t radius, uint16_t color);
	void drawBitmap(int16_t x, int16_t y, const uint8_t *bitmap, int16_t w, int16_t h, uint16_t color);
	void drawChar(int16_t x, int16_t y, unsigned char c, uint16_t color, uint16_t bg, uint8_t size);
	void setCursor(int16_t x, int16_t y);
    void getCursor(int16_t *x, int16_t *y);
	void setTextColor(uint16_t c);
	void setTextColor(uint16_t c, uint16_t bg);
	void setTextSize(uint8_t s);
	uint8_t getTextSize();
	void setTextWrap(boolean w);
	boolean getTextWrap();
	virtual size_t write(uint8_t);
	int16_t width(void)  { return _width; }
	int16_t height(void) { return _height; }
	uint8_t getRotation(void);
	void drawLine(int16_t x0, int16_t y0, int16_t x1, int16_t y1, uint16_t color);
	void drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color);
	int16_t getCursorX(void) const { return cursor_x; }
	int16_t getCursorY(void) const { return cursor_y; }
	void setFont(const ILI9341_t3_font_t &f) { font = &f; }
	void setFontAdafruit(void) { font = NULL; }
	void drawFontChar(unsigned int c);
	void measureChar(uint8_t c, uint16_t* w, uint16_t* h);
	uint16_t fontCapHeight() { return (font) ? font->cap_height : textsize * 8; }
	uint16_t fontLineSpace() { return (font) ? font->line_space : textsize * 8; }
	uint16_t fontGap() { return (font) ? font->line_space - font->cap_height : 0; };
	uint16_t measureTextWidth(const char* text, int chars = 0);
	uint16_t measureTextHeight(const char* text, int chars = 0);
	int16_t strPixelLen(char * str);

 protected:
        unsigned long _clock = ILI9341_SPICLOCK;
	int16_t _width, _height; // Display w/h as modified by current rotation
	int16_t  cursor_x, cursor_y;
	uint16_t textcolor, textbgcolor;
	uint8_t textsize, rotation;
	boolean wrap; // If set, 'wrap' text at right edge of display
	const ILI9341_t3_font_t *font;

  	uint8_t  _rst;
  	uint8_t _cs, _dc;
	uint8_t pcs_data, pcs_command;
	uint8_t _miso, _mosi, _sclk;
	// add support to allow only one hardware CS (used for dc)
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
    uint32_t _cspinmask;
    volatile uint32_t *_csport;
    uint32_t _spi_tcr_current;
    uint32_t _dcpinmask;
    uint8_t _pending_rx_count;
    volatile uint32_t *_dcport;
    uint32_t _tcr_dc_assert;
    uint32_t _tcr_dc_not_assert;
#else
    uint8_t _cspinmask;
    volatile uint8_t *_csport;
#endif
	uint16_t old_x0=-1, old_x1, old_y0=-1, old_y1;
	void setAddr(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
	  __attribute__((always_inline)) {
		if (x0 != old_x0 || x1 != old_x1) {
			writecommand_cont(ILI9341_CASET); // Column addr set
			writedata16_cont(x0);   // XSTART
			writedata16_cont(x1);   // XEND
			old_x0 = x0; old_x1 = x1;
		}
		if (y0 != old_y0 || y1 != old_y1) {
			writecommand_cont(ILI9341_PASET); // Row addr set
			writedata16_cont(y0);   // YSTART
			writedata16_cont(y1);   // YEND
			old_y0 = y0; old_y1 = y1;
		}
	}

//----------------------------------------------------------------------
// Processor Specific stuff

#if defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x
// T4
	void DIRECT_WRITE_LOW(volatile uint32_t * base, uint32_t mask)  __attribute__((always_inline)) {
		*(base+34) = mask;
	}
	void DIRECT_WRITE_HIGH(volatile uint32_t * base, uint32_t mask)  __attribute__((always_inline)) {
		*(base+33) = mask;
	}
	void waitFifoNotFull(void) {
    	uint32_t tmp __attribute__((unused));
    	do {
        	if ((IMXRT_LPSPI4_S.RSR & LPSPI_RSR_RXEMPTY) == 0)  {
            	tmp = IMXRT_LPSPI4_S.RDR;  // Read any pending RX bytes in
            	if (_pending_rx_count) _pending_rx_count--; //decrement count of bytes still levt
        	}
    	} while ((IMXRT_LPSPI4_S.SR & LPSPI_SR_TDF) == 0) ;
	}
	void waitTransmitComplete(void)  {
	    uint32_t tmp __attribute__((unused));
	//    digitalWriteFast(2, HIGH);

	    while (_pending_rx_count) {
	        if ((IMXRT_LPSPI4_S.RSR & LPSPI_RSR_RXEMPTY) == 0)  {
	            tmp = IMXRT_LPSPI4_S.RDR;  // Read any pending RX bytes in
	            _pending_rx_count--; //decrement count of bytes still levt
	        }
	    }
	    IMXRT_LPSPI4_S.CR = LPSPI_CR_MEN | LPSPI_CR_RRF;       // Clear RX FIFO
	//    digitalWriteFast(2, LOW);
	}


	#define TCR_MASK  (LPSPI_TCR_PCS(3) | LPSPI_TCR_FRAMESZ(31) | LPSPI_TCR_CONT | LPSPI_TCR_RXMSK )
	void maybeUpdateTCR(uint32_t requested_tcr_state) /*__attribute__((always_inline)) */ {
		if ((_spi_tcr_current & TCR_MASK) != requested_tcr_state) {
			bool dc_state_change = (_spi_tcr_current & LPSPI_TCR_PCS(3)) != (requested_tcr_state & LPSPI_TCR_PCS(3));
			_spi_tcr_current = (_spi_tcr_current & ~TCR_MASK) | requested_tcr_state ;
			// only output when Transfer queue is empty.
			if (!dc_state_change || !_dcpinmask) {
				while ((IMXRT_LPSPI4_S.FSR & 0x1f) )	;
				IMXRT_LPSPI4_S.TCR = _spi_tcr_current;	// update the TCR

			} else {
				waitTransmitComplete();
				if (requested_tcr_state & LPSPI_TCR_PCS(3)) DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
				else DIRECT_WRITE_LOW(_dcport, _dcpinmask);
				IMXRT_LPSPI4_S.TCR = _spi_tcr_current & ~(LPSPI_TCR_PCS(3) | LPSPI_TCR_CONT);	// go ahead and update TCR anyway?  

			}
		}
	}

	void beginSPITransaction(uint32_t clock = ILI9341_SPICLOCK) __attribute__((always_inline)) {
		SPI.beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
		if (!_dcport) _spi_tcr_current = IMXRT_LPSPI4_S.TCR; 	// Only if DC is on hardware CS 
		if (_csport)
			DIRECT_WRITE_LOW(_csport, _cspinmask);
	}
	void endSPITransaction() __attribute__((always_inline)) {
		if (_csport)
			DIRECT_WRITE_HIGH(_csport, _cspinmask);
		SPI.endTransaction();
	}

	// BUGBUG:: currently assumming we only have CS_0 as valid CS
	void writecommand_cont(uint8_t c) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7) /*| LPSPI_TCR_CONT*/);
		IMXRT_LPSPI4_S.TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	}
	void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_CONT);
		IMXRT_LPSPI4_S.TDR = c;
		_pending_rx_count++;	//
		waitFifoNotFull();
	}
	void writedata16_cont(uint16_t d) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15) | LPSPI_TCR_CONT);
		IMXRT_LPSPI4_S.TDR = d;
		_pending_rx_count++;	//
		waitFifoNotFull();
	}
	void writecommand_last(uint8_t c) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_assert | LPSPI_TCR_FRAMESZ(7));
		IMXRT_LPSPI4_S.TDR = c;
//		IMXRT_LPSPI4_S.SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	}
	void writedata8_last(uint8_t c) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));
		IMXRT_LPSPI4_S.TDR = c;
//		IMXRT_LPSPI4_S.SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	}
	void writedata16_last(uint16_t d) __attribute__((always_inline)) {
		maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(15));
		IMXRT_LPSPI4_S.TDR = d;
//		IMXRT_LPSPI4_S.SR = LPSPI_SR_WCF | LPSPI_SR_FCF | LPSPI_SR_TCF;
		_pending_rx_count++;	//
		waitTransmitComplete();
	}

#else
// T3.x	
	//void waitFifoNotFull(void) __attribute__((always_inline)) {
	void waitFifoNotFull(void) {
		uint32_t sr;
		uint32_t tmp __attribute__((unused));
		do {
			sr = KINETISK_SPI0.SR;
			if (sr & 0xF0) tmp = KINETISK_SPI0.POPR;  // drain RX FIFO
		} while ((sr & (15 << 12)) > (3 << 12));
	}
	void waitFifoEmpty(void) {
		uint32_t sr;
		uint32_t tmp __attribute__((unused));
		do {
			sr = KINETISK_SPI0.SR;
			if (sr & 0xF0) tmp = KINETISK_SPI0.POPR;  // drain RX FIFO
		} while ((sr & 0xF0F0) > 0);             // wait both RX & TX empty
	}
	void waitTransmitComplete(void) __attribute__((always_inline)) {
		uint32_t tmp __attribute__((unused));
		while (!(KINETISK_SPI0.SR & SPI_SR_TCF)) ; // wait until final output done
		tmp = KINETISK_SPI0.POPR;                  // drain the final RX FIFO word
	}
	void waitTransmitComplete(uint32_t mcr) __attribute__((always_inline)) {
		uint32_t tmp __attribute__((unused));
		while (1) {
			uint32_t sr = KINETISK_SPI0.SR;
			if (sr & SPI_SR_EOQF) break;  // wait for last transmit
			if (sr &  0xF0) tmp = KINETISK_SPI0.POPR;
		}
		KINETISK_SPI0.SR = SPI_SR_EOQF;
		SPI0_MCR = mcr;
		while (KINETISK_SPI0.SR & 0xF0) {
			tmp = KINETISK_SPI0.POPR;
		}
	}
	void beginSPITransaction(uint32_t clock = ILI9341_SPICLOCK) __attribute__((always_inline)) {
		SPI.beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
		if (_csport)
			*_csport  &= ~_cspinmask;
	}
	void endSPITransaction() __attribute__((always_inline)) {
		if (_csport)
			*_csport |= _cspinmask;
		SPI.endTransaction();
	}

	void writecommand_cont(uint8_t c) __attribute__((always_inline)) {
		KINETISK_SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
		waitFifoNotFull();
	}
	void writedata8_cont(uint8_t c) __attribute__((always_inline)) {
		KINETISK_SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
		waitFifoNotFull();
	}
	void writedata16_cont(uint16_t d) __attribute__((always_inline)) {
		KINETISK_SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_CONT;
		waitFifoNotFull();
	}
	void writecommand_last(uint8_t c) __attribute__((always_inline)) {
		uint32_t mcr = SPI0_MCR;
		KINETISK_SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	}
	void writedata8_last(uint8_t c) __attribute__((always_inline)) {
		uint32_t mcr = SPI0_MCR;
		KINETISK_SPI0.PUSHR = c | (pcs_data << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	}
	void writedata16_last(uint16_t d) __attribute__((always_inline)) {
		uint32_t mcr = SPI0_MCR;
		KINETISK_SPI0.PUSHR = d | (pcs_data << 16) | SPI_PUSHR_CTAS(1) | SPI_PUSHR_EOQ;
		waitTransmitComplete(mcr);
	}
#endif
	void HLine(int16_t x, int16_t y, int16_t w, uint16_t color)
	  __attribute__((always_inline)) {
	  	if((x >= _width) || (y >= _height) || (y < 0)) return;
		if(x < 0) {	w += x; x = 0; 	}
		if((x+w-1) >= _width)  w = _width-x;

		setAddr(x, y, x+w-1, y);
		writecommand_cont(ILI9341_RAMWR);
		do { writedata16_cont(color); } while (--w > 0);
	}
	void VLine(int16_t x, int16_t y, int16_t h, uint16_t color)
	  __attribute__((always_inline)) {
		if((x >= _width) || (x < 0) || (y >= _height)) return;
		if(y < 0) {	h += y; y = 0; 	}
		if((y+h-1) >= _height) h = _height-y;
		setAddr(x, y, x, y+h-1);
		writecommand_cont(ILI9341_RAMWR);
		do { writedata16_cont(color); } while (--h > 0);
	}
	void Pixel(int16_t x, int16_t y, uint16_t color)
	  __attribute__((always_inline)) {
		if((x >= _width) || (x < 0) || (y >= _height) || (y < 0)) return;
		setAddr(x, y, x, y);
		writecommand_cont(ILI9341_RAMWR);
		writedata16_cont(color);
	}
	void drawFontBits(uint32_t bits, uint32_t numbits, uint32_t x, uint32_t y, uint32_t repeat);
};

// To avoid conflict when also using Adafruit_GFX or any Adafruit library
// which depends on Adafruit_GFX, #include the Adafruit library *BEFORE*
// you #include ILI9341_t3.h.
#ifndef _ADAFRUIT_GFX_H
class Adafruit_GFX_Button {
public:
	Adafruit_GFX_Button(void) { _gfx = NULL; }
	void initButton(ILI9341_t3 *gfx, int16_t x, int16_t y,
		uint8_t w, uint8_t h,
		uint16_t outline, uint16_t fill, uint16_t textcolor,
		const char *label, uint8_t textsize);
	void drawButton(bool inverted = false);
	bool contains(int16_t x, int16_t y);
	void press(boolean p) {
		laststate = currstate;
		currstate = p;
	}
	bool isPressed() { return currstate; }
	bool justPressed() { return (currstate && !laststate); }
	bool justReleased() { return (!currstate && laststate); }
private:
	ILI9341_t3 *_gfx;
	int16_t _x, _y;
	uint16_t _w, _h;
	uint8_t _textsize;
	uint16_t _outlinecolor, _fillcolor, _textcolor;
	char _label[10];
	boolean currstate, laststate;
};
#endif

#endif // __cplusplus


#endif
