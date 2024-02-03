// https://github.com/PaulStoffregen/ILI9341_t3
// http://forum.pjrc.com/threads/26305-Highly-optimized-ILI9341-(320x240-TFT-color-display)-library

/***************************************************
  This is our library for the Adafruit ILI9341 Breakout and Shield
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
// 		writeRect8BPP - 	write 8 bit per pixel paletted bitmap
// 		writeRect4BPP - 	write 4 bit per pixel paletted bitmap
// 		writeRect2BPP - 	write 2 bit per pixel paletted bitmap
// 		writeRect1BPP - 	write 1 bit per pixel paletted bitmap

//TODO: transparent bitmap writing routines for sprites

//String Pixel Length support 
//---------------------------
//		strPixelLen			- gets pixel length of given ASCII string

// <\SoftEgg>

#include "ILI9341_t3.h"
#include <SPI.h>
#ifndef swap
#define swap(a, b) { typeof(a) t = a; a = b; b = t; }
#endif


// Teensy 3.1 can only generate 30 MHz SPI when running at 120 MHz (overclock)
// At all other speeds, SPI.beginTransaction() will use the fastest available clock

#define WIDTH  ILI9341_TFTWIDTH
#define HEIGHT ILI9341_TFTHEIGHT

// Constructor when using hardware SPI.  Faster, but must use SPI pins
// specific to each board type (e.g. 11,13 for Uno, 51,52 for Mega, etc.)
ILI9341_t3::ILI9341_t3(uint8_t cs, uint8_t dc, uint8_t rst, uint8_t mosi, uint8_t sclk, uint8_t miso)
{
	_cs   = cs;
	_dc   = dc;
	_rst  = rst;
	_mosi = mosi;
	_sclk = sclk;
	_miso = miso;
	_width    = WIDTH;
	_height   = HEIGHT;
	rotation  = 0;
	cursor_y  = cursor_x    = 0;
	textsize  = 1;
	textcolor = textbgcolor = 0xFFFF;
	wrap      = true;
	font      = NULL;
	// Added to see how much impact actually using non hardware CS pin might be
    _cspinmask = 0;
    _csport = NULL;
}

void ILI9341_t3::setAddrWindow(uint16_t x0, uint16_t y0, uint16_t x1, uint16_t y1)
{
	beginSPITransaction(_clock);
	setAddr(x0, y0, x1, y1);
	writecommand_last(ILI9341_RAMWR); // write to RAM
	endSPITransaction();
}

void ILI9341_t3::pushColor(uint16_t color)
{
	beginSPITransaction(_clock);
	writedata16_last(color);
	endSPITransaction();
}

void ILI9341_t3::drawPixel(int16_t x, int16_t y, uint16_t color) {

	if((x < 0) ||(x >= _width) || (y < 0) || (y >= _height)) return;

	beginSPITransaction(_clock);
	setAddr(x, y, x, y);
	writecommand_cont(ILI9341_RAMWR);
	writedata16_last(color);
	endSPITransaction();
}

void ILI9341_t3::drawFastVLine(int16_t x, int16_t y, int16_t h, uint16_t color)
{
	// Rudimentary clipping
	if((x >= _width) || (x < 0) || (y >= _height)) return;
	if(y < 0) {	h += y; y = 0; 	}
	if((y+h-1) >= _height) h = _height-y;
	beginSPITransaction(_clock);
	setAddr(x, y, x, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	while (h-- > 1) {
		writedata16_cont(color);
	}
	writedata16_last(color);
	endSPITransaction();
}

void ILI9341_t3::drawFastHLine(int16_t x, int16_t y, int16_t w, uint16_t color)
{
	// Rudimentary clipping
	if((x >= _width) || (y >= _height) || (y < 0)) return;
	if(x < 0) {	w += x; x = 0; 	}
	if((x+w-1) >= _width)  w = _width-x;
	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y);
	writecommand_cont(ILI9341_RAMWR);
	while (w-- > 1) {
		writedata16_cont(color);
	}
	writedata16_last(color);
	endSPITransaction();
}

void ILI9341_t3::fillScreen(uint16_t color)
{
	fillRect(0, 0, _width, _height, color);
}

// fill a rectangle
void ILI9341_t3::fillRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if(x < 0) {	w += x; x = 0; 	}
	if(y < 0) {	h += y; y = 0; 	}
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;

	// TODO: this can result in a very long transaction time
	// should break this into multiple transactions, even though
	// it'll cost more overhead, so we don't stall other SPI libs
	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16_cont(color);
		}
		writedata16_last(color);
		if (y > 1 && (y & 1)) {
			endSPITransaction();
			beginSPITransaction(_clock);
		}
	}
	endSPITransaction();
}

// fillRectVGradient	- fills area with vertical gradient
void ILI9341_t3::fillRectVGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;
	
	int16_t r1, g1, b1, r2, g2, b2, dr, dg, db, r, g, b;
	color565toRGB14(color1,r1,g1,b1);
	color565toRGB14(color2,r2,g2,b2);
	dr=(r2-r1)/h; dg=(g2-g1)/h; db=(b2-b1)/h;
	r=r1;g=g1;b=b1;	

	// TODO: this can result in a very long transaction time
	// should break this into multiple transactions, even though
	// it'll cost more overhead, so we don't stall other SPI libs
	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		uint16_t color = RGB14tocolor565(r,g,b);

		for(x=w; x>1; x--) {
			writedata16_cont(color);
		}
		writedata16_last(color);
		if (y > 1 && (y & 1)) {
			endSPITransaction();
			beginSPITransaction(_clock);
		}
		r+=dr;g+=dg; b+=db;
	}
	endSPITransaction();
}

// fillRectHGradient	- fills area with horizontal gradient
void ILI9341_t3::fillRectHGradient(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color1, uint16_t color2)
{
	// rudimentary clipping (drawChar w/big text requires this)
	if((x >= _width) || (y >= _height)) return;
	if((x + w - 1) >= _width)  w = _width  - x;
	if((y + h - 1) >= _height) h = _height - y;
	
	int16_t r1, g1, b1, r2, g2, b2, dr, dg, db, r, g, b;
	color565toRGB14(color1,r1,g1,b1);
	color565toRGB14(color2,r2,g2,b2);
	dr=(r2-r1)/h; dg=(g2-g1)/h; db=(b2-b1)/h;
	r=r1;g=g1;b=b1;	

	// TODO: this can result in a very long transaction time
	// should break this into multiple transactions, even though
	// it'll cost more overhead, so we don't stall other SPI libs
	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		uint16_t color;
		for(x=w; x>1; x--) {
			color = RGB14tocolor565(r,g,b);
			writedata16_cont(color);
			r+=dr;g+=dg; b+=db;
		}
		color = RGB14tocolor565(r,g,b);
		writedata16_last(color);
		if (y > 1 && (y & 1)) {
			endSPITransaction();
			beginSPITransaction(_clock);
		}
		r=r1;g=g1;b=b1;
	}
	endSPITransaction();
}

// fillScreenVGradient - fills screen with vertical gradient
void ILI9341_t3::fillScreenVGradient(uint16_t color1, uint16_t color2)
{
	fillRectVGradient(0,0,_width,_height,color1,color2);
}

// fillScreenHGradient - fills screen with horizontal gradient
void ILI9341_t3::fillScreenHGradient(uint16_t color1, uint16_t color2)
{
	fillRectHGradient(0,0,_width,_height,color1,color2);
}


#define MADCTL_MY  0x80
#define MADCTL_MX  0x40
#define MADCTL_MV  0x20
#define MADCTL_ML  0x10
#define MADCTL_RGB 0x00
#define MADCTL_BGR 0x08
#define MADCTL_MH  0x04

void ILI9341_t3::setRotation(uint8_t m)
{
	rotation = m % 4; // can't be higher than 3
	beginSPITransaction(_clock);
	writecommand_cont(ILI9341_MADCTL);
	switch (rotation) {
	case 0:
		writedata8_last(MADCTL_MX | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 1:
		writedata8_last(MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	case 2:
		writedata8_last(MADCTL_MY | MADCTL_BGR);
		_width  = ILI9341_TFTWIDTH;
		_height = ILI9341_TFTHEIGHT;
		break;
	case 3:
		writedata8_last(MADCTL_MX | MADCTL_MY | MADCTL_MV | MADCTL_BGR);
		_width  = ILI9341_TFTHEIGHT;
		_height = ILI9341_TFTWIDTH;
		break;
	}
	endSPITransaction();
	cursor_x = 0;
	cursor_y = 0;
}

void ILI9341_t3::setScroll(uint16_t offset)
{
	beginSPITransaction(_clock);
	writecommand_cont(ILI9341_VSCRSADD);
	writedata16_last(offset);
	endSPITransaction();
}

void ILI9341_t3::invertDisplay(boolean i)
{
	beginSPITransaction(_clock);
	writecommand_last(i ? ILI9341_INVON : ILI9341_INVOFF);
	endSPITransaction();
}










/*
uint8_t ILI9341_t3::readdata(void)
{
  uint8_t r;
       // Try to work directly with SPI registers...
       // First wait until output queue is empty
        uint16_t wTimeout = 0xffff;
        while (((KINETISK_SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        
//       	KINETISK_SPI0.MCR |= SPI_MCR_CLR_RXF; // discard any received data
//		KINETISK_SPI0.SR = SPI_SR_TCF;
        
        // Transfer a 0 out... 
        writedata8_cont(0);   
        
        // Now wait until completed. 
        wTimeout = 0xffff;
        while (((KINETISK_SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty
        r = KINETISK_SPI0.POPR;  // get the received byte... should check for it first...
    return r;
}
 */


uint8_t ILI9341_t3::readcommand8(uint8_t c, uint8_t index)
{
    // Bail if not valid miso
    if (_miso == 0xff) return 0;

 #ifdef KINETISK
    uint16_t wTimeout = 0xffff;
    uint8_t r=0;

    beginSPITransaction(_clock);
    while (((KINETISK_SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

    // Make sure the last frame has been sent...
    KINETISK_SPI0.SR = SPI_SR_TCF;   // dlear it out;
    wTimeout = 0xffff;
    while (!((KINETISK_SPI0.SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

    // clear out any current received bytes
    wTimeout = 0x10;    // should not go more than 4...
    while ((((KINETISK_SPI0.SR) >> 4) & 0xf) && (--wTimeout))  {
        r = KINETISK_SPI0.POPR;
    }

    //writecommand(0xD9); // sekret command
	KINETISK_SPI0.PUSHR = 0xD9 | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
//	while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // writedata(0x10 + index);
	KINETISK_SPI0.PUSHR = (0x10 + index) | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
//	while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // writecommand(c);
   	KINETISK_SPI0.PUSHR = c | (pcs_command << 16) | SPI_PUSHR_CTAS(0) | SPI_PUSHR_CONT;
//	while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // readdata
	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0);
//	while (((KINETISK_SPI0.SR) & (15 << 12)) > (3 << 12)) ; // wait if FIFO full

    // Now wait until completed.
    wTimeout = 0xffff;
    while (((KINETISK_SPI0.SR) & (15 << 12)) && (--wTimeout)) ; // wait until empty

    // Make sure the last frame has been sent...
    KINETISK_SPI0.SR = SPI_SR_TCF;   // dlear it out;
    wTimeout = 0xffff;
    while (!((KINETISK_SPI0.SR) & SPI_SR_TCF) && (--wTimeout)) ; // wait until it says the last frame completed

    wTimeout = 0x10;    // should not go more than 4...
    // lets get all of the values on the FIFO
    while ((((KINETISK_SPI0.SR) >> 4) & 0xf) && (--wTimeout))  {
        r = KINETISK_SPI0.POPR;
    }
    endSPITransaction();
    return r;  // get the received byte... should check for it first...


#elif defined(__IMXRT1062__)  // Teensy 4.x
	uint8_t r=0;

	//digitalWriteFast(2, HIGH); // oscilloscope trigger for testing
	//delayMicroseconds(10);
	//digitalWriteFast(2, LOW);

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	if (_dcport) {
		// DC pin is controlled by GPIO
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
		IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK | LPSPI_TCR_CONT;
		IMXRT_LPSPI4_S.TDR = 0xD9;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete

		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
		IMXRT_LPSPI4_S.TDR = 0x10 + index;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete

		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
		IMXRT_LPSPI4_S.TDR = c;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete

		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
		IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7);
		IMXRT_LPSPI4_S.TDR = 0x10 + index;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete
		while (((IMXRT_LPSPI4_S.FSR >> 16) & 0x1F) == 0) ; // wait until rx fifo not empty
		r = IMXRT_LPSPI4_S.RDR;
	} else {
		// DC pin is controlled by SPI CS hardware



	}
	endSPITransaction();
	return r;  // get the received byte... should check for it first...

#else
	beginSPITransaction(_clock);
	writecommand_cont(0xD9);
	writedata8_cont(0x10 + index);

	writecommand_cont(c);
	writedata8_cont(0);
	uint8_t r = waitTransmitCompleteReturnLast();
	endSPITransaction();
	return r;

#endif   
}

uint16_t ILI9341_t3::readScanLine()
{
#ifdef KINETISK
	return 0; // TODO...

#elif defined(__IMXRT1062__)
	uint16_t line=0;

	//digitalWriteFast(2, HIGH); // oscilloscope trigger for testing
	//delayMicroseconds(10);
	//digitalWriteFast(2, LOW);
	beginSPITransaction(ILI9341_SPICLOCK_READ);
	if (_dcport) {
		// DC pin is controlled by GPIO
		DIRECT_WRITE_LOW(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.SR = LPSPI_SR_TCF | LPSPI_SR_FCF | LPSPI_SR_WCF;
		IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(7) | LPSPI_TCR_RXMSK | LPSPI_TCR_CONT;
		IMXRT_LPSPI4_S.TDR = 0x45;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		IMXRT_LPSPI4_S.TDR = 0;
		IMXRT_LPSPI4_S.TCR = LPSPI_TCR_FRAMESZ(15);
		IMXRT_LPSPI4_S.TDR = 0;
		while (!(IMXRT_LPSPI4_S.SR & LPSPI_SR_WCF)) ; // wait until word complete
		while (((IMXRT_LPSPI4_S.FSR >> 16) & 0x1F) == 0) ; // wait until rx fifo not empty
		line = IMXRT_LPSPI4_S.RDR >> 7;
		//if (IMXRT_LPSPI4_S.FSR != 0) Serial.println("ERROR: junk remains in FIFO!!!");
	} else {
		// DC pin is controlled by SPI CS hardware
		// TODO...
	}
	endSPITransaction();
	return line;
#else
	return 0;
#endif
}



// Read Pixel at x,y and get back 16-bit packed color
uint16_t ILI9341_t3::readPixel(int16_t x, int16_t y)
{
   if (_miso == 0xff) return 0xffff;	// bail if not valid miso
#ifdef KINETISK	
	uint8_t dummy __attribute__((unused));
	uint8_t r,g,b;

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x, y);
	writecommand_cont(ILI9341_RAMRD); // read from RAM
	waitTransmitComplete();

	// Push 4 bytes over SPI
	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	waitFifoEmpty();    // wait for both queues to be empty.

	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_EOQ;

	// Wait for End of Queue
	while ((KINETISK_SPI0.SR & SPI_SR_EOQF) == 0) ;
	KINETISK_SPI0.SR = SPI_SR_EOQF;  // make sure it is clear

	// Read Pixel Data
	dummy = KINETISK_SPI0.POPR;	// Read a DUMMY byte of GRAM
	r = KINETISK_SPI0.POPR;		// Read a RED byte of GRAM
	g = KINETISK_SPI0.POPR;		// Read a GREEN byte of GRAM
	b = KINETISK_SPI0.POPR;		// Read a BLUE byte of GRAM

	endSPITransaction();
	return color565(r,g,b);
#else
	// Kinetisk
	uint16_t colors = 0;
	readRect(x, y, 1, 1, &colors);
	return colors;
#endif	
}

// Now lets see if we can read in multiple pixels
#ifdef KINETISK
void ILI9341_t3::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
   if (_miso == 0xff) return;		// bail if not valid miso
	uint8_t dummy __attribute__((unused));
	uint8_t r,g,b;
	uint16_t c = w * h;

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD); // read from RAM
	waitTransmitComplete();
	KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT | SPI_PUSHR_EOQ;
	while ((KINETISK_SPI0.SR & SPI_SR_EOQF) == 0) ;
	KINETISK_SPI0.SR = SPI_SR_EOQF;  // make sure it is clear
	while ((KINETISK_SPI0.SR & 0xf0)) {
		dummy = KINETISK_SPI0.POPR;	// Read a DUMMY byte but only once
	}
	c *= 3; // number of bytes we will transmit to the display
	while (c--) {
        	if (c) {
            		KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_CONT;
        	} else {
            		KINETISK_SPI0.PUSHR = 0 | (pcs_data << 16) | SPI_PUSHR_CTAS(0)| SPI_PUSHR_EOQ;
		}

		// If last byte wait until all have come in...
		if (c == 0) {
			while ((KINETISK_SPI0.SR & SPI_SR_EOQF) == 0) ;
			KINETISK_SPI0.SR = SPI_SR_EOQF;  // make sure it is clear
		}

		if ((KINETISK_SPI0.SR & 0xf0) >= 0x30) { // do we have at least 3 bytes in queue if so extract...
			r = KINETISK_SPI0.POPR;		// Read a RED byte of GRAM
			g = KINETISK_SPI0.POPR;		// Read a GREEN byte of GRAM
			b = KINETISK_SPI0.POPR;		// Read a BLUE byte of GRAM
			*pcolors++ = color565(r,g,b);
		}

		// like waitFiroNotFull but does not pop our return queue
		while ((KINETISK_SPI0.SR & (15 << 12)) > (3 << 12)) ;
	}
	endSPITransaction();
}
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
void ILI9341_t3::readRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t *pcolors)
{
   if (_miso == 0xff) return;		// bail if not valid miso

	uint8_t rgb[3];               // RGB bytes received from the display
	uint8_t rgbIdx = 0;
	uint32_t txCount = w * h * 3; // number of bytes we will transmit to the display
	uint32_t rxCount = txCount;   // number of bytes we will receive back from the display

	beginSPITransaction(ILI9341_SPICLOCK_READ);

	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMRD); // read from RAM


	// transmit a DUMMY byte before the color bytes
	writedata8_last(0);		// BUGBUG:: maybe fix this as this will wait until the byte fully transfers through.

	while (txCount || rxCount) {
		// transmit another byte if possible
		if (txCount && (IMXRT_LPSPI4_S.SR & LPSPI_SR_TDF)) {
			txCount--;
			if (txCount) {
				IMXRT_LPSPI4_S.TDR = 0;
			} else {
				maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7)); // remove the CONTINUE...
				while ((IMXRT_LPSPI4_S.SR & LPSPI_SR_TDF) == 0) ;		// wait if queue was full
				IMXRT_LPSPI4_S.TDR = 0;
			}
		}

		// receive another byte if possible, and either skip it or store the color
		if (rxCount && !(IMXRT_LPSPI4_S.RSR & LPSPI_RSR_RXEMPTY)) {
			rgb[rgbIdx] = IMXRT_LPSPI4_S.RDR;

			rxCount--;
			rgbIdx++;
			if (rgbIdx == 3) {
				rgbIdx = 0;
				*pcolors++ = color565(rgb[0], rgb[1], rgb[2]);
			}
		}
	}

	// We should have received everything so should be done
	endSPITransaction();
}

#endif
// Now lets see if we can writemultiple pixels
void ILI9341_t3::writeRect(int16_t x, int16_t y, int16_t w, int16_t h, const uint16_t *pcolors)
{
   	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16_cont(*pcolors++);
		}
		writedata16_last(*pcolors++);
	}
	endSPITransaction();
}

// writeRect8BPP - 	write 8 bit per pixel paletted bitmap
//					bitmap data in array at pixels, one byte per pixel
//					color palette data in array at palette
void ILI9341_t3::writeRect8BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
   	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>1; x--) {
			writedata16_cont(palette[*pixels++]);
		}
		writedata16_last(palette[*pixels++]);
	}
	endSPITransaction();
}

// writeRect4BPP - 	write 4 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 2 pixels
void ILI9341_t3::writeRect4BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
   	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>2; x-=2) {
			writedata16_cont(palette[((*pixels)>>4)&0xF]);
			writedata16_cont(palette[(*pixels++)&0xF]);
		}
		writedata16_cont(palette[((*pixels)>>4)&0xF]);
		writedata16_last(palette[(*pixels++)&0xF]);
	}
	endSPITransaction();
}

// writeRect2BPP - 	write 2 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 4 pixels
void ILI9341_t3::writeRect2BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
   	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>4; x-=4) {
			//unrolled loop might be faster?
			writedata16_cont(palette[((*pixels)>>6)&0x3]);
			writedata16_cont(palette[((*pixels)>>4)&0x3]);
			writedata16_cont(palette[((*pixels)>>2)&0x3]);
			writedata16_cont(palette[(*pixels++)&0x3]);
		}
		writedata16_cont(palette[((*pixels)>>6)&0x3]);
		writedata16_cont(palette[((*pixels)>>4)&0x3]);
		writedata16_cont(palette[((*pixels)>>2)&0x3]);
		writedata16_last(palette[(*pixels++)&0x3]);
	}
	endSPITransaction();
}

// writeRect1BPP - 	write 1 bit per pixel paletted bitmap
//					bitmap data in array at pixels, 4 bits per pixel
//					color palette data in array at palette
//					width must be at least 8 pixels
void ILI9341_t3::writeRect1BPP(int16_t x, int16_t y, int16_t w, int16_t h, const uint8_t *pixels, const uint16_t * palette )
{
   	beginSPITransaction(_clock);
	setAddr(x, y, x+w-1, y+h-1);
	writecommand_cont(ILI9341_RAMWR);
	for(y=h; y>0; y--) {
		for(x=w; x>8; x-=8) {
			//unrolled loop might be faster?
			writedata16_cont(palette[((*pixels)>>7)&0x1]);
			writedata16_cont(palette[((*pixels)>>6)&0x1]);
			writedata16_cont(palette[((*pixels)>>5)&0x1]);
			writedata16_cont(palette[((*pixels)>>4)&0x1]);
			writedata16_cont(palette[((*pixels)>>3)&0x1]);
			writedata16_cont(palette[((*pixels)>>2)&0x1]);
			writedata16_cont(palette[((*pixels)>>1)&0x1]);
			writedata16_cont(palette[(*pixels++)&0x1]);
		}
		writedata16_cont(palette[((*pixels)>>7)&0x1]);
		writedata16_cont(palette[((*pixels)>>6)&0x1]);
		writedata16_cont(palette[((*pixels)>>5)&0x1]);
		writedata16_cont(palette[((*pixels)>>4)&0x1]);
		writedata16_cont(palette[((*pixels)>>3)&0x1]);
		writedata16_cont(palette[((*pixels)>>2)&0x1]);
		writedata16_cont(palette[((*pixels)>>1)&0x1]);
		writedata16_last(palette[(*pixels++)&0x1]);
	}
	endSPITransaction();
}


static const uint8_t init_commands[] = {
	4, 0xEF, 0x03, 0x80, 0x02,
	4, 0xCF, 0x00, 0XC1, 0X30,
	5, 0xED, 0x64, 0x03, 0X12, 0X81,
	4, 0xE8, 0x85, 0x00, 0x78,
	6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
	2, 0xF7, 0x20,
	3, 0xEA, 0x00, 0x00,
	2, ILI9341_PWCTR1, 0x23, // Power control
	2, ILI9341_PWCTR2, 0x10, // Power control
	3, ILI9341_VMCTR1, 0x3e, 0x28, // VCM control
	2, ILI9341_VMCTR2, 0x86, // VCM control2
	2, ILI9341_MADCTL, 0x48, // Memory Access Control
	2, ILI9341_PIXFMT, 0x55,
	3, ILI9341_FRMCTR1, 0x00, 0x18,
	4, ILI9341_DFUNCTR, 0x08, 0x82, 0x27, // Display Function Control
	2, 0xF2, 0x00, // Gamma Function Disable
	2, ILI9341_GAMMASET, 0x01, // Gamma curve selected
	16, ILI9341_GMCTRP1, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
		0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
	16, ILI9341_GMCTRN1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
		0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
	3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
	0
};

void ILI9341_t3::begin(void)
{
    // verify SPI pins are valid;
    #ifdef KINETISK
    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
    // Allow to work with mimimum of MOSI and SCK
    if ((_mosi == 255 || _mosi == 11 || _mosi == 7 || _mosi == 28)  && (_sclk == 255 || _sclk == 13 || _sclk == 14 || _sclk == 27)) 
	#else
    if ((_mosi == 255 || _mosi == 11 || _mosi == 7) && (_sclk == 255 || _sclk == 13 || _sclk == 14)) 
    #endif	
    {
        
		if (_mosi != 255) SPI.setMOSI(_mosi);
        if (_sclk != 255) SPI.setSCK(_sclk);

        // Now see if valid MISO
	    #if defined(__MK64FX512__) || defined(__MK66FX1M0__)
	    if (_miso == 12 || _miso == 8 || _miso == 39)
		#else
	    if (_miso == 12 || _miso == 8)
	    #endif
		{	
        	SPI.setMISO(_miso);
    	} else {
			_miso = 0xff;	// set miso to 255 as flag it is bad
		}
	} else {
        return; // not valid pins...
	}
	SPI.begin();
	if (SPI.pinIsChipSelect(_cs, _dc)) {
		pcs_data = SPI.setCS(_cs);
		pcs_command = pcs_data | SPI.setCS(_dc);
	} else {
		// See if at least DC is on chipselect pin, if so try to limp along...
		if (SPI.pinIsChipSelect(_dc)) {
			pcs_data = 0;
			pcs_command = pcs_data | SPI.setCS(_dc);
			pinMode(_cs, OUTPUT);
			_csport    = portOutputRegister(digitalPinToPort(_cs));
			_cspinmask = digitalPinToBitMask(_cs);


		} else {
			pcs_data = 0;

		}
	}
#elif defined(__IMXRT1052__) || defined(__IMXRT1062__)  // Teensy 4.x 
	_pending_rx_count = 0;
	SPI.begin();
	_csport = portOutputRegister(_cs);
	_cspinmask = digitalPinToBitMask(_cs);
	pinMode(_cs, OUTPUT);	
	DIRECT_WRITE_HIGH(_csport, _cspinmask);
	_spi_tcr_current = IMXRT_LPSPI4_S.TCR; // get the current TCR value 

	// TODO:  Need to setup DC to actually work.
	if (SPI.pinIsChipSelect(_dc)) {
	 	uint8_t dc_cs_index = SPI.setCS(_dc);
	 	_dcport = 0;
	 	_dcpinmask = 0;
	 	dc_cs_index--;	// convert to 0 based
		_tcr_dc_assert = LPSPI_TCR_PCS(dc_cs_index);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(3);
	} else {
		//Serial.println("ILI9341_t3n: Error not DC is not valid hardware CS pin");
		_dcport = portOutputRegister(_dc);
		_dcpinmask = digitalPinToBitMask(_dc);
		pinMode(_dc, OUTPUT);	
		DIRECT_WRITE_HIGH(_dcport, _dcpinmask);
		_tcr_dc_assert = LPSPI_TCR_PCS(0);
    	_tcr_dc_not_assert = LPSPI_TCR_PCS(1);
	}
	maybeUpdateTCR(_tcr_dc_not_assert | LPSPI_TCR_FRAMESZ(7));


#endif
	// toggle RST low to reset
	if (_rst < 255) {
		pinMode(_rst, OUTPUT);
		digitalWrite(_rst, HIGH);
		delay(5);
		digitalWrite(_rst, LOW);
		delay(20);
		digitalWrite(_rst, HIGH);
		delay(150);
	}
	/*
	uint8_t x = readcommand8(ILI9341_RDMODE);
	Serial.print("\nDisplay Power Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDMADCTL);
	Serial.print("\nMADCTL Mode: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDPIXFMT);
	Serial.print("\nPixel Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDIMGFMT);
	Serial.print("\nImage Format: 0x"); Serial.println(x, HEX);
	x = readcommand8(ILI9341_RDSELFDIAG);
	Serial.print("\nSelf Diagnostic: 0x"); Serial.println(x, HEX);
	*/
	beginSPITransaction(_clock);
	const uint8_t *addr = init_commands;
	while (1) {
		uint8_t count = *addr++;
		if (count-- == 0) break;
		writecommand_cont(*addr++);
		while (count-- > 0) {
			writedata8_cont(*addr++);
		}
	}
	writecommand_last(ILI9341_SLPOUT);    // Exit Sleep
	endSPITransaction();

	delay(120); 		
	beginSPITransaction(_clock);
	writecommand_last(ILI9341_DISPON);    // Display on
	endSPITransaction();
}




/*
This is the core graphics library for all our displays, providing a common
set of graphics primitives (points, lines, circles, etc.).  It needs to be
paired with a hardware-specific library for each display device we carry
(to handle the lower-level functions).

Adafruit invests time and resources providing this open source code, please
support Adafruit & open-source hardware by purchasing products from Adafruit!

Copyright (c) 2013 Adafruit Industries.  All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

- Redistributions of source code must retain the above copyright notice,
  this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
*/

//#include "glcdfont.c"
extern "C" const unsigned char glcdfont[];

// Draw a circle outline
void ILI9341_t3::drawCircle(int16_t x0, int16_t y0, int16_t r,
    uint16_t color) {
  int16_t f = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x = 0;
  int16_t y = r;

  drawPixel(x0  , y0+r, color);
  drawPixel(x0  , y0-r, color);
  drawPixel(x0+r, y0  , color);
  drawPixel(x0-r, y0  , color);

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f += ddF_y;
    }
    x++;
    ddF_x += 2;
    f += ddF_x;

    drawPixel(x0 + x, y0 + y, color);
    drawPixel(x0 - x, y0 + y, color);
    drawPixel(x0 + x, y0 - y, color);
    drawPixel(x0 - x, y0 - y, color);
    drawPixel(x0 + y, y0 + x, color);
    drawPixel(x0 - y, y0 + x, color);
    drawPixel(x0 + y, y0 - x, color);
    drawPixel(x0 - y, y0 - x, color);
  }
}

void ILI9341_t3::drawCircleHelper( int16_t x0, int16_t y0,
               int16_t r, uint8_t cornername, uint16_t color) {
  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;
  int xold;

  xold = x;
  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;
    if (f >= 0 || x == y) { // time to draw the new line segment
        if (cornername & 0x4) {
            drawFastHLine(x0 + xold+1, y0 + y, x-xold, color);
            drawFastVLine(x0 + y, y0 + xold+1, x-xold, color);
        }
        if (cornername & 0x2) {
            drawFastHLine(x0 + xold+1, y0 - y, x-xold, color);
            drawFastVLine(x0 + y, y0 - x, x-xold, color);
        }
        if (cornername & 0x8) {
            drawFastVLine(x0 - y, y0 + xold+1, x-xold, color);
            drawFastHLine(x0 - x, y0 + y, x-xold, color); 
        }
        if (cornername & 0x1) {
            drawFastVLine(x0 - y, y0 - x, x-xold, color);
            drawFastHLine(x0 - x, y0 - y, x-xold, color);
        }
        xold = x;
     } // draw new line segment
  }
}

void ILI9341_t3::fillCircle(int16_t x0, int16_t y0, int16_t r,
			      uint16_t color) {
  drawFastVLine(x0, y0-r, 2*r+1, color);
  fillCircleHelper(x0, y0, r, 3, 0, color);
}

// Used to do circles and roundrects
void ILI9341_t3::fillCircleHelper(int16_t x0, int16_t y0, int16_t r,
    uint8_t cornername, int16_t delta, uint16_t color) {

  int16_t f     = 1 - r;
  int16_t ddF_x = 1;
  int16_t ddF_y = -2 * r;
  int16_t x     = 0;
  int16_t y     = r;

  while (x<y) {
    if (f >= 0) {
      y--;
      ddF_y += 2;
      f     += ddF_y;
    }
    x++;
    ddF_x += 2;
    f     += ddF_x;

    if (cornername & 0x1) {
      drawFastVLine(x0+x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0+y, y0-x, 2*x+1+delta, color);
    }
    if (cornername & 0x2) {
      drawFastVLine(x0-x, y0-y, 2*y+1+delta, color);
      drawFastVLine(x0-y, y0-x, 2*x+1+delta, color);
    }
  }
}


// Bresenham's algorithm - thx wikpedia
void ILI9341_t3::drawLine(int16_t x0, int16_t y0,
	int16_t x1, int16_t y1, uint16_t color)
{
	if (y0 == y1) {
		if (x1 > x0) {
			drawFastHLine(x0, y0, x1 - x0 + 1, color);
		} else if (x1 < x0) {
			drawFastHLine(x1, y0, x0 - x1 + 1, color);
		} else {
			drawPixel(x0, y0, color);
		}
		return;
	} else if (x0 == x1) {
		if (y1 > y0) {
			drawFastVLine(x0, y0, y1 - y0 + 1, color);
		} else {
			drawFastVLine(x0, y1, y0 - y1 + 1, color);
		}
		return;
	}

	bool steep = abs(y1 - y0) > abs(x1 - x0);
	if (steep) {
		swap(x0, y0);
		swap(x1, y1);
	}
	if (x0 > x1) {
		swap(x0, x1);
		swap(y0, y1);
	}

	int16_t dx, dy;
	dx = x1 - x0;
	dy = abs(y1 - y0);

	int16_t err = dx / 2;
	int16_t ystep;

	if (y0 < y1) {
		ystep = 1;
	} else {
		ystep = -1;
	}

	beginSPITransaction(_clock);
	int16_t xbegin = x0;
	if (steep) {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					VLine(y0, xbegin, len + 1, color);
				} else {
					Pixel(y0, x0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			VLine(y0, xbegin, x0 - xbegin, color);
		}

	} else {
		for (; x0<=x1; x0++) {
			err -= dy;
			if (err < 0) {
				int16_t len = x0 - xbegin;
				if (len) {
					HLine(xbegin, y0, len + 1, color);
				} else {
					Pixel(x0, y0, color);
				}
				xbegin = x0 + 1;
				y0 += ystep;
				err += dx;
			}
		}
		if (x0 > xbegin + 1) {
			HLine(xbegin, y0, x0 - xbegin, color);
		}
	}
	writecommand_last(ILI9341_NOP);
	endSPITransaction();
}

// Draw a rectangle
void ILI9341_t3::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color)
{
	beginSPITransaction(_clock);
	HLine(x, y, w, color);
	HLine(x, y+h-1, w, color);
	VLine(x, y, h, color);
	VLine(x+w-1, y, h, color);
	writecommand_last(ILI9341_NOP);
	endSPITransaction();
}

// Draw a rounded rectangle
void ILI9341_t3::drawRoundRect(int16_t x, int16_t y, int16_t w,
  int16_t h, int16_t r, uint16_t color) {
  // smarter version
  drawFastHLine(x+r  , y    , w-2*r, color); // Top
  drawFastHLine(x+r  , y+h-1, w-2*r, color); // Bottom
  drawFastVLine(x    , y+r  , h-2*r, color); // Left
  drawFastVLine(x+w-1, y+r  , h-2*r, color); // Right
  // draw four corners
  drawCircleHelper(x+r    , y+r    , r, 1, color);
  drawCircleHelper(x+w-r-1, y+r    , r, 2, color);
  drawCircleHelper(x+w-r-1, y+h-r-1, r, 4, color);
  drawCircleHelper(x+r    , y+h-r-1, r, 8, color);
}

// Fill a rounded rectangle
void ILI9341_t3::fillRoundRect(int16_t x, int16_t y, int16_t w,
				 int16_t h, int16_t r, uint16_t color) {
  // smarter version
  fillRect(x+r, y, w-2*r, h, color);

  // draw four corners
  fillCircleHelper(x+w-r-1, y+r, r, 1, h-2*r-1, color);
  fillCircleHelper(x+r    , y+r, r, 2, h-2*r-1, color);
}

// Draw a triangle
void ILI9341_t3::drawTriangle(int16_t x0, int16_t y0,
				int16_t x1, int16_t y1,
				int16_t x2, int16_t y2, uint16_t color) {
  drawLine(x0, y0, x1, y1, color);
  drawLine(x1, y1, x2, y2, color);
  drawLine(x2, y2, x0, y0, color);
}

// Fill a triangle
void ILI9341_t3::fillTriangle ( int16_t x0, int16_t y0,
				  int16_t x1, int16_t y1,
				  int16_t x2, int16_t y2, uint16_t color) {

  int16_t a, b, y, last;

  // Sort coordinates by Y order (y2 >= y1 >= y0)
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }
  if (y1 > y2) {
    swap(y2, y1); swap(x2, x1);
  }
  if (y0 > y1) {
    swap(y0, y1); swap(x0, x1);
  }

  if(y0 == y2) { // Handle awkward all-on-same-line case as its own thing
    a = b = x0;
    if(x1 < a)      a = x1;
    else if(x1 > b) b = x1;
    if(x2 < a)      a = x2;
    else if(x2 > b) b = x2;
    drawFastHLine(a, y0, b-a+1, color);
    return;
  }

  int32_t
    dx01 = x1 - x0,
    dy01 = y1 - y0,
    dx02 = x2 - x0,
    dy02 = y2 - y0,
    dx12 = x2 - x1,
    dy12 = y2 - y1,
    sa   = 0,
    sb   = 0;

  // For upper part of triangle, find scanline crossings for segments
  // 0-1 and 0-2.  If y1=y2 (flat-bottomed triangle), the scanline y1
  // is included here (and second loop will be skipped, avoiding a /0
  // error there), otherwise scanline y1 is skipped here and handled
  // in the second loop...which also avoids a /0 error here if y0=y1
  // (flat-topped triangle).
  if(y1 == y2) last = y1;   // Include y1 scanline
  else         last = y1-1; // Skip it

  for(y=y0; y<=last; y++) {
    a   = x0 + sa / dy01;
    b   = x0 + sb / dy02;
    sa += dx01;
    sb += dx02;
    /* longhand:
    a = x0 + (x1 - x0) * (y - y0) / (y1 - y0);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }

  // For lower part of triangle, find scanline crossings for segments
  // 0-2 and 1-2.  This loop is skipped if y1=y2.
  sa = dx12 * (y - y1);
  sb = dx02 * (y - y0);
  for(; y<=y2; y++) {
    a   = x1 + sa / dy12;
    b   = x0 + sb / dy02;
    sa += dx12;
    sb += dx02;
    /* longhand:
    a = x1 + (x2 - x1) * (y - y1) / (y2 - y1);
    b = x0 + (x2 - x0) * (y - y0) / (y2 - y0);
    */
    if(a > b) swap(a,b);
    drawFastHLine(a, y, b-a+1, color);
  }
}

void ILI9341_t3::drawBitmap(int16_t x, int16_t y,
			      const uint8_t *bitmap, int16_t w, int16_t h,
			      uint16_t color) {

  int16_t i, j, byteWidth = (w + 7) / 8;

  for(j=0; j<h; j++) {
    for(i=0; i<w; i++ ) {
      if(pgm_read_byte(bitmap + j * byteWidth + i / 8) & (128 >> (i & 7))) {
	drawPixel(x+i, y+j, color);
      }
    }
  }
}

size_t ILI9341_t3::write(uint8_t c)
{
	if (font) {
		if (c == '\n') {
			cursor_y += font->line_space; // Fix linefeed. Added by T.T., SoftEgg
			cursor_x = 0;
		} else {
			drawFontChar(c);
		}
	} else {
		if (c == '\n') {
			cursor_y += textsize*8;
			cursor_x  = 0;
		} else if (c == '\r') {
			// skip em
		} else {
			drawChar(cursor_x, cursor_y, c, textcolor, textbgcolor, textsize);
			cursor_x += textsize*6;
			if (wrap && (cursor_x > (_width - textsize*6))) {
				cursor_y += textsize*8;
				cursor_x = 0;
			}
		}
	}
	return 1;
}

// Draw a character
void ILI9341_t3::drawChar(int16_t x, int16_t y, unsigned char c,
			    uint16_t fgcolor, uint16_t bgcolor, uint8_t size)
{
	if((x >= _width)            || // Clip right
	   (y >= _height)           || // Clip bottom
	   ((x + 6 * size - 1) < 0) || // Clip left  TODO: is this correct?
	   ((y + 8 * size - 1) < 0))   // Clip top   TODO: is this correct?
		return;

	if (fgcolor == bgcolor) {
		int16_t xoff, yoff, count;
		uint8_t mask = 0x1;
		uint8_t *s = (uint8_t *)&glcdfont[c * 5];
		for (yoff=0; yoff < 8; yoff++) {
			xoff = 0;
			while (xoff < 5) {
				while (xoff < 5 && !(s[xoff] & mask))
					xoff++; // skip transparent bg
				count = 0;
				while (xoff < 5 && s[xoff] & mask) { // fg
					count++;
					xoff++;
				}
				if (count) {
					if (size == 1)
						drawFastHLine(x+xoff-count, y + yoff, count, fgcolor);
					else
						fillRect(x + (xoff-count) * size, y + yoff * size, count * size, size, fgcolor);
				}
			} // while xoff
			mask = mask << 1;
		} // for y
	} else {
		// This solid background approach is about 5 time faster
		beginSPITransaction(_clock);
		setAddr(x, y, x + 6 * size - 1, y + 8 * size - 1);
		writecommand_cont(ILI9341_RAMWR);
		uint8_t xr, yr;
		uint8_t mask = 0x01;
		uint16_t color;
		for (y=0; y < 8; y++) {
			for (yr=0; yr < size; yr++) {
				for (x=0; x < 5; x++) {
					color = (glcdfont[c * 5 + x] & mask) ? fgcolor : bgcolor;
					for (xr=0; xr < size; xr++) {
						writedata16_cont(color);
					}
				}
				for (xr=0; xr < size; xr++) {
					writedata16_cont(bgcolor);
				}
			}
			mask = mask << 1;
		}
		writecommand_last(ILI9341_NOP);
		endSPITransaction();
	}
}

static inline uint32_t fetchbit(const uint8_t *p, uint32_t index)
{
	return (p[index >> 3] & (0x80 >> (index & 7)));
}

static uint32_t fetchbits_unsigned(const uint8_t *p, uint32_t index, uint32_t required)
{
uint32_t val;
uint8_t *s = (uint8_t *)&p[index>>3];

#ifdef UNALIGNED_IS_SAFE
	val = *(uint32_t *)s; // read 4 bytes - unaligned is ok
	val = __builtin_bswap32(val); // change to big-endian order
#else
	val = s[0] << 24;
	val |= (s[1] << 16);
	val |= (s[2] << 8);
	val |= s[3];
#endif
	val <<= (index & 7); // shift out used bits
	if (32 - (index & 7) < required) { // need to get more bits
		val |= (s[4] >> (8 - (index & 7))); 
        }
	val >>= (32-required); // right align the bits
	return val;
}

static uint32_t fetchbits_signed(const uint8_t *p, uint32_t index, uint32_t required)
{
	uint32_t val = fetchbits_unsigned(p, index, required);
	if (val & (1 << (required - 1))) {
		return (int32_t)val - (1 << required);
	}
	return (int32_t)val;
}

// Measure the dimensions for a single character
void ILI9341_t3::measureChar(unsigned char c, uint16_t* w, uint16_t* h) {
	// Treat non-breaking space as normal space
	if (c == 0xa0) {
		c = ' ';
	}

	// Is current font a T3 font or the default Adafruit-GFX font?
	if (font) {
		// ILI9341_T3 font
		*h = font->cap_height;
		*w = 0;

		uint32_t bitoffset;
		const uint8_t *data;

		if (c >= font->index1_first && c <= font->index1_last) {
			bitoffset = c - font->index1_first;
			bitoffset *= font->bits_index;
		}
		else if (c >= font->index2_first && c <= font->index2_last) {
			bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
			bitoffset *= font->bits_index;
		}
		else if (font->unicode) {
			return; // TODO: implement sparse unicode
		}
		else {
			return;
		}

		data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

		uint32_t encoding = fetchbits_unsigned(data, 0, 3);

		if (encoding != 0) return;

		//uint32_t width =
		fetchbits_unsigned(data, 3, font->bits_width);
		bitoffset = font->bits_width + 3;

		//uint32_t height =
		fetchbits_unsigned(data, bitoffset, font->bits_height);
		bitoffset += font->bits_height;

		//int32_t xoffset =
		fetchbits_signed(data, bitoffset, font->bits_xoffset);
		bitoffset += font->bits_xoffset;

		//int32_t yoffset =
		fetchbits_signed(data, bitoffset, font->bits_yoffset);
		bitoffset += font->bits_yoffset;

		uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
		*w = delta;
	}
	else {
		// Adafruit-GFX default font has fixed 6x8 dimensions
		*w = 6 * textsize;
		*h = 8 * textsize;
	}

}


void ILI9341_t3::drawFontChar(unsigned int c)
{
	uint32_t bitoffset;
	const uint8_t *data;

	//Serial.printf("drawFontChar %d\n", c);

	if (c >= font->index1_first && c <= font->index1_last) {
		bitoffset = c - font->index1_first;
		bitoffset *= font->bits_index;
	} else if (c >= font->index2_first && c <= font->index2_last) {
		bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
		bitoffset *= font->bits_index;
	} else if (font->unicode) {
		return; // TODO: implement sparse unicode
	} else {
		return;
	}
	//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
	data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

	uint32_t encoding = fetchbits_unsigned(data, 0, 3);
	if (encoding != 0) return;
	uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
	bitoffset = font->bits_width + 3;
	uint32_t height = fetchbits_unsigned(data, bitoffset, font->bits_height);
	bitoffset += font->bits_height;
	//Serial.printf("  size =   %d,%d\n", width, height);

	int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
	bitoffset += font->bits_xoffset;
	int32_t yoffset = fetchbits_signed(data, bitoffset, font->bits_yoffset);
	bitoffset += font->bits_yoffset;
	//Serial.printf("  offset = %d,%d\n", xoffset, yoffset);

	uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
	bitoffset += font->bits_delta;
	//Serial.printf("  delta =  %d\n", delta);

	//Serial.printf("  cursor = %d,%d\n", cursor_x, cursor_y);

	// horizontally, we draw every pixel, or none at all
	if (cursor_x < 0) cursor_x = 0;
	int32_t origin_x = cursor_x + xoffset;
	if (origin_x < 0) {
		cursor_x -= xoffset;
		origin_x = 0;
	}
	if (origin_x + (int)width > _width) {
		if (!wrap) return;
		origin_x = 0;
		if (xoffset >= 0) {
			cursor_x = 0;
		} else {
			cursor_x = -xoffset;
		}
		cursor_y += font->line_space;
	}
	if (cursor_y >= _height) return;
	cursor_x += delta;

	// vertically, the top and/or bottom can be clipped
	int32_t origin_y = cursor_y + font->cap_height - height - yoffset;
	//Serial.printf("  origin = %d,%d\n", origin_x, origin_y);

	// TODO: compute top skip and number of lines
	int32_t linecount = height;
	//uint32_t loopcount = 0;
	uint32_t y = origin_y;
	while (linecount) {
		//Serial.printf("    linecount = %d\n", linecount);
		uint32_t b = fetchbit(data, bitoffset++);
		if (b == 0) {
			//Serial.println("    single line");
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, 1);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y++;
			linecount--;
		} else {
			uint32_t n = fetchbits_unsigned(data, bitoffset, 3) + 2;
			bitoffset += 3;
			uint32_t x = 0;
			do {
				uint32_t xsize = width - x;
				if (xsize > 32) xsize = 32;
				//Serial.printf("    multi line %d\n", n);
				uint32_t bits = fetchbits_unsigned(data, bitoffset, xsize);
				drawFontBits(bits, xsize, origin_x + x, y, n);
				bitoffset += xsize;
				x += xsize;
			} while (x < width);
			y += n;
			linecount -= n;
		}
		//if (++loopcount > 100) {
			//Serial.println("     abort draw loop");
			//break;
		//}
	}
}

//strPixelLen			- gets pixel length of given ASCII string
int16_t ILI9341_t3::strPixelLen(char * str)
{
//	Serial.printf("strPixelLen %s\n", str);
	if (!str) return(0);
	uint16_t len=0, maxlen=0;
	while (*str)
	{
		if (*str=='\n')
		{
			if ( len > maxlen )
			{
				maxlen=len;
				len=0;
			}
		}
		else
		{
			if (!font)
			{
				len+=textsize*6;
			}
			else
			{

				uint32_t bitoffset;
				const uint8_t *data;
				uint16_t c = *str;

//				Serial.printf("char %c(%d)\n", c,c);

				if (c >= font->index1_first && c <= font->index1_last) {
					bitoffset = c - font->index1_first;
					bitoffset *= font->bits_index;
				} else if (c >= font->index2_first && c <= font->index2_last) {
					bitoffset = c - font->index2_first + font->index1_last - font->index1_first + 1;
					bitoffset *= font->bits_index;
				} else if (font->unicode) {
					continue;
				} else {
					continue;
				}
				//Serial.printf("  index =  %d\n", fetchbits_unsigned(font->index, bitoffset, font->bits_index));
				data = font->data + fetchbits_unsigned(font->index, bitoffset, font->bits_index);

				uint32_t encoding = fetchbits_unsigned(data, 0, 3);
				if (encoding != 0) continue;
//				uint32_t width = fetchbits_unsigned(data, 3, font->bits_width);
//				Serial.printf("  width =  %d\n", width);
				bitoffset = font->bits_width + 3;
				bitoffset += font->bits_height;

//				int32_t xoffset = fetchbits_signed(data, bitoffset, font->bits_xoffset);
//				Serial.printf("  xoffset =  %d\n", xoffset);
				bitoffset += font->bits_xoffset;
				bitoffset += font->bits_yoffset;

				uint32_t delta = fetchbits_unsigned(data, bitoffset, font->bits_delta);
				bitoffset += font->bits_delta;
//				Serial.printf("  delta =  %d\n", delta);

				len += delta;//+width-xoffset;
//				Serial.printf("  len =  %d\n", len);
				if ( len > maxlen )
				{
					maxlen=len;
//					Serial.printf("  maxlen =  %d\n", maxlen);
				}
			
			}
		}
		str++;
	}
//	Serial.printf("Return  maxlen =  %d\n", maxlen);
	return( maxlen );
}

void ILI9341_t3::drawFontBits(uint32_t bits, uint32_t numbits, uint32_t x, uint32_t y, uint32_t repeat)
{
	if (bits == 0) return;
	beginSPITransaction(_clock);
	uint32_t w;
	bits <<= (32-numbits); // left align bits	
	do {
		w = __builtin_clz(bits); // skip over zeros
		if (w > numbits) w = numbits;
		numbits -= w;
		x += w;
		bits <<= w;
		bits = ~bits; // invert to count 1s as 0s
		w = __builtin_clz(bits);
		if (w > numbits) w = numbits; 
		numbits -= w;
		bits <<= w;
		bits = ~bits; // invert back to original polarity
		if (w > 0) {
			x += w;
			setAddr(x-w, y, x-1, y+repeat-1); // write a block of pixels w x repeat sized
			writecommand_cont(ILI9341_RAMWR); // write to RAM
			w *= repeat;
			while (w-- > 1) { // draw line
				writedata16_cont(textcolor);
			}
			writedata16_last(textcolor);
		}
	} while (numbits > 0);
	endSPITransaction();
}

void ILI9341_t3::setCursor(int16_t x, int16_t y) {
	if (x < 0) x = 0;
	else if (x >= _width) x = _width - 1;
	cursor_x = x;
	if (y < 0) y = 0;
	else if (y >= _height) y = _height - 1;
	cursor_y = y;
}

void ILI9341_t3::getCursor(int16_t *x, int16_t *y) {
  *x = cursor_x;
  *y = cursor_y;
}

void ILI9341_t3::setTextSize(uint8_t s) {
  textsize = (s > 0) ? s : 1;
}

uint8_t ILI9341_t3::getTextSize() {
	return textsize;
}

void ILI9341_t3::setTextColor(uint16_t c) {
  // For 'transparent' background, we'll set the bg
  // to the same as fg instead of using a flag
  textcolor = textbgcolor = c;
}

void ILI9341_t3::setTextColor(uint16_t c, uint16_t b) {
  textcolor   = c;
  textbgcolor = b;
}

void ILI9341_t3::setTextWrap(boolean w) {
  wrap = w;
}

boolean ILI9341_t3::getTextWrap()
{
	return wrap;
}

// Return the width of a text string
// - num =  max characters to process, or 0 for entire string (null-terminated)
uint16_t ILI9341_t3::measureTextWidth(const char* text, int num) {
	uint16_t maxH = 0;
	uint16_t currH = 0;
	uint16_t n = num;

	if (n == 0) {
		n = strlen(text);
	};

	for (int i = 0; i < n; i++) {
		if (text[i] == '\n') {
			// For multi-line strings, retain max width
			if (currH > maxH)
				maxH = currH;
			currH = 0;
		}
		else {
			uint16_t h, w;
			measureChar(text[i], &w, &h);
			currH += w;
		}
	}
	uint16_t h = maxH > currH ? maxH : currH;
	return h;
}

// Return the height of a text string
// - num =  max characters to process, or 0 for entire string (null-terminated)
uint16_t ILI9341_t3::measureTextHeight(const char* text, int num) {
	int lines = 1;
	uint16_t n = num;
	if (n == 0) {
		n = strlen(text);
	};
	for (int i = 0; i < n; i++) {
		if (text[i] == '\n') {
			lines++;
		}
	}
	return ((lines - 1) * fontLineSpace() + fontCapHeight());
}

uint8_t ILI9341_t3::getRotation(void) {
  return rotation;
}

void ILI9341_t3::sleep(bool enable) {
	beginSPITransaction(_clock);
	if (enable) {
		writecommand_cont(ILI9341_DISPOFF);		
		writecommand_last(ILI9341_SLPIN);	
		  endSPITransaction();
	} else {
		writecommand_cont(ILI9341_DISPON);
		writecommand_last(ILI9341_SLPOUT);
		endSPITransaction();
		delay(5);
	}
}

void Adafruit_GFX_Button::initButton(ILI9341_t3 *gfx,
	int16_t x, int16_t y, uint8_t w, uint8_t h,
	uint16_t outline, uint16_t fill, uint16_t textcolor,
	const char *label, uint8_t textsize)
{
	_x = x;
	_y = y;
	_w = w;
	_h = h;
	_outlinecolor = outline;
	_fillcolor = fill;
	_textcolor = textcolor;
	_textsize = textsize;
	_gfx = gfx;
	strncpy(_label, label, 9);
	_label[9] = 0;
}

void Adafruit_GFX_Button::drawButton(bool inverted)
{
	uint16_t fill, outline, text;

	if (! inverted) {
		fill = _fillcolor;
		outline = _outlinecolor;
		text = _textcolor;
	} else {
		fill =  _textcolor;
		outline = _outlinecolor;
		text = _fillcolor;
	}
	_gfx->fillRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, fill);
	_gfx->drawRoundRect(_x - (_w/2), _y - (_h/2), _w, _h, min(_w,_h)/4, outline);
	_gfx->setCursor(_x - strlen(_label)*3*_textsize, _y-4*_textsize);
	_gfx->setTextColor(text);
	_gfx->setTextSize(_textsize);
	_gfx->print(_label);
}

bool Adafruit_GFX_Button::contains(int16_t x, int16_t y)
{
	if ((x < (_x - _w/2)) || (x > (_x + _w/2))) return false;
	if ((y < (_y - _h/2)) || (y > (_y + _h/2))) return false;
	return true;
}

