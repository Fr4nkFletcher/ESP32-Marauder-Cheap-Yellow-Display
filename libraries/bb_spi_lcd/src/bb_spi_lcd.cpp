// LCD direct communication using the SPI interface
// Copyright (c) 2017 Larry Bank
// email: bitbank@pobox.com
// Project started 5/15/2017
// 
// Copyright 2017 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
//#define LOG_OUTPUT

// The ILITEK LCD display controllers communicate through the SPI interface
// and two GPIO pins to control the RESET, and D/C (data/command)
// control lines. 
//#if defined(ADAFRUIT_PYBADGE_M4_EXPRESS)
//#define SPI SPI1
#ifndef __LINUX__
#include <SPI.h>
#include <Wire.h>
#endif
//#define SPI mySPI
// MISO, SCK, MOSI
//#endif

#if defined(__SAMD51__)
#define ARDUINO_SAMD_ZERO
#endif

#if defined( ARDUINO_SAMD_ZERO )
#include <SPI.h>
#include "wiring_private.h" // pinPeripheral() function
#include <Arduino.h>
#include <Adafruit_ZeroDMA.h>
#include "utility/dma.h"
#define HAS_DMA

void dma_callback(Adafruit_ZeroDMA *dma);

Adafruit_ZeroDMA myDMA;
ZeroDMAstatus    stat; // DMA status codes returned by some functions
DmacDescriptor *desc;

//SPIClass mySPI (&sercom0, 6, 8, 7, SPI_PAD_2_SCK_3, SERCOM_RX_PAD_1);

#ifdef SEEED_WIO_TERMINAL
SPIClass mySPI(
  &PERIPH_SPI3,         // -> Sercom peripheral
  PIN_SPI3_MISO,    // MISO pin (also digital pin 12)
  PIN_SPI3_SCK,     // SCK pin  (also digital pin 13)
  PIN_SPI3_MOSI,    // MOSI pin (also digital pin 11)
  PAD_SPI3_TX,  // TX pad (MOSI, SCK pads)
  PAD_SPI3_RX); // RX pad (MISO pad)
#else
SPIClass mySPI(
  &PERIPH_SPI,      // -> Sercom peripheral
  PIN_SPI_MISO,     // MISO pin (also digital pin 12)
  PIN_SPI_SCK,      // SCK pin  (also digital pin 13)
  PIN_SPI_MOSI,     // MOSI pin (also digital pin 11)
  PAD_SPI_TX,       // TX pad (MOSI, SCK pads)
  PAD_SPI_RX);      // RX pad (MISO pad)
#endif // WIO
#endif // ARDUINO_SAMD_ZERO

#ifdef __LINUX__
#include <stdint.h>
#include <stdlib.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/i2c-dev.h>
#include <linux/spi/spidev.h>
#include <gpiod.h>
#include <math.h>
#ifndef CONSUMER
#define CONSUMER "Consumer"
#endif
struct gpiod_chip *chip = NULL;
struct gpiod_line *lines[64];
uint8_t ucTXBuf[4096];
volatile uint8_t *pDMA = ucTXBuf;
static uint8_t transfer_is_done = 1;
static int spi_fd; // SPI handle
#else // Arduino
// Use the default (non DMA) SPI library for boards we don't currently support
#if !defined(__SAMD51__) && !defined(ARDUINO_SAMD_ZERO) && !defined(ARDUINO_ARCH_RP2040)
#define mySPI SPI
//SPIClass mySPI(HSPI);
#elif defined(ARDUINO_ARCH_RP2040)
#ifdef ARDUINO_ARCH_MBED
MbedSPI *pSPI = new MbedSPI(12,11,13);
#else // Pico SDK
SPIClassRP2040 *pSPI = &SPI;
#endif
//MbedSPI *pSPI = new MbedSPI(4,7,6); // use GPIO numbers, not pin #s
#endif

#include <Arduino.h>
#include <SPI.h>
#endif // LINUX

#include <bb_spi_lcd.h>

#if defined( ESP_PLATFORM )
#include "esp_cache.h"
#include <rom/cache.h>
//#define ESP32_SPI_HOST VSPI_HOST
#ifdef VSPI_HOST
#define ESP32_SPI_HOST VSPI_HOST
#else
#define ESP32_SPI_HOST SPI2_HOST
#endif // VSPI_HOST
//#define ESP32_DMA
#define HAS_DMA
#define SPI_DMA_CHAN SPI_DMA_CH_AUTO
#endif

volatile int iCurrentCS;

#ifdef ARDUINO_ARCH_ESP32
#include "driver/spi_master.h"
// CS=12, SCK=17, D0=13, D1=18, D2=21, D3=14, RST=16, BL=1
int qspiInit(SPILCD *pLCD, int iLCDType, int iFLAGS, uint32_t u32Freq, uint8_t u8CS, uint8_t u8CLK, uint8_t u8D0, uint8_t u8D1, uint8_t u8D2, uint8_t u8D3, uint8_t u8RST, uint8_t u8LED);
void qspiSendCMD(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pParams, int iLen);
void qspiSendDATA(SPILCD *pLCD, uint8_t *pData, int iLen, int iFlags);
void qspiRotate(SPILCD *pLCD, int iOrient);
void qspiSetPosition(SPILCD *pLCD, int x, int y, int w, int h);
void qspiSetBrightness(SPILCD *pLCD, uint8_t u8Brightness);
static void spi_pre_transfer_callback(spi_transaction_t *t);
static spi_device_interface_config_t devcfg;
static spi_bus_config_t buscfg;
static void spi_post_transfer_callback(spi_transaction_t *t);
// ODROID-GO
//const gpio_num_t SPI_PIN_NUM_MISO = GPIO_NUM_19;
//const gpio_num_t SPI_PIN_NUM_MOSI = GPIO_NUM_23;
//const gpio_num_t SPI_PIN_NUM_CLK  = GPIO_NUM_18;
// M5StickC
//const gpio_num_t SPI_PIN_NUM_MISO = GPIO_NUM_19;
//const gpio_num_t SPI_PIN_NUM_MOSI = GPIO_NUM_15;
//const gpio_num_t SPI_PIN_NUM_CLK  = GPIO_NUM_13;
static spi_transaction_t trans[2];
static spi_device_handle_t spi;
//static TaskHandle_t xTaskToNotify = NULL;
// ESP32 has enough memory to spare 8K
DMA_ATTR uint8_t ucTXBuf[8192];
uint8_t *pDMA0 = ucTXBuf;
uint8_t *pDMA1 = &ucTXBuf[4096]; // 2 ping-pong buffers
volatile uint8_t *pDMA = pDMA0;
static unsigned char ucRXBuf[4096];
//#ifndef ESP32_DMA
static int iTXBufSize = 4096; // max reasonable size
//#endif // ESP32_DMA
#else
//static int iTXBufSize;
//static unsigned char *ucTXBuf;
#ifdef __AVR__
static unsigned char ucRXBuf[512];
#elif defined( ARDUINO_ARCH_RP2040 )
// RP2040 somehow allocates this on an odd byte boundary if we don't
// explicitly align the memory
static uint8_t ucTXBuf[8192];
static bool transfer_is_done = true; // not used the same way with RP2040
static unsigned char ucRXBuf[2048] __attribute__((aligned (16)));
uint8_t *pDMA0 = ucTXBuf;
uint8_t *pDMA1 = &ucTXBuf[sizeof(ucTXBuf)/2]; // 2 ping-pong buffers
volatile uint8_t *pDMA = pDMA0;
#else
static uint8_t ucTXBuf[4096];
static uint8_t ucRXBuf[1024];
static int iTXBufSize = sizeof(ucTXBuf);;
uint8_t *pDMA0 = ucTXBuf;
uint8_t *pDMA1 = &ucTXBuf[sizeof(ucTXBuf)/2]; // 2 ping-pong buffers
volatile uint8_t *pDMA = pDMA0;
volatile bool transfer_is_done = true; // Done yet?
#endif // AVR | RP2040
#endif // !ESP32
#define LCD_DELAY 0xff
#ifdef __AVR__
volatile uint8_t *outDC, *outCS; // port registers for fast I/O
uint8_t bitDC, bitCS; // bit mask for the chosen pins
#endif
#ifdef HAS_DMA
volatile bool transfer_is_done = true; // Done yet?
void spilcdWaitDMA(void);
void spilcdWriteDataDMA(SPILCD *pLCD, uint8_t *pBuf, int iLen);
#endif
static void myPinWrite(int iPin, int iValue);
//static int32_t iSPISpeed;
//static int iCSPin, iDCPin, iResetPin, iLEDPin; // pin numbers for the GPIO control lines
//static int iScrollOffset; // current scroll amount
//static int iOrientation = LCD_ORIENTATION_0; // default to 'natural' orientation
//static int iLCDType, iLCDFlags;
//static int iOldX=-1, iOldY=-1, iOldCX, iOldCY;
//static int iWidth, iHeight;
//static int iCurrentWidth, iCurrentHeight; // reflects virtual size due to orientation
// User-provided callback for writing data/commands
//static DATACALLBACK pfnDataCallback = NULL;
//static RESETCALLBACK pfnResetCallback = NULL;
// For back buffer support
//static int iScreenPitch, iOffset, iMaxOffset;
//static int iSPIMode;
//static int iColStart, iRowStart, iMemoryX, iMemoryY; // display oddities with smaller LCDs
//static uint8_t *pBackBuffer = NULL;
//static int iWindowX, iWindowY, iCurrentX, iCurrentY;
//static int iWindowCX, iWindowCY;
volatile int bSetPosition = 0; // flag telling myspiWrite() to ignore data writes to memory
void spilcdParallelCMDParams(uint8_t ucCMD, uint8_t *pParams, int iLen);
void spilcdParallelData(uint8_t *pData, int iLen);
void spilcdWriteCommand(SPILCD *pLCD, unsigned char);
void spilcdWriteCmdParams(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pParams, int iLen);
void spilcdWriteCommand16(SPILCD *pLCD, uint16_t us);
static void spilcdWriteData8(SPILCD *pLCD, unsigned char c);
static void spilcdWriteData16(SPILCD *pLCD, unsigned short us, int iFlags);
void spilcdSetPosition(SPILCD *pLCD, int x, int y, int w, int h, int iFlags);
int spilcdFill(SPILCD *pLCD, unsigned short usData, int iFlags);
// commands to initialize st7701 parallel display controller
// for the Makerfabs 4" 480x480 board
const uint8_t st7701list[] = {
        6, 0xff, 0x77, 0x01, 0x00, 0x00, 0x10,
        3, 0xc0, 0x3b, 0x00,
        3, 0xc1, 0x0d, 0x02,
        2, 0xcd, 0x08,
        17, 0xb0, 0x00, 0x11, 0x18, 0x0e, 0x11, 0x06, 0x07, 0x08, 0x07, 0x22, 0x04, 0x12, 0x0f, 0xaa, 0x31, 0x18,
        17, 0xb1, 0x00, 0x11, 0x19, 0x0e, 0x12, 0x07, 0x08, 0x08, 0x08, 0x22, 0x04, 0x11, 0x11, 0xa9, 0x32, 0x18,
        6, 0xff, 0x77, 0x01, 0x00, 0x00, 0x11,
        2, 0xb0, 0x60,
        2, 0xb1, 0x32,
        2, 0xb2, 0x07,
        2, 0xb3, 0x80,
        2, 0xb5, 0x49,
        2, 0xb7, 0x85,
        2, 0xb8, 0x21,
        2, 0xc1, 0x78,
        2, 0xc2, 0x78,
        4, 0xe0, 0x00, 0x1b, 0x02,
        12, 0xe1, 0x08, 0xa0, 0x00, 0x00, 0x07, 0xa0, 0x00, 0x00, 0x00, 0x44, 0x44,
        13, 0xe2, 0x11, 0x11, 0x44, 0x44, 0xed, 0xa0, 0x00, 0x00, 0xec, 0xa0, 0x00, 0x00,
        5, 0xe3, 0x00, 0x00, 0x11, 0x11,
        3, 0xe4, 0x44, 0x44,
        17, 0xe5, 0x0a, 0xe9, 0xd8, 0xa0, 0x0c, 0xeb, 0xd8, 0xa0, 0x0e, 0xed, 0xd8, 0xa0, 0x10, 0xef, 0xd8, 0xa0,
        5, 0xe6, 0x00, 0x00, 0x11, 0x11,
        3, 0xe7, 0x44, 0x44,
        17, 0xe8, 0x09, 0xe8, 0xe8, 0xa0, 0x0b, 0xea, 0xd8, 0xa0, 0x0d, 0xec, 0xd8, 0xa0, 0x0f, 0xee, 0xd8, 0xa0,
        8, 0xeb, 0x02, 0x00, 0xe4, 0xe4, 0x88, 0x00, 0x40,
        3, 0xec, 0x3c, 0x00,
        17, 0xed, 0xab, 0x89, 0x76, 0x54, 0x02, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x20, 0x45, 0x67, 0x98, 0xba,
        6, 0xff, 0x77, 0x01, 0x00, 0x00, 0x13,
        2, 0xe5, 0xe4,
        6, 0xff, 0x77, 0x01, 0x00, 0x00, 0x00,
        1, 0x21,
        2, 0x3a, 0x60,
        1, 0x11, // sleep out
        LCD_DELAY, 120,
	1, 0x29, // display on
        0
};
// 16-bit RGB panel 480x480
const BB_RGB rgbpanel_480x480 = { 
    1 /* CS */, 12 /* SCK */, 11 /* SDA */,
    45 /* DE */, 4 /* VSYNC */, 5 /* HSYNC */, 21 /* PCLK */,
    39 /* R0 */, 40 /* R1 */, 41 /* R2 */, 42 /* R3 */, 2 /* R4 */,          
    0 /* G0 */, 9 /* G1 */, 14 /* G2 */, 47 /* G3 */, 48 /* G4 */, 3 /* G5 */, 
    6 /* B0 */, 7 /* B1 */, 15 /* B2 */, 16 /* B3 */, 8 /* B4 */,
    50 /* hsync_back_porch */, 10 /* hsync_front_porch */, 8 /* hsync_pulse_width */,
    20 /* vsync_back_porch */, 10 /* vsync_front_porch */, 8 /* vsync_pulse_width */,
    1 /* hsync_polarity */, 1 /* vsync_polarity */,
    480, 480,
    16000000 // speed
};
// 16-bit RGB panel 800x480 for JC8048W700 (7.0" 800x480)
const BB_RGB rgbpanel_800x480_7 = { 
    -1 /* CS */, -1 /* SCK */, -1 /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,           
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */,
    16 /* hsync_back_porch */, 180 /* hsync_front_porch */, 30 /* hsync_pulse_width */,     
    10 /* vsync_back_porch */, 12 /* vsync_front_porch */, 13 /* vsync_pulse_width */,     
    0 /* hsync_polarity */, 0 /* vsync_polarity */,
    800, 480,
    16000000 // speed
};
// 16-bit RGB panel 800x480 for JC8048W543 and JC8048W550 (4.3"/5.5" 800x480)
const BB_RGB rgbpanel_800x480 = {
    -1 /* CS */, -1 /* SCK */, -1 /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */,
    8 /* hsync_back_porch */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */,
    8 /* vsync_back_porch */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */,
    0 /* hsync_polarity */, 0 /* vsync_polarity */,
    800, 480,
    14000000 // speed
};
// 8-bit parallel pins for the CYD 2.2" ST7789
uint8_t u8_22C_Pins[8] = {15,13,12,14,27,25,33,32};
// 8-bit parallel pins for the WT32-SC01-PLUS
uint8_t u8_WT32_Pins[8] = {9,46,3,8,18,17,16,15};
// 8-bit parallel pins for the Wemos D1 R32 ESP32 + LCD shield
uint8_t u8D1R32DataPins[8] = {12,13,26,25,17,16,27,14};
const unsigned char ucE0_0[] PROGMEM = {0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08, 0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00};
const unsigned char ucE1_0[] PROGMEM = {0x00, 0x0E, 0x14, 0x03, 0x11, 0x07, 0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F};
const unsigned char ucE0_1[] PROGMEM = {0x1f, 0x1a, 0x18, 0x0a, 0x0f, 0x06, 0x45, 0x87, 0x32, 0x0a, 0x07, 0x02, 0x07, 0x05, 0x00};
const unsigned char ucE1_1[] PROGMEM = {0x00, 0x25, 0x27, 0x05, 0x10, 0x09, 0x3a, 0x78, 0x4d, 0x05, 0x18, 0x0d, 0x38, 0x3a, 0x1f};
// Table to convert a group of 4 1-bit pixels to a 2-bit gray level
const uint8_t ucGray2BPP[256] PROGMEM =
{   0x00,0x01,0x01,0x02,0x04,0x05,0x05,0x06,0x04,0x05,0x05,0x06,0x08,0x09,0x09,0x0a,
0x01,0x02,0x02,0x02,0x05,0x06,0x06,0x06,0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,
0x01,0x02,0x02,0x02,0x05,0x06,0x06,0x06,0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,
0x02,0x02,0x02,0x03,0x06,0x06,0x06,0x07,0x06,0x06,0x06,0x07,0x0a,0x0a,0x0a,0x0b,
0x04,0x05,0x05,0x06,0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,
0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,
0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,
0x06,0x06,0x06,0x07,0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,
0x04,0x05,0x05,0x06,0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,
0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,
0x05,0x06,0x06,0x06,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,
0x06,0x06,0x06,0x07,0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,
0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,0x08,0x09,0x09,0x0a,0x0c,0x0d,0x0d,0x0e,
0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x0d,0x0e,0x0e,0x0e,
0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x09,0x0a,0x0a,0x0a,0x0d,0x0e,0x0e,0x0e,
0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,0x0a,0x0a,0x0a,0x0b,0x0e,0x0e,0x0e,0x0f};

// small (8x8) font
const uint8_t ucFont[]PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x5f,0x5f,0x06,0x00,0x00,
    0x00,0x07,0x07,0x00,0x07,0x07,0x00,0x00,0x14,0x7f,0x7f,0x14,0x7f,0x7f,0x14,0x00,
    0x24,0x2e,0x2a,0x6b,0x6b,0x3a,0x12,0x00,0x46,0x66,0x30,0x18,0x0c,0x66,0x62,0x00,
    0x30,0x7a,0x4f,0x5d,0x37,0x7a,0x48,0x00,0x00,0x04,0x07,0x03,0x00,0x00,0x00,0x00,
    0x00,0x1c,0x3e,0x63,0x41,0x00,0x00,0x00,0x00,0x41,0x63,0x3e,0x1c,0x00,0x00,0x00,
    0x08,0x2a,0x3e,0x1c,0x1c,0x3e,0x2a,0x08,0x00,0x08,0x08,0x3e,0x3e,0x08,0x08,0x00,
    0x00,0x00,0x80,0xe0,0x60,0x00,0x00,0x00,0x00,0x08,0x08,0x08,0x08,0x08,0x08,0x00,
    0x00,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x60,0x30,0x18,0x0c,0x06,0x03,0x01,0x00,
    0x3e,0x7f,0x59,0x4d,0x47,0x7f,0x3e,0x00,0x40,0x42,0x7f,0x7f,0x40,0x40,0x00,0x00,
    0x62,0x73,0x59,0x49,0x6f,0x66,0x00,0x00,0x22,0x63,0x49,0x49,0x7f,0x36,0x00,0x00,
    0x18,0x1c,0x16,0x53,0x7f,0x7f,0x50,0x00,0x27,0x67,0x45,0x45,0x7d,0x39,0x00,0x00,
    0x3c,0x7e,0x4b,0x49,0x79,0x30,0x00,0x00,0x03,0x03,0x71,0x79,0x0f,0x07,0x00,0x00,
    0x36,0x7f,0x49,0x49,0x7f,0x36,0x00,0x00,0x06,0x4f,0x49,0x69,0x3f,0x1e,0x00,0x00,
    0x00,0x00,0x00,0x66,0x66,0x00,0x00,0x00,0x00,0x00,0x80,0xe6,0x66,0x00,0x00,0x00,
    0x08,0x1c,0x36,0x63,0x41,0x00,0x00,0x00,0x00,0x14,0x14,0x14,0x14,0x14,0x14,0x00,
    0x00,0x41,0x63,0x36,0x1c,0x08,0x00,0x00,0x00,0x02,0x03,0x59,0x5d,0x07,0x02,0x00,
    0x3e,0x7f,0x41,0x5d,0x5d,0x5f,0x0e,0x00,0x7c,0x7e,0x13,0x13,0x7e,0x7c,0x00,0x00,
    0x41,0x7f,0x7f,0x49,0x49,0x7f,0x36,0x00,0x1c,0x3e,0x63,0x41,0x41,0x63,0x22,0x00,
    0x41,0x7f,0x7f,0x41,0x63,0x3e,0x1c,0x00,0x41,0x7f,0x7f,0x49,0x5d,0x41,0x63,0x00,
    0x41,0x7f,0x7f,0x49,0x1d,0x01,0x03,0x00,0x1c,0x3e,0x63,0x41,0x51,0x33,0x72,0x00,
    0x7f,0x7f,0x08,0x08,0x7f,0x7f,0x00,0x00,0x00,0x41,0x7f,0x7f,0x41,0x00,0x00,0x00,
    0x30,0x70,0x40,0x41,0x7f,0x3f,0x01,0x00,0x41,0x7f,0x7f,0x08,0x1c,0x77,0x63,0x00,
    0x41,0x7f,0x7f,0x41,0x40,0x60,0x70,0x00,0x7f,0x7f,0x0e,0x1c,0x0e,0x7f,0x7f,0x00,
    0x7f,0x7f,0x06,0x0c,0x18,0x7f,0x7f,0x00,0x1c,0x3e,0x63,0x41,0x63,0x3e,0x1c,0x00,
    0x41,0x7f,0x7f,0x49,0x09,0x0f,0x06,0x00,0x1e,0x3f,0x21,0x31,0x61,0x7f,0x5e,0x00,
    0x41,0x7f,0x7f,0x09,0x19,0x7f,0x66,0x00,0x26,0x6f,0x4d,0x49,0x59,0x73,0x32,0x00,
    0x03,0x41,0x7f,0x7f,0x41,0x03,0x00,0x00,0x7f,0x7f,0x40,0x40,0x7f,0x7f,0x00,0x00,
    0x1f,0x3f,0x60,0x60,0x3f,0x1f,0x00,0x00,0x3f,0x7f,0x60,0x30,0x60,0x7f,0x3f,0x00,
    0x63,0x77,0x1c,0x08,0x1c,0x77,0x63,0x00,0x07,0x4f,0x78,0x78,0x4f,0x07,0x00,0x00,
    0x47,0x63,0x71,0x59,0x4d,0x67,0x73,0x00,0x00,0x7f,0x7f,0x41,0x41,0x00,0x00,0x00,
    0x01,0x03,0x06,0x0c,0x18,0x30,0x60,0x00,0x00,0x41,0x41,0x7f,0x7f,0x00,0x00,0x00,
    0x08,0x0c,0x06,0x03,0x06,0x0c,0x08,0x00,0x80,0x80,0x80,0x80,0x80,0x80,0x80,0x80,
    0x00,0x00,0x03,0x07,0x04,0x00,0x00,0x00,0x20,0x74,0x54,0x54,0x3c,0x78,0x40,0x00,
    0x41,0x7f,0x3f,0x48,0x48,0x78,0x30,0x00,0x38,0x7c,0x44,0x44,0x6c,0x28,0x00,0x00,
    0x30,0x78,0x48,0x49,0x3f,0x7f,0x40,0x00,0x38,0x7c,0x54,0x54,0x5c,0x18,0x00,0x00,
    0x48,0x7e,0x7f,0x49,0x03,0x06,0x00,0x00,0x98,0xbc,0xa4,0xa4,0xf8,0x7c,0x04,0x00,
    0x41,0x7f,0x7f,0x08,0x04,0x7c,0x78,0x00,0x00,0x44,0x7d,0x7d,0x40,0x00,0x00,0x00,
    0x60,0xe0,0x80,0x84,0xfd,0x7d,0x00,0x00,0x41,0x7f,0x7f,0x10,0x38,0x6c,0x44,0x00,
    0x00,0x41,0x7f,0x7f,0x40,0x00,0x00,0x00,0x7c,0x7c,0x18,0x78,0x1c,0x7c,0x78,0x00,
    0x7c,0x78,0x04,0x04,0x7c,0x78,0x00,0x00,0x38,0x7c,0x44,0x44,0x7c,0x38,0x00,0x00,
    0x84,0xfc,0xf8,0xa4,0x24,0x3c,0x18,0x00,0x18,0x3c,0x24,0xa4,0xf8,0xfc,0x84,0x00,
    0x44,0x7c,0x78,0x4c,0x04,0x0c,0x18,0x00,0x48,0x5c,0x54,0x74,0x64,0x24,0x00,0x00,
    0x04,0x04,0x3e,0x7f,0x44,0x24,0x00,0x00,0x3c,0x7c,0x40,0x40,0x3c,0x7c,0x40,0x00,
    0x1c,0x3c,0x60,0x60,0x3c,0x1c,0x00,0x00,0x3c,0x7c,0x60,0x30,0x60,0x7c,0x3c,0x00,
    0x44,0x6c,0x38,0x10,0x38,0x6c,0x44,0x00,0x9c,0xbc,0xa0,0xa0,0xfc,0x7c,0x00,0x00,
    0x4c,0x64,0x74,0x5c,0x4c,0x64,0x00,0x00,0x08,0x08,0x3e,0x77,0x41,0x41,0x00,0x00,
    0x00,0x00,0x00,0x77,0x77,0x00,0x00,0x00,0x41,0x41,0x77,0x3e,0x08,0x08,0x00,0x00,
    0x02,0x03,0x01,0x03,0x02,0x03,0x01,0x00,0x70,0x78,0x4c,0x46,0x4c,0x78,0x70,0x00};
// AVR MCUs have very little memory; save 6K of FLASH by stretching the 'normal'
// font instead of using this large font
#ifndef __AVR__
const uint8_t ucBigFont[]PROGMEM = {
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xfc,0xfc,0xff,0xff,0xff,0xff,0xfc,0xfc,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x0f,0x0f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x3f,0x3f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xfc,0xfc,0xfc,0xfc,0xc0,0xc0,0xfc,0xfc,0xfc,0xfc,0xc0,0xc0,0x00,0x00,
  0xc0,0xc0,0xff,0xff,0xff,0xff,0xc0,0xc0,0xff,0xff,0xff,0xff,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0x0f,0x0f,0x3c,0x3c,0x00,0x00,
  0xf0,0xf0,0xc3,0xc3,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0x03,0x03,0x03,0x03,0x3f,0x3f,0x3f,0x3f,0x03,0x03,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x3c,0x3c,0xff,0xff,0xc3,0xc3,0xff,0xff,0x3c,0x3c,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x0f,0x0f,0xfc,0xfc,0xff,0xff,0x03,0x03,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x30,0x30,0x3f,0x3f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0xfc,0xfc,0xf0,0xf0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0x0c,0x0c,0xcc,0xcc,0xff,0xff,0x3f,0x3f,0x3f,0x3f,0xff,0xff,0xcc,0xcc,0x0c,0x0c,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x0c,0x0c,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x30,0x30,0x3f,0x3f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0xc3,0xc3,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0xff,0xff,0xff,0xff,0x30,0x30,0x0f,0x0f,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x30,0x30,0x3c,0x3c,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x03,0x03,0xc3,0xc3,0xff,0xff,0x3c,0x3c,0x00,0x00,
  0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x0f,0x0f,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x00,0x00,0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x3f,0x3f,0xf3,0xf3,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0xc3,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x3c,0x3c,0xf0,0xf0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xf3,0xf3,0x3f,0x3f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x3c,0x3c,0x3f,0x3f,0x03,0x03,0x03,0x03,0xc3,0xc3,0xff,0xff,0x3c,0x3c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x3f,0x3f,0x3f,0x3f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0xfc,0xfc,0xf0,0xf0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0xff,0xff,0xff,0xff,0xff,0xff,0x3f,0x3f,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x3c,0x3c,0xf0,0xf0,0xc0,0xc0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x03,0x03,0x0f,0x0f,0x3c,0x3c,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x0f,0x0f,0xfc,0xfc,0xf0,0xf0,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0xc3,0xc3,0x0f,0x0f,0x3f,0x3f,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x0f,0x0f,0x00,0x00,0xc0,0xc0,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0xc3,0xc3,0x0f,0x0f,0x3f,0x3f,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x03,0x03,0x0f,0x0f,0x3c,0x3c,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x0c,0x0c,0x0c,0x0c,0xfc,0xfc,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x03,0x03,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,
  0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,0xf0,0xf0,0xff,0xff,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x0f,0x0f,0x3f,0x3f,0xf0,0xf0,0xc0,0xc0,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0xfc,0xfc,0xf0,0xf0,0xfc,0xfc,0xff,0xff,0xff,0xff,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x03,0x03,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0xfc,0xfc,0xf0,0xf0,0xc0,0xc0,0xff,0xff,0xff,0xff,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x03,0x03,0x0f,0x0f,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0f,0x0f,0x03,0x03,0x0f,0x0f,0xfc,0xfc,0xf0,0xf0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0xc0,0xc0,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0f,0x0f,0xff,0xff,0xff,0xff,0xc3,0xc3,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x0f,0x0f,0xff,0xff,0xf0,0xf0,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x3c,0x3c,0xff,0xff,0xc3,0xc3,0x03,0x03,0x03,0x03,0x3f,0x3f,0x3c,0x3c,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x03,0x03,0x03,0x03,0x0f,0x0f,0xfc,0xfc,0xf0,0xf0,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x3f,0x3f,0x0f,0x0f,0xff,0xff,0xff,0xff,0x0f,0x0f,0x3f,0x3f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x3f,0x3f,0xff,0xff,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xff,0xff,0x3f,0x3f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0xff,0xff,0xff,0xff,0xc0,0xc0,0xfc,0xfc,0xc0,0xc0,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0xff,0xff,0xf0,0xf0,0x00,0x00,0xf0,0xf0,0xff,0xff,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0xf0,0xf0,0xff,0xff,0x0f,0x0f,0xff,0xff,0xf0,0xf0,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x3f,0x3f,0x0f,0x0f,0x03,0x03,0x03,0x03,0xc3,0xc3,0xff,0xff,0x3f,0x3f,0x00,0x00,
  0xc0,0xc0,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x03,0x03,0x00,0x00,0xc0,0xc0,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xf0,0xf0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x03,0x03,0x0f,0x0f,0x3f,0x3f,0xfc,0xfc,0xf0,0xf0,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xc0,0xc0,0xf0,0xf0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0xf0,0xf0,0xfc,0xfc,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xc0,0xc0,0xc3,0xc3,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xfc,0xfc,0xff,0xff,0x03,0x03,0x0f,0x0f,0x3c,0x3c,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0xc3,0xc3,0xcf,0xcf,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xc0,0xc0,0xcf,0xcf,0xcf,0xcf,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xcf,0xcf,0xcf,0xcf,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0xf0,0xf0,0xf0,0xf0,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,
  0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x3c,0x3c,0xff,0xff,0xc3,0xc3,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x03,0x03,0xff,0xff,0x03,0x03,0xff,0xff,0xff,0xff,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x0f,0x0f,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x03,0x03,0x00,0x00,0x03,0x03,0x0f,0x0f,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x3c,0x3c,0x30,0x30,0xf0,0xf0,0xc3,0xc3,0x03,0x03,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0xfc,0xfc,0xff,0xff,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0xf0,0xf0,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0f,0x0f,0x03,0x03,0x0f,0x0f,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0x00,0x00,0x03,0x03,0xff,0xff,0xfc,0xfc,0xff,0xff,0x03,0x03,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,
  0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0xcc,0xcc,0xff,0xff,0x3f,0x3f,0x00,0x00,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0xc0,0x00,0x00,
  0x03,0x03,0xc3,0xc3,0xf0,0xf0,0x3c,0x3c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,
  0x0f,0x0f,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x0f,0x0f,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x00,0x00,
  0x00,0x00,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0c,0x0c,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xff,0xff,0xff,0xff,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x0f,0x0f,0x0f,0x0f,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x03,0x03,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0xfc,0xfc,0xff,0xff,0x03,0x03,0x03,0x03,0x00,0x00,
  0x00,0x00,0x0c,0x0c,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x0f,0x0f,0x0c,0x0c,0x0f,0x0f,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,
  0x00,0x00,0x00,0x00,0xc0,0xc0,0xf0,0xf0,0xc0,0xc0,0x00,0x00,0x00,0x00,0x00,0x00,
  0xfc,0xfc,0xff,0xff,0x03,0x03,0x00,0x00,0x03,0x03,0xff,0xff,0xfc,0xfc,0x00,0x00,
  0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x03,0x00,0x00,
  0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00};
//
// Table of sine values for 0-360 degrees expressed as a signed 16-bit value
// from -32768 (-1) to 32767 (1)
//
int16_t i16SineTable[] = {0,572, 1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,  // 0-9
        5690,  6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668, // 10-19
        11207,  11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,// 20-29
        16384,  16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,// 30-39
        21063,  21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730,// 40-49
        25102,  25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,// 50-59
        28378,  28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,// 60-69
        30792,  30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,// 70-79
        32270,  32365, 32440, 32524, 32599, 32643, 32688, 32723, 32748, 32763,// 80-89
        32767,  32763, 32748, 32723, 32688, 32643, 32588, 32524, 32449, 32365,// 90-99
        32270,  32166, 32052, 31928, 31795, 31651, 31499, 31336, 31164, 30983,// 100-109
        30792,  30592, 30382, 30163, 29935, 29698, 29452, 29197, 28932, 28660,// 110-119
        28378,  28088, 27789, 27482, 27166, 26842, 26510, 26170, 25822, 25466,// 120-129
        25102,  24730, 24351, 23965, 23571, 23170, 22763, 22348, 21926, 21498,// 130-139
        21063,  20622, 20174, 19720, 19261, 18795, 18324, 17847, 17364, 16877,// 140-149
        16384,  15886, 15384, 14876, 14365, 13848, 13328, 12803, 12275, 11743,// 150-159
        11207,  10668, 10126, 9580, 9032, 8481, 7927, 7371, 6813, 6252,// 160-169
        5690,  5126, 4560, 3993, 3425, 2856, 2286, 1715, 1144, 572,//  170-179
        0,  -572, -1144, -1715, -2286, -2856, -3425, -3993, -4560, -5126,// 180-189
        -5690,  -6252, -6813, -7371, -7927, -8481, -9032, -9580, -10126, -10668,// 190-199
        -11207,  -11743, -12275, -12803, -13328, -13848, -14365, -14876, -15384, -15886,// 200-209
        -16384,  -16877, -17364, -17847, -18324, -18795, -19261, -19720, -20174, -20622,// 210-219
        -21063,  -21498, -21926, -22348, -22763, -23170, -23571, -23965, -24351, -24730, // 220-229
        -25102,  -25466, -25822, -26170, -26510, -26842, -27166, -27482, -27789, -28088, // 230-239
        -28378,  -28660, -28932, -29196, -29452, -29698, -29935, -30163, -30382, -30592, // 240-249
        -30792,  -30983, -31164, -31336, -31499, -31651, -31795, -31928, -32052, -32166, // 250-259
        -32270,  -32365, -32449, -32524, -32588, -32643, -32688, -32723, -32748, -32763, // 260-269
        -32768,  -32763, -32748, -32723, -32688, -32643, -32588, -32524, -32449, -32365, // 270-279
        -32270,  -32166, -32052, -31928, -31795, -31651, -31499, -31336, -31164, -30983, // 280-289
        -30792,  -30592, -30382, -30163, -29935, -29698, -29452, -29196, -28932, -28660, // 290-299
        -28378,  -28088, -27789, -27482, -27166, -26842, -26510, -26170, -25822, -25466, // 300-309
        -25102,  -24730, -24351, -23965, -23571, -23170, -22763, -22348, -21926, -21498, // 310-319
        -21063,  -20622, -20174, -19720, -19261, -18795, -18234, -17847, -17364, -16877, // 320-329
        -16384,  -15886, -15384, -14876, -14365, -13848, -13328, -12803, -12275, -11743, // 330-339
        -11207,  -10668, -10126, -9580, -9032, -8481, -7927, -7371, -6813, -6252,// 340-349
        -5690,  -5126, -4560, -3993, -3425, -2856, -2286, -1715, -1144, -572, // 350-359
// an extra 90 degrees to simulate the cosine function
        0,572,  1144, 1715, 2286, 2856, 3425, 3993, 4560, 5126,// 0-9
        5690,  6252, 6813, 7371, 7927, 8481, 9032, 9580, 10126, 10668,// 10-19
        11207,  11743, 12275, 12803, 13328, 13848, 14365, 14876, 15384, 15886,// 20-29
        16384,  16877, 17364, 17847, 18324, 18795, 19261, 19720, 20174, 20622,// 30-39
        21063,  21498, 21926, 22348, 22763, 23170, 23571, 23965, 24351, 24730,// 40-49
        25102,  25466, 25822, 26170, 26510, 26842, 27166, 27482, 27789, 28088,// 50-59
        28378,  28660, 28932, 29197, 29452, 29698, 29935, 30163, 30382, 30592,// 60-69
        30792,  30983, 31164, 31336, 31499, 31651, 31795, 31928, 32052, 32166,// 70-79
    32270,  32365, 32440, 32524, 32599, 32643, 32688, 32723, 32748, 32763}; // 80-89

#endif // !__AVR__

// 5x7 font (in 6x8 cell)
const uint8_t ucSmallFont[]PROGMEM = {
    0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x06,0x5f,0x06,0x00,0x00,0x07,0x03,0x00,
    0x07,0x03,0x00,0x24,0x7e,0x24,0x7e,0x24,0x00,0x24,0x2b,0x6a,0x12,0x00,0x00,0x63,
    0x13,0x08,0x64,0x63,0x00,0x36,0x49,0x56,0x20,0x50,0x00,0x00,0x07,0x03,0x00,0x00,
    0x00,0x00,0x3e,0x41,0x00,0x00,0x00,0x00,0x41,0x3e,0x00,0x00,0x00,0x08,0x3e,0x1c,
    0x3e,0x08,0x00,0x08,0x08,0x3e,0x08,0x08,0x00,0x00,0xe0,0x60,0x00,0x00,0x00,0x08,
    0x08,0x08,0x08,0x08,0x00,0x00,0x60,0x60,0x00,0x00,0x00,0x20,0x10,0x08,0x04,0x02,
    0x00,0x3e,0x51,0x49,0x45,0x3e,0x00,0x00,0x42,0x7f,0x40,0x00,0x00,0x62,0x51,0x49,
    0x49,0x46,0x00,0x22,0x49,0x49,0x49,0x36,0x00,0x18,0x14,0x12,0x7f,0x10,0x00,0x2f,
    0x49,0x49,0x49,0x31,0x00,0x3c,0x4a,0x49,0x49,0x30,0x00,0x01,0x71,0x09,0x05,0x03,
    0x00,0x36,0x49,0x49,0x49,0x36,0x00,0x06,0x49,0x49,0x29,0x1e,0x00,0x00,0x6c,0x6c,
    0x00,0x00,0x00,0x00,0xec,0x6c,0x00,0x00,0x00,0x08,0x14,0x22,0x41,0x00,0x00,0x24,
    0x24,0x24,0x24,0x24,0x00,0x00,0x41,0x22,0x14,0x08,0x00,0x02,0x01,0x59,0x09,0x06,
    0x00,0x3e,0x41,0x5d,0x55,0x1e,0x00,0x7e,0x11,0x11,0x11,0x7e,0x00,0x7f,0x49,0x49,
    0x49,0x36,0x00,0x3e,0x41,0x41,0x41,0x22,0x00,0x7f,0x41,0x41,0x41,0x3e,0x00,0x7f,
    0x49,0x49,0x49,0x41,0x00,0x7f,0x09,0x09,0x09,0x01,0x00,0x3e,0x41,0x49,0x49,0x7a,
    0x00,0x7f,0x08,0x08,0x08,0x7f,0x00,0x00,0x41,0x7f,0x41,0x00,0x00,0x30,0x40,0x40,
    0x40,0x3f,0x00,0x7f,0x08,0x14,0x22,0x41,0x00,0x7f,0x40,0x40,0x40,0x40,0x00,0x7f,
    0x02,0x04,0x02,0x7f,0x00,0x7f,0x02,0x04,0x08,0x7f,0x00,0x3e,0x41,0x41,0x41,0x3e,
    0x00,0x7f,0x09,0x09,0x09,0x06,0x00,0x3e,0x41,0x51,0x21,0x5e,0x00,0x7f,0x09,0x09,
    0x19,0x66,0x00,0x26,0x49,0x49,0x49,0x32,0x00,0x01,0x01,0x7f,0x01,0x01,0x00,0x3f,
    0x40,0x40,0x40,0x3f,0x00,0x1f,0x20,0x40,0x20,0x1f,0x00,0x3f,0x40,0x3c,0x40,0x3f,
    0x00,0x63,0x14,0x08,0x14,0x63,0x00,0x07,0x08,0x70,0x08,0x07,0x00,0x71,0x49,0x45,
    0x43,0x00,0x00,0x00,0x7f,0x41,0x41,0x00,0x00,0x02,0x04,0x08,0x10,0x20,0x00,0x00,
    0x41,0x41,0x7f,0x00,0x00,0x04,0x02,0x01,0x02,0x04,0x00,0x80,0x80,0x80,0x80,0x80,
    0x00,0x00,0x03,0x07,0x00,0x00,0x00,0x20,0x54,0x54,0x54,0x78,0x00,0x7f,0x44,0x44,
    0x44,0x38,0x00,0x38,0x44,0x44,0x44,0x28,0x00,0x38,0x44,0x44,0x44,0x7f,0x00,0x38,
    0x54,0x54,0x54,0x08,0x00,0x08,0x7e,0x09,0x09,0x00,0x00,0x18,0xa4,0xa4,0xa4,0x7c,
    0x00,0x7f,0x04,0x04,0x78,0x00,0x00,0x00,0x00,0x7d,0x40,0x00,0x00,0x40,0x80,0x84,
    0x7d,0x00,0x00,0x7f,0x10,0x28,0x44,0x00,0x00,0x00,0x00,0x7f,0x40,0x00,0x00,0x7c,
    0x04,0x18,0x04,0x78,0x00,0x7c,0x04,0x04,0x78,0x00,0x00,0x38,0x44,0x44,0x44,0x38,
    0x00,0xfc,0x44,0x44,0x44,0x38,0x00,0x38,0x44,0x44,0x44,0xfc,0x00,0x44,0x78,0x44,
    0x04,0x08,0x00,0x08,0x54,0x54,0x54,0x20,0x00,0x04,0x3e,0x44,0x24,0x00,0x00,0x3c,
    0x40,0x20,0x7c,0x00,0x00,0x1c,0x20,0x40,0x20,0x1c,0x00,0x3c,0x60,0x30,0x60,0x3c,
    0x00,0x6c,0x10,0x10,0x6c,0x00,0x00,0x9c,0xa0,0x60,0x3c,0x00,0x00,0x64,0x54,0x54,
    0x4c,0x00,0x00,0x08,0x3e,0x41,0x41,0x00,0x00,0x00,0x00,0x77,0x00,0x00,0x00,0x00,
    0x41,0x41,0x3e,0x08,0x00,0x02,0x01,0x02,0x01,0x00,0x00,0x3c,0x26,0x23,0x26,0x3c};

// wrapper/adapter functions to make the code work on Linux
#ifdef __LINUX__
int digitalRead(int iPin)
{
  return gpiod_line_get_value(lines[iPin]);
} /* digitalRead() */

void digitalWrite(int iPin, int iState)
{
   gpiod_line_set_value(lines[iPin], iState);
} /* digitalWrite() */

void pinMode(int iPin, int iMode)
{
   if (chip == NULL) {
       chip = gpiod_chip_open_by_name("gpiochip0");
   }
   lines[iPin] = gpiod_chip_get_line(chip, iPin);
   if (iMode == OUTPUT) {
       gpiod_line_request_output(lines[iPin], CONSUMER, 0);
   } else if (iMode == INPUT_PULLUP) {
       gpiod_line_request_input_flags(lines[iPin], CONSUMER, GPIOD_LINE_REQUEST_FLAG_BIAS_PULL_UP);
   } else { // plain input
       gpiod_line_request_input(lines[iPin], CONSUMER);
   }
} /* pinMode() */

static void delay(int iMS)
{
  usleep(iMS * 1000);
} /* delay() */

static void delayMicroseconds(int iMS)
{
  usleep(iMS);
} /* delayMicroseconds() */

static uint8_t pgm_read_byte(uint8_t *ptr)
{
  return *ptr;
}
#ifdef FUTURE
static int16_t pgm_read_word(uint8_t *ptr)
{
  return ptr[0] + (ptr[1]<<8);
}
#endif // FUTURE
#endif // __LINUX__
//
// Provide a small temporary buffer for use by the graphics functions
//
void spilcdSetTXBuffer(uint8_t *pBuf, int iSize)
{
#if !defined( ARDUINO_ARCH_ESP32 ) && !defined( __LINUX__ )
  //ucTXBuf = pBuf;
  //iTXBufSize = iSize;
#endif
} /* spilcdSetTXBuffer() */

// Sets the D/C pin to data or command mode
void spilcdSetMode(SPILCD *pLCD, int iMode)
{
#ifdef __AVR__
    if (iMode == MODE_DATA)
       *outDC |= bitDC;
    else
       *outDC &= ~bitDC;
#else
	myPinWrite(pLCD->iDCPin, iMode == MODE_DATA);
#endif
#ifdef ARDUINO_ARCH_ESP32
	delayMicroseconds(1); // some systems are so fast that it needs to be delayed
#endif
} /* spilcdSetMode() */

const unsigned char ucST7796InitList[] PROGMEM = {
    1, 0x01, // software reset
    LCD_DELAY, 120,
    1, 0x11, // sleep exit
    LCD_DELAY, 120,
    2, 0xf0,0xc3, // command set control
    2, 0xf0,0x96, // enable extension command 2 part
    2, 0x36,0x48, // x-mirror
    1, 0x20, // invert off
    2, 0x3a,0x55, // pixel format RGB565
    2, 0xb4,0x01, // column inversion
    4, 0xb6,0x80,0x02,0x3b,
    9, 0xe8,0x40,0x8a,0x00,0x00,0x29,0x19,0xa5,0x33,
    2, 0xc1,0x06,
    2, 0xc2,0xa7,
    2, 0xc5,0x18,
    LCD_DELAY, 120,
    15, 0xe0,0xf0,0x09,0x0b,0x06,0x04,0x15,0x2f,0x54,0x42,0x3c,0x17,0x14,0x18,0x1b, // gamma +
    15, 0xe1,0xe0,0x09,0x0b,0x06,0x04,0x03,0x2b,0x43,0x42,0x3b,0x16,0x14,0x17,0x1b, // gamma -
    LCD_DELAY, 120,
    2, 0xf0,0x3c, // disable extension cmd pt 1
    2, 0xf0,0x69, // disable extension cmd pt 2
    LCD_DELAY, 120,
    1, 0x29,  // display on
    0
};

const unsigned char ucJD9613InitList[] PROGMEM = {
    2, 0xfe, 0x01,
    4, 0xf7, 0x96, 0x13, 0xa9,
    2, 0x90, 0x01,
    15, 0x2c, 0x19, 0x0b, 0x24, 0x1b, 0x1b, 0x1b, 0xaa, 0x50, 0x01, 0x16, 0x04, 0x04, 0x04, 0xd7,
    4, 0x2d, 0x66, 0x56, 0x55,
    10, 0x2e, 0x24, 0x04, 0x3f, 0x30, 0x30, 0xa8, 0xb8, 0xb8, 0x07,
    13, 0x33, 0x03, 0x03, 0x03, 0x19, 0x19, 0x19, 0x13, 0x13, 0x13, 0x1a, 0x1a, 0x1a,
    14, 0x10, 0x0b, 0x08, 0x64, 0xae, 0x0b, 0x08, 0x64, 0xae, 0x00, 0x80, 0x00, 0x00, 0x01,
    6, 0x11, 0x01, 0x1e, 0x01, 0x1e, 0x00,
    6, 0x03, 0x93, 0x1c, 0x00, 0x01, 0x7e,
    2, 0x19, 0x00,
    7, 0x31, 0x1b, 0x00, 0x06, 0x05, 0x05, 0x05,
    5, 0x35, 0x00, 0x80, 0x80, 0x00,
    2, 0x12, 0x1b,
    9, 0x1a, 0x01, 0x20, 0x00, 0x08, 0x01, 0x06, 0x06, 0x06,
    8, 0x74, 0xbd, 0x00, 0x01, 0x08, 0x01, 0xbb, 0x98,
    10, 0x6c, 0xdc, 0x08, 0x02, 0x01, 0x08, 0x01, 0x30, 0x08, 0x00,
    10, 0x6d, 0xdc, 0x08, 0x02, 0x01, 0x08, 0x02, 0x30, 0x08, 0x00,
    10, 0x76, 0xda, 0x00, 0x02, 0x20, 0x39, 0x80, 0x80, 0x50, 0x05,
    10, 0x6e, 0xdc, 0x00, 0x02, 0x01, 0x00, 0x02, 0x4f, 0x02, 0x00,
    10, 0x6f, 0xdc, 0x00, 0x02, 0x01, 0x00, 0x01, 0x4f, 0x02, 0x00,
    8, 0x80, 0xbd, 0x00, 0x01, 0x08, 0x01, 0xbb, 0x98,
    10, 0x78, 0xdc, 0x08, 0x02, 0x01, 0x08, 0x01, 0x30, 0x08, 0x00,
    10, 0x79, 0xdc, 0x08, 0x02, 0x01, 0x08, 0x02, 0x30, 0x08, 0x00,
    10, 0x82, 0xda, 0x40, 0x02, 0x20, 0x39, 0x00, 0x80, 0x50, 0x05,
    10, 0x7a, 0xdc, 0x00, 0x02, 0x01, 0x00, 0x02, 0x4f, 0x02, 0x00,
    10, 0x7b, 0xdc, 0x00, 0x02, 0x01, 0x00, 0x01, 0x4f, 0x02, 0x00,
    11, 0x84, 0x01, 0x00, 0x09, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19, 0x19,
    11, 0x85, 0x19, 0x19, 0x19, 0x03, 0x02, 0x08, 0x19, 0x19, 0x19, 0x19,
    13, 0x20, 0x20, 0x00, 0x08, 0x00, 0x02, 0x00, 0x40, 0x00, 0x10, 0x00, 0x04, 0x00,
    13, 0x1e, 0x40, 0x00, 0x10, 0x00, 0x04, 0x00, 0x20, 0x00, 0x08, 0x00, 0x02, 0x00,
    13, 0x24, 0x20, 0x00, 0x08, 0x00, 0x02, 0x00, 0x40, 0x00, 0x10, 0x00, 0x04, 0x00,
    13, 0x22, 0x40, 0x00, 0x10, 0x00, 0x04, 0x00, 0x20, 0x00, 0x08, 0x00, 0x02, 0x00,
    4, 0x13, 0x63, 0x52, 0x41,
    4, 0x14, 0x36, 0x25, 0x14,
    4, 0x15, 0x63, 0x52, 0x41,
    4, 0x16, 0x36, 0x25, 0x14,
    4, 0x1d, 0x10, 0x00, 0x00,
    3, 0x2a, 0x0d, 0x07,
    7, 0x27, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    7, 0x28, 0x00, 0x01, 0x02, 0x03, 0x04, 0x05,
    3, 0x26, 0x01, 0x01,
    3, 0x86, 0x01, 0x01,
    2, 0xfe, 0x02,
    6, 0x16, 0x81, 0x43, 0x23, 0x1e, 0x03,
    2, 0xfe, 0x03,
    2, 0x60, 0x01,
    16, 0x61, 0x00, 0x00, 0x00, 0x00, 0x11, 0x00, 0x0d, 0x26, 0x5a, 0x80, 0x80, 0x95, 0xf8, 0x3b, 0x75,
    16, 0x62, 0x21, 0x22, 0x32, 0x43, 0x44, 0xd7, 0x0a, 0x59, 0xa1, 0xe1, 0x52, 0xb7, 0x11, 0x64, 0xb1,
    12, 0x63, 0x54, 0x55, 0x66, 0x06, 0xfb, 0x3f, 0x81, 0xc6, 0x06, 0x45, 0x83,
    16, 0x64, 0x00, 0x00, 0x11, 0x11, 0x21, 0x00, 0x23, 0x6a, 0xf8, 0x63, 0x67, 0x70, 0xa5, 0xdc, 0x02,
    16, 0x65, 0x22, 0x22, 0x32, 0x43, 0x44, 0x24, 0x44, 0x82, 0xc1, 0xf8, 0x61, 0xbf, 0x13, 0x62, 0xad,
    12, 0x66, 0x54, 0x55, 0x65, 0x06, 0xf5, 0x37, 0x76, 0xb8, 0xf5, 0x31, 0x6c,
    16, 0x67, 0x00, 0x10, 0x22, 0x22, 0x22, 0x00, 0x37, 0xa4, 0x7e, 0x22, 0x25, 0x2c, 0x4c, 0x72, 0x9a,
    16, 0x68, 0x22, 0x33, 0x43, 0x44, 0x55, 0xc1, 0xe5, 0x2d, 0x6f, 0xaf, 0x23, 0x8f, 0xf3, 0x50, 0xa6,
    12, 0x69, 0x65, 0x66, 0x77, 0x07, 0xfd, 0x4e, 0x9c, 0xed, 0x39, 0x86, 0xd3,
    2, 0xfe, 0x05,
    16, 0x61, 0x00, 0x31, 0x44, 0x54, 0x55, 0x00, 0x92, 0xb5, 0x88, 0x19, 0x90, 0xe8, 0x3e, 0x71, 0xa5,
    16, 0x62, 0x55, 0x66, 0x76, 0x77, 0x88, 0xce, 0xf2, 0x32, 0x6e, 0xc4, 0x34, 0x8b, 0xd9, 0x2a, 0x7d,
    12, 0x63, 0x98, 0x99, 0xaa, 0x0a, 0xdc, 0x2e, 0x7d, 0xc3, 0x0d, 0x5b, 0x9e,
    16, 0x64, 0x00, 0x31, 0x44, 0x54, 0x55, 0x00, 0xa2, 0xe5, 0xcd, 0x5c, 0x94, 0xcf, 0x09, 0x4a, 0x72,
    16, 0x65, 0x55, 0x65, 0x66, 0x77, 0x87, 0x9c, 0xc2, 0xff, 0x36, 0x6a, 0xec, 0x45, 0x91, 0xd8, 0x20,
    12, 0x66, 0x88, 0x98, 0x99, 0x0a, 0x68, 0xb0, 0xfb, 0x43, 0x8c, 0xd5, 0x0e,
    16, 0x67, 0x00, 0x42, 0x55, 0x55, 0x55, 0x00, 0xcb, 0x62, 0xc5, 0x09, 0x44, 0x72, 0xa9, 0xd6, 0xfd,
    16, 0x68, 0x66, 0x66, 0x77, 0x87, 0x98, 0x21, 0x45, 0x96, 0xed, 0x29, 0x90, 0xee, 0x4b, 0xb1, 0x13,
    12, 0x69, 0x99, 0xaa, 0xba, 0x0b, 0x6a, 0xb8, 0x0d, 0x62, 0xb8, 0x0e, 0x54,
    2, 0xfe, 0x07,
    2, 0x3e, 0x00,
    3, 0x42, 0x03, 0x10,
    2, 0x4a, 0x31,
    2, 0x5c, 0x01,
    7, 0x3c, 0x07, 0x00, 0x24, 0x04, 0x3f, 0xe2,
    5, 0x44, 0x03, 0x40, 0x3f, 0x02,
    11, 0x12, 0xaa, 0xaa, 0xc0, 0xc8, 0xd0, 0xd8, 0xe0, 0xe8, 0xf0, 0xf8,
    16, 0x11, 0xaa, 0xaa, 0xaa, 0x60, 0x68, 0x70, 0x78, 0x80, 0x88, 0x90, 0x98, 0xa0, 0xa8, 0xb0, 0xb8,
    16, 0x10, 0xaa, 0xaa, 0xaa, 0x00, 0x08, 0x10, 0x18, 0x20, 0x28, 0x30, 0x38, 0x40, 0x48, 0x50, 0x58,
    17, 0x14, 0x03, 0x1f, 0x3f, 0x5f, 0x7f, 0x9f, 0xbf, 0xdf, 0x03, 0x1f, 0x3f, 0x5f, 0x7f, 0x9f, 0xbf, 0xdf,
    13, 0x18, 0x70, 0x1a, 0x22, 0xbb, 0xaa, 0xff, 0x24, 0x71, 0x0f, 0x01, 0x00, 0x03,
    2, 0xfe, 0x00,
    2, 0x3a, 0x55,
    2, 0xc4, 0x80,
    5, 0x2a, 0x00, 0x00, 0x00, 0x7d,
    5, 0x2b, 0x00, 0x00, 0x01, 0x25,
    2, 0x35, 0x00,
    2, 0x53, 0x28,
    2, 0x51, 0xff,
    2, 0x11, 0x00,
    LCD_DELAY, 120,
    2, 0x29, 0x00,
    LCD_DELAY, 120,
}; // JD9613

const unsigned char ucGC9D01InitList[] PROGMEM = {
    1, 0xFE,// Inter Register Enable1 (FEh)
    1, 0xEF,// Inter Register Enable2 (EFh)
    2,0x80,0xff,
    2,0x81,0xFF,
    2,0x82,0xFF,
    2,0x83,0xFF,
    2,0x84,0xFF,
    2,0x85,0xFF,
    2,0x86,0xFF,
    2,0x87,0xFF,
    2,0x88,0xFF,
    2,0x89,0xFF,
    2,0x8A,0xFF,
    2,0x8B,0xFF,
    2,0x8C,0xFF,
    2,0x8D,0xFF,
    2,0x8E,0xFF,
    2,0x8F,0xFF,
    2,0x3A,0x05, // COLMOD: Pixel Format Set (3Ah) MCU interface, 16 bits / pixel
    2,0xEC,0x10, // Inversion (ECh) DINV=1+2 column for Single Gate (BFh=0)
    2,0x7E,0x7A,
    8,0x74,0x02,0x0E,0x00,0x00,0x28,0x00,0x00,
    2,0x98,0x3E,
    2,0x99,0x3E,
    3,0xB5,0x0E,0x0E, // Blanking Porch Control (B5h) VFP=14 VBP=14 HBP=Off
    5,0x60,0x38,0x09,0x6D,0x67,
    6,0x63,0x38,0xAD,0x6D,0x67,0x05,
    7,0x64,0x38,0x0B,0x70,0xAB,0x6D,0x67,
    7,0x66,0x38,0x0F,0x70,0xAF,0x6D,0x67,
    3,0x6A,0x00,0x00,
    8,0x68,0x3B,0x08,0x04,0x00,0x04,0x64,0x67,
    8,0x6C,0x22,0x02,0x22,0x02,0x22,0x22,0x50,
   33,0x6E,0x00,0x00,0x00,0x00,0x07,0x01,0x13,0x11,0x0B,0x09,0x16,0x15,0x1D,0x1E,0x00,0x00,0x00,0x00,0x1E,0x1D,0x15,0x16,0x0A,0x0C,0x12,0x14,0x02,0x08,0x00,0x00,0x00,0x00,
    2,0xA9,0x1B,
    2,0xA8,0x6B,
    2,0xA8,0x6D,
    2,0xA7,0x40,
    2,0xAD,0x47,
    2,0xAF,0x73,
    2,0xAF,0x73,
    2,0xAC,0x44,
    2,0xA3,0x6C,
    2,0xCB,0x00,
    2,0xCD,0x22,
    2,0xC2,0x10,
    2,0xC5,0x00,
    2,0xC6,0x0E,
    2,0xC7,0x1F,
    2,0xC8,0x0E,
    2,0xBF,0x00,  // Dual-Single Gate Select (BFh) 0=>Single gate
    2,0xF9,0x20,
    2,0x9B,0x3B,
    4,0x93,0x33,0x7F,0x00,
    7,0x70,0x0E,0x0F,0x03,0x0E,0x0F,0x03,
    4,0x71,0x0E,0x16,0x03,
    3,0x91,0x0E,0x09,
    2,0xC3,0x2C,  // Vreg1a Voltage Control 2 (C3h) vreg1_vbp_d=0x2C
    2,0xC4,0x1A,  // Vreg1b Voltage Control 2 (C4h) vreg1_vbn_d=0x1A
    7,0xF0,0x51,0x13,0x0C,0x06,0x00,0x2F,  // SET_GAMMA1 (F0h)
    7,0xF2,0x51,0x13,0x0C,0x06,0x00,0x33,  // SET_GAMMA3 (F2h)
    7,0xF1,0x3C,0x94,0x4F,0x33,0x34,0xCF,  // SET_GAMMA2 (F1h)
    7,0xF3,0x4D,0x94,0x4F,0x33,0x34,0xCF,  // SET_GAMMA4 (F3h)
    2,0x36,0x00,  // Memory Access Control (36h) MY=0, MX=0, MV=0, ML=0, BGR=0, MH=0
    1,0x11,  // Sleep Out Mode (11h) and delay(200)
    LCD_DELAY, 200,
    1,0x29,  // Display ON (29h) and delay(20)
    LCD_DELAY, 20,
    1,0x2C,  // Memory Write (2Ch) D=0
    0
}; // GC9D01 160x160 round

const unsigned char ucGC9A01InitList[]PROGMEM = {
    1, 0xEF,
    2, 0xEB, 0x14,
    1, 0xFE,
    1, 0xEF,
    2, 0xEB, 0x14,
    2, 0x84, 0x40,
    2, 0x85, 0xff,
    2, 0x86, 0xff,
    2, 0x87, 0xff,
    2, 0x88, 0x0a,
    2, 0x89, 0x21,
    2, 0x8a, 0x00,
    2, 0x8b, 0x80,
    2, 0x8c, 0x01,
    2, 0x8d, 0x01,
    2, 0x8e, 0xff,
    2, 0x8f, 0xff,
    3, 0xb6, 0x00, 0x00,
    2, 0x3a, 0x55,
    5, 0x90, 0x08,0x08,0x08,0x08,
    2, 0xbd, 0x06,
    2, 0xbc, 0x00,
    4, 0xff, 0x60,0x01,0x04,
    2, 0xc3, 0x13,
    2, 0xc4, 0x13,
    2, 0xc9, 0x22,
    2, 0xbe, 0x11,
    3, 0xe1, 0x10,0x0e,
    4, 0xdf, 0x21,0x0c,0x02,
    7, 0xf0, 0x45,0x09,0x08,0x08,0x26,0x2a,
    7, 0xf1, 0x43,0x70,0x72,0x36,0x37,0x6f,
    7, 0xf2, 0x45,0x09,0x08,0x08,0x26,0x2a,
    7, 0xf3, 0x43,0x70,0x72,0x36,0x37,0x6f,
    3, 0xed, 0x1b,0x0b,
    2, 0xae, 0x77,
    2, 0xcd, 0x63,
    10,0x70, 0x07,0x07,0x04,0x0e,0x0f,0x09,0x07,0x08,0x03,
    2, 0xe8, 0x34,
    13,0x62, 0x18,0x0d,0x71,0xed,0x70,0x70,0x18,0x0f,0x71,0xef,0x70,0x70,
    13,0x63, 0x18,0x11,0x71,0xf1,0x70,0x70,0x18,0x13,0x71,0xf3,0x70,0x70,
    8, 0x64, 0x28,0x29,0xf1,0x01,0xf1,0x00,0x07,
    11,0x66, 0x3c,0x00,0xcd,0x67,0x45,0x45,0x10,0x00,0x00,0x00,
    11,0x67, 0x00,0x3c,0x00,0x00,0x00,0x01,0x54,0x10,0x32,0x98,
    8, 0x74, 0x10,0x85,0x80,0x00,0x00,0x4e,0x00,
    3, 0x98, 0x3e,0x07,
    2, 0x36, 0x48,
    1, 0x35,
    1, 0x21,
    1, 0x11,
    LCD_DELAY, 120,
    1, 0x29,
    LCD_DELAY, 120,
    0
}; // GC9A01

const unsigned char ucGC9107InitList[]PROGMEM = {
    1, 0xEF,
    2, 0xEB, 0x14,
    1, 0xFE,
    1, 0xEF,
    2, 0xEB, 0x14,
    2, 0x84, 0x40,
    2, 0x85, 0xff,
    2, 0x86, 0xff,
    2, 0x87, 0xff,
    2, 0x88, 0x0a,
    2, 0x89, 0x21,
    2, 0x8a, 0x00,
    2, 0x8b, 0x80,
    2, 0x8c, 0x01,
    2, 0x8d, 0x01,
    2, 0x8e, 0xff,
    2, 0x8f, 0xff,
    3, 0xb6, 0x00, 0x00,
    2, 0x3a, 0x55,
    5, 0x90, 0x08,0x08,0x08,0x08,
    2, 0xbd, 0x06,
    2, 0xbc, 0x00,
    4, 0xff, 0x60,0x01,0x04,
    2, 0xc3, 0x13,
    2, 0xc4, 0x13,
    2, 0xc9, 0x22,
    2, 0xbe, 0x11,
    3, 0xe1, 0x10,0x0e,
    4, 0xdf, 0x21,0x0c,0x02,
    7, 0xf0, 0x45,0x09,0x08,0x08,0x26,0x2a,
    7, 0xf1, 0x43,0x70,0x72,0x36,0x37,0x6f,
    7, 0xf2, 0x45,0x09,0x08,0x08,0x26,0x2a,
    7, 0xf3, 0x43,0x70,0x72,0x36,0x37,0x6f,
    3, 0xed, 0x1b,0x0b,
    2, 0xae, 0x77,
    2, 0xcd, 0x63,
    10,0x70, 0x07,0x07,0x04,0x0e,0x0f,0x09,0x07,0x08,0x03,
    2, 0xe8, 0x34,
    13,0x62, 0x18,0x0d,0x71,0xed,0x70,0x70,0x18,0x0f,0x71,0xef,0x70,0x70,
    13,0x63, 0x18,0x11,0x71,0xf1,0x70,0x70,0x18,0x13,0x71,0xf3,0x70,0x70,
    8, 0x64, 0x28,0x29,0xf1,0x01,0xf1,0x00,0x07,
    11,0x66, 0x3c,0x00,0xcd,0x67,0x45,0x45,0x10,0x00,0x00,0x00,
    11,0x67, 0x00,0x3c,0x00,0x00,0x00,0x01,0x54,0x10,0x32,0x98,
    8, 0x74, 0x10,0x85,0x80,0x00,0x00,0x4e,0x00,
    3, 0x98, 0x3e,0x07,
    2, 0x36, 0xC8,
    1, 0x35,
    1, 0x21,
    1, 0x11,
    LCD_DELAY, 120,
    1, 0x29,
    LCD_DELAY, 120,
    0
}; // GC9107

const uint16_t u16ST7793_Init[]PROGMEM = {
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            0x0000, 0x0000,
            LCD_DELAY, 15,
            0x0400, 0x6200,     //NL=0x31 (49) i.e. 400 rows
            0x0008, 0x0808,
            //gamma
            0x0300, 0x0C00,
            0x0301, 0x5A0B,
            0x0302, 0x0906,
            0x0303, 0x1017,
            0x0304, 0x2300,
            0x0305, 0x1700,
            0x0306, 0x6309,
            0x0307, 0x0C09,
            0x0308, 0x100C,
            0x0309, 0x2232,

            0x0010, 0x0016,     //69.5Hz         0016
            0x0011, 0x0101,
            0x0012, 0x0000,
            0x0013, 0x0001,
            0x0100, 0x0330,     //BT,AP
            0x0101, 0x0237,     //DC0,DC1,VC
            0x0103, 0x0D00,     //VDV
            0x0280, 0x6100,     //VCM
            0x0102, 0xC1B0,     //VRH,VCMR,PSON,PON
            LCD_DELAY, 50,

            0x0001, 0x0100,
            0x0002, 0x0100,
            0x0003, 0x1030,     //1030
            0x0009, 0x0001,
            0x000C, 0x0000,
            0x0090, 0x8000,
            0x000F, 0x0000,

            0x0210, 0x0000,
            0x0211, 0x00EF,
            0x0212, 0x0000,
            0x0213, 0x018F,     //432=01AF,400=018F
            0x0500, 0x0000,
            0x0501, 0x0000,
            0x0502, 0x005F,     //???
            0x0401, 0x0001,     //REV=1
            0x0404, 0x0000,
            LCD_DELAY, 50,

            0x0007, 0x0100,     //BASEE
            LCD_DELAY, 50,

            0x0200, 0x0000,
            0x0201, 0x0000,
            0 // end
}; // ST7793 240x400

// List of command/parameters to initialize the ST7789 LCD
const unsigned char uc240x240InitList[]PROGMEM = {
    1, 0x13, // partial mode off
    1, 0x21, // display inversion off
    2, 0x36,0x08,    // memory access 0xc0 for 180 degree flipped
    2, 0x3a,0x55,    // pixel format; 5=RGB565
    3, 0x37,0x00,0x00, //
    6, 0xb2,0x0c,0x0c,0x00,0x33,0x33, // Porch control
    2, 0xb7,0x35,    // gate control
    2, 0xbb,0x1a,    // VCOM
    2, 0xc0,0x2c,    // LCM
    2, 0xc2,0x01,    // VDV & VRH command enable
    2, 0xc3,0x0b,    // VRH set
    2, 0xc4,0x20,    // VDV set
    2, 0xc6,0x0f,    // FR control 2
    3, 0xd0, 0xa4, 0xa1,     // Power control 1
    15, 0xe0, 0x00,0x19,0x1e,0x0a,0x09,0x15,0x3d,0x44,0x51,0x12,0x03,
        0x00,0x3f,0x3f,     // gamma 1
    15, 0xe1, 0x00,0x18,0x1e,0x0a,0x09,0x25,0x3f,0x43,0x52,0x33,0x03,
        0x00,0x3f,0x3f,        // gamma 2
    1, 0x29,    // display on
    0
};
// List of command/parameters to initialize the ili9341 display
const unsigned char uc240InitList[]PROGMEM = {
        4, 0xEF, 0x03, 0x80, 0x02,
        4, 0xCF, 0x00, 0XC1, 0X30,
        5, 0xED, 0x64, 0x03, 0X12, 0X81,
        4, 0xE8, 0x85, 0x00, 0x78,
        6, 0xCB, 0x39, 0x2C, 0x00, 0x34, 0x02,
        2, 0xF7, 0x20,
        3, 0xEA, 0x00, 0x00,
        2, 0xc0, 0x23, // Power control
        2, 0xc1, 0x10, // Power control
        3, 0xc5, 0x3e, 0x28, // VCM control
        2, 0xc7, 0x86, // VCM control2
        2, 0x36, 0x48, // Memory Access Control
        1, 0x20,        // non inverted
        2, 0x3a, 0x55,
        3, 0xb1, 0x00, 0x18,
        4, 0xb6, 0x08, 0x82, 0x27, // Display Function Control
        2, 0xF2, 0x00, // Gamma Function Disable
        2, 0x26, 0x01, // Gamma curve selected
        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
        0
};
// Init sequence for the SSD1331 OLED display
// Courtesy of Adafruit's SSD1331 library
const unsigned char ucSSD1331InitList[] PROGMEM = {
    1, 0xae,    // display off
    1, 0xa0,    // set remap
    1, 0x72,    // RGB 0x76 == BGR
    2, 0xa1, 0x00,  // set start line
    2, 0xa2, 0x00,  // set display offset
    1, 0xa4,    // normal display
    2, 0xa8, 0x3f,  // set multiplex 1/64 duty
    2, 0xad, 0x8e, // set master configuration
    2, 0xb0, 0x0b, // disable power save
    2, 0xb1, 0x31, // phase period adjustment
    2, 0xb3, 0xf0, // clock divider
    2, 0x8a, 0x64, // precharge a
    2, 0x8b, 0x78, // precharge b
    2, 0x8c, 0x64, // precharge c
    2, 0xbb, 0x3a, // set precharge level
    2, 0xbe, 0x3e, // set vcomh
    2, 0x87, 0x06, // master current control
    2, 0x81, 0x91, // contrast for color a
    2, 0x82, 0x50, // contrast for color b
    2, 0x83, 0x7D, // contrast for color c
    1, 0xAF, // display on, normal
    0,0
};
// List of command/parameters to initialize the SSD1351 OLED display
const unsigned char ucOLEDInitList[] PROGMEM = {
	2, 0xfd, 0x12, // unlock the controller
	2, 0xfd, 0xb1, // unlock the command
	1, 0xae,	// display off
	2, 0xb3, 0xf1,  // clock divider
	2, 0xca, 0x7f,	// mux ratio
	2, 0xa0, 0x74,	// set remap
	3, 0x15, 0x00, 0x7f,	// set column
	3, 0x75, 0x00, 0x7f,	// set row
	2, 0xb5, 0x00,	// set GPIO state
	2, 0xab, 0x01,	// function select (internal diode drop)
	2, 0xb1, 0x32,	// precharge
	2, 0xbe, 0x05,	// vcomh
	1, 0xa6,	// set normal display mode
	4, 0xc1, 0xc8, 0x80, 0xc8, // contrast ABC
	2, 0xc7, 0x0f,	// contrast master
	4, 0xb4, 0xa0,0xb5,0x55,	// set VSL
	2, 0xb6, 0x01,	// precharge 2
	1, 0xaf,	// display ON
	0};
// List of command/parameters for the SSD1286 LCD
const unsigned char uc132x176InitList[] PROGMEM = {
   3, 0x00, 0x00, 0x01, // osc start
   3, 0x10, 0x1f, 0x92, // pwr ctrl1
   3, 0x11, 0x00, 0x14, // pwr ctrl2
   3, 0x28, 0x00, 0x06, // test enable
   3, 0x00, 0x00, 0x01, // osc start
   3, 0x10, 0x1f, 0x92, // pwr ctrl1
   3, 0x11, 0x00, 0x14, // pwr ctrl2
   3, 0x02, 0x00, 0x00, // LCD drive AC
   3, 0x12, 0x04, 0x0b, // pwr ctrl3
   3, 0x03, 0x68, 0x30, // entry mode + RGB565 pixels
   3, 0x01, 0x31, 0xaf, // driver output ctrl
   3, 0x07, 0x00, 0x33, // display ctrl
   3, 0x42, 0xaf, 0x00, // 1st screen driving
   3, 0x21, 0x00, 0x00, // RAM Address set
   3, 0x44, 0x83, 0x00, // horizontal RAM addr (132-1)
   3, 0x45, 0xaf, 0x00, // vertical RAM addr (176-1)
   3, 0x2c, 0x30, 0x00, // osc freq
   3, 0x29, 0x00, 0x00, // analog enable
//   3, 0x2d, 0x31, 0x0f, // analog tune
   3, 0x13, 0x30, 0x00, // pwr ctrl 4
   1, 0x22 // RAM Data write
};

// List of command/parameters for the SSD1283A display
const unsigned char uc132InitList[]PROGMEM = {
    3, 0x10, 0x2F,0x8E,
    3, 0x11, 0x00,0x0C,
    3, 0x07, 0x00,0x21,
    3, 0x28, 0x00,0x06,
    3, 0x28, 0x00,0x05,
    3, 0x27, 0x05,0x7F,
    3, 0x29, 0x89,0xA1,
    3, 0x00, 0x00,0x01,
    LCD_DELAY, 100,
    3, 0x29, 0x80,0xB0,
    LCD_DELAY, 30,
    3, 0x29, 0xFF,0xFE,
    3, 0x07, 0x02,0x23,
    LCD_DELAY, 30,
    3, 0x07, 0x02,0x33,
    3, 0x01, 0x21,0x83,
    3, 0x03, 0x68,0x30,
    3, 0x2F, 0xFF,0xFF,
    3, 0x2C, 0x80,0x00,
    3, 0x27, 0x05,0x70,
    3, 0x02, 0x03,0x00,
    3, 0x0B, 0x58,0x0C,
    3, 0x12, 0x06,0x09,
    3, 0x13, 0x31,0x00,
    0
};

const unsigned char uc320InitList[]PROGMEM = {
        2, 0xc0, 0x23, // Power control
        2, 0xc1, 0x10, // Power control
        3, 0xc5, 0x3e, 0x28, // VCM control
        2, 0xc7, 0x86, // VCM control2
        2, 0x36, 0x08, // Memory Access Control (flip x/y/bgr/rgb)
        2, 0x3a, 0x55,
	1, 0x21,	// inverted display off
//        2, 0x26, 0x01, // Gamma curve selected
//        16, 0xe0, 0x0F, 0x31, 0x2B, 0x0C, 0x0E, 0x08,
//                0x4E, 0xF1, 0x37, 0x07, 0x10, 0x03, 0x0E, 0x09, 0x00, // Set Gamma
//        16, 0xe1, 0x00, 0x0E, 0x14, 0x03, 0x11, 0x07,
//                0x31, 0xC1, 0x48, 0x08, 0x0F, 0x0C, 0x31, 0x36, 0x0F, // Set Gamma
//        3, 0xb1, 0x00, 0x10, // FrameRate Control 119Hz
        0
};
// List of command/parameters to initialize the st7735s display
const unsigned char uc80InitList[]PROGMEM = {
//        4, 0xb1, 0x01, 0x2c, 0x2d,    // frame rate control
//        4, 0xb2, 0x01, 0x2c, 0x2d,    // frame rate control (idle mode)
//        7, 0xb3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d, // frctrl - partial mode
//        2, 0xb4, 0x07,    // non-inverted
//        4, 0xc0, 0xa2, 0x02, 0x84,    // power control
//        2, 0xc1, 0xc5,     // pwr ctrl2
//        2, 0xc2, 0x0a, 0x00, // pwr ctrl3
//        3, 0xc3, 0x8a, 0x2a, // pwr ctrl4
//        3, 0xc4, 0x8a, 0xee, // pwr ctrl5
//        2, 0xc5, 0x0e,        // vm ctrl1
    2, 0x3a, 0x05,    // pixel format RGB565
    2, 0x36, 0xc0, // MADCTL
    5, 0x2a, 0x00, 0x02, 0x00, 0x7f+0x02, // column address start
    5, 0x2b, 0x00, 0x01, 0x00, 0x9f+0x01, // row address start
    17, 0xe0, 0x09, 0x16, 0x09,0x20,
    0x21,0x1b,0x13,0x19,
    0x17,0x15,0x1e,0x2b,
    0x04,0x05,0x02,0x0e, // gamma sequence
    17, 0xe1, 0x0b,0x14,0x08,0x1e,
    0x22,0x1d,0x18,0x1e,
    0x1b,0x1a,0x24,0x2b,
    0x06,0x06,0x02,0x0f,
    1, 0x21,    // display inversion on
    0
};
// List of command/parameters to initialize the ILI9225 176x220 LCD
const unsigned char ucILI9225InitList[] PROGMEM = {
    //************* Start Initial Sequence **********//
    // These are 16-bit commands and data
    // Set SS bit and direction output from S528 to S1
    2, 0x00, 0x10, 0x00, 0x00, // Set SAP, DSTB, STB
    2, 0x00, 0x11, 0x00, 0x00, // Set APON,PON,AON,VCI1EN,VC
    2, 0x00, 0x12, 0x00, 0x00, // Set BT,DC1,DC2,DC3
    2, 0x00, 0x13, 0x00, 0x00, // Set GVDD
    2, 0x00, 0x14, 0x00, 0x00, // Set VCOMH/VCOML voltage
    LCD_DELAY, 40,
    // Power on sequence
    2, 0x00, 0x11, 0x00, 0x18, // Set APON,PON,AON,VCI1EN,VC
    2, 0x00, 0x12, 0x61, 0x21, // Set BT,DC1,DC2,DC3
    2, 0x00, 0x13, 0x00, 0x6f, // Set GVDD
    2, 0x00, 0x14, 0x49, 0x5f, // Set VCOMH/VCOML voltage
    2, 0x00, 0x10, 0x08, 0x00, // Set SAP, DSTB, STB
    LCD_DELAY, 10,
    2, 0x00, 0x11, 0x10, 0x3B, // Set APON,PON,AON,VCI1EN,VC
    LCD_DELAY, 50,
    2, 0x00, 0x01, 0x01, 0x1C, // set display line number and direction
    2, 0x00, 0x02, 0x01, 0x00, // set 1 line inversion
    2, 0x00, 0x03, 0x10, 0x30, // set GRAM write direction and BGR=1
    2, 0x00, 0x07, 0x00, 0x00, // display off
    2, 0x00, 0x08, 0x08, 0x08,  // set back porch and front porch
    2, 0x00, 0x0b, 0x11, 0x00, // set the clocks number per line
    2, 0x00, 0x0C, 0x00, 0x00, // interface control - CPU
    2, 0x00, 0x0F, 0x0d, 0x01, // Set frame rate (OSC control)
    2, 0x00, 0x15, 0x00, 0x20, // VCI recycling
    2, 0x00, 0x20, 0x00, 0x00, // Set GRAM horizontal address
    2, 0x00, 0x21, 0x00, 0x00, // Set GRAM vertical address
    //------------------------ Set GRAM area --------------------------------//
    2, 0x00, 0x30, 0x00, 0x00,
    2, 0x00, 0x31, 0x00, 0xDB,
    2, 0x00, 0x32, 0x00, 0x00,
    2, 0x00, 0x33, 0x00, 0x00,
    2, 0x00, 0x34, 0x00, 0xDB,
    2, 0x00, 0x35, 0x00, 0x00,
    2, 0x00, 0x36, 0x00, 0xAF,
    2, 0x00, 0x37, 0x00, 0x00,
    2, 0x00, 0x38, 0x00, 0xDB,
    2, 0x00, 0x39, 0x00, 0x00,
// set GAMMA curve
    2, 0x00, 0x50, 0x00, 0x00,
    2, 0x00, 0x51, 0x08, 0x08,
    2, 0x00, 0x52, 0x08, 0x0A,
    2, 0x00, 0x53, 0x00, 0x0A,
    2, 0x00, 0x54, 0x0A, 0x08,
    2, 0x00, 0x55, 0x08, 0x08,
    2, 0x00, 0x56, 0x00, 0x00,
    2, 0x00, 0x57, 0x0A, 0x00,
    2, 0x00, 0x58, 0x07, 0x10,
    2, 0x00, 0x59, 0x07, 0x10,
    2, 0x00, 0x07, 0x00, 0x12, // display ctrl
    LCD_DELAY, 50,
    2, 0x00, 0x07, 0x10, 0x17, // display on
    0
};
// List of command/parameters to initialize the st7735r display
const unsigned char uc128InitList[]PROGMEM = {
//	4, 0xb1, 0x01, 0x2c, 0x2d,	// frame rate control
//	4, 0xb2, 0x01, 0x2c, 0x2d,	// frame rate control (idle mode)
//	7, 0xb3, 0x01, 0x2c, 0x2d, 0x01, 0x2c, 0x2d, // frctrl - partial mode
//	2, 0xb4, 0x07,	// non-inverted
//	4, 0xc0, 0x82, 0x02, 0x84,	// power control
//	2, 0xc1, 0xc5, 	// pwr ctrl2
//	2, 0xc2, 0x0a, 0x00, // pwr ctrl3
//	3, 0xc3, 0x8a, 0x2a, // pwr ctrl4
//	3, 0xc4, 0x8a, 0xee, // pwr ctrl5
//	2, 0xc5, 0x0e,		// pwr ctrl
//	1, 0x20,	// display inversion off
	2, 0x3a, 0x55,	// pixel format RGB565
	2, 0x36, 0xc0, // MADCTL
	17, 0xe0, 0x09, 0x16, 0x09,0x20,
		0x21,0x1b,0x13,0x19,
		0x17,0x15,0x1e,0x2b,
		0x04,0x05,0x02,0x0e, // gamma sequence
	17, 0xe1, 0x0b,0x14,0x08,0x1e,
		0x22,0x1d,0x18,0x1e,
		0x1b,0x1a,0x24,0x2b,
		0x06,0x06,0x02,0x0f,
	0
};
// List of command/parameters to initialize the ILI9488 display
const unsigned char ucILI9488InitList[] PROGMEM = {
   3, 0xc0, 0x17, 0x15, // CMD_PWCTR1 - VRH1, VRH2
   2, 0xc1, 0x41, // CMD_PWCTR2 - VGH, VGL
//   4, 0xc5, 0x00, 0x12, 0x80, // CMD_VMCTR
   3, 0xb1, 0xa0, 0x11, // frame rate 60hz
//   2, 0xb4, 0x02, // frame inversion control = 2dot
   4, 0xb6, 0x02, 0x22, 0x3b, // normal scan, 5 frames
//   2, 0xb7, 0xc6, // ETMOD
   2, 0x36,0x0a, // MEM_CTL
   1, 0x20, // not inverted
   2, 0x3a, 0x55, // pixel format RGB565
   5, 0xf7, 0xa9, 0x51, 0x2c, 0x82, // ADJCTL3
   1, 0x11, // sleep mode out
   LCD_DELAY, 120,
   1, 0x29, // display on
   LCD_DELAY, 100,
   0
};
// List of command/parameters to initialize the ILI9486 display
const unsigned char ucILI9486InitList[] PROGMEM = {
   2, 0x01, 0x00,
   LCD_DELAY, 50,
   2, 0x28, 0x00,
   3, 0xc0, 0xd, 0xd,
   3, 0xc1, 0x43, 0x00,
   2, 0xc2, 0x00,
   3, 0xc5, 0x00, 0x48,
   4, 0xb6, 0x00, 0x22, 0x3b,
   16, 0xe0,0x0f,0x24,0x1c,0x0a,0x0f,0x08,0x43,0x88,0x32,0x0f,0x10,0x06,0x0f,0x07,0x00,
   16, 0xe1,0x0f,0x38,0x30,0x09,0x0f,0x0f,0x4e,0x77,0x3c,0x07,0x10,0x05,0x23,0x1b,0x00,
   1, 0x20,
   2, 0x36,0x0a,
   2, 0x3a,0x55,
   1, 0x11,
   LCD_DELAY, 150,
   1, 0x29,
   LCD_DELAY, 25,
   0
};
// List of command/parameters to initialize the hx8357 display
const unsigned char uc480InitList[]PROGMEM = {
	2, 0x3a, 0x55,
	2, 0xc2, 0x44,
	5, 0xc5, 0x00, 0x00, 0x00, 0x00,
	16, 0xe0, 0x0f, 0x1f, 0x1c, 0x0c, 0x0f, 0x08, 0x48, 0x98, 0x37,
		0x0a,0x13, 0x04, 0x11, 0x0d, 0x00,
	16, 0xe1, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
		0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
	16, 0xe2, 0x0f, 0x32, 0x2e, 0x0b, 0x0d, 0x05, 0x47, 0x75, 0x37,
		0x06, 0x10, 0x03, 0x24, 0x20, 0x00,
	2, 0x36, 0x48,
	0	
};

//
// 16-bit memset
//
void memset16(uint16_t *pDest, uint16_t usPattern, int iCount)
{
    while (iCount--)
        *pDest++ = usPattern;
}

//
// Returns true if DMA is currently busy
//
int spilcdIsDMABusy(void)
{
#ifdef HAS_DMA
    return !transfer_is_done;
#endif
    return 0;
} /* spilcdIsDMABusy() */
//
// Send the data by bit-banging the GPIO ports
//
void SPI_BitBang(SPILCD *pLCD, uint8_t *pData, int iLen, int iMode)
{
    int iMOSI, iCLK; // local vars to speed access
    uint8_t c, j;
    
    iMOSI = pLCD->iMOSIPin;
    iCLK = pLCD->iCLKPin;
    myPinWrite(pLCD->iCSPin, 0);
    if (iMode == MODE_COMMAND)
        myPinWrite(pLCD->iDCPin, 0);
    while (iLen)
    {
        c = *pData++;
        if (pLCD->iLCDFlags & FLAGS_CS_EACHBYTE)
           myPinWrite(pLCD->iCSPin, 0);
        if (c == 0 || c == 0xff) // quicker for all bits equal
        {
            digitalWrite(iMOSI, c);
            for (j=0; j<8; j++)
            {
                myPinWrite(iCLK, HIGH);
                delayMicroseconds(0);
                myPinWrite(iCLK, LOW);
            }
        }
        else
        {
            for (j=0; j<8; j++)
            {
                myPinWrite(iMOSI, c & 0x80);
                myPinWrite(iCLK, HIGH);
                c <<= 1;
                delayMicroseconds(0);
                myPinWrite(iCLK, LOW);
            }
        }
        if (pLCD->iLCDFlags & FLAGS_CS_EACHBYTE)
           myPinWrite(pLCD->iCSPin, 1);
        iLen--;
    }
    myPinWrite(pLCD->iCSPin, 1);
    if (iMode == MODE_COMMAND) // restore it to MODE_DATA before leaving
        myPinWrite(pLCD->iDCPin, 1);
} /* SPI_BitBang() */
//
// Wrapper function for writing to SPI
//
static void myspiWrite(SPILCD *pLCD, unsigned char *pBuf, int iLen, int iMode, int iFlags)
{
    if (iLen == 0) return;
 //   Serial.printf("myspiWrite len=%d, flags=%d\n", iLen, iFlags);
// swap DMA buffers
#ifndef __LINUX__
    if (pDMA == pDMA0) {
        pDMA = pDMA1;
    } else {
        pDMA = pDMA0;
    }
#endif
    if ((iFlags & DRAW_WITH_DMA) && !pLCD->bUseDMA) {
        iFlags &= ~DRAW_WITH_DMA; // DMA is not being used, so remove the flag
    }
    if (pLCD->iLCDType == LCD_VIRTUAL_MEM) iFlags = DRAW_TO_RAM;

    if (iMode == MODE_DATA && pLCD->pBackBuffer != NULL && !bSetPosition && iFlags & DRAW_TO_RAM) // write it to the back buffer
    {
        uint16_t *s, *d;
        int j, iOff, iStrip, iMaxX, iMaxY, i;
        iMaxX = pLCD->iWindowX + pLCD->iWindowCX;
        iMaxY = pLCD->iWindowY + pLCD->iWindowCY;
        iOff = 0;
        i = iLen/2;
        while (i > 0)
        {
            iStrip = iMaxX - pLCD->iCurrentX; // max pixels that can be written in one shot
            if (iStrip > i)
                iStrip = i;
            s = (uint16_t *)&pBuf[iOff];
            d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset];
            if (pLCD->iOffset > pLCD->iMaxOffset || (pLCD->iOffset+iStrip*2) > pLCD->iMaxOffset)
            { // going to write past the end of memory, don't even try
                i = iStrip = 0;
            }
            for (j=0; j<iStrip; j++) // memcpy could be much slower for small runs
            {
                *d++ = *s++;
            }
#if defined ARDUINO_ESP32S3_DEV
            Cache_WriteBack_Addr((uint32_t)&pLCD->pBackBuffer[pLCD->iOffset], iStrip*2);
#endif
#if defined ARDUINO_ESP32P4_DEV
            esp_cache_msync(&pLCD->pBackBuffer[pLCD->iOffset], iStrip*2, ESP_CACHE_MSYNC_FLAG_DIR_C2M | ESP_CACHE_MSYNC_FLAG_UNALIGNED);
#endif
            pLCD->iOffset += iStrip*2; iOff += iStrip*2;
            i -= iStrip;
            pLCD->iCurrentX += iStrip;
            if (pLCD->iCurrentX >= iMaxX) // need to wrap around to the next line
            {
                pLCD->iCurrentX = pLCD->iWindowX;
                pLCD->iCurrentY++;
                if (pLCD->iCurrentY >= iMaxY)
                    pLCD->iCurrentY = pLCD->iWindowY;
                pLCD->iOffset = (pLCD->iCurrentWidth * 2 * pLCD->iCurrentY) + (pLCD->iCurrentX * 2);
            }
        }
    }
    if (!(iFlags & DRAW_TO_LCD) || pLCD->iLCDType == LCD_VIRTUAL_MEM)
        return; // don't write it to spi
#if defined( ARDUINO_ARCH_ESP32 )// && !defined ( CONFIG_IDF_TARGET_ESP32 )
    if (pLCD->pfnDataCallback) { // only ESP32-S2 and S3
        spilcdParallelData(pBuf, iLen);
        return;
    }
#endif
    if (pLCD->pfnDataCallback != NULL)
    {
       (*pLCD->pfnDataCallback)(pBuf, iLen, iMode);
       return;
    }
#ifdef ARDUINO_ARCH_ESP32
    if (pLCD->iLCDType > LCD_QUAD_SPI) {
        qspiSendDATA(pLCD, pBuf, iLen, iFlags);
        return;
    }
#endif // ESP32
    if (pLCD->iLCDFlags & FLAGS_BITBANG)
    {
        SPI_BitBang(pLCD, pBuf, iLen, iMode);
        return;
    }
    if (iMode == MODE_COMMAND)
    {
#ifdef HAS_DMA
        spilcdWaitDMA(); // wait for any previous transaction to finish
#endif
        spilcdSetMode(pLCD, MODE_COMMAND);
    }
    if ((iFlags & DRAW_WITH_DMA) == 0 || iMode == MODE_COMMAND)
    {
#ifdef __AVR__
      *outCS &= ~bitCS;
#else
      myPinWrite(pLCD->iCSPin, 0);
#endif // __AVR__
    }
#ifdef ARDUINO_ARCH_ESP32
    // DMA was requested, so we initialized the SPI peripheral, but
    // The user is asking to not use it, so use polling SPI
    if (pLCD->bUseDMA && (iMode == MODE_COMMAND || !(iFlags & DRAW_WITH_DMA)))
    {
        esp_err_t ret;
        static spi_transaction_t t;
        iCurrentCS = -1;
        memset(&t, 0, sizeof(t));       //Zero out the transaction
        while (iLen) {
          int l = iLen;
          if (l > 4000) { // transmit maximum length (full duplex mode)
             l = 4000;
          }
          t.length=l*8;  // length in bits
          t.rxlength = 0;
          t.tx_buffer=pBuf;
          t.user=(void*)iMode;
    // Queue the transaction
//    ret = spi_device_queue_trans(spi, &t, portMAX_DELAY);
          ret=spi_device_polling_transmit(spi, &t);  //Transmit!
          assert(ret==ESP_OK);            //Should have had no issues.
          iLen -= l;
          pBuf += l;
        } // while (iLen)
        if (iMode == MODE_COMMAND) // restore D/C pin to DATA
            spilcdSetMode(pLCD, MODE_DATA);
        myPinWrite(pLCD->iCSPin, 1);
        return;
    }
    // wait for it to complete
//    spi_transaction_t *rtrans;
//    spi_device_get_trans_result(spi, &rtrans, portMAX_DELAY);
#endif // ESP32
#ifdef HAS_DMA
    if (iMode == MODE_DATA && (iFlags & DRAW_WITH_DMA)) // only pixels will get DMA treatment
    {
        while (iLen) {
            int l = iLen;
            if (l > 4000) {
                l = 4000;
            }
            spilcdWaitDMA(); // wait for any previous transaction to finish
            iCurrentCS = pLCD->iCSPin;
            myPinWrite(pLCD->iCSPin, 0);
            spilcdWriteDataDMA(pLCD, pBuf, l);
            pBuf += l;
            iLen -= l;
        }
        return;
    }
#endif // HAS_DMA
        
// No DMA requested or available, fall through to here
#ifdef __LINUX__
{
struct spi_ioc_transfer spi;
   memset(&spi, 0, sizeof(spi));
   spi.tx_buf = (unsigned long)pBuf;
   spi.len = iLen;
   spi.speed_hz = pLCD->iSPISpeed;
   spi.bits_per_word = 8;
   ioctl(spi_fd, SPI_IOC_MESSAGE(1), &spi);
}
#else
#ifdef ARDUINO_ARCH_RP2040
    pSPI->beginTransaction(SPISettings(pLCD->iSPISpeed, MSBFIRST, pLCD->iSPIMode));
#else
    mySPI.beginTransaction(SPISettings(pLCD->iSPISpeed, MSBFIRST, pLCD->iSPIMode));
#endif
#ifdef ARDUINO_ARCH_ESP32
    mySPI.transferBytes(pBuf, NULL, iLen);
#else
#ifdef ARDUINO_ARCH_RP2040
    pSPI->transfer(pBuf, iLen);
#else
    mySPI.transfer(pBuf, iLen);
#endif // RP2040
#endif
#ifdef ARDUINO_ARCH_RP2040
    pSPI->endTransaction();
#else
    mySPI.endTransaction();
#endif
#endif // __LINUX__
    if (iMode == MODE_COMMAND) // restore D/C pin to DATA
        spilcdSetMode(pLCD, MODE_DATA);
    if (pLCD->iCSPin != -1)
    {
#ifdef __AVR__
       *outCS |= bitCS;
#else
       myPinWrite(pLCD->iCSPin, 1);
#endif
    }
} /* myspiWrite() */

//
// Public wrapper function to write data to the display
//
void spilcdWriteDataBlock(SPILCD *pLCD, uint8_t *pData, int iLen, int iFlags)
{
#ifdef LOG_OUTPUT
    Serial.printf("writeDataBlock: %d\n", iLen);
#endif
  myspiWrite(pLCD, pData, iLen, MODE_DATA, iFlags);
} /* spilcdWriteDataBlock() */
//
// spilcdWritePixelsMasked
//
void spilcdWritePixelsMasked(SPILCD *pLCD, int x, int y, uint8_t *pData, uint8_t *pMask, int iCount, int iFlags)
{
    int i, pix_count, bit_count;
    uint8_t c, *s, *sPixels, *pEnd;
    s = pMask; sPixels = pData;
    pEnd = &pMask[(iCount+7)>>3];
    i = 0;
    bit_count = 8;
    c = *s++; // get first byte
    while (i<iCount && s < pEnd) {
        // Count the number of consecutive pixels to skip
        pix_count = 0;
        while (bit_count && (c & 0x80) == 0) { // count 0's
            if (c == 0) { // quick count remaining 0 bits
                pix_count += bit_count;
                bit_count = 0;
                if (s < pEnd) {
                    bit_count = 8;
                    c = *s++;
                }
                continue;
            }
            pix_count++;
            bit_count--;
            c <<= 1;
            if (bit_count == 0 && s < pEnd) {
                bit_count = 8;
                c = *s++;
            }
        }
        // we've hit the first 1 bit, skip the source pixels we've counted
        i += pix_count;
        sPixels += pix_count*2; // skip RGB565 pixels
        // Count the number of consecutive pixels to draw
        pix_count = 0;
        while (bit_count && (c & 0x80) == 0x80) { // count 1's
            if (c == 0xff) {
                pix_count += 8;
                bit_count = 0;
                if (s < pEnd) {
                    c = *s++;
                    bit_count = 8;
                }
                continue;
            }
            pix_count++;
            bit_count--;
            c <<= 1;
            if (bit_count == 0 && s < pEnd) {
                bit_count = 8;
                c = *s++;
            }
        }
        if (pix_count) {
            spilcdSetPosition(pLCD, x+i, y, pix_count, 1, iFlags);
            spilcdWriteDataBlock(pLCD, sPixels, pix_count*2, iFlags);
        }
        i += pix_count;
        sPixels += pix_count*2;
    } // while counting pixels
} /* spilcdWritePixelsMasked() */
//
// Wrapper function to control a GPIO line
//
static void myPinWrite(int iPin, int iValue)
{
    if (iPin != -1)
        digitalWrite(iPin, (iValue) ? HIGH: LOW);
} /* myPinWrite() */

//
// Choose the gamma curve between 2 choices (0/1)
// ILI9341 only
//
int spilcdSetGamma(SPILCD *pLCD, int iMode)
{
int i;
unsigned char *sE0, *sE1;

	if (iMode < 0 || iMode > 1 || pLCD->iLCDType != LCD_ILI9341)
		return 1;
	if (iMode == 0)
	{
		sE0 = (unsigned char *)ucE0_0;
		sE1 = (unsigned char *)ucE1_0;
	}
	else
	{
		sE0 = (unsigned char *)ucE0_1;
		sE1 = (unsigned char *)ucE1_1;
	}
	spilcdWriteCommand(pLCD, 0xe0);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(pLCD, pgm_read_byte(sE0++));
	}
	spilcdWriteCommand(pLCD, 0xe1);
	for(i=0; i<16; i++)
	{
		spilcdWriteData8(pLCD, pgm_read_byte(sE1++));
	}

	return 0;
} /* spilcdSetGamma() */

// *****************
//
// Configure a GPIO pin for input
// Returns 0 if successful, -1 if unavailable
// all input pins are assumed to use internal pullup resistors
// and are connected to ground when pressed
//
int spilcdConfigurePin(int iPin)
{
        if (iPin == -1) // invalid
                return -1;
        pinMode(iPin, INPUT_PULLUP);
        return 0;
} /* spilcdConfigurePin() */
// Read from a GPIO pin
int spilcdReadPin(int iPin)
{
   if (iPin == -1)
      return -1;
   return (digitalRead(iPin) == HIGH);
} /* spilcdReadPin() */
//
// Give bb_spi_lcd two callback functions to talk to the LCD
// useful when not using SPI or providing an optimized interface
//
void spilcdSetCallbacks(SPILCD *pLCD, RESETCALLBACK pfnReset, DATACALLBACK pfnData)
{
    pLCD->pfnDataCallback = pfnData;
    pLCD->pfnResetCallback = pfnReset;
}
int spilcdParallelInit(SPILCD *pLCD, int iType, int iFlags, uint8_t RST_PIN, uint8_t RD_PIN, uint8_t WR_PIN, uint8_t CS_PIN, uint8_t DC_PIN, int iBusWidth, uint8_t *data_pins, uint32_t u32Freq)
{
    memset(pLCD, 0, sizeof(SPILCD));
    if (RST_PIN != 0xff) {
        pinMode(RST_PIN, OUTPUT);
        digitalWrite(RST_PIN, LOW);
        delay(100);
        digitalWrite(RST_PIN, HIGH);
        delay(100);
    }
    ParallelDataInit(RD_PIN, WR_PIN, CS_PIN, DC_PIN, iBusWidth, data_pins, iFlags, u32Freq);
    spilcdSetCallbacks(pLCD, ParallelReset, ParallelDataWrite);
    return spilcdInit(pLCD, iType, iFlags, 0,0,0,0,0,0,0,0,0);
} /* spilcdParallelInit() */

//
// Initialize the LCD controller and clear the display
// LED pin is optional - pass as -1 to disable
//
int spilcdInit(SPILCD *pLCD, int iType, int iFlags, int32_t iSPIFreq, int iCS, int iDC, int iReset, int iLED, int iMISOPin, int iMOSIPin, int iCLKPin, int bUseDMA)
{
unsigned char *s, *d;
int i, iCount;
#ifdef ARDUINO_ARCH_ESP32
static int iStarted = 0; // indicates if the master driver has already been initialized 
#endif

    if (pLCD->pFont != NULL && iSPIFreq != 0) { // the structure is probably not initialized
        memset(pLCD, 0, sizeof(SPILCD));
    }   
    pLCD->bUseDMA = bUseDMA;
    pLCD->iColStart = pLCD->iRowStart = pLCD->iMemoryX = pLCD->iMemoryY = 0;
    pLCD->iOrientation = 0;
    pLCD->iLCDType = iType;
    pLCD->iLCDFlags = iFlags;

  if (pLCD->pfnResetCallback != NULL)
  {
     (*pLCD->pfnResetCallback)();
     goto start_of_init;
  }
#ifndef ARDUINO_ARCH_ESP32
    (void)iMISOPin;
#endif
#ifdef __AVR__
{ // setup fast I/O
  uint8_t port;
    port = digitalPinToPort(iDC);
    outDC = portOutputRegister(port);
    bitDC = digitalPinToBitMask(iDC);
    if (iCS != -1) {
      port = digitalPinToPort(iCS);
      outCS = portOutputRegister(port);
      bitCS = digitalPinToBitMask(iCS);
    }
}
#endif

    pLCD->iLEDPin = -1; // assume it's not defined
	if (iType <= LCD_INVALID || iType >= LCD_VALID_MAX)
	{
#ifndef __LINUX__
		Serial.println("Unsupported display type\n");
#endif // __LINUX__
		return -1;
	}
#ifndef __LINUX__
    pLCD->iSPIMode = (iType == LCD_ST7789_NOCS || iType == LCD_ST7789) ? SPI_MODE3 : SPI_MODE0;
#endif
    pLCD->iSPISpeed = iSPIFreq;
	pLCD->iScrollOffset = 0; // current hardware scroll register value

    pLCD->iDCPin = iDC;
    pLCD->iCSPin = iCS;
    pLCD->iResetPin = iReset;
    pLCD->iLEDPin = iLED;
	if (pLCD->iDCPin == -1)
	{
#ifndef __LINUX__
		Serial.println("One or more invalid GPIO pin numbers\n");
#endif
		return -1;
	}
    if (iFlags & FLAGS_BITBANG)
    {
        pinMode(pLCD->iMOSIPin, OUTPUT);
        pinMode(pLCD->iCLKPin, OUTPUT);
        goto skip_spi_init;
    }
#ifdef ARDUINO_ARCH_ESP32
    if (!iStarted && bUseDMA) {
    esp_err_t ret;

        if (iMOSIPin == -1 || iMOSIPin == 0xff) {
            // use the Arduino defaults
            iMISOPin = MISO;
            iMOSIPin = MOSI;
            iCLKPin = SCK;
        }
    memset(&buscfg, 0, sizeof(buscfg));
    buscfg.miso_io_num = iMISOPin;
    buscfg.mosi_io_num = iMOSIPin;
    buscfg.sclk_io_num = iCLKPin;
    buscfg.max_transfer_sz=4096;
    buscfg.quadwp_io_num=-1;
    buscfg.quadhd_io_num=-1;
    //Initialize the SPI bus
    ret=spi_bus_initialize(ESP32_SPI_HOST, &buscfg, SPI_DMA_CHAN);
    assert(ret==ESP_OK);
    
    memset(&devcfg, 0, sizeof(devcfg));
    devcfg.clock_speed_hz = iSPIFreq;
    devcfg.mode = pLCD->iSPIMode;                         //SPI mode 0 or 3
    devcfg.spics_io_num = -1;               //CS pin, set to -1 to disable since we handle it outside of the master driver
    devcfg.queue_size = 2;                          //We want to be able to queue 2 transactions at a time
// These callbacks currently don't do anything
    devcfg.pre_cb = spi_pre_transfer_callback;  //Specify pre-transfer callback to handle D/C line
    devcfg.post_cb = spi_post_transfer_callback;
    devcfg.flags = SPI_DEVICE_NO_DUMMY; // allow speeds > 26Mhz
//    devcfg.flags = SPI_DEVICE_HALFDUPLEX; // this disables SD card access
    //Attach the LCD to the SPI bus
    ret=spi_bus_add_device(ESP32_SPI_HOST, &devcfg, &spi);
    assert(ret==ESP_OK);
    memset(&trans[0], 0, sizeof(spi_transaction_t));
        iStarted = 1; // don't re-initialize this code
    } else { // no DMA
       if (iMISOPin != iMOSIPin) {
          mySPI.begin(iCLKPin, iMISOPin, iMOSIPin, -1); //iCS);
       } else {
          mySPI.begin();
       }
    } // bUseDMA
#else
#ifdef __LINUX__
    spi_fd = open("/dev/spidev0.1", O_RDWR); // DEBUG - open SPI channel 0 
#else
#ifdef ARDUINO_ARCH_RP2040
  pSPI->begin();
#else
  mySPI.begin(); // simple Arduino init (e.g. AVR)
#endif
#ifdef ARDUINO_SAMD_ZERO

  myDMA.setTrigger(mySPI.getDMAC_ID_TX());
  myDMA.setAction(DMA_TRIGGER_ACTON_BEAT);

  stat = myDMA.allocate();
  desc = myDMA.addDescriptor(
    ucTXBuf,                    // move data from here
    (void *)(mySPI.getDataRegister()),
    100,                      // this many...
    DMA_BEAT_SIZE_BYTE,               // bytes/hword/words
    true,                             // increment source addr?
    false);                           // increment dest addr?

  myDMA.setCallback(dma_callback);
#endif // ARDUINO_SAMD_ZERO

#endif // __LINUX__
#endif
//
// Start here if bit bang enabled
//
    skip_spi_init:
    if (pLCD->iCSPin != -1)
    {
        pinMode(pLCD->iCSPin, OUTPUT);
        myPinWrite(pLCD->iCSPin, HIGH);
    }
	pinMode(pLCD->iDCPin, OUTPUT);
	if (pLCD->iLEDPin != -1)
    {
#if (ESP_IDF_VERSION_MAJOR > 4)
        ledcAttach(pLCD->iLEDPin, 5000, 8); // attach pin to channel 0
        ledcWrite(pLCD->iLEDPin, 192); // set to moderate brightness to begin
#else   
	pinMode(pLCD->iLEDPin, OUTPUT);
        myPinWrite(pLCD->iLEDPin, 1); // turn on the backlight
#endif
    }
	if (pLCD->iResetPin != -1)
	{
        pinMode(pLCD->iResetPin, OUTPUT);
		myPinWrite(pLCD->iResetPin, 1);
		delayMicroseconds(60000);
		myPinWrite(pLCD->iResetPin, 0); // reset the controller
		delayMicroseconds(60000);
		myPinWrite(pLCD->iResetPin, 1);
		delayMicroseconds(460000);
	}
	if (pLCD->iLCDType != LCD_SSD1351 && pLCD->iLCDType != LCD_SSD1331) // no backlight and no soft reset on OLED
	{
        spilcdWriteCommand(pLCD, 0x01); // software reset
        delayMicroseconds(60000);

        spilcdWriteCommand(pLCD, 0x11);
        delayMicroseconds(60000);
        delayMicroseconds(60000);
	}
start_of_init:
    d = ucRXBuf;
        switch (pLCD->iLCDType) {
           case LCD_ST7793:
           {
            uint16_t u16CMD, *s;
            int iSize;
            pLCD->iCMDType = CMD_TYPE_SITRONIX_16BIT;
            pLCD->iCurrentWidth = pLCD->iWidth = 240;
            pLCD->iCurrentHeight = pLCD->iHeight = 400;
            // 16-bit commands
            s = (uint16_t *)u16ST7793_Init;
            iSize = sizeof(u16ST7793_Init)/4;
            for (int i=0; i<iSize; i++) {
               u16CMD = *s++;
               if (u16CMD == LCD_DELAY) {
                  delay(s[0]);
                  s++;
               } else {
                  spilcdWriteCommand16(pLCD, u16CMD);
                  spilcdWriteData16(pLCD, *s++, DRAW_TO_LCD);
               } // if u16CMD
            } // for
           } // LCD_ST7793
           break;
        case LCD_ST7796:
        case LCD_ST7796_222:
           {
            uint8_t iBGR = (pLCD->iLCDFlags & FLAGS_SWAP_RB) ? 0x08:0;
            uint8_t iFlipX = (pLCD->iLCDFlags & FLAGS_FLIPX) ? 0x40:0;
            s = (unsigned char *)&ucST7796InitList[0];
            memcpy_P(d, s, sizeof(ucST7796InitList));
            s = d;
            s[16] = iFlipX | iBGR;
            if (pLCD->iLCDFlags & FLAGS_INVERT)
               s[18] = 0x21; // change inversion off (default) to on
            pLCD->iCurrentWidth = pLCD->iWidth = 320;
            pLCD->iCurrentHeight = pLCD->iHeight = 480;
            if (pLCD->iLCDType == LCD_ST7796_222) {
                pLCD->iCurrentWidth = pLCD->iWidth = 222;
                pLCD->iColStart = pLCD->iMemoryX = 49;
            }
            pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
           }
           break;
	case LCD_ST7789:
        case LCD_ST7789_172:
        case LCD_ST7789_280:
        case LCD_ST7789_240:
        case LCD_ST7789_135:
        case LCD_ST7789_NOCS:
	{
        uint8_t iBGR = (pLCD->iLCDFlags & FLAGS_SWAP_RB) ? 8:0;
		s = (unsigned char *)&uc240x240InitList[0];
        memcpy_P(d, s, sizeof(uc240x240InitList));
        s = d;
        s[6] = 0x00 + iBGR;
        if (pLCD->iLCDFlags & FLAGS_INVERT)
           s[3] = 0x20; // change inversion on (default) to off
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 240;
        pLCD->iCurrentHeight = pLCD->iHeight = 320;
	if (pLCD->iLCDType == LCD_ST7789_240 || pLCD->iLCDType == LCD_ST7789_NOCS)
	{
            pLCD->iCurrentWidth = pLCD->iWidth = 240;
            pLCD->iCurrentHeight = pLCD->iHeight = 240;
	}
	else if (pLCD->iLCDType == LCD_ST7789_135)
	{
            pLCD->iCurrentWidth = pLCD->iWidth = 135;
            pLCD->iCurrentHeight = pLCD->iHeight = 240;
            pLCD->iColStart = pLCD->iMemoryX = 52;
            pLCD->iRowStart = pLCD->iMemoryY = 40;
	} else if (pLCD->iLCDType == LCD_ST7789_280) {
	    pLCD->iCurrentWidth = pLCD->iWidth = 240;
	    pLCD->iCurrentHeight = pLCD->iHeight = 280;
            pLCD->iRowStart = pLCD->iMemoryY = 20;
	} else if (pLCD->iLCDType == LCD_ST7789_172) {
            pLCD->iCurrentWidth = pLCD->iWidth = 174;
            pLCD->iCurrentHeight = pLCD->iHeight = 320;
            pLCD->iColStart = pLCD->iMemoryX = 33;
            pLCD->iRowStart = pLCD->iMemoryY = 0;
	}
        pLCD->iLCDType = LCD_ST7789; // treat them the same from here on
       } // ST7789
       break;
    case LCD_JD9613:
       {
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 126;
        pLCD->iCurrentHeight = pLCD->iHeight = 294;
        pLCD->iColStart = pLCD->iMemoryX = 0;
        pLCD->iRowStart = pLCD->iMemoryY = 0;
        s = (unsigned char *)&ucJD9613InitList[0];
        memcpy_P(d, s, sizeof(ucJD9613InitList));
        s = d;
       }
       break;

    case LCD_GC9D01:
       {
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 120;
        pLCD->iCurrentHeight = pLCD->iHeight = 120;
        pLCD->iColStart = pLCD->iMemoryX = 0;
        pLCD->iRowStart = pLCD->iMemoryY = 0;
        s = (unsigned char *)&ucGC9D01InitList[0];
        memcpy_P(d, s, sizeof(ucGC9D01InitList));
        s = d;
       } // GC9D01
       break;

    case LCD_GC9A01:
       {
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 240;
        pLCD->iCurrentHeight = pLCD->iHeight = 240;
        pLCD->iColStart = pLCD->iMemoryX = 0;
        pLCD->iRowStart = pLCD->iMemoryY = 0;
        s = (unsigned char *)&ucGC9A01InitList[0];
        memcpy_P(d, s, sizeof(ucGC9A01InitList));
        s = d;
       } // GC9A01
       break;
    case LCD_GC9107:
       {
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 128;
        pLCD->iCurrentHeight = pLCD->iHeight = 128;
        pLCD->iColStart = pLCD->iMemoryX = 2;
        pLCD->iRowStart = pLCD->iMemoryY = 1;
        s = (unsigned char *)&ucGC9107InitList[0];
        memcpy_P(d, s, sizeof(ucGC9107InitList));
        s = d;
       } // GC9107
       break;
    case LCD_SSD1331:
       {
        s = (unsigned char *)ucSSD1331InitList;
        memcpy_P(d, ucSSD1331InitList, sizeof(ucSSD1331InitList));
        s = d;

        pLCD->iCMDType = CMD_TYPE_SOLOMON_OLED2;
        pLCD->iCurrentWidth = pLCD->iWidth = 96;
        pLCD->iCurrentHeight = pLCD->iHeight = 64;

        if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
        { // copy to RAM to modify it
            s[6] = 0x76;
        }
       }
       break;
    case LCD_SSD1351:
	{
	s = (unsigned char *)ucOLEDInitList; // do the commands manually
        memcpy_P(d, s, sizeof(ucOLEDInitList));
        pLCD->iCMDType = CMD_TYPE_SOLOMON_OLED1;
        pLCD->iCurrentWidth = pLCD->iWidth = 128;
        pLCD->iCurrentHeight = pLCD->iHeight = 128;
	}
        break;
    // Send the commands/parameters to initialize the LCD controller
    case LCD_ILI9341:
	{  // copy to RAM to modify
        s = (unsigned char *)uc240InitList;
        memcpy_P(d, s, sizeof(uc240InitList));
        s = d;
        if (pLCD->iLCDFlags & FLAGS_INVERT)
            s[52] = 0x21; // invert pixels
        else
            s[52] = 0x20; // non-inverted
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 240;
        pLCD->iCurrentHeight = pLCD->iHeight = 320;
	}
       break;
    case LCD_SSD1283A:
       {
        s = (unsigned char *)uc132InitList;
        memcpy_P(d, s, sizeof(uc132InitList));
        pLCD->iCMDType = CMD_TYPE_SOLOMON_LCD;
        pLCD->iCurrentWidth = pLCD->iWidth = 132;
        pLCD->iCurrentHeight = pLCD->iHeight = 132;
       }
       break;
    case LCD_SSD1286:
       {
        s = (unsigned char *)uc132x176InitList;
        memcpy_P(d, s, sizeof(uc132x176InitList));
        pLCD->iCMDType = CMD_TYPE_SOLOMON_LCD;
        pLCD->iCurrentWidth = pLCD->iWidth = 132;
        pLCD->iCurrentHeight = pLCD->iHeight = 176;
       }
       break;
    case LCD_ILI9342:
	{
	s = (unsigned char *)uc320InitList;
        memcpy_P(d, s, sizeof(uc320InitList));
        s = d;
        if (pLCD->iLCDFlags & FLAGS_INVERT)
            s[24] = 0x20; // invert pixels
        else
            s[24] = 0x21; // non-inverted
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        pLCD->iCurrentWidth = pLCD->iWidth = 320;
        pLCD->iCurrentHeight = pLCD->iHeight = 240;
	}
       break;
    case LCD_HX8357:
	{
        pLCD->iCMDType = CMD_TYPE_SITRONIX_16BIT;
        spilcdWriteCommand(pLCD, 0xb0);
        spilcdWriteData16(pLCD, 0x00FF, DRAW_TO_LCD);
        spilcdWriteData16(pLCD, 0x0001, DRAW_TO_LCD);
        delayMicroseconds(60000);

        s = (unsigned char *)uc480InitList;
        memcpy_P(d, s, sizeof(uc480InitList));
        s = d;
        pLCD->iCurrentWidth = pLCD->iWidth = 320;
        pLCD->iCurrentHeight = pLCD->iHeight = 480;
	}
        break;
    case LCD_ILI9486:
    case LCD_ILI9488:
        {
            uint8_t ucBGRFlags, iBGROffset, iInvertOffset;
            if (pLCD->iLCDType == LCD_ILI9488) { // slightly different init sequence
                s = (unsigned char *)ucILI9488InitList;
                memcpy_P(d, s, sizeof(ucILI9488InitList));
                iBGROffset = 18;
                iInvertOffset = 20;
            } else {
                s = (unsigned char *)ucILI9486InitList;
                memcpy_P(d, s, sizeof(ucILI9486InitList));
                iBGROffset = 66;
                iInvertOffset = 63;
            }
            pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
            s = d;
            ucBGRFlags = 0xa; // normal direction, RGB color order
            if (pLCD->iLCDFlags & FLAGS_FLIPX) {
                ucBGRFlags ^= 0x40;
                d[iBGROffset] = ucBGRFlags;
            }
            if (pLCD->iLCDFlags & FLAGS_INVERT)
               d[iInvertOffset] = 0x21; // invert display command
            if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
            {
                ucBGRFlags |= 8;
               d[iBGROffset] = ucBGRFlags;
            }
            pLCD->iCurrentWidth = pLCD->iWidth = 320;
            pLCD->iCurrentHeight = pLCD->iHeight = 480;
        }
        break;
    case LCD_ILI9225:
        {
        uint8_t iBGR = 0x10;
        if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
            iBGR = 0;
        s = (unsigned char *)ucILI9225InitList;
        memcpy_P(d, s, sizeof(ucILI9225InitList));
        s = d;
        pLCD->iCMDType = CMD_TYPE_ILITEK_16BIT;
//        if (iFlags & FLAGS_INVERT)
//           s[55] = 0x21; // invert on
//        else
//           s[55] = 0x20; // invert off
        s[74] = iBGR;
        pLCD->iCurrentWidth = pLCD->iWidth = 176;
        pLCD->iCurrentHeight = pLCD->iHeight = 220;
       }
       break;
    case LCD_ST7735S:
    case LCD_ST7735S_B:
       {
        uint8_t iBGR = 0;
        if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
            iBGR = 8;
        s = (unsigned char *)uc80InitList;
        memcpy_P(d, s, sizeof(uc80InitList));
        pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        s = d;
        if (pLCD->iLCDFlags & FLAGS_INVERT)
           s[55] = 0x21; // invert on
        else
           s[55] = 0x20; // invert off
        s[5] = 0x00 + iBGR; // normal orientation
        pLCD->iCurrentWidth = pLCD->iWidth = 80;
        pLCD->iCurrentHeight = pLCD->iHeight = 160;
        if (pLCD->iLCDType == LCD_ST7735S_B)
        {
            pLCD->iLCDType = LCD_ST7735S; // the rest is the same
            pLCD->iColStart = pLCD->iMemoryX = 26; // x offset of visible area
            pLCD->iRowStart = pLCD->iMemoryY = 1;
        }
        else
        {
            pLCD->iColStart = pLCD->iMemoryX = 24;
        }
       }
       break;
    case LCD_ST7735R:
    case LCD_ST7735_128:
	{
		s = (unsigned char *)uc128InitList;
                memcpy_P(d, s, sizeof(uc128InitList));
                s = d;
                pLCD->iCMDType = CMD_TYPE_SITRONIX_8BIT;
        	pLCD->iCurrentWidth = pLCD->iWidth = 128;
                if (pLCD->iLCDType == LCD_ST7735R) {
        	   pLCD->iCurrentHeight = pLCD->iHeight = 160;
                } else {
                   pLCD->iColStart = pLCD->iMemoryX = 2;
                   pLCD->iRowStart = pLCD->iMemoryY = 3;
                   pLCD->iCurrentHeight = pLCD->iHeight = 128;
                   if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
                      s[5] = 0xc0;
                   else
                      s[5] = 0xc8;
                }
	}
        break;
        default:
          return -1;
      } // switch on LCD type
    iCount = 1;
    bSetPosition = 1; // don't let the data writes affect RAM
    s = d; // start of RAM copy of our data
	while (iCount)
	{
		iCount = *s++;
		if (iCount != 0)
		{
               unsigned char uc;
               if (iCount == LCD_DELAY)
               {
                 uc = *s++;
                 delay(uc);
               }
               else
               {
                 if (pLCD->iLCDType == LCD_ILI9225) // 16-bit commands and data
                 {
                     uint8_t uc2;
                     for (i=0; i<iCount; i++)
                     {
                        uc = *s++;
                         uc2 = *s++;
                        spilcdWriteData16(pLCD, (uc << 8) | uc2, DRAW_TO_LCD);
                     } // for i
                 } else {
                     if (pLCD->iLCDType == LCD_SSD1331) {
                         // SSD1331 expects the parameters to
                         // be written in command mode
                         for (i=0; i<iCount; i++)
                         {
                            uc = *s++;
                            spilcdWriteCommand(pLCD, uc);
                         } // for i
                     } else { // normal 1 byte CMD + data params
                         spilcdWriteCmdParams(pLCD, s[0], &s[1], iCount-1);
                         s += iCount;
                     }
                 }
              }
          }
	  }
        bSetPosition = 0;
	if (pLCD->iLCDType != LCD_SSD1351 && pLCD->iLCDType != LCD_SSD1331)
	{
		spilcdWriteCommand(pLCD, 0x11); // sleep out
		delayMicroseconds(60000);
		spilcdWriteCommand(pLCD, 0x29); // Display ON
		delayMicroseconds(10000);
	}
//	spilcdFill(0, 1); // erase memory
	spilcdScrollReset(pLCD);
   
	return 0;

} /* spilcdInit() */

//
// Reset the scroll position to 0
//
void spilcdScrollReset(SPILCD *pLCD)
{
uint32_t u32Temp;
int iLen;
    
    if (pLCD->iLCDType == LCD_VIRTUAL_MEM) {
        return;
    }
	pLCD->iScrollOffset = 0;
	if (pLCD->iLCDType == LCD_SSD1351) {
		spilcdWriteCommand(pLCD, 0xa1); // set scroll start line
		spilcdWriteData8(pLCD, 0x00);
		spilcdWriteCommand(pLCD, 0xa2); // display offset
		spilcdWriteData8(pLCD, 0x00);
		return;
	}
    else if (pLCD->iLCDType == LCD_SSD1331) {
        spilcdWriteCommand(pLCD, 0xa1);
        spilcdWriteCommand(pLCD, 0x00);
        spilcdWriteCommand(pLCD, 0xa2);
        spilcdWriteCommand(pLCD, 0x00);
        return;
    }
    u32Temp = 0;
    iLen = (pLCD->iLCDType == LCD_HX8357) ? 4:2;
	spilcdWriteCmdParams(pLCD, 0x37, (uint8_t *)&u32Temp, iLen); // scroll start address
} /* spilcdScrollReset() */

//
// Scroll the screen N lines vertically (positive or negative)
// The value given represents a delta which affects the current scroll offset
// If iFillColor != -1, the newly exposed lines will be filled with that color
//
void spilcdScroll(SPILCD *pLCD, int iLines, int iFillColor)
{

	pLCD->iScrollOffset = (pLCD->iScrollOffset + iLines) % pLCD->iHeight;
	if (pLCD->iLCDType == LCD_SSD1351)
	{
		spilcdWriteCommand(pLCD, 0xa1); // set scroll start line
		spilcdWriteData8(pLCD, pLCD->iScrollOffset);
		return;
	}
    else if (pLCD->iLCDType == LCD_SSD1331)
    {
        spilcdWriteCommand(pLCD, 0xa1);
        spilcdWriteCommand(pLCD, pLCD->iScrollOffset);
        return;
    }
	else
	{
		spilcdWriteCommand(pLCD, 0x37); // Vertical scrolling start address
		if (pLCD->iLCDType == LCD_ILI9341 || pLCD->iLCDType == LCD_ILI9342 || pLCD->iLCDType == LCD_ST7735R || pLCD->iLCDType == LCD_ST7789 || pLCD->iLCDType == LCD_ST7789_135 || pLCD->iLCDType == LCD_ST7735S)
		{
			spilcdWriteData16(pLCD, pLCD->iScrollOffset, DRAW_TO_LCD);
		}
		else
		{
			spilcdWriteData16(pLCD, pLCD->iScrollOffset >> 8, DRAW_TO_LCD);
			spilcdWriteData16(pLCD, pLCD->iScrollOffset & -1, DRAW_TO_LCD);
		}
	}
	if (iFillColor != -1) // fill the exposed lines
	{
	int i, iStart;
	uint16_t *usTemp = (uint16_t *)ucRXBuf;
	uint32_t *d;
	uint32_t u32Fill;
		// quickly prepare a full line's worth of the color
                if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
	            u32Fill = (iFillColor >> 8) | ((iFillColor & -1) << 8);
                } else {
                    u32Fill = iFillColor;
                }
		u32Fill |= (u32Fill << 16);
		d = (uint32_t *)&usTemp[0];
		for (i=0; i<pLCD->iWidth/2; i++)
			*d++ = u32Fill;
		if (iLines < 0)
		{
			iStart = 0;
			iLines = 0 - iLines;
		}
		else
			iStart = pLCD->iHeight - iLines;
        spilcdSetPosition(pLCD, 0, iStart, pLCD->iWidth, iLines, DRAW_TO_LCD);
		for (i=0; i<iLines; i++)
		{
			myspiWrite(pLCD, (unsigned char *)usTemp, pLCD->iWidth*2, MODE_DATA, DRAW_TO_LCD);
		}
	}

} /* spilcdScroll() */
//
// Draw a 24x24 RGB565 tile scaled to 40x40
// The main purpose of this function is for GameBoy emulation
// Since the original display is 160x144, this function allows it to be
// stretched 166% larger (266x240). Not a perfect fit for 320x240, but better
// Each group of 3x3 pixels becomes a group of 5x5 pixels by averaging the pixels
//
// +-+-+-+ becomes +----+----+----+----+----+
// |A|B|C|         |A   |ab  |B   |bc  |C   |
// +-+-+-+         +----+----+----+----+----+
// |D|E|F|         |ad  |abde|be  |becf|cf  |
// +-+-+-+         +----+----+----+----+----+
// |G|H|I|         |D   |de  |E   |ef  |F   |
// +-+-+-+         +----+----+----+----+----+
//                 |dg  |dgeh|eh  |ehfi|fi  |
//                 +----+----+----+----+----+
//                 |G   |gh  |H   |hi  |I   |
//                 +----+----+----+----+----+
//
// The x/y coordinates will be scaled as well
//
int spilcdDraw53Tile(SPILCD *pLCD, int x, int y, int cx, int cy, unsigned char *pTile, int iPitch, int iFlags)
{
    int i, j, iPitch16;
    uint16_t *s, *d;
    uint16_t u32A, u32B, u32C, u32D, u32E, u32F;
    uint16_t t1, t2, u32ab, u32bc, u32de, u32ef, u32ad, u32be, u32cf;
    uint16_t u32Magic = 0xf7de;
    
    // scale coordinates for stretching
    x = (x * 5)/3;
    y = (y * 5)/3;
    iPitch16 = iPitch/2;
    if (cx < 24 || cy < 24)
        memset((void *)pDMA, 0, 40*40*2);
    for (j=0; j<cy/3; j++) // 8 blocks of 3 lines
    {
        s = (uint16_t *)&pTile[j*3*iPitch];
        d = (uint16_t *)&pDMA[j*40*5*2];
        for (i=0; i<cx-2; i+=3) // source columns (3 at a time)
        {
            u32A = s[i];
            u32B = s[i+1];
            u32C = s[i+2];
            u32D = s[i+iPitch16];
            u32E = s[i+iPitch16+1];
            u32F = s[i+iPitch16 + 2];
            u32bc = u32ab = (u32B & u32Magic) >> 1;
            u32ab += ((u32A & u32Magic) >> 1);
            u32bc += (u32C & u32Magic) >> 1;
            u32de = u32ef = ((u32E & u32Magic) >> 1);
            u32de += ((u32D & u32Magic) >> 1);
            u32ef += ((u32F & u32Magic) >> 1);
            u32ad = ((u32A & u32Magic) >> 1) + ((u32D & u32Magic) >> 1);
            u32be = ((u32B & u32Magic) >> 1) + ((u32E & u32Magic) >> 1);
            u32cf = ((u32C & u32Magic) >> 1) + ((u32F & u32Magic) >> 1);
            // first row
            d[0] = __builtin_bswap16(u32A); // swap byte order
            d[1] = __builtin_bswap16(u32ab);
            d[2] = __builtin_bswap16(u32B);
            d[3] = __builtin_bswap16(u32bc);
            d[4] = __builtin_bswap16(u32C);
            // second row
            t1 = ((u32ab & u32Magic) >> 1) + ((u32de & u32Magic) >> 1);
            t2 = ((u32be & u32Magic) >> 1) + ((u32cf & u32Magic) >> 1);
            d[40] = __builtin_bswap16(u32ad);
            d[41] = __builtin_bswap16(t1);
            d[42] = __builtin_bswap16(u32be);
            d[43] = __builtin_bswap16(t2);
            d[44] = __builtin_bswap16(u32cf);
            // third row
            d[80] = __builtin_bswap16(u32D);
            d[81] = __builtin_bswap16(u32de);
            d[82] = __builtin_bswap16(u32E);
            d[83] = __builtin_bswap16(u32ef);
            d[84] = __builtin_bswap16(u32F);
            // fourth row
            u32A = s[i+iPitch16*2];
            u32B = s[i+iPitch16*2 + 1];
            u32C = s[i+iPitch16*2 + 2];
            u32bc = u32ab = (u32B & u32Magic) >> 1;
            u32ab += ((u32A & u32Magic) >> 1);
            u32bc += (u32C & u32Magic) >> 1;
            u32ad = ((u32A & u32Magic) >> 1) + ((u32D & u32Magic) >> 1);
            u32be = ((u32B & u32Magic) >> 1) + ((u32E & u32Magic) >> 1);
            u32cf = ((u32C & u32Magic) >> 1) + ((u32F & u32Magic) >> 1);
            t1 = ((u32ab & u32Magic) >> 1) + ((u32de & u32Magic) >> 1);
            t2 = ((u32be & u32Magic) >> 1) + ((u32cf & u32Magic) >> 1);
            d[120] = __builtin_bswap16(u32ad);
            d[121] = __builtin_bswap16(t1);
            d[122] = __builtin_bswap16(u32be);
            d[123] = __builtin_bswap16(t2);
            d[124] = __builtin_bswap16(u32cf);
            // fifth row
            d[160] = __builtin_bswap16(u32A);
            d[161] = __builtin_bswap16(u32ab);
            d[162] = __builtin_bswap16(u32B);
            d[163] = __builtin_bswap16(u32bc);
            d[164] = __builtin_bswap16(u32C);
            d += 5;
        } // for i
    } // for j
    spilcdSetPosition(pLCD, x, y, 40, 40, iFlags);
    myspiWrite(pLCD, (uint8_t *)pDMA, 40*40*2, MODE_DATA, iFlags);
    return 0;
} /* spilcdDraw53Tile() */
//
// Draw a NxN RGB565 tile
// This reverses the pixel byte order and sets a memory "window"
// of pixels so that the write can occur in one shot
//
int spilcdDrawTile(SPILCD *pLCD, int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch, int iFlags)
{
    int i, j;
    //uint32_t ul32;
    //unsigned char *s, *d;
    uint16_t *s16, *d16;

    if (iTileWidth*iTileHeight > 2048) {
        return -1; // tile must fit in 4k SPI block size
    }
    // First convert to big-endian order
    d16 = (uint16_t *)pDMA;
    for (j=0; j<iTileHeight; j++)
    {
        s16 = (uint16_t*)&pTile[j*iPitch];
        for (i=0; i<iTileWidth; i++)
        {
            *d16++ = __builtin_bswap16(*s16++);
        } // for i;
    } // for j
    spilcdSetPosition(pLCD, x, y, iTileWidth, iTileHeight, iFlags);
    myspiWrite(pLCD, (uint8_t *)pDMA, iTileWidth*iTileHeight*2, MODE_DATA, iFlags);
    return 0;
} /* spilcdDrawTile() */

//
// Draw a 16x16 tile as 16x14 (with pixel averaging)
// This is for drawing 160x144 video games onto a 160x128 display
// It is assumed that the display is set to LANDSCAPE orientation
//
int spilcdDrawSmallTile(SPILCD *pLCD, int x, int y, unsigned char *pTile, int iPitch, int iFlags)
{
    unsigned char ucTemp[448];
    int i, j, iPitch32;
    uint16_t *d;
    uint32_t *s;
    uint32_t u32A, u32B, u32a, u32b, u32C, u32D;
    uint32_t u32Magic = 0xf7def7de;
    uint32_t u32Mask = 0xffff;
    
    // scale y coordinate for shrinking
    y = (y * 7)/8;
    iPitch32 = iPitch/4;
    for (j=0; j<16; j+=2) // 16 source lines (2 at a time)
    {
        s = (uint32_t *)&pTile[j * 2];
        d = (uint16_t *)&ucTemp[j*28];
        for (i=0; i<16; i+=2) // 16 source columns (2 at a time)
        {
            u32A = s[(15-i)*iPitch32]; // read A+C
            u32B = s[(14-i)*iPitch32]; // read B+D
            u32C = u32A >> 16;
            u32D = u32B >> 16;
            u32A &= u32Mask;
            u32B &= u32Mask;
            if (i == 0 || i == 8) // pixel average a pair
            {
                u32a = (u32A & u32Magic) >> 1;
                u32a += ((u32B & u32Magic) >> 1);
                u32b = (u32C & u32Magic) >> 1;
                u32b += ((u32D & u32Magic) >> 1);
                d[0] = __builtin_bswap16(u32a);
                d[14] = __builtin_bswap16(u32b);
                d++;
            }
            else
            {
                d[0] = __builtin_bswap16(u32A);
                d[1] = __builtin_bswap16(u32B);
                d[14] = __builtin_bswap16(u32C);
                d[15] = __builtin_bswap16(u32D);
                d += 2;
            }
        } // for i
    } // for j
    spilcdSetPosition(pLCD, x, y, 16, 14, iFlags);
    myspiWrite(pLCD, ucTemp, 448, MODE_DATA, iFlags);
    return 0;
} /* spilcdDrawSmallTile() */

//
// Draw a 16x16 tile as 16x13 (with priority to non-black pixels)
// This is for drawing a 224x288 image onto a 320x240 display in landscape
//
int spilcdDrawRetroTile(SPILCD *pLCD, int x, int y, unsigned char *pTile, int iPitch, int iFlags)
{
    unsigned char ucTemp[416];
    int i, j, iPitch16;
    uint16_t *s, *d, u16A, u16B;
    
    // scale y coordinate for shrinking
    y = (y * 13)/16;
    iPitch16 = iPitch/2;
    for (j=0; j<16; j++) // 16 destination columns
    {
        s = (uint16_t *)&pTile[j * 2];
        d = (uint16_t *)&ucTemp[j*26];
        for (i=0; i<16; i++) // 13 actual source rows
        {
            if (i == 0 || i == 5 || i == 10) // combined pixels
            {
                u16A = s[(15-i)*iPitch16];
                u16B = s[(14-i)*iPitch16];
                if (u16A == 0)
                    *d++ = __builtin_bswap16(u16B);
                else
                    *d++ = __builtin_bswap16(u16A);
                i++; // advance count since we merged 2 lines
            }
            else // just copy
            {
                *d++ = __builtin_bswap16(s[(15-i)*iPitch16]);
            }
        } // for i
    } // for j
    spilcdSetPosition(pLCD, x, y, 16, 13, iFlags);
    myspiWrite(pLCD, ucTemp, 416, MODE_DATA, iFlags);
    return 0;
    
} /* spilcdDrawRetroTile() */
//
// Draw a NxN RGB565 tile
// This reverses the pixel byte order and sets a memory "window"
// of pixels so that the write can occur in one shot
// Scales the tile by 150% (for GameBoy/GameGear)
//
int spilcdDrawTile150(SPILCD *pLCD, int x, int y, int iTileWidth, int iTileHeight, unsigned char *pTile, int iPitch, int iFlags)
{
    int i, j, iPitch32, iLocalPitch;
    uint32_t ul32A, ul32B, ul32Avg, ul32Avg2;
    uint16_t u16Avg, u16Avg2;
    uint32_t u32Magic = 0xf7def7de;
    uint16_t u16Magic = 0xf7de;
    uint16_t *d16;
    uint32_t *s32;
    
    if (iTileWidth*iTileHeight > 1365)
        return -1; // tile must fit in 4k SPI block size
    
    iPitch32 = iPitch / 4;
    iLocalPitch = (iTileWidth * 3)/2; // offset to next output line
    d16 = (uint16_t *)pDMA;
    for (j=0; j<iTileHeight; j+=2)
    {
        s32 = (uint32_t*)&pTile[j*iPitch];
        for (i=0; i<iTileWidth; i+=2) // turn 2x2 pixels into 3x3
        {
            ul32A = s32[0];
            ul32B = s32[iPitch32]; // get 2x2 pixels
            // top row
            ul32Avg = ((ul32A & u32Magic) >> 1);
            ul32Avg2 = ((ul32B & u32Magic) >> 1);
            u16Avg = (uint16_t)(ul32Avg + (ul32Avg >> 16)); // average the 2 pixels
            d16[0] = __builtin_bswap16((uint16_t)ul32A); // first pixel
            d16[1] = __builtin_bswap16(u16Avg); // middle (new) pixel
            d16[2] = __builtin_bswap16((uint16_t)(ul32A >> 16)); // 3rd pixel
            u16Avg2 = (uint16_t)(ul32Avg2 + (ul32Avg2 >> 16)); // bottom line averaged pixel
            d16[iLocalPitch] = __builtin_bswap16((uint16_t)(ul32Avg + ul32Avg2)); // vertical average
            d16[iLocalPitch+2] = __builtin_bswap16((uint16_t)((ul32Avg + ul32Avg2)>>16)); // vertical average
            d16[iLocalPitch*2] = __builtin_bswap16((uint16_t)ul32B); // last line 1st
            d16[iLocalPitch*2+1] = __builtin_bswap16(u16Avg2); // middle pixel
            d16[iLocalPitch*2+2] = __builtin_bswap16((uint16_t)(ul32B >> 16)); // 3rd pixel
            u16Avg = (u16Avg & u16Magic) >> 1;
            u16Avg2 = (u16Avg2 & u16Magic) >> 1;
            d16[iLocalPitch+1] = __builtin_bswap16(u16Avg + u16Avg2); // middle pixel
            d16 += 3;
            s32 += 1;
        } // for i;
        d16 += iLocalPitch*2; // skip lines we already output
    } // for j
    spilcdSetPosition(pLCD, (x*3)/2, (y*3)/2, (iTileWidth*3)/2, (iTileHeight*3)/2, iFlags);
    myspiWrite(pLCD, (uint8_t *)pDMA, (iTileWidth*iTileHeight*9)/2, MODE_DATA, iFlags);
    return 0;
} /* spilcdDrawTile150() */

//
// Draw a 1-bpp pattern with the given color and translucency
// 1 bits are drawn as color, 0 are transparent
// The translucency value can range from 1 (barely visible) to 32 (fully opaque)
// If there is a backbuffer, the bitmap is draw only into memory
// If there is no backbuffer, the bitmap is drawn on the screen with a black background color
//
void spilcdDrawPattern(SPILCD *pLCD, uint8_t *pPattern, int iSrcPitch, int iDestX, int iDestY, int iCX, int iCY, uint16_t usColor, int iTranslucency)
{
    int x, y;
    uint8_t *s, uc, ucMask;
    uint16_t us, *d;
    uint32_t ulMask = 0x07e0f81f; // this allows all 3 values to be multipled at once
    uint32_t ulSrcClr, ulDestClr, ulDestTrans;
    
    ulDestTrans = 32-iTranslucency; // inverted to combine src+dest
    ulSrcClr = (usColor & 0xf81f) | ((uint32_t)(usColor & 0x06e0) << 16); // shift green to upper 16-bits
    ulSrcClr *= iTranslucency; // prepare for color blending
    if (iDestX+iCX > pLCD->iCurrentWidth) // trim to fit on display
        iCX = (pLCD->iCurrentWidth - iDestX);
    if (iDestY+iCY > pLCD->iCurrentHeight)
        iCY = (pLCD->iCurrentHeight - iDestY);
    if (pPattern == NULL || iDestX < 0 || iDestY < 0 || iCX <=0 || iCY <= 0 || iTranslucency < 1 || iTranslucency > 32)
        return;
    if (pLCD->pBackBuffer == NULL) // no back buffer, draw opaque colors
    {
      uint16_t u16Clr;
      u16Clr = (usColor >> 8) | (usColor << 8); // swap low/high bytes
      spilcdSetPosition(pLCD, iDestX, iDestY, iCX, iCY, DRAW_TO_LCD);
      for (y=0; y<iCY; y++)
      {
        s = &pPattern[y * iSrcPitch];
        ucMask = uc = 0;
        d = (uint16_t *)pDMA;
        for (x=0; x<iCX; x++)
        {
            ucMask >>= 1;
            if (ucMask == 0)
            {
                ucMask = 0x80;
                uc = *s++;
            }
            if (uc & ucMask) // active pixel
               *d++ = u16Clr;
            else
               *d++ = 0;
        } // for x
        myspiWrite(pLCD, (uint8_t *)pDMA, iCX*2, MODE_DATA, DRAW_TO_LCD);
      } // for y
      return;
    }
    for (y=0; y<iCY; y++)
    {
        int iDelta;
        iDelta = 1;
        d = (uint16_t *)&pLCD->pBackBuffer[((iDestY+y)*pLCD->iScreenPitch) + (iDestX*2)];
        s = &pPattern[y * iSrcPitch];
        ucMask = uc = 0;
        for (x=0; x<iCX; x++)
        {
            ucMask >>= 1;
            if (ucMask == 0)
            {
                ucMask = 0x80;
                uc = *s++;
            }
            if (uc & ucMask) // active pixel
            {
                us = d[0]; // read destination pixel
                us = (us >> 8) | (us << 8); // fix the byte order
                // The fast way to combine 2 RGB565 colors
                ulDestClr = (us & 0xf81f) | ((uint32_t)(us & 0x06e0) << 16);
                ulDestClr = (ulDestClr * ulDestTrans);
                ulDestClr += ulSrcClr; // combine old and new colors
                ulDestClr = (ulDestClr >> 5) & ulMask; // done!
                ulDestClr = (ulDestClr >> 16) | (ulDestClr); // move green back into place
                us = (uint16_t)ulDestClr;
                if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                    us = (us >> 8) | (us << 8); // swap bytes for LCD
                }
                d[0] = us;
            }
            d += iDelta;
        } // for x
    } // for y
    
}
void spilcdRectangle(SPILCD *pLCD, int x, int y, int w, int h, unsigned short usColor1, unsigned short usColor2, int bFill, int iFlags)
{
unsigned short *usTemp = (unsigned short *)pDMA;
int i, ty, th, iStart;
uint16_t usColor;
    
	// check bounds
	if (x < 0 || x >= pLCD->iCurrentWidth || x+w > pLCD->iCurrentWidth)
		return; // out of bounds
	if (y < 0 || y >= pLCD->iCurrentHeight || y+h > pLCD->iCurrentHeight)
		return;
	if (w <= 0 || h <= 0) return; // nothing to draw
	ty = y;
	th = h;
	if (bFill)
	{
        int32_t iDR, iDG, iDB; // current colors and deltas
        int32_t iRAcc, iGAcc, iBAcc;
        uint16_t usRInc, usGInc, usBInc;
        iRAcc = iGAcc = iBAcc = 0; // color fraction accumulators
        iDB = (int32_t)(usColor2 & 0x1f) - (int32_t)(usColor1 & 0x1f); // color deltas
        usBInc = (iDB < 0) ? 0xffff : 0x0001;
        iDB = abs(iDB);
        iDR = (int32_t)(usColor2 >> 11) - (int32_t)(usColor1 >> 11);
        usRInc = (iDR < 0) ? 0xf800 : 0x0800;
        iDR = abs(iDR);
        iDG = (int32_t)((usColor2 & 0x06e0) >> 5) - (int32_t)((usColor1 & 0x06e0) >> 5);
        usGInc = (iDG < 0) ? 0xffe0 : 0x0020;
        iDG = abs(iDG);
        iDB = (iDB << 16) / th;
        iDR = (iDR << 16) / th;
        iDG = (iDG << 16) / th;
        spilcdSetPosition(pLCD, x, y, w, h, iFlags);
	for (i=0; i<h; i++)
            {
                if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                    usColor = (usColor1 >> 8) | (usColor1 << 8); // swap byte order
                } else {
                    usColor = usColor1;
                }
                memset16((uint16_t*)usTemp, usColor, w);
                myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
                // Update the color components
                iRAcc += iDR;
                if (iRAcc >= 0x10000) // time to increment
                {
                    usColor1 += usRInc;
                    iRAcc -= 0x10000;
                }
                iGAcc += iDG;
                if (iGAcc >= 0x10000) // time to increment
                {
                    usColor1 += usGInc;
                    iGAcc -= 0x10000;
                }
                iBAcc += iDB;
                if (iBAcc >= 0x10000) // time to increment
                {
                    usColor1 += usBInc;
                    iBAcc -= 0x10000;
                }
            }
	}
	else // outline
	{
        if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
            usColor = (usColor1 >> 8) | (usColor1 << 8); // swap byte order
        } else {
            usColor = usColor1;
        }
		// draw top/bottom
	spilcdSetPosition(pLCD, x, y, w, 1, iFlags);
        memset16((uint16_t*)usTemp, usColor, w);
        myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
	spilcdSetPosition(pLCD, x, y + h-1, w, 1, iFlags);
        memset16((uint16_t*)usTemp, usColor, w);
	myspiWrite(pLCD, (unsigned char *)usTemp, w*2, MODE_DATA, iFlags);
		// draw left/right
		if (((ty + pLCD->iScrollOffset) % pLCD->iCurrentHeight) > pLCD->iCurrentHeight-th)
		{
			iStart = (pLCD->iCurrentHeight - ((ty+pLCD->iScrollOffset) % pLCD->iCurrentHeight));
			spilcdSetPosition(pLCD, x, y, 1, iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, iStart*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x+w-1, y, 1, iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, iStart*2, MODE_DATA, iFlags);
			// second half
			spilcdSetPosition(pLCD, x,y+iStart, 1, h-iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, h-iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, (h-iStart)*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x+w-1, y+iStart, 1, h-iStart, iFlags);
            memset16((uint16_t*)usTemp, usColor, h-iStart);
			myspiWrite(pLCD, (unsigned char *)usTemp, (h-iStart)*2, MODE_DATA, iFlags);
		}
		else // can do it in 1 shot
		{
			spilcdSetPosition(pLCD, x, y, 1, h, iFlags);
            memset16((uint16_t*)usTemp, usColor, h);
			myspiWrite(pLCD, (unsigned char *)usTemp, h*2, MODE_DATA, iFlags);
			spilcdSetPosition(pLCD, x + w-1, y, 1, h, iFlags);
            memset16((uint16_t*)usTemp, usColor, h);
			myspiWrite(pLCD, (unsigned char *)usTemp, h*2, MODE_DATA, iFlags);
		}
	} // outline
} /* spilcdRectangle() */

//
// Show part or all of the back buffer on the display
// Used after delayed rendering of graphics
//
void spilcdShowBuffer(SPILCD *pLCD, int iStartX, int iStartY, int cx, int cy, int iFlags)
{
    int y;
    uint8_t *s;
    
    if (pLCD->pBackBuffer == NULL)
        return; // nothing to do
    if (iStartX + cx > pLCD->iCurrentWidth || iStartY + cy > pLCD->iCurrentHeight || iStartX < 0 || iStartY < 0)
        return; // invalid area
    spilcdSetPosition(pLCD, iStartX, iStartY, cx, cy, iFlags);
    bSetPosition = 1;
#ifdef ARDUINO_ARCH_RP2040
    if (pLCD->pfnDataCallback == ParallelDataWrite) {
        // write the entire Tufty screen in a single transaction
        s = &pLCD->pBackBuffer[(iStartY * pLCD->iCurrentWidth * 2) + iStartX*2];
        myspiWrite(pLCD, s, cx * cy * 2, MODE_DATA, iFlags);
    } else
#endif
    {
        for (y=iStartY; y<iStartY+cy; y++)
        {
            s = &pLCD->pBackBuffer[(y * pLCD->iCurrentWidth * 2) + iStartX*2];
            myspiWrite(pLCD, s, cx * 2, MODE_DATA, iFlags);
        }
    }
    bSetPosition = 0;
} /* spilcdShowBuffer() */

//
// Sends a command to turn off the LCD display
// Turns off the backlight LED
// Closes the SPI file handle
//
void spilcdShutdown(SPILCD *pLCD)
{
	if (pLCD->iLCDType == LCD_SSD1351 || pLCD->iLCDType == LCD_SSD1331)
		spilcdWriteCommand(pLCD, 0xae); // Display Off
	else
		spilcdWriteCommand(pLCD, 0x28); // Display OFF
    myPinWrite(pLCD->iLEDPin, 0); // turn off the backlight
    spilcdFreeBackbuffer(pLCD);
#ifdef __LINUX__
	close(spi_fd);
	//AIORemoveGPIO(pLCD->iDCPin);
	//AIORemoveGPIO(pLCD->iResetPin);
	//AIORemoveGPIO(pLCD->iLEDPin);
#endif // __LINUX__
} /* spilcdShutdown() */
//
// Write a command byte followed by parameters
//
void spilcdWriteCmdParams(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pParams, int iLen)
{
#if defined ( ARDUINO_ARCH_ESP32 )// && !defined( CONFIG_IDF_TARGET_ESP32 )
    if (pLCD->pfnDataCallback) { // only ESP32-S2 and S3
        spilcdParallelCMDParams(ucCMD, pParams, iLen);
        return;
    }
#endif
    myspiWrite(pLCD, &ucCMD, 1, MODE_COMMAND, DRAW_TO_LCD);
    if (iLen) {
        myspiWrite(pLCD, pParams, iLen, MODE_DATA, DRAW_TO_LCD);
    }
} /* spilcdWriteCmdParams() */
//
// Send a command byte to the LCD controller
// In SPI 8-bit mode, the D/C line must be set
// high during the write
//
void spilcdWriteCommand(SPILCD *pLCD, unsigned char c)
{
unsigned char buf[2];
#if defined ( ARDUINO_ARCH_ESP32 ) //&& !defined ( CONFIG_IDF_TARGET_ESP32 )
    if (pLCD->pfnDataCallback) { // only ESP32-S2 and S3
        spilcdParallelCMDParams(c, NULL, 0);
        return;
    }
#endif
	buf[0] = c;
	myspiWrite(pLCD, buf, 1, MODE_COMMAND, DRAW_TO_LCD);
} /* spilcdWriteCommand() */

void spilcdWriteCommand16(SPILCD *pLCD, uint16_t us)
{
unsigned char buf[2];

    buf[0] = (uint8_t)(us >> 8);
    buf[1] = (uint8_t)us;
//    myspiWrite(pLCD, buf, 2, MODE_COMMAND, DRAW_TO_LCD);
    spilcdSetMode(pLCD, MODE_COMMAND);
    spilcdParallelData(buf, 2);
    spilcdSetMode(pLCD, MODE_DATA);
} /* spilcdWriteCommand() */

//
// Write a single byte of data
//
static void spilcdWriteData8(SPILCD *pLCD, unsigned char c)
{
unsigned char buf[2];

	buf[0] = c;
    myspiWrite(pLCD, buf, 1, MODE_DATA, DRAW_TO_LCD);

} /* spilcdWriteData8() */

//
// Write 16-bits of data
// The ILI9341 receives data in big-endian order
// (MSB first)
//
static void spilcdWriteData16(SPILCD *pLCD, unsigned short us, int iFlags)
{
unsigned char buf[2];

    buf[0] = (unsigned char)(us >> 8);
    buf[1] = (unsigned char)us;
    spilcdParallelData(buf, 2);
//    myspiWrite(pLCD, buf, 2, MODE_DATA, iFlags);

} /* spilcdWriteData16() */
//
// Set the text cursor position in pixels
//
void spilcdSetCursor(SPILCD *pLCD, int x, int y)
{
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
} /* spilcdSetCursor() */

//
// Position the "cursor" to the given
// row and column. The width and height of the memory
// 'window' must be specified as well. The controller
// allows more efficient writing of small blocks (e.g. tiles)
// by bounding the writes within a small area and automatically
// wrapping the address when reaching the end of the window
// on the curent row
//
void spilcdSetPosition(SPILCD *pLCD, int x, int y, int w, int h, int iFlags)
{
unsigned char ucBuf[8];
int iLen;
#ifdef LOG_OUTPUT
    Serial.printf("setPosition: %d, %d, %d, %d\n", x, y, w, h);
#endif
    pLCD->iWindowX = pLCD->iCurrentX = x; pLCD->iWindowY = pLCD->iCurrentY = y;
    pLCD->iWindowCX = w; pLCD->iWindowCY = h;
    pLCD->iOffset = (pLCD->iCurrentWidth * 2 * y) + (x * 2);

    if (!(iFlags & DRAW_TO_LCD) || pLCD->iLCDType == LCD_VIRTUAL_MEM) return; // nothing to do
    // Only ESP32 supports QSPI for now
#ifdef ARDUINO_ARCH_ESP32
    if (pLCD->iLCDType > LCD_QUAD_SPI) {
        qspiSetPosition(pLCD, x, y, w, h);
        return;
    }
#endif
    bSetPosition = 1; // flag to let myspiWrite know to ignore data writes
    y = (y + pLCD->iScrollOffset) % pLCD->iHeight; // scroll offset affects writing position

    if (pLCD->iCMDType == CMD_TYPE_ILITEK_16BIT) // 16-bit commands and data
    { // The display flipping bits don't change the address information, just
        // the horizontal and vertical inc/dec, so we must place the starting
        // address correctly for the 4 different orientations
        int xs=0, xb=0, xe=0, ys=0, yb= 0,ye=0;
        switch (pLCD->iOrientation)
        {
            case LCD_ORIENTATION_0:
                xs = xb = x; xe = (x + w - 1);
                ys = yb = y; ye = (y + h - 1);
                break;
            case LCD_ORIENTATION_90:
                yb = ys = x;
                ye = x + w - 1;
                xb = pLCD->iCurrentHeight - y - h;
                xe = xs = pLCD->iCurrentHeight - 1 - y;
                break;
            case LCD_ORIENTATION_180:
                xb = (pLCD->iCurrentWidth - x - w);
                xe = xs = (pLCD->iCurrentWidth - 1 - x);
                yb = (pLCD->iCurrentHeight - y - h);
                ye = ys = (pLCD->iCurrentHeight - 1 - y);
                break;
            case LCD_ORIENTATION_270:
                ye = ys = pLCD->iCurrentWidth - 1 - x;
                yb = pLCD->iCurrentWidth - x - w;
                xb = xs = y;
                xe = y + h - 1;
                break;
        }
        spilcdWriteCommand16(pLCD, 0x36); // Horizontal Address end
        spilcdWriteData16(pLCD, xe, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x37); // Horizontal Address start
        spilcdWriteData16(pLCD, xb, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x38); // Vertical Address end
        spilcdWriteData16(pLCD, ye, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x39); // Vertical Address start
        spilcdWriteData16(pLCD, yb, DRAW_TO_LCD);

        spilcdWriteCommand16(pLCD, 0x20); // Horizontal RAM address set
        spilcdWriteData16(pLCD, xs, DRAW_TO_LCD);
        spilcdWriteCommand16(pLCD, 0x21); // Vertical RAM address set
        spilcdWriteData16(pLCD, ys, DRAW_TO_LCD);

        spilcdWriteCommand16(pLCD, 0x22); // write to RAM
        bSetPosition = 0;
        return;
    }
	if (pLCD->iCMDType == CMD_TYPE_SOLOMON_OLED1) // OLED has very different commands
	{
		spilcdWriteCommand(pLCD, 0x15); // set column
		ucBuf[0] = x;
		ucBuf[1] = x + w - 1;
		myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
		spilcdWriteCommand(pLCD, 0x75); // set row
		ucBuf[0] = y;
		ucBuf[1] = y + h - 1;
		myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
		spilcdWriteCommand(pLCD, 0x5c); // write RAM
		bSetPosition = 0;
		return;
	}
        else if (pLCD->iCMDType == CMD_TYPE_SOLOMON_LCD) // so does the SSD1283A
        {
            switch (pLCD->iOrientation) {
                case LCD_ORIENTATION_0:
                    spilcdWriteCommand(pLCD, 0x44); // set col
                    ucBuf[0] = x + w - 1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45); // set row
                    ucBuf[0] = y + h - 1;
                    ucBuf[1] = y;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21); // set col+row
                    ucBuf[0] = y;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    break;
                case LCD_ORIENTATION_90:
                    spilcdWriteCommand(pLCD, 0x44); // set col
                    ucBuf[0] = y + h;
                    ucBuf[1] = y + 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45); // set row
                    ucBuf[0] = x + w -1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21); // set col+row
                    ucBuf[0] = x+1;
                    ucBuf[1] = y+1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    break;
                case LCD_ORIENTATION_180:
                    spilcdWriteCommand(pLCD, 0x44);
                    ucBuf[0] = pLCD->iCurrentWidth - x - 1;
                    ucBuf[1] = pLCD->iCurrentWidth - (x+w);
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45);
                    ucBuf[0] = pLCD->iCurrentHeight - y - 1;
                    ucBuf[1] = pLCD->iCurrentHeight - (y+h) - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21);
                    ucBuf[0] = pLCD->iCurrentHeight - y - 1;
                    ucBuf[1] = pLCD->iCurrentWidth - x - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                  break;
                case LCD_ORIENTATION_270:
                    spilcdWriteCommand(pLCD, 0x44);
                    ucBuf[0] = pLCD->iCurrentHeight - (y + h) - 1;
                    ucBuf[1] = pLCD->iCurrentHeight - y - 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x45);
                    ucBuf[0] = x + w - 1;
                    ucBuf[1] = x;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                    spilcdWriteCommand(pLCD, 0x21);
                    ucBuf[0] = x + 2;
                    ucBuf[1] = y + 1;
                    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
                  break;
            } // switch on orientation
            spilcdWriteCommand(pLCD, 0x22); // write RAM
            bSetPosition = 0;
            return;
        }
    else if (pLCD->iCMDType == CMD_TYPE_SOLOMON_OLED2)
    {
        spilcdWriteCommand(pLCD, 0x15);
        ucBuf[0] = x;
        ucBuf[1] = x + w - 1;
        myspiWrite(pLCD, ucBuf, 2, MODE_COMMAND, iFlags);

        spilcdWriteCommand(pLCD, 0x75);
        ucBuf[0] = y;
        ucBuf[1] = y + h - 1;
        myspiWrite(pLCD, ucBuf, 2, MODE_COMMAND, iFlags);

        bSetPosition = 0;
        return;
    }
    if (x != pLCD->iOldX || w != pLCD->iOldCX)
    {
        pLCD->iOldX = x; pLCD->iOldCX = w;
	if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
	{
		x += pLCD->iMemoryX;
		ucBuf[0] = (unsigned char)(x >> 8);
		ucBuf[1] = (unsigned char)x;
		x = x + w - 1;
//		if ((x-iMemoryX) > iWidth-1) x = iMemoryX + iWidth-1;
		ucBuf[2] = (unsigned char)(x >> 8);
		ucBuf[3] = (unsigned char)x;
        iLen = 4;
	}
	else // must be Sitronix 16-bit
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
 		ucBuf[1] = (unsigned char)(x >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)x;
		x = x + w -1;
		if (x > pLCD->iWidth-1) x = pLCD->iWidth-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(x >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)x;
        iLen = 8;
	}
        spilcdWriteCmdParams(pLCD, 0x2a, ucBuf, iLen); // set column address
    } // if X changed
    if (y != pLCD->iOldY || h != pLCD->iOldCY)
    {
        pLCD->iOldY = y; pLCD->iOldCY = h;
	if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
	{
                if (pLCD->iCurrentHeight == 135 && pLCD->iOrientation == LCD_ORIENTATION_90)
                   pLCD->iMemoryY+= 1; // ST7789 240x135 rotated 90 is off by 1
		y += pLCD->iMemoryY;
		ucBuf[0] = (unsigned char)(y >> 8);
		ucBuf[1] = (unsigned char)y;
		y = y + h;
		if ((y-pLCD->iMemoryY) > pLCD->iCurrentHeight-1) y = pLCD->iMemoryY + pLCD->iCurrentHeight;
		ucBuf[2] = (unsigned char)(y >> 8);
		ucBuf[3] = (unsigned char)y;
        iLen = 4;
                if (pLCD->iCurrentHeight == 135 && pLCD->iOrientation == LCD_ORIENTATION_90)
                   pLCD->iMemoryY -=1; // ST7789 240x135 rotated 90 is off by 1
	}
	else // must be Sitronix 16-bit
	{
// combine coordinates into 1 write to save time
		ucBuf[0] = 0;
		ucBuf[1] = (unsigned char)(y >> 8); // MSB first
		ucBuf[2] = 0;
		ucBuf[3] = (unsigned char)y;
		y = y + h - 1;
		if (y > pLCD->iHeight-1) y = pLCD->iHeight-1;
		ucBuf[4] = 0;
		ucBuf[5] = (unsigned char)(y >> 8);
		ucBuf[6] = 0;
		ucBuf[7] = (unsigned char)y;
        iLen = 8;
	}
        spilcdWriteCmdParams(pLCD, 0x2b, ucBuf, iLen); // set row address
    } // if Y changed
    if (!(pLCD->iLCDFlags & FLAGS_MEM_RESTART))
        spilcdWriteCommand(pLCD, 0x2c); // write memory begin
//	spilcdWriteCommand(0x3c); // write memory continue
    bSetPosition = 0;
} /* spilcdSetPosition() */

//
// Draw an individual RGB565 pixel
//
int spilcdSetPixel(SPILCD *pLCD, int x, int y, unsigned short usColor, int iFlags)
{
	spilcdSetPosition(pLCD, x, y, 1, 1, iFlags);
	spilcdWriteData16(pLCD, usColor, iFlags);
	return 0;
} /* spilcdSetPixel() */

uint8_t * spilcdGetDMABuffer(void)
{
  return (uint8_t *)pDMA;
}

#ifdef ARDUINO_ARCH_ESP32
//
// In Quad SPI mode, commands are 16-bits and send only on D0
// after a command "introducer"
// e.g. to send the MADCTR command (0x36)
// 0x02 0x00 = write, 0x03 0x00 = read
// send (on D0 only) 0x02, 0x00, 0x36, 0x00, then the parameters as single bytes
// For sending pixels, send command 0x32 0x00 0x2c/0x3c 0x00, followed by pixel data
//
void qspiSendCMD(SPILCD *pLCD, uint8_t u8CMD, uint8_t *pParams, int iLen)
{
    spi_transaction_t t;
    memset(&t, 0, sizeof(t));
    
    spilcdWaitDMA();
    t.flags = (SPI_TRANS_MULTILINE_CMD | SPI_TRANS_MULTILINE_ADDR);
    t.cmd = 0x02;
    t.addr = u8CMD << 8;
    if (iLen) {
        t.tx_buffer = pParams;
        t.length = 8 * iLen; // length in bits
    }
    spi_device_polling_transmit(spi, &t);
} /* qspiSendCMD() */

void qspiSendDATA(SPILCD *pLCD, uint8_t *pData, int iLen, int iFlags)
{
esp_err_t ret;

    spilcdWaitDMA();
    memset(&trans[0], 0, sizeof(trans[0]));
    
    trans[0].flags = SPI_TRANS_MODE_QIO;
    trans[0].cmd = 0x32;
    trans[0].addr = 0;
    if (bSetPosition) { // first data after a setPosition
        trans[0].addr = 0x2c00;
        bSetPosition = 0;
    } else { // all data after...
        trans[0].addr = 0x3c00;
    }
    trans[0].tx_buffer = pData;
    trans[0].length = iLen*8;
    if (iFlags & DRAW_WITH_DMA) {
        transfer_is_done = false;
        ret = spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
        assert (ret==ESP_OK);
    } else { // use polling
        spi_device_polling_transmit(spi, &trans[0]);
    }
} /* qspiSendDATA() */

void qspiSetPosition(SPILCD *pLCD, int x, int y, int w, int h)
{
    uint8_t u8Temp[8];
    int x2, y2;
    
    if (x != pLCD->iOldX || w != pLCD->iOldCX)
    {
        x2 = x + w - 1;
        pLCD->iOldX = x; pLCD->iOldCX = w;
        x += pLCD->iMemoryX;
        x2 += pLCD->iMemoryX;
        u8Temp[0] = (uint8_t)(x >> 8);
        u8Temp[1] = (uint8_t)x;
        u8Temp[2] = (uint8_t)(x2 >> 8);
        u8Temp[3] = (uint8_t)x2;
        qspiSendCMD(pLCD, 0x2a, u8Temp, 8); // set x1/x2
    }
    if (y != pLCD->iOldY || h != pLCD->iOldCY) {
        pLCD->iOldY = y; pLCD->iOldCY = h;
        y2 = y + h - 1;
        y += pLCD->iMemoryY;
        y2 += pLCD->iMemoryY;
        u8Temp[0] = (uint8_t)(y >> 8);
        u8Temp[1] = (uint8_t)y;
        u8Temp[2] = (uint8_t)(y2 >> 8);
        u8Temp[3] = (uint8_t)y2;
        qspiSendCMD(pLCD, 0x2b, u8Temp, 4); // set y1/y2
    }
    bSetPosition = 1;
} /* qspiSetPosition() */

void qspiSetBrightness(SPILCD *pLCD ,uint8_t u8Brightness)
{
    if (pLCD->iLEDPin < 99) {
#if (ESP_IDF_VERSION_MAJOR > 4)
       ledcWrite(pLCD->iLEDPin, u8Brightness); // PWM
#endif
    } else { // AMOLED display
       qspiSendCMD(pLCD, 0x51, &u8Brightness, 1);
    }
} /* qspiSetBrightness() */

void qspiRotate(SPILCD *pLCD, int iOrient)
{
    uint8_t u8Mad = 0;
    switch (iOrient) {
        case LCD_ORIENTATION_0:
            break;
        case LCD_ORIENTATION_90:
            u8Mad = 0x60;
            break;
        case LCD_ORIENTATION_180:
            u8Mad = 0xc0;
            break;
        case LCD_ORIENTATION_270:
            u8Mad = 0xa0;
            break;
    }
    qspiSendCMD(pLCD, 0x36, &u8Mad, 1);
} /* qspiRotate() */

void ST77916Init(SPILCD *pLCD)
{
int iCount;
uint8_t *s;

const uint8_t st77916_init[] = {
    2, 0xF0, 0x08,
    2, 0xF2, 0x08,
    2, 0x9B, 0x51,
    2, 0x86, 0x53,
    2, 0xF2, 0x80,
    2, 0xF0, 0x00,
    2, 0xF0, 0x01,
    2, 0xF1, 0x01,
    2, 0xB0, 0x54,
    2, 0xB1, 0x3F,
    2, 0xB2, 0x2A,
    2, 0xB4, 0x46,
    2, 0xB5, 0x34,
    2, 0xB6, 0xD5,
    2, 0xB7, 0x30,
    2, 0xBA, 0x00,
    2, 0xBB, 0x08,
    2, 0xBC, 0x08,
    2, 0xBD, 0x00,
    2, 0xC0, 0x80,
    2, 0xC1, 0x10,
    2, 0xC2, 0x37,
    2, 0xC3, 0x80,
    2, 0xC4, 0x10,
    2, 0xC5, 0x37,
    2, 0xC6, 0xA9,
    2, 0xC7, 0x41,
    2, 0xC8, 0x51,
    2, 0xC9, 0xA9,
    2, 0xCA, 0x41,
    2, 0xCB, 0x51,
    2, 0xD0, 0x91,
    2, 0xD1, 0x68,
    2, 0xD2, 0x69,
    3, 0xF5, 0x00, 0xA5,
    2, 0xDD, 0x3F,
    2, 0xDE, 0x3F,
    2, 0xF1, 0x10,
    2, 0xF0, 0x00,
    2, 0xF0, 0x02,
    15, 0xE0, 0x70, 0x09, 0x12, 0x0C, 0x0B, 0x27, 0x38, 0x54, 0x4E, 0x19, 0x15, 0x15, 0x2C, 0x2F,
    15, 0xE1, 0x70, 0x08, 0x11, 0x0C, 0x0B, 0x27, 0x38, 0x43, 0x4C, 0x18, 0x14, 0x14, 0x2B, 0x2D,
    2, 0xF0, 0x10,
    2, 0xF3, 0x10,
    2, 0xE0, 0x08,
    2, 0xE1, 0x00,
    2, 0xE2, 0x00,
    2, 0xE3, 0x00,
    2, 0xE4, 0xE0,
    2, 0xE5, 0x06,
    2, 0xE6, 0x21,
    2, 0xE7, 0x00,
    2, 0xE8, 0x05,
    2, 0xE9, 0x82,
    2, 0xEA, 0xDF,
    2, 0xEB, 0x89,
    2, 0xEC, 0x20,
    2, 0xED, 0x14,
    2, 0xEE, 0xFF,
    2, 0xEF, 0x00,
    2, 0xF8, 0xFF,
    2, 0xF9, 0x00,
    2, 0xFA, 0x00,
    2, 0xFB, 0x30,
    2, 0xFC, 0x00,
    2, 0xFD, 0x00,
    2, 0xFE, 0x00,
    2, 0xFF, 0x00,
    2, 0x60, 0x42,
    2, 0x61, 0xE0,
    2, 0x62, 0x40,
    2, 0x63, 0x40,
    2, 0x64, 0x02,
    2, 0x65, 0x00,
    2, 0x66, 0x40,
    2, 0x67, 0x03,
    2, 0x68, 0x00,
    2, 0x69, 0x00,
    2, 0x6A, 0x00,
    2, 0x6B, 0x00,
    2, 0x70, 0x42,
    2, 0x71, 0xE0,
    2, 0x72, 0x40,
    2, 0x73, 0x40,
    2, 0x74, 0x02,
    2, 0x75, 0x00,
    2, 0x76, 0x40,
    2, 0x77, 0x03,
    2, 0x78, 0x00,
    2, 0x79, 0x00,
    2, 0x7A, 0x00,
    2, 0x7B, 0x00,
    2, 0x80, 0x48,
    2, 0x81, 0x00,
    2, 0x82, 0x05,
    2, 0x83, 0x02,
    2, 0x84, 0xDD,
    2, 0x85, 0x00,
    2, 0x86, 0x00,
    2, 0x87, 0x00,
    2, 0x88, 0x48,
    2, 0x89, 0x00,
    2, 0x8A, 0x07,
    2, 0x8B, 0x02,
    2, 0x8C, 0xDF,
    2, 0x8D, 0x00,
    2, 0x8E, 0x00,
    2, 0x8F, 0x00,
    2, 0x90, 0x48,
    2, 0x91, 0x00,
    2, 0x92, 0x09,
    2, 0x93, 0x02,
    2, 0x94, 0xE1,
    2, 0x95, 0x00,
    2, 0x96, 0x00,
    2, 0x97, 0x00,
    2, 0x98, 0x48,
    2, 0x99, 0x00,
    2, 0x9A, 0x0B,
    2, 0x9B, 0x02,
    2, 0x9C, 0xE3,
    2, 0x9D, 0x00,
    2, 0x9E, 0x00,
    2, 0x9F, 0x00,
    2, 0xA0, 0x48,
    2, 0xA1, 0x00,
    2, 0xA2, 0x04,
    2, 0xA3, 0x02,
    2, 0xA4, 0xDC,
    2, 0xA5, 0x00,
    2, 0xA6, 0x00,
    2, 0xA7, 0x00,
    2, 0xA8, 0x48,
    2, 0xA9, 0x00,
    2, 0xAA, 0x06,
    2, 0xAB, 0x02,
    2, 0xAC, 0xDE,
    2, 0xAD, 0x00,
    2, 0xAE, 0x00,
    2, 0xAF, 0x00,
    2, 0xB0, 0x48,
    2, 0xB1, 0x00,
    2, 0xB2, 0x08,
    2, 0xB3, 0x02,
    2, 0xB4, 0xE0,
    2, 0xB5, 0x00,
    2, 0xB6, 0x00,
    2, 0xB7, 0x00,
    2, 0xB8, 0x48,
    2, 0xB9, 0x00,
    2, 0xBA, 0x0A,
    2, 0xBB, 0x02,
    2, 0xBC, 0xE2,
    2, 0xBD, 0x00,
    2, 0xBE, 0x00,
    2, 0xBF, 0x00,
    2, 0xC0, 0x12,
    2, 0xC1, 0xAA,
    2, 0xC2, 0x65,
    2, 0xC3, 0x74,
    2, 0xC4, 0x47,
    2, 0xC5, 0x56,
    2, 0xC6, 0x00,
    2, 0xC7, 0x88,
    2, 0xC8, 0x99,
    2, 0xC9, 0x33,
    2, 0xD0, 0x21,
    2, 0xD1, 0xAA,
    2, 0xD2, 0x65,
    2, 0xD3, 0x74,
    2, 0xD4, 0x47,
    2, 0xD5, 0x56,
    2, 0xD6, 0x00,
    2, 0xD7, 0x88,
    2, 0xD8, 0x99,
    2, 0xD9, 0x33,
    2, 0xF3, 0x01,
    2, 0xF0, 0x00,
    2, 0xF0, 0x01,
    2, 0xF1, 0x01,
    2, 0xA0, 0x0B,
    2, 0xA3, 0x2A,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x2B,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x2C,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x2D,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x2E,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x2F,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x30,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x31,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x32,
    2, 0xA5, 0xC3,
    LCD_DELAY, 1,
    2, 0xA3, 0x33,
    2, 0xA5, 0xC3,
    2, 0xA0, 0x09,
    2, 0xF1, 0x10,
    2, 0xF0, 0x00,
    5, 0x2A, 0x00, 0x00, 0x01, 0x67,
    5, 0x2B, 0x01, 0x68, 0x01, 0x68,
    2, 0x4D, 0x00,
    2, 0x4E, 0x00,
    2, 0x4F, 0x00,
    2, 0x4C, 0x01,
    LCD_DELAY, 10,
    2, 0x4C, 0x00,
    5, 0x2A, 0x00, 0x00, 0x01, 0x67,
    5, 0x2B, 0x00, 0x00, 0x01, 0x67,
    2, 0x21, 0x00,
    2, 0x3A, 0x55, // color=16
    2, 0x11, 0x00,
    LCD_DELAY, 120,
    2, 0x29, 0x00,
    0
};
    pLCD->iCurrentWidth = pLCD->iWidth = 360;
    pLCD->iCurrentHeight = pLCD->iHeight = 360;
    iCount = 1;
    s = (uint8_t *)st77916_init;
    while (iCount) {
        iCount = *s++;
        if (iCount == LCD_DELAY) {
           delay(*s++);
        } else {
           qspiSendCMD(pLCD, s[0], &s[1], iCount-1);
           s += iCount;
        }
    }
} /* ST77916Init() */

void NV3041AInit(SPILCD *pLCD)
{

DRAM_ATTR static const uint8_t nv3041a_init[] = {
    0xff, 0xa5,
    //0xE7, 0x10,
    0x35, 0x00,
    0x36, 0xd0,
    0x3A, 0x01, // 01---565，00---666
    0x40, 0x01,
    0x41, 0x03, // 01--8bit, 03-16bit
    0x44, 0x15,
    0x45, 0x15,
    0x7d, 0x03,
    0xc1, 0xbb,
    0xc2, 0x05,
    0xc3, 0x10,
    0xc6, 0x3e,
    0xc7, 0x25,
    0xc8, 0x11,
    0x7a, 0x5f,
    0x6f, 0x44,
    0x78, 0x70,
    0xc9, 0x00,
    0x67, 0x21,

    0x51, 0x0a,
    0x52, 0x76,
    0x53, 0x0a,
    0x54, 0x76,

    0x46, 0x0a,
    0x47, 0x2a,
    0x48, 0x0a,
    0x49, 0x1a,
    0x56, 0x43,
    0x57, 0x42,
    0x58, 0x3c,
    0x59, 0x64,
    0x5a, 0x41,
    0x5b, 0x3c,
    0x5c, 0x02,
    0x5d, 0x3c,
    0x5e, 0x1f,
    0x60, 0x80,
    0x61, 0x3f,
    0x62, 0x21,
    0x63, 0x07,
    0x64, 0xe0,
    0x65, 0x02,
    0xca, 0x20,
    0xcb, 0x52,
    0xcc, 0x10,
    0xcD, 0x42,

    0xD0, 0x20,
    0xD1, 0x52,
    0xD2, 0x10,
    0xD3, 0x42,
    0xD4, 0x0a,
    0xD5, 0x32,

    0xf8, 0x03,
    0xf9, 0x20,

    0x80, 0x00,
    0xA0, 0x00,

    0x81, 0x07,
    0xA1, 0x06,

    0x82, 0x02,
    0xA2, 0x01,

    0x86, 0x11,
    0xA6, 0x10,

    0x87, 0x27,
    0xA7, 0x27,

    0x83, 0x37,
    0xA3, 0x37,

    0x84, 0x35,
    0xA4, 0x35,

    0x85, 0x3f,
    0xA5, 0x3f,

    0x88, 0x0b,
    0xA8, 0x0b,

    0x89, 0x14,
    0xA9, 0x14,

    0x8a, 0x1a,
    0xAa, 0x1a,

    0x8b, 0x0a,
    0xAb, 0x0a,

    0x8c, 0x14,
    0xAc, 0x08,

    0x8d, 0x17,
    0xAd, 0x07,

    0x8e, 0x16,
    0xAe, 0x06,

    0x8f, 0x1B,
    0xAf, 0x07,

    0x90, 0x04,
    0xB0, 0x04,

    0x91, 0x0A,
    0xB1, 0x0A,

    0x92, 0x16,
    0xB2, 0x15,

    0xff, 0x00,
};
static const uint8_t u8ScrollInit[4] = {0,0,1,16};

    pLCD->iCurrentWidth = pLCD->iWidth = 480;
    pLCD->iCurrentHeight = pLCD->iHeight = 272;
    for (int i=0; i<sizeof(nv3041a_init); i += 2) {
        qspiSendCMD(pLCD, nv3041a_init[i], (uint8_t*)&nv3041a_init[i+1], 1);
    }
    qspiSendCMD(pLCD, 0x20, NULL, 0); // invert off
    qspiSendCMD(pLCD, 0x11, NULL, 0); // sleep out
    delay(200);
    qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
    // set up whole screen scrolling as the default
    qspiSendCMD(pLCD, 0x33, (uint8_t *)u8ScrollInit, 4); // initialize scrolling
    qspiSendCMD(pLCD, 0x37, (uint8_t *)u8ScrollInit, 2); // 0 offset to start    
} /* NV3041AInit() */

void AXS15231Init(SPILCD *pLCD, int iSubType)
{
    uint8_t u8Temp[4];
    
    if (iSubType == 0) {
        pLCD->iCurrentWidth = pLCD->iWidth = 180;
        pLCD->iCurrentHeight = pLCD->iHeight = 640;
    } else {
        pLCD->iCurrentWidth = pLCD->iWidth = 320;
        pLCD->iCurrentHeight = pLCD->iHeight = 480;
    }
    if (iSubType == 1) {
        // exit sleep mode first
        qspiSendCMD(pLCD, 0x11, u8Temp, 0);
        delay(200);
        u8Temp[0] = 0x05; // color mode (RGB565)
        qspiSendCMD(pLCD, 0x3a, u8Temp, 1);
        u8Temp[0] = 0x0; // madctl
        qspiSendCMD(pLCD, 0x36, u8Temp, 1);
    } else {
        qspiSendCMD(pLCD, 0x28, NULL, 0);
        delay(20);
        qspiSendCMD(pLCD, 0x10, NULL, 0);
        delay(5);
        qspiSendCMD(pLCD, 0x11, NULL, 0);
        delay(200);
        qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
        u8Temp[0] = 0x0; //0xa0; // can't set 90/270
        qspiSendCMD(pLCD, 0x36, u8Temp, 1);
    }
} /* AXS15231Init() */

void ICNA3311Init(SPILCD *pLCD)
{
    uint8_t u8Temp[4];
        
    pLCD->iCurrentWidth = pLCD->iWidth = 280;
    pLCD->iCurrentHeight = pLCD->iHeight = 456;
    pLCD->iMemoryX = 20;

    qspiSendCMD(pLCD, 0x11, NULL, 0); // sleep out
    delay(120);
    u8Temp[0] = 0x00;
    qspiSendCMD(pLCD, 0xfe, u8Temp, 1);
    u8Temp[0] = 0x80; // SPI mode control
    qspiSendCMD(pLCD, 0xc4, u8Temp, 1);
    u8Temp[0] = 0x55; // pixel format
    qspiSendCMD(pLCD, 0x3a, u8Temp, 1);

    u8Temp[0] = 0x20;
    qspiSendCMD(pLCD, 0x53, u8Temp, 1); // write ctrl 1

    u8Temp[0] = 0xff;
    qspiSendCMD(pLCD, 0x63, u8Temp, 1); // brightness in HBM mode

    u8Temp[0] = 0; u8Temp[1] = 0x14; u8Temp[2] = 0x1; u8Temp[3] = 0x2b;
    qspiSendCMD(pLCD, 0x2a, u8Temp, 4); // CASET

    u8Temp[0] = 0; u8Temp[1] = 0; u8Temp[2] = 1; u8Temp[3] = 0xc7;
    qspiSendCMD(pLCD, 0x2b, u8Temp, 4); // PASET

    qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
    u8Temp[0] = 0xd0; // display brightness (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);
    u8Temp[0] = 0x0; // sunlight mode off
    qspiSendCMD(pLCD, 0x58, u8Temp, 1);
    delay(10);

    qspiSendCMD(pLCD, 0x20, NULL, 0); // inversion off
} /* ICNA3311Init() */

void RM67162Init(SPILCD *pLCD)
{
    uint8_t u8Temp[4];
    
    pLCD->iCurrentWidth = pLCD->iWidth = 240;
    pLCD->iCurrentHeight = pLCD->iHeight = 536;
    qspiSendCMD(pLCD, 0x11, NULL, 0); // sleep out
    delay(20);
    u8Temp[0] = 0x55;
    qspiSendCMD(pLCD, 0x3a, u8Temp, 1); // pixel format
    u8Temp[0] = 1; // display brightness dark (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);
    qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
    delay(200);
    u8Temp[0] = 0xd0; // display brightness (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);
} /* RM67162Init() */

void SH8601Init(SPILCD *pLCD)
{
    uint8_t u8Temp[4];

    pLCD->iCurrentWidth = pLCD->iWidth = 368;
    pLCD->iCurrentHeight = pLCD->iHeight = 448;
    qspiSendCMD(pLCD, 0x11, NULL, 0); // sleep out
    delay(120);
    qspiSendCMD(pLCD, 0x13, NULL, 0); // normal display mode on
    qspiSendCMD(pLCD, 0x20, NULL, 0); // inversion off
    u8Temp[0] = 0x05;
    qspiSendCMD(pLCD, 0x3a, u8Temp, 1); // pixel format
    qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
    u8Temp[0] = 0x28;
    qspiSendCMD(pLCD, 0x53, u8Temp, 1); // brightness control on and display dimming on
    u8Temp[0] = 0;
    qspiSendCMD(pLCD, 0x58, u8Temp, 1); // high contrast mode off
    u8Temp[0] = 0xd0; // display brightness (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);
} /* SH8601Init() */

const uint8_t spd2010_init[] = {
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x0c, 0x11,
    2, 0x10, 0x02,
    2, 0x11, 0x11,
    2, 0x15, 0x42,
    2, 0x16, 0x11,
    2, 0x1a, 0x02,
    2, 0x1b, 0x11,
    2, 0x61, 0x80,
    2, 0x62, 0x80,
    2, 0x54, 0x44,
    2, 0x58, 0x88,
    2, 0x5c, 0xcc,
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x20, 0x80,
    2, 0x21, 0x81,
    2, 0x22, 0x31,
    2, 0x23, 0x20,
    2, 0x24, 0x11,
    2, 0x25, 0x11,
    2, 0x26, 0x12,
    2, 0x27, 0x12,
    2, 0x30, 0x80,
    2, 0x31, 0x81,
    2, 0x32, 0x31,
    2, 0x33, 0x20,
    2, 0x34, 0x11,
    2, 0x35, 0x11,
    2, 0x36, 0x12,
    2, 0x37, 0x12,
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x41, 0x11,
    2, 0x42, 0x22,
    2, 0x43, 0x33,
    2, 0x49, 0x11,
    2, 0x4a, 0x22,
    2, 0x4b, 0x33,
    4, 0xff, 0x20, 0x10, 0x15,
    2, 0x00, 0x00,
    2, 0x01, 0x00,
    2, 0x02, 0x00,
    2, 0x03, 0x00,
    2, 0x04, 0x10,
    2, 0x05, 0x0c,
    2, 0x06, 0x23,
    2, 0x07, 0x22,
    2, 0x08, 0x21,
    2, 0x09, 0x20,
    2, 0x0a, 0x33,
    2, 0x0b, 0x32,
    2, 0x0c, 0x34,
    2, 0x0d, 0x35,
    2, 0x0e, 0x01,
    2, 0x0f, 0x01,
    2, 0x20, 0x00,
    2, 0x21, 0x00,
    2, 0x22, 0x00,
    2, 0x23, 0x00,
    2, 0x24, 0x0c,
    2, 0x25, 0x10,
    2, 0x26, 0x20,
    2, 0x27, 0x21,
    2, 0x28, 0x22,
    2, 0x29, 0x23,
    2, 0x2a, 0x33,
    2, 0x2b, 0x32,
    2, 0x2c, 0x34,
    2, 0x2d, 0x35,
    2, 0x2e, 0x01,
    2, 0x2f, 0x01,
    4, 0xff, 0x20, 0x10, 0x16,
    2, 0x00, 0x00,
    2, 0x01, 0x00,
    2, 0x02, 0x00,
    2, 0x03, 0x00,
    2, 0x04, 0x08,
    2, 0x05, 0x04,
    2, 0x06, 0x19,
    2, 0x07, 0x18,
    2, 0x08, 0x17,
    2, 0x09, 0x16,
    2, 0x0a, 0x33,
    2, 0x0b, 0x32,
    2, 0x0c, 0x34,
    2, 0x0d, 0x35,
    2, 0x0e, 0x01,
    2, 0x0f, 0x01,
    2, 0x20, 0x00,
    2, 0x21, 0x00,
    2, 0x22, 0x00,
    2, 0x23, 0x00,
    2, 0x24, 0x04,
    2, 0x25, 0x08,
    2, 0x26, 0x16,
    2, 0x27, 0x17,
    2, 0x28, 0x18,
    2, 0x29, 0x19,
    2, 0x2a, 0x33,
    2, 0x2b, 0x32,
    2, 0x2c, 0x34,
    2, 0x2d, 0x35,
    2, 0x2e, 0x01,
    2, 0x2f, 0x01,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x00, 0x99,
    2, 0x2a, 0x28,
    2, 0x2b, 0x0f,
    2, 0x2c, 0x16,
    2, 0x2d, 0x28,
    2, 0x2e, 0x0f,
    4, 0xff, 0x20, 0x10, 0xa0,
    2, 0x08, 0xdc,
    4, 0xff, 0x20, 0x10, 0x45,
    2, 0x01, 0x9c,
    2, 0x03, 0x9c,
    4, 0xff, 0x20, 0x10, 0x42,
    2, 0x05, 0x2c,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x50, 0x01,
    4, 0xff, 0x20, 0x10, 0x00,
    5, 0x2a, 0x00, 0x00, 0x01, 0x9b,
    5, 0x2b, 0x00, 0x00, 0x01, 0x9b,
    4, 0xff, 0x20, 0x10, 0x40,
    2, 0x86, 0x00,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x0d, 0x66,
    4, 0xff, 0x20, 0x10, 0x17,
    2, 0x39, 0x3c,
    4, 0xff, 0x20, 0x10, 0x31,
    2, 0x38, 0x03,
    2, 0x39, 0xf0,
    2, 0x36, 0x03,
    2, 0x37, 0xe8,
    2, 0x34, 0x03,
    2, 0x35, 0xcf,
    2, 0x32, 0x03,
    2, 0x33, 0xba,
    2, 0x30, 0x03,
    2, 0x31, 0xa2,
    2, 0x2e, 0x03,
    2, 0x2f, 0x95,
    2, 0x2c, 0x03,
    2, 0x2d, 0x7e,
    2, 0x2a, 0x03,
    2, 0x2b, 0x62,
    2, 0x28, 0x03,
    2, 0x29, 0x44,
    2, 0x26, 0x02,
    2, 0x27, 0xfc,
    2, 0x24, 0x02,
    2, 0x25, 0xd0,
    2, 0x22, 0x02,
    2, 0x23, 0x98,
    2, 0x20, 0x02,
    2, 0x21, 0x6f,
    2, 0x1e, 0x02,
    2, 0x1f, 0x32,
    2, 0x1c, 0x01,
    2, 0x1d, 0xf6,
    2, 0x1a, 0x01,
    2, 0x1b, 0xb8,
    2, 0x18, 0x01,
    2, 0x19, 0x6e,
    2, 0x16, 0x01,
    2, 0x17, 0x41,
    2, 0x14, 0x00,
    2, 0x15, 0xfd,
    2, 0x12, 0x00,
    2, 0x13, 0xcf,
    2, 0x10, 0x00,
    2, 0x11, 0x98,
    2, 0x0e, 0x00,
    2, 0x0f, 0x89,
    2, 0x0c, 0x00,
    2, 0x0d, 0x79,
    2, 0x0a, 0x00,
    2, 0x0b, 0x67,
    2, 0x08, 0x00,
    2, 0x09, 0x55,
    2, 0x06, 0x00,
    2, 0x07, 0x3f,
    2, 0x04, 0x00,
    2, 0x05, 0x28,
    2, 0x02, 0x00,
    2, 0x03, 0x0e,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x32,
    2, 0x38, 0x03,
    2, 0x39, 0xf0,
    2, 0x36, 0x03,
    2, 0x37, 0xe8,
    2, 0x34, 0x03,
    2, 0x35, 0xcf,
    2, 0x32, 0x03,
    2, 0x33, 0xba,
    2, 0x30, 0x03,
    2, 0x31, 0xa2,
    2, 0x2e, 0x03,
    2, 0x2f, 0x95,
    2, 0x2c, 0x03,
    2, 0x2d, 0x7e,
    2, 0x2a, 0x03,
    2, 0x2b, 0x62,
    2, 0x28, 0x03,
    2, 0x29, 0x44,
    2, 0x26, 0x02,
    2, 0x27, 0xfc,
    2, 0x24, 0x02,
    2, 0x25, 0xd0,
    2, 0x22, 0x02,
    2, 0x23, 0x98,
    2, 0x20, 0x02,
    2, 0x21, 0x6f,
    2, 0x1e, 0x02,
    2, 0x1f, 0x32,
    2, 0x1c, 0x01,
    2, 0x1d, 0xf6,
    2, 0x1a, 0x01,
    2, 0x1b, 0xb8,
    2, 0x18, 0x01,
    2, 0x19, 0x6e,
    2, 0x16, 0x01,
    2, 0x17, 0x41,
    2, 0x14, 0x00,
    2, 0x15, 0xfd,
    2, 0x12, 0x00,
    2, 0x13, 0xcf,
    2, 0x10, 0x00,
    2, 0x11, 0x98,
    2, 0x0e, 0x00,
    2, 0x0f, 0x89,
    2, 0x0c, 0x00,
    2, 0x0d, 0x79,
    2, 0x0a, 0x00,
    2, 0x0b, 0x67,
    2, 0x08, 0x00,
    2, 0x09, 0x55,
    2, 0x06, 0x00,
    2, 0x07, 0x3f,
    2, 0x04, 0x00,
    2, 0x05, 0x28,
    2, 0x02, 0x00,
    2, 0x03, 0x0e,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x60, 0x01,
    2, 0x65, 0x03,
    2, 0x66, 0x38,
    2, 0x67, 0x04,
    2, 0x68, 0x34,
    2, 0x69, 0x03,
    2, 0x61, 0x03,
    2, 0x62, 0x38,
    2, 0x63, 0x04,
    2, 0x64, 0x34,
    2, 0x0a, 0x11,
    2, 0x0b, 0x20,
    2, 0x0c, 0x20,
    2, 0x55, 0x06,
    4, 0xff, 0x20, 0x10, 0x42,
    2, 0x05, 0x3d,
    2, 0x06, 0x03,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x1f, 0xdc,
    4, 0xff, 0x20, 0x10, 0x17,
    2, 0x11, 0xaa,
    2, 0x16, 0x12,
    2, 0x0b, 0xc3,
    2, 0x10, 0x0e,
    2, 0x14, 0xaa,
    2, 0x18, 0xa0,
    2, 0x1a, 0x80,
    2, 0x1f, 0x80,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x30, 0xee,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x15, 0x0f,
    4, 0xff, 0x20, 0x10, 0x2d,
    2, 0x01, 0x3e,
    4, 0xff, 0x20, 0x10, 0x40,
    2, 0x83, 0xc4,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x00, 0xcc,
    2, 0x36, 0xa0,
    2, 0x2a, 0x2d,
    2, 0x2b, 0x1e,
    2, 0x2c, 0x26,
    2, 0x2d, 0x2d,
    2, 0x2e, 0x1e,
    2, 0x1f, 0xe6,
    4, 0xff, 0x20, 0x10, 0xa0,
    2, 0x08, 0xe6,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x10, 0x0f,
    4, 0xff, 0x20, 0x10, 0x18,
    2, 0x01, 0x01,
    2, 0x00, 0x1e,
    4, 0xff, 0x20, 0x10, 0x43,
    2, 0x03, 0x04,
    4, 0xff, 0x20, 0x10, 0x18,
    2, 0x3a, 0x01,
    4, 0xff, 0x20, 0x10, 0x50,
    2, 0x05, 0x08,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x50,
    2, 0x00, 0xa6,
    2, 0x01, 0xa6,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x50,
    2, 0x08, 0x55,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x0b, 0x43,
    2, 0x0c, 0x12,
    2, 0x10, 0x01,
    2, 0x11, 0x12,
    2, 0x15, 0x00,
    2, 0x16, 0x00,
    2, 0x1a, 0x00,
    2, 0x1b, 0x00,
    2, 0x61, 0x00,
    2, 0x62, 0x00,
    2, 0x51, 0x11,
    2, 0x55, 0x55,
    2, 0x58, 0x00,
    2, 0x5c, 0x00,
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x20, 0x81,
    2, 0x21, 0x82,
    2, 0x22, 0x72,
    2, 0x30, 0x00,
    2, 0x31, 0x00,
    2, 0x32, 0x00,
    4, 0xff, 0x20, 0x10, 0x10,
    2, 0x44, 0x44,
    2, 0x45, 0x55,
    2, 0x46, 0x66,
    2, 0x47, 0x77,
    2, 0x49, 0x00,
    2, 0x4a, 0x00,
    2, 0x4b, 0x00,
    4, 0xff, 0x20, 0x10, 0x17,
    2, 0x37, 0x00,
    4, 0xff, 0x20, 0x10, 0x15,
    2, 0x04, 0x08,
    2, 0x05, 0x04,
    2, 0x06, 0x1c,
    2, 0x07, 0x1a,
    2, 0x08, 0x18,
    2, 0x09, 0x16,
    2, 0x24, 0x05,
    2, 0x25, 0x09,
    2, 0x26, 0x17,
    2, 0x27, 0x19,
    2, 0x28, 0x1b,
    2, 0x29, 0x1d,
    4, 0xff, 0x20, 0x10, 0x16,
    2, 0x04, 0x09,
    2, 0x05, 0x05,
    2, 0x06, 0x1d,
    2, 0x07, 0x1b,
    2, 0x08, 0x19,
    2, 0x09, 0x17,
    2, 0x24, 0x04,
    2, 0x25, 0x08,
    2, 0x26, 0x16,
    2, 0x27, 0x18,
    2, 0x28, 0x1a,
    2, 0x29, 0x1c,
    4, 0xff, 0x20, 0x10, 0x18,
    2, 0x1f, 0x02,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x15, 0x99,
    2, 0x16, 0x99,
    2, 0x1c, 0x88,
    2, 0x1d, 0x88,
    2, 0x1e, 0x88,
    2, 0x13, 0xf0,
    2, 0x14, 0x34,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x12, 0x89,
    2, 0x06, 0x06,
    2, 0x18, 0x00,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x0a, 0x00,
    2, 0x0b, 0xf0,
    2, 0x0c, 0xf0,
    2, 0x6a, 0x10,
    4, 0xff, 0x20, 0x10, 0x00,
    4, 0xff, 0x20, 0x10, 0x11,
    2, 0x08, 0x70,
    2, 0x09, 0x00,
    4, 0xff, 0x20, 0x10, 0x00,
    2, 0x35, 0x00,
    4, 0xff, 0x20, 0x10, 0x12,
    2, 0x21, 0x70,
    4, 0xff, 0x20, 0x10, 0x2d,
    2, 0x02, 0x00,
    4, 0xff, 0x20, 0x10, 0x00,
    2, 0x11, 0x00,
    LCD_DELAY, 120,
    0
};
void SPD2010Init(SPILCD *pLCD)
{
    int iCount;
    uint8_t *s, u8Temp[4];

    pLCD->iCurrentWidth = pLCD->iWidth = 412;
    pLCD->iCurrentHeight = pLCD->iHeight = 412;
    iCount = 1;
    s = (uint8_t *)spd2010_init;
    while (iCount) {
        iCount = *s++;
        if (iCount == LCD_DELAY) {
           delay(*s++);
        } else {
           qspiSendCMD(pLCD, s[0], &s[1], iCount-1);
           s += iCount;
        }
    }
    u8Temp[0] = 0x55;
    qspiSendCMD(pLCD, 0x3a, u8Temp, 1); // pixel format
    u8Temp[0] = 1; // display brightness dark (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);
    qspiSendCMD(pLCD, 0x29, NULL, 0); // display on
    delay(200);
    u8Temp[0] = 0xd0; // display brightness (max = 0xff)
    qspiSendCMD(pLCD, 0x51, u8Temp, 1);

} /* SPD2010Init() */

// Initialize a Quad SPI display
int qspiInit(SPILCD *pLCD, int iLCDType, int iFLAGS, uint32_t u32Freq, uint8_t u8CS, uint8_t u8CLK, uint8_t u8D0, uint8_t u8D1, uint8_t u8D2, uint8_t u8D3, uint8_t u8RST, uint8_t u8LED)
{
    esp_err_t ret;
    
    if (u8CS != 0xff) {
        pinMode(u8CS, OUTPUT);
        digitalWrite(u8CS, HIGH);
    }
    if (u8LED != 0xff) {
#if (ESP_IDF_VERSION_MAJOR > 4)
        ledcAttach(u8LED, 5000, 8); // attach pin to channel 0
        ledcWrite(u8LED, 192); // set to moderate brightness to begin
#else
        pinMode(u8LED, OUTPUT);
        digitalWrite(u8LED, HIGH); // turn on backlight
#endif
    }
    pLCD->iLEDPin = u8LED;
    pLCD->iCSPin = u8CS;
    pLCD->iLCDType = iLCDType;
    if (u8RST != 0xff) { // has a reset GPIO?
      pinMode(u8RST, OUTPUT);
      digitalWrite(u8RST, HIGH);
      delay(130);
      digitalWrite(u8RST, LOW);
      delay(130);
      digitalWrite(u8RST, HIGH);
      delay(300);
    }

    spi_bus_config_t buscfg = {
        .data0_io_num = u8D0,
        .data1_io_num = u8D1,
        .sclk_io_num = u8CLK,
        .data2_io_num = u8D2,
        .data3_io_num = u8D3,
        .max_transfer_sz = 8192, //(14400 * 16) + 8,
        .flags = SPICOMMON_BUSFLAG_MASTER | SPICOMMON_BUSFLAG_GPIO_PINS /* |
                 SPICOMMON_BUSFLAG_QUAD */
        ,
    };
    spi_device_interface_config_t devcfg = {
        .command_bits = 8,
        .address_bits = 24,
        .mode = SPI_MODE0,
        .clock_speed_hz = (int)u32Freq,
        //.spics_io_num = -1,
        .spics_io_num = u8CS,
        .flags = SPI_DEVICE_HALFDUPLEX,
        .queue_size = 17,
        .post_cb = spi_post_transfer_callback,
    };
    if (u8CS == 0xff) devcfg.spics_io_num = -1;
    ret = spi_bus_initialize(SPI2_HOST, &buscfg, SPI_DMA_CH_AUTO);
    ESP_ERROR_CHECK(ret);
    ret = spi_bus_add_device(SPI2_HOST, &devcfg, &spi);
    ESP_ERROR_CHECK(ret);
    
    switch (iLCDType) {
        case LCD_SPD2010:
            SPD2010Init(pLCD);
            break;
        case LCD_SH8601:
            SH8601Init(pLCD);
            break;
        case LCD_ICNA3311:
            ICNA3311Init(pLCD);
            break;
        case LCD_AXS15231:
            AXS15231Init(pLCD, 1);
            break;
        case LCD_AXS15231B:
            AXS15231Init(pLCD, 0);
            break;
        case LCD_RM67162:
            RM67162Init(pLCD);
            break;
        case LCD_NV3041A:
            NV3041AInit(pLCD);
            break;
        case LCD_ST77916:
            ST77916Init(pLCD);
            break;
    }
    return 1;
} /* qspiInit() */

//This function is called (in irq context!) just before a transmission starts. It will
//set the D/C line to the value indicated in the user field.
static void spi_pre_transfer_callback(spi_transaction_t *t)
{
//    int iMode=(int)t->user;
//    spilcdSetMode(iMode);
}
static void IRAM_ATTR spi_post_transfer_callback(spi_transaction_t *t)
{
//    SPILCD *pLCD = (SPILCD*)t;
    transfer_is_done = true;
//    myPinWrite(iCurrentCS, 1);
//    iCurrentCS = -1;
}
#endif

#ifdef ARDUINO_SAMD_ZERO
// Callback for end-of-DMA-transfer
void dma_callback(Adafruit_ZeroDMA *dma) {
//    SPILCD *pLCD = (SPILCD*)dma; // not needed
    transfer_is_done = true;
    myPinWrite(iCurrentCS, 1);
    iCurrentCS = -1;
//    myPinWrite(pLCD->iCSPin, 1);
}
#endif // ARDUINO_SAMD_ZERO

// wait for previous transaction to complete
void spilcdWaitDMA(void)
{
//#ifdef HAS_DMA
    while (!transfer_is_done);
//    myPinWrite(iCurrentCS, 1);
//    iCurrentCS = -1;
#ifdef ARDUINO_SAMD_ZERO
    mySPI.endTransaction();
#endif // ARDUINO_SAMD_ZERO
//#endif // HAS_DMA
}
// Queue a new transaction for the SPI DMA
void spilcdWriteDataDMA(SPILCD *pLCD, uint8_t *pBuf, int iLen)
{
#ifdef ARDUINO_ARCH_ESP32
esp_err_t ret;

    trans[0].tx_buffer = pBuf;
    trans[0].length = iLen * 8; // Length in bits
    trans[0].rxlength = 0; // defaults to the same length as tx length
    
//    spi_device_polling_end(spi, portMAX_DELAY); // make sure it's ready
    // Queue the transaction
//    ret = spi_device_polling_transmit(spi, &t);
    transfer_is_done = false;
    ret = spi_device_queue_trans(spi, &trans[0], portMAX_DELAY);
    assert (ret==ESP_OK);
//    iFirst = 0;
    return;
#endif // ARDUINO_ARCH_ESP32
#ifdef ARDUINO_SAMD_ZERO
  myDMA.changeDescriptor(
    desc,
    ucTXBuf,     // src
    NULL, // dest
    iLen);                      // this many...
  mySPI.beginTransaction(SPISettings(pLCD->iSPISpeed, MSBFIRST, pLCD->iSPIMode));
  transfer_is_done = false;
  myDMA.startJob();
  return;
#endif
} /* spilcdWriteDataDMA() */

//
// Smooth expanded text
//
//  A    --\ 1 2
//C P B  --/ 3 4
//  D
// 1=P; 2=P; 3=P; 4=P;
// IF C==A AND C!=D AND A!=B => 1=A
// IF A==B AND A!=C AND B!=D => 2=B
// IF B==D AND B!=A AND D!=C => 4=D
// IF D==C AND D!=B AND C!=A => 3=C
void SmoothImg(uint16_t *pSrc, uint16_t *pDest, int iSrcPitch, int iDestPitch, int iWidth, int iHeight)
{
int x, y;
unsigned short *s,*s2;
uint32_t *d,*d2;
unsigned short A, B, C, D, P;
uint32_t ulPixel, ulPixel2;

// copy the edge pixels as-is
// top+bottom lines first
s = pSrc;
s2 = (unsigned short *)&pSrc[(iHeight-1)*iSrcPitch];
d = (uint32_t *)pDest;
d2 = (uint32_t *)&pDest[(iHeight-1)*2*iDestPitch];
for (x=0; x<iWidth; x++)
{     
      ulPixel = *s++;
      ulPixel2 = *s2++;
      ulPixel |= (ulPixel << 16); // simply double it
      ulPixel2 |= (ulPixel2 << 16);
      d[0] = ulPixel; 
      d[iDestPitch/2] = ulPixel;
      d2[0] = ulPixel2;
      d2[iDestPitch/2] = ulPixel2;
      d++; d2++;
}
for (y=1; y<iHeight-1; y++)
  {
  s = (unsigned short *)&pSrc[y * iSrcPitch];
  d = (uint32_t *)&pDest[(y * 2 * iDestPitch)];
// first pixel is just stretched
  ulPixel = *s++;
  ulPixel |= (ulPixel << 16);
  d[0] = ulPixel;
  d[iDestPitch/2] = ulPixel;
  d++;
  for (x=1; x<iWidth-1; x++)
     {
     A = s[-iSrcPitch];
     C = s[-1];
     P = s[0];
     B = s[1];
     D = s[iSrcPitch];
     if (C==A && C!=D && A!=B)
        ulPixel = A;
     else
        ulPixel = P;
     if (A==B && A!=C && B!=D)
        ulPixel |= (B << 16);
     else
        ulPixel |= (P << 16);
     d[0] = ulPixel;
     if (D==C && D!=B && C!=A)
        ulPixel = C;
     else
        ulPixel = P;
     if (B==D && B!=A && D!=C)
        ulPixel |= (D << 16);
     else
        ulPixel |= (P << 16);
     d[iDestPitch/2] = ulPixel;
     d++;
     s++;
     } // for x
// last pixel is just stretched
  ulPixel = s[0];
  ulPixel |= (ulPixel << 16);
  d[0] = ulPixel;
  d[iDestPitch/2] = ulPixel;
  } // for y
} /* SmoothImg() */
//
// Width is the doubled pixel width
// Convert 1-bpp into 2-bit grayscale
//
static void Scale2Gray(uint8_t *source, int width, int iPitch)
{
    int x;
    uint8_t ucPixels, c, d, *dest;

    dest = source; // write the new pixels over the old to save memory

    for (x=0; x<width/8; x+=2) /* Convert a pair of lines to gray */
    {
        c = source[x];  // first 4x2 block
        d = source[x+iPitch];
        /* two lines of 8 pixels are converted to one line of 4 pixels */
        ucPixels = (ucGray2BPP[(unsigned char)((c & 0xf0) | (d >> 4))] << 4);
        ucPixels |= (ucGray2BPP[(unsigned char)((c << 4) | (d & 0x0f))]);
        *dest++ = ucPixels;
        c = source[x+1];  // next 4x2 block
        d = source[x+iPitch+1];
        ucPixels = (ucGray2BPP[(unsigned char)((c & 0xf0) | (d >> 4))])<<4;
        ucPixels |= ucGray2BPP[(unsigned char)((c << 4) | (d & 0x0f))];
        *dest++ = ucPixels;
    }
    if (width & 4) // 2 more pixels to do
    {
        c = source[x];
        d = source[x + iPitch];
        ucPixels = (ucGray2BPP[(unsigned char) ((c & 0xf0) | (d >> 4))]) << 4;
        ucPixels |= (ucGray2BPP[(unsigned char) ((c << 4) | (d & 0x0f))]);
        dest[0] = ucPixels;
    }
} /* Scale2Gray() */
//
// Draw a string of characters in a custom font antialiased
// at 1/2 its original size
// A back buffer must be defined
//
int spilcdWriteStringAntialias(SPILCD *pLCD, GFXfont *pFont, int x, int y, char *szMsg, uint16_t usFGColor, uint16_t usBGColor, int iFlags)
{
int i, end_y, cx, dx, dy, tx, ty, c, iBitOff;
uint8_t *s, *d, bits, /*ucMask,*/ ucClr, uc;
GFXfont font;
GFXglyph glyph, *pGlyph;
const uint32_t ulClrMask = 0x07E0F81F;
uint32_t ulFG, ulBG;
uint8_t ucTemp[64]; // enough space for a 256 pixel wide font
uint16_t usTemp[128];
    
   if (pLCD == NULL || pFont == NULL)
      return -1;
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0)
        return -1;
    // Prepare the foreground and background colors for alpha calculations
    ulFG = usFGColor | ((uint32_t)usFGColor << 16);
    ulBG = usBGColor | ((uint32_t)usBGColor << 16);
    ulFG &= ulClrMask; ulBG &= ulClrMask;
   // in case of running on Harvard CPU, get copy of data from FLASH
   memcpy_P(&font, pFont, sizeof(font));
   pGlyph = &glyph;

   i = 0;
   while (szMsg[i] && x < pLCD->iCurrentWidth)
   {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      memcpy_P(&glyph, &font.glyph[c], sizeof(glyph));
      dx = x + pGlyph->xOffset/2; // offset from character UL to start drawing
       cx = (pGlyph->width+1)/2;
       if (dx+cx > pLCD->iCurrentWidth)
           cx = pLCD->iCurrentWidth - dx;
      dy = y + (pGlyph->yOffset/2);
      s = font.bitmap + pGlyph->bitmapOffset; // start of bitmap data
      // Bitmap drawing loop. Image is MSB first and each pixel is packed next
      // to the next (continuing on to the next character line)
      iBitOff = 0; // bitmap offset (in bits)
      bits = uc = 0; // bits left in this font byte
      end_y = dy + (pGlyph->height+1)/2;
//      if (dy < 0) { // skip these lines
//          iBitOff += (pGlyph->width * (-dy));
//          dy = 0;
//      }
       spilcdSetPosition(pLCD, dx, dy, cx, end_y-dy, iFlags);
       memset(ucTemp, 0, sizeof(ucTemp));
       for (ty=0; ty<pGlyph->height; ty++) {
         d = &ucTemp[(ty & 1) * (sizeof(ucTemp)/2)]; // internal buffer dest
         for (tx=0; tx<pGlyph->width; tx++) {
            if (bits == 0) { // need to read more font data
               uc = pgm_read_byte(&s[iBitOff>>3]); // get more font bitmap data
               bits = 8;
               iBitOff += bits;
            } // if we ran out of bits
            if (uc & 0x80) { // set the pixel
                d[(tx>>3)] |= (0x80 >> (tx & 7));
            }
            bits--; // next bit
            uc <<= 1;
         } // for x
           if ((ty & 1) || ty == pGlyph->height-1) {
               uint8_t *pg; // pointer to gray source pixels
               uint16_t *pus = usTemp;
               uint32_t ulAlpha, ulPixel;
               //int j;
               const uint8_t ucClrConvert[4] = {0,5,11,16};
               // Convert this pair of lines to grayscale output
               Scale2Gray(ucTemp, pGlyph->width, sizeof(ucTemp)/2);
               // the Scale2Gray code writes the bits horizontally; crop and convert them for the internal memory format
               pg = ucTemp;
               ucClr = *pg++;
               for (tx=0; tx<cx; tx++) {
                   ulAlpha = ucClrConvert[((ucClr & 0xc0) >> 6)]; // 0-3 scaled from 0 to 100% in thirds
                   ulPixel = ((ulFG * ulAlpha) + (ulBG * (16-ulAlpha))) >> 4;
                   ulPixel &= ulClrMask; // separate the RGBs
                   ulPixel |= (ulPixel >> 16); // bring G back to RB
                   *pus++ = __builtin_bswap16(ulPixel); // final pixel
                   ucClr <<= 2;
                   if ((tx & 3) == 3)
                       ucClr = *pg++; // get 4 more pixels
               }
               myspiWrite(pLCD, (uint8_t *)usTemp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
               memset(ucTemp, 0, sizeof(ucTemp));
           }
      } // for y
      x += pGlyph->xAdvance/2; // width of this character
   } // while drawing characters
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
   return 0;
} /* spilcdWriteStringAntialias() */
//
// Draw a string in a proportional font you supply
//
int spilcdWriteStringCustom(SPILCD *pLCD, GFXfont *pFont, int x, int y, char *szMsg, uint16_t usFGColor, uint16_t usBGColor, int bBlank, int iFlags)
{
int i, /*j, iLen, */ k, dx, dy, cx, cy, c, iBitOff;
int tx, ty;
uint8_t *s, bits, uc;
GFXfont font;
GFXglyph *pGlyph;
uint16_t *d;
uint8_t ucBitmap[32]; // 256 pixels wide should be big enough?
    
   if (pFont == NULL)
      return -1;
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0)
        return -1;
   // in case of running on AVR, get copy of data from FLASH
   memcpy_P(&font, pFont, sizeof(font));
   if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
       usFGColor = (usFGColor >> 8) | (usFGColor << 8); // swap h/l bytes
       usBGColor = (usBGColor >> 8) | (usBGColor << 8);
   }
   i = 0;
   while (szMsg[i] && x < pLCD->iCurrentWidth)
   {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      pGlyph = &font.glyph[c];
      // set up the destination window (rectangle) on the display
      dx = x + pGlyph->xOffset; // offset from character UL to start drawing
      dy = y + pGlyph->yOffset;
      cx = pGlyph->width;
      cy = pGlyph->height;
      iBitOff = 0; // bitmap offset (in bits)
      if (dy + cy > pLCD->iCurrentHeight)
         cy = pLCD->iCurrentHeight - dy; // clip bottom edge
      else if (dy < 0) {
         cy += dy;
         iBitOff += (pGlyph->width * (-dy));
         dy = 0;
      }
      if (dx + cx > pLCD->iCurrentWidth)
         cx = pLCD->iCurrentWidth - dx; // clip right edge
      s = font.bitmap + pGlyph->bitmapOffset; // start of bitmap data
      // Bitmap drawing loop. Image is MSB first and each pixel is packed next
      // to the next (continuing on to the next character line)
      bits = uc = 0; // bits left in this font byte

      if (bBlank) { // erase the areas around the char to not leave old bits
         int miny, maxy;
         c = '0' - font.first;
         miny = y + pGlyph->yOffset;
         c = 'y' - font.first;
         maxy = miny + pGlyph->height;
         if (maxy > pLCD->iCurrentHeight)
            maxy = pLCD->iCurrentHeight;
         cx = pGlyph->xAdvance;
         if (cx + x > pLCD->iCurrentWidth) {
            cx = pLCD->iCurrentWidth - x;
         }
         // Serial.println("Blank");
         spilcdSetPosition(pLCD, x, miny, cx, maxy-miny, iFlags);
            // blank out area above character
//            cy = font.yAdvance - pGlyph->height;
//            for (ty=miny; ty<miny+cy && ty < maxy; ty++) {
//               for (tx=0; tx<cx; tx++)
//                  u16Temp[tx] = usBGColor;
//               myspiWrite(pLCD, (uint8_t *)u16Temp, cx*sizeof(uint16_t), MODE_DATA, iFlags);
//            } // for ty
            // character area (with possible padding on L+R)
            for (ty=0; ty<pGlyph->height && ty+miny < maxy; ty++) {
               d = (uint16_t *)ucRXBuf;
               for (tx=0; tx<pGlyph->xOffset && tx < cx; tx++) { // left padding
                  *d++ = usBGColor;
               }
            // character bitmap (center area)
               for (tx=0; tx<pGlyph->width; tx++) {
                  if (bits == 0) { // need more data
                     uc = pgm_read_byte(&s[iBitOff>>3]);
                     bits = 8;
                     iBitOff += bits;
                  }
                  if (tx + pGlyph->xOffset < cx) {
                     *d++ = (uc & 0x80) ? usFGColor : usBGColor;
                  }
                  bits--;
                  uc <<= 1;
               } // for tx
               // right padding
               k = pGlyph->xAdvance - (int)(d - (uint16_t *)ucRXBuf); // remaining amount
               for (tx=0; tx<k && (tx+pGlyph->xOffset+pGlyph->width) < cx; tx++)
                  *d++ = usBGColor;
               myspiWrite(pLCD, ucRXBuf, cx*sizeof(uint16_t), MODE_DATA, iFlags);
            } // for ty
            // padding below the current character
            ty = y + pGlyph->yOffset + pGlyph->height;
            for (; ty < maxy; ty++) {
               for (tx=0; tx<cx; tx++)
                  ucRXBuf[tx] = usBGColor;
               myspiWrite(pLCD, ucRXBuf, cx*sizeof(uint16_t), MODE_DATA, iFlags);
            } // for ty
      } else if (usFGColor == usBGColor) { // transparent
          int iCount; // opaque pixel count
          //Serial.println("Transparent");
          d = (uint16_t *)ucRXBuf;
          for (iCount=0; iCount < cx; iCount++)
              d[iCount] = usFGColor; // set up a line of solid color
          iCount = 0; // number of sequential opaque pixels
             for (ty=0; ty<cy; ty++) {
             for (tx=0; tx<pGlyph->width; tx++) {
                if (bits == 0) { // need to read more font data
                   uc = pgm_read_byte(&s[iBitOff>>3]); // get more font bitmap data
                   bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
                   iBitOff += bits; // because of a clipped line
                   uc <<= (8-bits);
                } // if we ran out of bits
                if (tx < cx) {
                    if (uc & 0x80) {
                        iCount++; // one more opaque pixel
                    } else { // any opaque pixels to write?
                        if (iCount) {
                            spilcdSetPosition(pLCD, dx+tx-iCount, dy+ty, iCount, 1, iFlags);
                       d = (uint16_t *)ucRXBuf; // point to start of output buffer
                            myspiWrite(pLCD, ucRXBuf, iCount*sizeof(uint16_t), MODE_DATA, iFlags);
                            iCount = 0;
                        } // if opaque pixels to write
                    } // if transparent pixel hit
                }
                bits--; // next bit
                uc <<= 1;
             } // for tx
             } // for ty
       // quicker drawing
      } else { // just draw the current character box fast
         spilcdSetPosition(pLCD, dx, dy, cx, cy, iFlags);
            d = (uint16_t *)ucRXBuf; // point to start of output buffer
            for (ty=0; ty<cy; ty++) {
            for (tx=0; tx<pGlyph->width; tx++) {
               if (bits == 0) { // need to read more font data
                  uc = pgm_read_byte(&s[iBitOff>>3]); // get more font bitmap data
                  bits = 8 - (iBitOff & 7); // we might not be on a byte boundary
                  iBitOff += bits; // because of a clipped line
                  uc <<= (8-bits);
                  k = (int)(d-(uint16_t *)ucRXBuf); // number of words in output buffer
                  if (k >= sizeof(ucRXBuf)/4) { // time to write it
                     myspiWrite(pLCD, ucRXBuf, k*sizeof(uint16_t), MODE_DATA, iFlags);
                     d = (uint16_t *)ucRXBuf;
                  }
               } // if we ran out of bits
               if (tx < cx) {
                  *d++ = (uc & 0x80) ? usFGColor : usBGColor;
               }
               bits--; // next bit
               uc <<= 1;
            } // for tx
            } // for ty
            k = (int)(d-(uint16_t *)ucRXBuf);
            if (k) // write any remaining data
               myspiWrite(pLCD, ucRXBuf, k*sizeof(uint16_t), MODE_DATA, iFlags);
      } // quicker drawing
      x += pGlyph->xAdvance; // width of this character
   } // while drawing characters
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
   return 0;
} /* spilcdWriteStringCustom() */
//
// Get the width of text in a custom font
//
void spilcdGetStringBox(GFXfont *pFont, char *szMsg, int *width, int *top, int *bottom)
{
int cx = 0;
int c, i = 0;
GFXfont font;
GFXglyph glyph, *pGlyph;
int miny, maxy;

   if (pFont == NULL)
      return;
   // in case of running on AVR, get copy of data from FLASH
   memcpy_P(&font, pFont, sizeof(font));
   pGlyph = &glyph;
   if (width == NULL || top == NULL || bottom == NULL || pFont == NULL || szMsg == NULL) return; // bad pointers
   miny = 1000; maxy = 0;
   while (szMsg[i]) {
      c = szMsg[i++];
      if (c < font.first || c > font.last) // undefined character
         continue; // skip it
      c -= font.first; // first char of font defined
      memcpy_P(&glyph, &font.glyph[c], sizeof(glyph));
      cx += pGlyph->xAdvance;
      if (pGlyph->yOffset < miny) miny = pGlyph->yOffset;
      if (pGlyph->height+pGlyph->yOffset > maxy) maxy = pGlyph->height+pGlyph->yOffset;
   }
   *width = cx;
   *top = miny;
   *bottom = maxy;
} /* spilcdGetStringBox() */

#ifndef __AVR__
//
// Draw a string of text as quickly as possible
//
int spilcdWriteStringFast(SPILCD *pLCD, int x, int y, char *szMsg, unsigned short usFGColor, unsigned short usBGColor, int iFontSize, int iFlags)
{
int i, j, k, iLen;
int iStride;
uint8_t *s;
uint16_t usFG, usBG;
uint16_t *usD;
int cx;
uint8_t *pFont;

    if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
        usFG = (usFGColor >> 8) | ((usFGColor & -1)<< 8);
        usBG = (usBGColor >> 8) | ((usBGColor & -1)<< 8);
    } else {
        usFG = usFGColor;
        usBG = usBGColor;
    }
    if (iFontSize != FONT_6x8 && iFontSize != FONT_8x8 && iFontSize != FONT_12x16)
        return -1; // invalid size
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0) return -1;
    
    if (iFontSize == FONT_12x16) {
        iLen = strlen(szMsg);
        if ((12*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/12; // can't display it all
        if (iLen < 0) return -1;
        iStride = iLen*12;
        spilcdSetPosition(pLCD, x, y, iStride, 16, iFlags);
        for (k = 0; k<8; k++) { // create a pair of scanlines from each original
           uint8_t ucMask = (1 << k);
           usD = (uint16_t *)pDMA;
           for (i=0; i<iStride*2; i++)
              usD[i] = usBG; // set to background color first
           for (i=0; i<iLen; i++)
           {
               uint8_t c0, c1;
               s = (uint8_t *)&ucSmallFont[((unsigned char)szMsg[i]-32) * 6];
               for (j=1; j<6; j++)
               {
                   uint8_t ucMask1 = ucMask << 1;
                   uint8_t ucMask2 = ucMask >> 1;
                   c0 = pgm_read_byte(&s[j]);
                   if (c0 & ucMask)
                      usD[0] = usD[1] = usD[iStride] = usD[iStride+1] = usFG;
                   // test for smoothing diagonals
                   if (j < 5) {
                      c1 = pgm_read_byte(&s[j+1]);
                      if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask1) && (c1 & ucMask1)) { // first diagonal condition
                          usD[iStride+2] = usFG;
                      } else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask1) && (~c1 & ucMask1)) { // second condition
                          usD[iStride+1] = usFG;
                      }
                      if ((c0 & ucMask2) && (~c1 & ucMask2) && (~c0 & ucMask) && (c1 & ucMask)) { // repeat for previous line
                          usD[1] = usFG;
                      } else if ((~c0 & ucMask2) && (c1 & ucMask2) && (c0 & ucMask) && (~c1 & ucMask)) {
                          usD[2] = usFG;
                      }
                   }
                   usD+=2;
               } // for j
               usD += 2; // leave "6th" column blank
            } // for each character
            myspiWrite(pLCD, (uint8_t *)pDMA, iStride*4, MODE_DATA, iFlags);
        } // for each scanline
        return 0;
    } // 12x16
    
    cx = (iFontSize == FONT_8x8) ? 8:6;
    pFont = (iFontSize == FONT_8x8) ? (uint8_t *)ucFont : (uint8_t *)ucSmallFont;
    iLen = strlen(szMsg);
	if (iLen <=0) return -1; // can't use this function

    if ((cx*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/cx; // can't display it all
    if (iLen > 32) iLen = 32;
    iStride = iLen * cx*2;
    for (i=0; i<iLen; i++)
    {
        s = &pFont[((unsigned char)szMsg[i]-32) * cx];
        uint8_t ucMask = 1;
        for (k=0; k<8; k++) // for each scanline
        {
            usD = (uint16_t *)&pDMA[(k*iStride) + (i * cx*2)];
            for (j=0; j<cx; j++)
            {
                if (s[j] & ucMask)
                    *usD++ = usFG;
                else
                    *usD++ = usBG;
            } // for j
            ucMask <<= 1;
        } // for k
    } // for i
    // write the data in one shot
    spilcdSetPosition(pLCD, x, y, cx*iLen, 8, iFlags);
    myspiWrite(pLCD, (uint8_t *)pDMA, iLen*cx*16, MODE_DATA, iFlags);
    pLCD->iCursorX = x + (cx*iLen);
    pLCD->iCursorY = y;
	return 0;
} /* spilcdWriteStringFast() */
#endif // !__AVR__

//
// Draw a string of small (8x8) or large (16x32) characters
// At the given col+row
//
int spilcdWriteString(SPILCD *pLCD, int x, int y, char *szMsg, int usFGColor, int usBGColor, int iFontSize, int iFlags)
{
int i, j, k, iLen;
#ifndef __AVR__
int l;
#endif
unsigned char *s;
unsigned short usFG, usBG;
uint16_t usPitch = pLCD->iScreenPitch/2;


    if (pLCD->iLCDFlags & FLAGS_SWAP_COLOR) {
        usFG = usFGColor;
        usBG = usBGColor;
    } else {
        usFG = (usFGColor >> 8) | (usFGColor << 8);
        usBG = (usBGColor >> 8) | (usBGColor << 8);
    } 
    if (x == -1)
        x = pLCD->iCursorX;
    if (y == -1)
        y = pLCD->iCursorY;
    if (x < 0 || y < 0) return -1;
	iLen = strlen(szMsg);
    if (usBGColor == -1)
        iFlags = DRAW_TO_RAM; // transparent text doesn't get written to the display
    if (usFG == usBG) usBG = 0; // DEBUG!! no transparent option for now
#ifndef __AVR__
	if (iFontSize == FONT_16x32) // draw 16x32 font
	{
		if (iLen*16 + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x) / 16;
		if (iLen < 0) return -1;
		for (i=0; i<iLen; i++)
		{
			uint16_t *usD, *usTemp = (uint16_t *)pDMA;
			s = (uint8_t *)&ucBigFont[((unsigned char)szMsg[i]-32)*64];
			usD = &usTemp[0];
            if (usBGColor == -1) // transparent text is not rendered to the display
               iFlags = DRAW_TO_RAM;
            spilcdSetPosition(pLCD, x+(i*16), y,16,32, iFlags);
            for (l=0; l<4; l++) // 4 sets of 8 rows
            {
                uint8_t ucMask = 1;
                for (k=0; k<8; k++) // for each scanline
                { // left half
                    if (usBGColor == -1) // transparent text
                    {
                        uint16_t *d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + ((l*8+k)*pLCD->iScreenPitch)];
                        for (j=0; j<16; j++)
                        {
                            if (s[j] & ucMask)
                                *d = usFG;
                            d++;
                        } // for j
                    }
                    else
                    {
                        for (j=0; j<16; j++)
                        {
                            if (s[j] & ucMask)
                                *usD++ = usFG;
                            else
                                *usD++ = usBG;
                        } // for j
                    }
                    ucMask <<= 1;
                } // for each scanline
                s += 16;
            } // for each set of 8 scanlines
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)usTemp, 1024, MODE_DATA, iFlags);
		} // for each character
        x += (i*16);
    }
#endif // !__AVR__
    if (iFontSize == FONT_8x8 || iFontSize == FONT_6x8) // draw the 6x8 or 8x8 font
	{
		uint16_t *usD, *usTemp = (uint16_t *)pDMA;
        int cx;
        uint8_t *pFont;

        cx = (iFontSize == FONT_8x8) ? 8:6;
        pFont = (iFontSize == FONT_8x8) ? (uint8_t *)ucFont : (uint8_t *)ucSmallFont;
		if ((cx*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/cx; // can't display it all
		if (iLen < 0)return -1;

		for (i=0; i<iLen; i++)
		{
			s = &pFont[((unsigned char)szMsg[i]-32) * cx];
			usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*cx), y, cx, 8, iFlags);
            uint8_t ucMask = 1;
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k * pLCD->iScreenPitch)];
                    for (j=0; j<cx; j++)
                    {
                        if (pgm_read_byte(&s[j]) & ucMask)
                            *usD = usFG;
                        usD++;
                    } // for j
                }
                else // regular text
                {
                    for (j=0; j<cx; j++)
                    {
                        if (pgm_read_byte(&s[j]) & ucMask)
                            *usD++ = usFG;
                        else
                            *usD++ = usBG;
                    } // for j
                }
                ucMask <<= 1;
            } // for k
    // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)usTemp, cx*16, MODE_DATA, iFlags);
		}	
        x += (i*cx);
    } // 6x8 and 8x8
    if (iFontSize == FONT_12x16) // 6x8 stretched to 12x16 (with smoothing)
    {
        uint16_t *usD, *usTemp = (uint16_t *)pDMA;
        
        if ((12*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/12; // can't display it all
        if (iLen < 0)return -1;
        
        for (i=0; i<iLen; i++)
        {
            s = (uint8_t *)&ucSmallFont[((unsigned char)szMsg[i]-32) * 6];
            usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*12), y, 12, 16, iFlags);
            uint8_t ucMask = 1;
            if (usBGColor != -1) // start with all BG color
            {
               for (k=0; k<12*16; k++)
                  usD[k] = usBG;
            }
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    uint8_t c0, c1;
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k*2*pLCD->iScreenPitch)];
                    for (j=0; j<6; j++)
                    {
                        c0 = pgm_read_byte(&s[j]);
                        if (c0 & ucMask)
                            usD[0] = usD[1] = usD[usPitch] = usD[usPitch+1] = usFG;
                        // test for smoothing diagonals
                        if (k < 7 && j < 5) {
                           uint8_t ucMask2 = ucMask << 1;
                           c1 = pgm_read_byte(&s[j+1]);
                           if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask2) && (c1 & ucMask2)) // first diagonal condition
                               usD[usPitch+2] = usD[2*usPitch+1] = usFG;
                           else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && (~c1 & ucMask2))
                               usD[usPitch+1] = usD[2*usPitch+2] = usFG;
                        } // if not on last row and last col
                        usD += 2;
                    } // for j
                }
                else // regular text drawing
                {
                    uint8_t c0, c1;
                    for (j=0; j<6; j++)
                    {
                        c0 = pgm_read_byte(&s[j]);
                        if (c0 & ucMask)
                           usD[0] = usD[1] = usD[12] = usD[13] = usFG;
                        // test for smoothing diagonals
                        if (k < 7 && j < 5) {
                           uint8_t ucMask2 = ucMask << 1;
                           c1 = pgm_read_byte(&s[j+1]);
                           if ((c0 & ucMask) && (~c1 & ucMask) && (~c0 & ucMask2) && (c1 & ucMask2)) // first diagonal condition
                               usD[14] = usD[25] = usFG;
                           else if ((~c0 & ucMask) && (c1 & ucMask) && (c0 & ucMask2) && (~c1 & ucMask2))
                               usD[13] = usD[26] = usFG;
                        } // if not on last row and last col
                        usD+=2;
                    } // for j
                }
                usD += 12; // skip the extra line
                ucMask <<= 1;
            } // for k
        // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)&usTemp[0], 12*16*2, MODE_DATA, iFlags);
        }
        x += i*12;
    } // FONT_12x16
    if (iFontSize == FONT_16x16) // 8x8 stretched to 16x16
    {
        uint16_t *usD, *usTemp = (uint16_t *)pDMA;
        
        if ((16*iLen) + x > pLCD->iCurrentWidth) iLen = (pLCD->iCurrentWidth - x)/16; // can't display it all
        if (iLen < 0)return -1;
        
        for (i=0; i<iLen; i++)
        {
            s = (uint8_t *)&ucFont[((unsigned char)szMsg[i]-32) * 8];
            usD = &usTemp[0];
            spilcdSetPosition(pLCD, x+(i*16), y, 16, 16, iFlags);
            uint8_t ucMask = 1;
            if (usBGColor != -1) // start with all BG color
            {
               for (k=0; k<256; k++)
                  usD[k] = usBG;
            }
            for (k=0; k<8; k++) // for each scanline
            {
                if (usBGColor == -1) // transparent text
                {
                    usD = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (k*2*pLCD->iScreenPitch)];
                    for (j=0; j<8; j++)
                    {
                        if (pgm_read_byte(&s[j]) & ucMask)
                            usD[0] = usD[1] = usD[usPitch] = usD[usPitch+1] = usFG;
                        usD += 2;
                    } // for j
                }
                else // regular text drawing
                {
                    uint8_t c0;
                    for (j=0; j<8; j++)
                    {
                        c0 = pgm_read_byte(&s[j]);
                        if (c0 & ucMask)
                        usD[0] = usD[1] = usD[16] = usD[17] = usFG;
                        usD+=2;
                    } // for j
                }
                usD += 16; // skip the next line
                ucMask <<= 1;
            } // for k
        // write the data in one shot
        if (usBGColor != -1) // don't write anything if we're doing transparent text
            myspiWrite(pLCD, (unsigned char *)&usTemp[0], 512, MODE_DATA, iFlags);
        }
        x += (i*16);
    } // FONT_16x16
    pLCD->iCursorX = x;
    pLCD->iCursorY = y;
	return 0;
} /* spilcdWriteString() */
//
// For drawing ellipses, a circle is drawn and the x and y pixels are scaled by a 16-bit integer fraction
// This function draws a single pixel and scales its position based on the x/y fraction of the ellipse
//
void DrawScaledPixel(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, unsigned short usColor, int iFlags)
{
    uint8_t ucBuf[2];
    if (iXFrac != 0x10000) x = (x * iXFrac) >> 16;
    if (iYFrac != 0x10000) y = (y * iYFrac) >> 16;
    x += iCX; y += iCY;
    if (x < 0 || x >= pLCD->iCurrentWidth || y < 0 || y >= pLCD->iCurrentHeight)
        return; // off the screen
    ucBuf[0] = (uint8_t)(usColor >> 8);
    ucBuf[1] = (uint8_t)usColor;
    spilcdSetPosition(pLCD, x, y, 1, 1, iFlags);
    myspiWrite(pLCD, ucBuf, 2, MODE_DATA, iFlags);
} /* DrawScaledPixel() */
        
void DrawScaledLine(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, uint16_t *pBuf, int iFlags)
{
    int32_t iLen, x2;
    if (iXFrac != 0x10000) x = (x * iXFrac) >> 16;
    if (iYFrac != 0x10000) y = (y * iYFrac) >> 16;
    iLen = x * 2;
    x = iCX - x; y += iCY;
    x2 = x + iLen;
    if (y < 0 || y >= pLCD->iCurrentHeight)
        return; // completely off the screen
    if (x < 0) x = 0;
    if (x2 >= pLCD->iCurrentWidth) x2 = pLCD->iCurrentWidth-1;
    iLen = x2 - x + 1; // new length
    spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
//#ifdef ESP32_DMA
//    myspiWrite(pLCD, (uint8_t*)pBuf, iLen*2, MODE_DATA, iFlags);
//#else
    // need to refresh the output data each time
    {
    int i;
    unsigned short us = pBuf[0];
      for (i=1; i<iLen; i++)
        pBuf[i] = us;
    }
    myspiWrite(pLCD, (uint8_t*)&pBuf[1], iLen*2, MODE_DATA, iFlags);
//#endif
} /* DrawScaledLine() */
//
// Draw the 8 pixels around the Bresenham circle
// (scaled to make an ellipse)
//
void BresenhamCircle(SPILCD *pLCD, int32_t iCX, int32_t iCY, int32_t x, int32_t y, int32_t iXFrac, int32_t iYFrac, uint16_t iColor, uint16_t *pFill, uint8_t u8Parts, int iFlags)
{
    if (pFill != NULL) // draw a filled ellipse
    {
        static int prev_y = -1;
        // for a filled ellipse, draw 4 lines instead of 8 pixels
        DrawScaledLine(pLCD, iCX, iCY, y, x, iXFrac, iYFrac, pFill, iFlags);
        DrawScaledLine(pLCD, iCX, iCY, y, -x, iXFrac, iYFrac, pFill, iFlags);
        if (y != prev_y) {
            DrawScaledLine(pLCD, iCX, iCY, x, y, iXFrac, iYFrac, pFill, iFlags);
            DrawScaledLine(pLCD, iCX, iCY, x, -y, iXFrac, iYFrac, pFill, iFlags);
            prev_y = y;
        }
    }
    else // draw 8 pixels around the edges
    {
        if (u8Parts & 1) {
            DrawScaledPixel(pLCD, iCX, iCY, -x, -y, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, -y, -x, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 2) {
            DrawScaledPixel(pLCD, iCX, iCY, x, -y, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, y, -x, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 4) {
            DrawScaledPixel(pLCD, iCX, iCY, y, x, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, x, y, iXFrac, iYFrac, iColor, iFlags);
        }
        if (u8Parts & 8) {
            DrawScaledPixel(pLCD, iCX, iCY, -y, x, iXFrac, iYFrac, iColor, iFlags);
            DrawScaledPixel(pLCD, iCX, iCY, -x, y, iXFrac, iYFrac, iColor, iFlags);
        }
    }
} /* BresenhamCircle() */

void spilcdEllipse(SPILCD *pLCD, int32_t iCenterX, int32_t iCenterY, int32_t iRadiusX, int32_t iRadiusY, uint8_t u8Parts, unsigned short usColor, int bFilled, int iFlags)
{
    int32_t iRadius, iXFrac, iYFrac;
    int32_t iDelta, x, y;
    uint16_t us, *pus, *usTemp = (uint16_t *)pDMA; // up to 320 pixels wide
    
    if (iCenterX < 0 || iCenterY < 0 || iRadiusX <= 0 || iRadiusY <= 0) return;

    if (iRadiusX > iRadiusY) // use X as the primary radius
    {
        iRadius = iRadiusX;
        iXFrac = 65536;
        iYFrac = (iRadiusY * 65536) / iRadiusX;
    }
    else
    {
        iRadius = iRadiusY;
        iXFrac = (iRadiusX * 65536) / iRadiusY;
        iYFrac = 65536;
    }
    // set up a buffer with the widest possible run of pixels to dump in 1 shot
    if (bFilled)
    {
        if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
            us = (usColor >> 8) | (usColor << 8); // swap byte order
        } else {
            us = usColor;
        }
        y = iRadius*2;
        if (y > 320) y = 320; // max size
//#ifdef ESP32_DMA
        for (x=0; x<(y+2); x++)
        {
            usTemp[x] = us;
        }
//#else
//	usTemp[0] = us; // otherwise just set the first one to the color
//#endif
        pus = usTemp;
    }
    else
    {
        pus = NULL;
    }
    iDelta = 3 - (2 * iRadius);
    x = 0; y = iRadius;
    while (x <= y)
    {
        BresenhamCircle(pLCD, iCenterX, iCenterY, x, y, iXFrac, iYFrac, usColor, pus, u8Parts, iFlags);
        x++;
        if (iDelta < 0)
        {
            iDelta += (4*x) + 6;
        }
        else
        {
            iDelta += 4 * (x-y) + 10;
            y--;
        }
    }

} /* spilcdEllipse() */

//
// Set the (software) orientation of the display
// The hardware is permanently oriented in 240x320 portrait mode
// The library can draw characters/tiles rotated 90
// degrees if set into landscape mode
//
int spilcdSetOrientation(SPILCD *pLCD, int iOrient)
{
int bX=0, bY=0, bV=0;

    pLCD->iOrientation = iOrient;
    // Make sure next setPos() resets both x and y
    pLCD->iOldX=-1; pLCD->iOldY=-1; pLCD->iOldCX=-1; pLCD->iOldCY = -1;
   switch(iOrient)
   {
     case LCD_ORIENTATION_0:
           bX = bY = bV = 0;
           pLCD->iMemoryX = pLCD->iColStart;
           pLCD->iMemoryY = pLCD->iRowStart;
           pLCD->iCurrentHeight = pLCD->iHeight;
           pLCD->iCurrentWidth = pLCD->iWidth;
        break;
     case LCD_ORIENTATION_90:
        bX = bV = 1;
        bY = 0;
           pLCD->iMemoryX = pLCD->iRowStart;
           pLCD->iMemoryY = pLCD->iColStart;
           pLCD->iCurrentHeight = pLCD->iWidth;
           pLCD->iCurrentWidth = pLCD->iHeight;
        break;
     case LCD_ORIENTATION_180:
        bX = bY = 1;
        bV = 0;
        if (pLCD->iColStart != 0)
            pLCD->iMemoryX = pLCD->iColStart;// - 1;
           pLCD->iMemoryY = pLCD->iRowStart;
           pLCD->iCurrentHeight = pLCD->iHeight;
           pLCD->iCurrentWidth = pLCD->iWidth;
        break;
     case LCD_ORIENTATION_270:
        bY = bV = 1;
        bX = 0;
           pLCD->iMemoryX = pLCD->iRowStart;
           pLCD->iMemoryY = pLCD->iColStart;
           pLCD->iCurrentHeight = pLCD->iWidth;
           pLCD->iCurrentWidth = pLCD->iHeight;
        break;
   }
   pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;
#ifdef ARDUINO_ARCH_ESP32
    if (pLCD->iLCDType > LCD_QUAD_SPI) {
        qspiRotate(pLCD, iOrient);
        return 0;
    }
#endif
    if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT && pLCD->iHeight == 240 && pLCD->iWidth == 240) {
        // special issue with memory offsets in certain orientations
        if (pLCD->iOrientation == LCD_ORIENTATION_180) {
            pLCD->iMemoryX = 0; pLCD->iMemoryY = 80;
        } else if (pLCD->iOrientation == LCD_ORIENTATION_270) {
            pLCD->iMemoryX = 80; pLCD->iMemoryY = 0;
        } else {
            pLCD->iMemoryX = pLCD->iMemoryY = 0;
        }
    }
   if (pLCD->iCMDType == CMD_TYPE_SITRONIX_8BIT)
   {
      uint8_t uc = 0;
       if (pLCD->iLCDType == LCD_JD9613 || pLCD->iLCDType == LCD_ILI9342) // x is reversed
           bX = !bX;
      if (bY) uc |= 0x80;
      if (bX) uc |= 0x40;
      if (bV) uc |= 0x20;
       if (pLCD->iLCDType == LCD_ILI9341 || pLCD->iLCDType == LCD_ILI9342) {
          uc |= 8; // R/B inverted from other LCDs
           uc ^= 0x40; // x is inverted too
       }
       if (pLCD->iLCDFlags & FLAGS_FLIPX)
           uc ^= 0x40;
      if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
          uc ^= 0x8;
      spilcdWriteCmdParams(pLCD, 0x36, &uc, 1); // MADCTL
   }
    if (pLCD->iLCDType == LCD_SSD1283A) {
        uint16_t u1=0, u3=0;
        switch (pLCD->iOrientation)
        {
            case LCD_ORIENTATION_0:
                u1 = (pLCD->iHeight == 132) ? 0x2183 : 0x31AF;
                u3 = 0x6830;
                break;
            case LCD_ORIENTATION_90:
                u1 = (pLCD->iHeight == 132) ? 0x2283 : 0x31AF;
                u3 = 0x6828;
                break;
            case LCD_ORIENTATION_180:
                u1 = (pLCD->iHeight == 132) ? 0x2183 : 0x31AF;
                u3 = 0x6800;
                break;
            case LCD_ORIENTATION_270:
                u1 = (pLCD->iHeight == 132) ? 0x2283 : 0x32AF;
                u3 = 0x6828;
                break;
        }
        spilcdWriteCommand(pLCD, 0x1); // output control
        spilcdWriteData8(pLCD, (uint8_t)(u1 >> 8));
        spilcdWriteData8(pLCD, (uint8_t)u1);
        spilcdWriteCommand(pLCD, 0x3); // Entry Mode
        spilcdWriteData8(pLCD, (uint8_t)(u3 >> 8));
        spilcdWriteData8(pLCD, (uint8_t)u3);
    }
    if (pLCD->iLCDType == LCD_ILI9225)
    {
        uint16_t usVal = 0x1000; // starting Entry Mode (0x03) bits
        if (bX == bY) // for 0 and 180, invert the bits
        {
            bX = !bX;
            bY = !bY;
        }
        usVal |= (bV << 3);
        usVal |= (bY << 4);
        usVal |= (bX << 5);
        if (pLCD->iLCDFlags & FLAGS_SWAP_RB)
            usVal ^= 0x1000;
        spilcdWriteCommand16(pLCD, 0x03);
        spilcdWriteData16(pLCD, usVal, DRAW_TO_LCD);
    }

    return 0;
} /* spilcdSetOrientation() */
//
// Fill the frame buffer with a single color
//
int spilcdFill(SPILCD *pLCD, unsigned short usData, int iFlags)
{
#ifdef __AVR__
int cx, tx;
#endif
int x, y;
uint16_t *u16Temp = (uint16_t *)pDMA;

    // make sure we're in landscape mode to use the correct coordinates
    spilcdScrollReset(pLCD);
    if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
        usData = (usData >> 8) | (usData << 8); // swap hi/lo byte for LCD
    }
    if (pLCD->iLCDFlags & FLAGS_MEM_RESTART) {
        // special case for parllel LCD using ESP32 LCD API
        for (x=0; x<pLCD->iCurrentWidth; x++)
            u16Temp[x] = usData;
        for (y=0; y<pLCD->iCurrentHeight; y++)
        {
            spilcdSetPosition(pLCD, 0,y,pLCD->iCurrentWidth,1, iFlags);
            myspiWrite(pLCD, (uint8_t *)u16Temp, pLCD->iCurrentWidth*2, MODE_DATA, iFlags);
        } // for y
        return 0;
    }
    spilcdSetPosition(pLCD, 0,0,pLCD->iCurrentWidth,pLCD->iCurrentHeight, iFlags);
#ifdef __AVR__
    cx = 16;
    tx = pLCD->iCurrentWidth/16;
    for (y=0; y<pLCD->iCurrentHeight; y++)
    {
       u16Temp = (uint16_t *)pDMA;
       for (x=0; x<cx; x++)
       {
// have to do this every time because the buffer gets overrun (no half-duplex mode in Arduino SPI library)
            for (i=0; i<tx; i++)
                u16Temp[i] = usData;
            myspiWrite(pLCD, (uint8_t *)u16Temp, tx*2, MODE_DATA, iFlags); // fill with data byte
       } // for x
    } // for y
#else
    for (y=0; y<pLCD->iCurrentHeight; y+=2)
    {
        // MCUs with more RAM can do it faster
        for (x=0; x<pLCD->iCurrentWidth*2; x++) {
            u16Temp[x] = usData; // data will be overwritten
        }
        myspiWrite(pLCD, (uint8_t *)u16Temp, pLCD->iCurrentWidth*4, MODE_DATA, iFlags);
    } // for y

#endif
    return 0;
} /* spilcdFill() */

//
// Draw a line between 2 points using Bresenham's algorithm
// An optimized version of the algorithm where each continuous run of pixels is written in a
// single shot to reduce the total number of SPI transactions. Perfectly vertical or horizontal
// lines are the most extreme version of this condition and will write the data in a single
// operation.
//
void spilcdDrawLine(SPILCD *pLCD, int x1, int y1, int x2, int y2, unsigned short usColor, int iFlags)
{
    int temp;
    int dx = x2 - x1;
    int dy = y2 - y1;
    int error;
    int xinc, yinc;
    int iLen, x, y;
//#ifndef ESP32_DMA
    int i;
//#endif
    uint16_t *usTemp = (uint16_t *)pDMA, us;

    if (x1 < 0 || x2 < 0 || y1 < 0 || y2 < 0 || x1 >= pLCD->iCurrentWidth || x2 >= pLCD->iCurrentWidth || y1 >= pLCD->iCurrentHeight || y2 >= pLCD->iCurrentHeight)
        return;
    if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
        us = (usColor >> 8) | (usColor << 8); // byte swap for LCD byte order
    } else {
        us = usColor;
    }
    if(abs(dx) > abs(dy)) {
        // X major case
        if(x2 < x1) {
            dx = -dx;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
//#ifdef ESP32_DMA
        for (x=0; x<dx+1; x++) // prepare color data for max length line
            usTemp[x] = us;
//#endif
//        spilcdSetPosition(x1, y1, dx+1, 1); // set the starting position in both X and Y
        y = y1;
        dy = (y2 - y1);
        error = dx >> 1;
        yinc = 1;
        if (dy < 0)
        {
            dy = -dy;
            yinc = -1;
        }
        for(x = x1; x1 <= x2; x1++) {
            error -= dy;
            if (error < 0) // y needs to change, write existing pixels
            {
                error += dx;
		iLen = (x1-x+1);
                spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
//#ifndef ESP32_DMA
	        for (i=0; i<iLen; i++) // prepare color data for max length line
                   usTemp[i] = us;
//#endif
                myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
                y += yinc;
//                spilcdSetPosY(y, 1); // update the y position only
                x = x1+1; // we've already written the pixel at x1
            }
        } // for x1
        if (x != x1) // some data needs to be written
        {
	    iLen = (x1-x);
//#ifndef ESP32_DMA
            for (temp=0; temp<iLen; temp++) // prepare color data for max length line
               usTemp[temp] = us;
//#endif
            spilcdSetPosition(pLCD, x, y, iLen, 1, iFlags);
            myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
        }
    }
    else {
        // Y major case
        if(y1 > y2) {
            dy = -dy;
            temp = x1;
            x1 = x2;
            x2 = temp;
            temp = y1;
            y1 = y2;
            y2 = temp;
        }
//#ifdef ESP32_DMA
        for (x=0; x<dy+1; x++) // prepare color data for max length line
            usTemp[x] = us;
//#endif
//        spilcdSetPosition(x1, y1, 1, dy+1); // set the starting position in both X and Y
        dx = (x2 - x1);
        error = dy >> 1;
        xinc = 1;
        if (dx < 0)
        {
            dx = -dx;
            xinc = -1;
        }
        x = x1;
        for(y = y1; y1 <= y2; y1++) {
            error -= dx;
            if (error < 0) { // x needs to change, write any pixels we traversed
                error += dy;
                iLen = y1-y+1;
//#ifndef ESP32_DMA
      		for (i=0; i<iLen; i++) // prepare color data for max length line
       		    usTemp[i] = us;
//#endif
                spilcdSetPosition(pLCD, x, y, 1, iLen, iFlags);
                myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
                x += xinc;
//                spilcdSetPosX(x, 1); // update the x position only
                y = y1+1; // we've already written the pixel at y1
            }
        } // for y
        if (y != y1) // write the last byte we modified if it changed
        {
	    iLen = y1-y;
//#ifndef ESP32_DMA
            for (i=0; i<iLen; i++) // prepare color data for max length line
               usTemp[i] = us;
//#endif
            spilcdSetPosition(pLCD, x, y, 1, iLen, iFlags);
            myspiWrite(pLCD, (uint8_t*)usTemp, iLen*2, MODE_DATA, iFlags); // write the row we changed
        }
    } // y major case
} /* spilcdDrawLine() */
//
// Decompress one line of 8-bit RLE data
//
unsigned char * DecodeRLE8(unsigned char *s, int iWidth, uint16_t *d, uint16_t *usPalette)
{
unsigned char c;
static unsigned char ucColor, ucRepeat, ucCount;
int iStartWidth = iWidth;
long l;

   if (s == NULL) { // initialize
      ucRepeat = 0;
      ucCount = 0;
      return s;
   }

   while (iWidth > 0)
   {
      if (ucCount) // some non-repeating bytes to deal with
      {  
         while (ucCount && iWidth > 0)
         {  
            ucCount--;
            iWidth--; 
            ucColor = *s++;
            *d++ = usPalette[ucColor];
         } 
         l = (long)s;
         if (l & 1) s++; // compressed data pointer must always be even
      }
      if (ucRepeat == 0 && iWidth > 0) // get a new repeat code or command byte
      {
         ucRepeat = *s++;
         if (ucRepeat == 0) // command code
         {
            c = *s++;
            switch (c)
            {
               case 0: // end of line
                    if (iStartWidth != iWidth) {
                        return s; // true end of line
                    }
                break; // otherwise do nothing because it was from the last line
               case 1: // end of bitmap
                 return s;
               case 2: // move
                 c = *s++; // debug - delta X
                 d += c; iWidth -= c;
                 c = *s++; // debug - delta Y
                 break;
               default: // uncompressed data
                 ucCount = c;
                 break;
            } // switch on command byte
         }
         else
         {
            ucColor = *s++; // get the new colors
         }     
      }
      while (ucRepeat && iWidth > 0)
      {
         ucRepeat--;
         *d++ = usPalette[ucColor];
         iWidth--;
      } // while decoding the current line
   } // while pixels on the current line to draw
   return s;
} /* DecodeRLE8() */

//
// Decompress one line of 4-bit RLE data
//
unsigned char * DecodeRLE4(uint8_t *s, int iWidth, uint16_t *d, uint16_t *usPalette)
{
uint8_t c, ucOdd=0, ucColor;
static uint8_t uc1, uc2, ucRepeat, ucCount;
int iStartWidth = iWidth;

   if (s == NULL) { // initialize this bitmap
      ucRepeat = 0;
      ucCount = 0;
      return s;
   }

   while (iWidth > 0)
   {
      if (ucCount) // some non-repeating bytes to deal with
      {
         while (ucCount > 0 && iWidth > 0)
         {
            ucCount--;
            iWidth--;
            ucColor = *s++;
            uc1 = ucColor >> 4; uc2 = ucColor & 0xf;
            *d++ = usPalette[uc1];
            if (ucCount > 0 && iWidth > 0)
            {
               *d++ = usPalette[uc2];
               ucCount--;
               iWidth--;
            }
         }
         if ((int)(intptr_t)s & 1) {
            s++; // compressed data pointer must always be even
         }
      }
      if (ucRepeat == 0 && iWidth > 0) // get a new repeat code or command byte
      {
         ucRepeat = *s++;
         if (ucRepeat == 0) // command code
         {
            c = *s++;
            switch (c)
            {
               case 0: // end of line
                 if (iStartWidth - iWidth >= 2)
                    return s; // true end of line
                 break; // otherwise do nothing because it was from the last line
               case 1: // end of bitmap
                 return s;
               case 2: // move
                 c = *s++; // debug - delta X
                 d += c; iWidth -= c;
                 c = *s++; // debug - delta Y
                 break;
               default: // uncompressed data
                 ucCount = c;
                 break;
            } // switch on command byte
         }
         else
         {
            ucOdd = 0; // start on an even source pixel
            ucColor = *s++; // get the new colors
            uc1 = ucColor >> 4; uc2 = ucColor & 0xf;
         }     
      }
      while (ucRepeat > 0 && iWidth > 0)
      {
         ucRepeat--;
         iWidth--;
         *d++ = (ucOdd) ? usPalette[uc2] : usPalette[uc1]; 
         ucOdd = !ucOdd;
      } // while decoding the current line
   } // while pixels on the current line to draw
   return s;
} /* DecodeRLE4() */

//
// Draw a 4, 8 or 16-bit Windows uncompressed bitmap onto the display
// Pass the pointer to the beginning of the BMP file
// Optionally stretch to 2x size
// Optimized for drawing to the backbuffer. The transparent color index is only used
// when drawinng to the back buffer. Set it to -1 to disable
// returns -1 for error, 0 for success
//
int spilcdDrawBMP(SPILCD *pLCD, uint8_t *pBMP, int iDestX, int iDestY, int bStretch, int iTransparent, int iFlags)
{
    int iOffBits, iPitch;
    uint16_t *usPalette = (uint16_t *)ucRXBuf;
    uint8_t *pCompressed;
    uint8_t ucCompression;
    int16_t cx, cy, bpp, y; // offset to bitmap data
    int j, x;
    uint16_t *pus, us, *d, *usTemp = (uint16_t *)pDMA; // process a line at a time
    uint8_t bFlipped = false;
    
    if (pBMP[0] != 'B' || pBMP[1] != 'M') // must start with 'BM'
        return -1; // not a BMP file
    cx = pBMP[18] | pBMP[19]<<8;
    cy = pBMP[22] | pBMP[23]<<8;
    ucCompression = pBMP[30]; // 0 = uncompressed, 1/2/4 = RLE compressed
    if (ucCompression > 4) // unsupported feature
        return -1;
    if (cy > 0) // BMP is flipped vertically (typical)
        bFlipped = true;
    else
        cy = -cy;
    bpp = pBMP[28] | pBMP[29]<<8;
    if (bpp != 16 && bpp != 4 && bpp != 8) // must be 4/8/16 bits per pixel
        return -1;
    if (iDestX + cx > pLCD->iCurrentWidth || iDestX < 0 || cx < 0)
        return -1; // invalid
    if (iDestY + cy > pLCD->iCurrentHeight || iDestY < 0 || cy < 0)
        return -1;
    if (iTransparent != -1) // transparent drawing can only happen on the back buffer
        iFlags = DRAW_TO_RAM;
    iOffBits = pBMP[10] | pBMP[11]<<8;
    iPitch = (cx * bpp) >> 3; // bytes per line
    iPitch = (iPitch + 3) & 0xfffc; // must be dword aligned
    // Get the palette as RGB565 values (if there is one)
    if (bpp == 4 || bpp == 8)
    {
        uint16_t r, g, b, us;
        int iOff, iColors;
        iColors = pBMP[46]; // colors used BMP field
        if (iColors == 0 || iColors > (1<<bpp))
            iColors = (1 << bpp); // full palette
        iOff = iOffBits - (4 * iColors); // start of color palette
        for (x=0; x<iColors; x++)
        {
            b = pBMP[iOff++];
            g = pBMP[iOff++];
            r = pBMP[iOff++];
            iOff++; // skip extra byte
            r >>= 3;
            us = (r  << 11);
            g >>= 2;
            us |= (g << 5);
            us |= (b >> 3);
            if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                usPalette[x] = (us >> 8) | (us << 8); // swap byte order for writing to the display
            } else {
                usPalette[x] = us;
            }
        }
    }
    if (ucCompression == 2) // need to do it differently for RLE compressed
    {
    uint16_t *d;
    int y, iStartY, iEndY, iDeltaY;
        
       pCompressed = &pBMP[iOffBits]; // start of compressed data
       if (bFlipped)
       {  
          iStartY = iDestY + cy - 1;
          iEndY = iDestY - 1;
          iDeltaY = -1;
       }
       else
       {  
          iStartY = iDestY;
          iEndY = iDestY + cy;
          iDeltaY = 1;
       }
       DecodeRLE4(NULL, 0,NULL,NULL); // initialize
       DecodeRLE8(NULL, 0,NULL,NULL);
       for (y=iStartY; y!= iEndY; y += iDeltaY)
       { 
          d = (uint16_t *)pDMA;
          spilcdSetPosition(pLCD, iDestX, y, cx, 1, iFlags);
          if (bpp == 4)
             pCompressed = DecodeRLE4(pCompressed, cx, d, usPalette);
          else
             pCompressed = DecodeRLE8(pCompressed, iPitch, d, usPalette);
          myspiWrite(pLCD, (uint8_t *)d, cx*2, MODE_DATA, iFlags);
       }
       return 0;
    } // RLE compressed

    if (bFlipped)
    {
        iOffBits += (cy-1) * iPitch; // start from bottom
        iPitch = -iPitch;
    }

        if (bStretch)
        {
            spilcdSetPosition(pLCD, iDestX, iDestY, cx*2, cy*2, iFlags);
            for (y=0; y<cy; y++)
            {
                pus = (uint16_t *)&pBMP[iOffBits + (y * iPitch)]; // source line
                for (j=0; j<2; j++) // for systems without half-duplex, we need to prepare the data for each write
                {
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (bpp == 16)
                    {
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x++)
                            {
                                us = pus[x];
                                if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {

                                    d[0] = d[1] = (us >> 8) | (us << 8); // swap byte order
                                } else {
                                    d[0] = d[1] = us;
                                }
                                d += 2;
                            } // for x
                        }
                        else
                        {
                            for (x=0; x<cx; x++)
                            {
                                us = pus[x];
                                if (us != (uint16_t)iTransparent)
                                    if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                                        d[0] = d[1] = (us >> 8) | (us << 8); // swap byte order
                                    } else {
                                        d[0] = d[1] = us;
                                    }
                                d += 2;
                            } // for x
                        }
                    }
                    else if (bpp == 8)
                    {
                        uint8_t *s = (uint8_t *)pus;
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x++)
                            {
                                d[0] = d[1] = usPalette[*s++];
                                d += 2;
                            }
                        }
                        else
                        {
                            for (x=0; x<cx; x++)
                            {
                                uint8_t uc = *s++;
                                if (uc != (uint8_t)iTransparent)
                                    d[0] = d[1] = usPalette[uc];
                                d += 2;
                            }
                        }
                    }
                    else // 4 bpp
                    {
                        uint8_t uc, *s = (uint8_t *)pus;
                        if (iTransparent == -1) // no transparency
                        {
                            for (x=0; x<cx; x+=2)
                            {
                                uc = *s++;
                                d[0] = d[1] = usPalette[uc >> 4];
                                d[2] = d[3] = usPalette[uc & 0xf];
                                d += 4;
                            }
                        }
                        else
                        {
                            for (x=0; x<cx; x+=2)
                            {
                                uc = *s++;
                                if ((uc >> 4) != (uint8_t)iTransparent)
                                    d[0] = d[1] = usPalette[uc >> 4];
                                if ((uc & 0xf) != (uint8_t)iTransparent)
                                    d[2] = d[3] = usPalette[uc & 0xf];
                                d += 4;
                            }
                        }
                    }
                    if (iFlags & DRAW_TO_LCD)
                        spilcdWriteDataBlock(pLCD, (uint8_t *)usTemp, cx*4, iFlags); // write the same line twice
                } // for j
            } // for y
        } // 2:1
        else // 1:1
        {
            spilcdSetPosition(pLCD, iDestX, iDestY, cx, cy, iFlags);
            for (y=0; y<cy; y++)
            {
                pus = (uint16_t *)&pBMP[iOffBits + (y * iPitch)]; // source line
                if (bpp == 16)
                {
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y * pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x++)
                        {
                           us = *pus++;
                           if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                               *d++ = (us >> 8) | (us << 8); // swap byte order
                           } else {
                               *d++ = us;
                           }
                        }
                    }
                    else // skip transparent pixels
                    {
                        for (x=0; x<cx; x++)
                        {
                            us = *pus++;
                            if (us != (uint16_t)iTransparent)
                                if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
                                 d[0] = (us >> 8) | (us << 8); // swap byte order
                                } else {
                                  d[0] = us;
                                }
                            d++;
                        }
                    }
                }
                else if (bpp == 8)
                {
                    uint8_t uc, *s = (uint8_t *)pus;
                    if (iFlags & DRAW_TO_LCD)
                        d = usTemp;
                    else
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x++)
                        {
                            *d++ = usPalette[*s++];
                        }
                    }
                    else
                    {
                        for (x=0; x<cx; x++)
                        {
                            uc = *s++;
                            if (uc != iTransparent)
                                d[0] = usPalette[*s++];
                            d++;
                        }
                    }
                }
                else // 4 bpp
                {
                    uint8_t uc, *s = (uint8_t *)pus;
                    if (iFlags & DRAW_TO_LCD)
                        d = (uint16_t *)pDMA;
                    else // write to the correct spot directly to save time
                        d = (uint16_t *)&pLCD->pBackBuffer[pLCD->iOffset + (y*pLCD->iScreenPitch)];
                    if (iTransparent == -1) // no transparency
                    {
                        for (x=0; x<cx; x+=2)
                        {
                            uc = *s++;
                            *d++ = usPalette[uc >> 4];
                            *d++ = usPalette[uc & 0xf];
                        }
                    }
                    else // check transparent color
                    {
                        for (x=0; x<cx; x+=2)
                        {
                            uc = *s++;
                            if ((uc >> 4) != iTransparent)
                               d[0] = usPalette[uc >> 4];
                            if ((uc & 0xf) != iTransparent)
                               d[1] = usPalette[uc & 0xf];
                            d += 2;
                        }
                    }
                }
                if (iFlags & DRAW_TO_LCD)
                    spilcdWriteDataBlock(pLCD, (uint8_t *)pDMA, cx*2, iFlags);
            } // for y
        } // 1:1
    return 0;
} /* spilcdDrawBMP() */

#ifndef __AVR__
//
// Returns the current backbuffer address
//
uint16_t * spilcdGetBuffer(SPILCD *pLCD)
{
    return (uint16_t *)pLCD->pBackBuffer;
}
//
// Set the back buffer
//
void spilcdSetBuffer(SPILCD *pLCD, void *pBuffer)
{
    pLCD->pBackBuffer = (uint8_t *)pBuffer;
    pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;
    pLCD->iOffset = 0;
    pLCD->iMaxOffset = pLCD->iScreenPitch * pLCD->iCurrentHeight; // can't write past this point
    pLCD->iWindowX = pLCD->iWindowY = 0; // current window = whole display
    pLCD->iWindowCX = pLCD->iCurrentWidth;
    pLCD->iWindowCY = pLCD->iCurrentHeight;

} /* spilcdSetBuffer() */

//
// Allocate the back buffer for delayed rendering operations
// returns -1 for failure, 0 for success
//
int spilcdAllocBackbuffer(SPILCD *pLCD)
{
    if (pLCD->pBackBuffer != NULL) // already allocated
        return -1;
    pLCD->iScreenPitch = pLCD->iCurrentWidth * 2;
    pLCD->pBackBuffer = (uint8_t *)malloc(pLCD->iScreenPitch * pLCD->iCurrentHeight);
    if (pLCD->pBackBuffer == NULL) // no memory
        return -1;
    memset(pLCD->pBackBuffer, 0, pLCD->iScreenPitch * pLCD->iCurrentHeight);
    pLCD->iOffset = 0; // starting offset
    pLCD->iMaxOffset = pLCD->iScreenPitch * pLCD->iCurrentHeight; // can't write past this point
    pLCD->iWindowX = pLCD->iWindowY = 0; // current window = whole display
    pLCD->iWindowCX = pLCD->iCurrentWidth;
    pLCD->iWindowCY = pLCD->iCurrentHeight;
    return 0;
}
//
// Free the back buffer
//
void spilcdFreeBackbuffer(SPILCD *pLCD)
{
    if (pLCD->pBackBuffer)
    {
        free(pLCD->pBackBuffer);
        pLCD->pBackBuffer = NULL;
    }
}
//
// Rotate a 1-bpp mask image around a given center point
// valid angles are 0-359
//
void spilcdRotateBitmap(uint8_t *pSrc, uint8_t *pDest, int iBpp, int iWidth, int iHeight, int iPitch, int iCenterX, int iCenterY, int iAngle)
{
int32_t i, x, y;
int32_t tx, ty, sa, ca;
uint8_t *s, *d, uc, ucMask;
uint16_t *uss, *usd;

    if (pSrc == NULL || pDest == NULL || iWidth < 1 || iHeight < 1 || iPitch < 1 || iAngle < 0 || iAngle > 359 || iCenterX < 0 || iCenterX >= iWidth || iCenterY < 0 || iCenterY >= iHeight || (iBpp != 1 && iBpp != 16))
        return;
    // since we're rotating from dest back to source, reverse the angle
    iAngle = 360 - iAngle;
    if (iAngle == 360) // just copy src to dest
    {
        memcpy(pDest, pSrc, iHeight * iPitch);
        return;
    }
    // Create a quicker lookup table for sin/cos pre-multiplied at the given angle
    sa = (int32_t)i16SineTable[iAngle]; // sine of given angle
    ca = (int32_t)i16SineTable[iAngle+90]; // cosine of given angle
    for (y=0; y<iHeight; y++)
    {
        int32_t siny = (sa * (y-iCenterY));
        int32_t cosy = (ca * (y-iCenterY));
        d = &pDest[y * iPitch];
        usd = (uint16_t *)d;
        ucMask = 0x80;
        uc = 0;
        for (x=0; x<iWidth; x++)
        {
            // Rotate from the destination pixel back to the source to not have gaps
            // x' = cos*x - sin*y, y' = sin*x + cos*y
            tx = iCenterX + (((ca * (x-iCenterX)) - siny) >> 15);
            ty = iCenterY + (((sa * (x-iCenterX)) + cosy) >> 15);
            if (iBpp == 1)
            {
                if (tx > 0 && ty > 0 && tx < iWidth && ty < iHeight) // check source pixel
                {
                    s = &pSrc[(ty*iPitch)+(tx>>3)];
                    if (s[0] & (0x80 >> (tx & 7)))
                        uc |= ucMask; // set destination pixel
                }
                ucMask >>= 1;
                if (ucMask == 0) // write the byte into the destination bitmap
                {
                    ucMask = 0x80;
                    *d++ = uc;
                    uc = 0;
                }
            }
            else // 16-bpp
            {
                if (tx >= 0 && ty >= 0 && tx < iWidth && ty < iHeight) // check source pixel
                {
                    uss = (uint16_t *)&pSrc[(ty*iPitch)+(tx*2)];
                    *usd = uss[0]; // copy the pixel
                }
                usd++;
            }
        }
        if (iBpp == 1 && ucMask != 0x80) // store partial byte
            *d++ = uc;
    } // for y
} /* spilcdRotateMask() */
#endif // !__AVR__
void spilcdScroll1Line(SPILCD *pLCD, int iAmount)
{
int i, iCount;
int iPitch = pLCD->iCurrentWidth * sizeof(uint16_t);
uint16_t *d, us;
    if (pLCD == NULL || pLCD->pBackBuffer == NULL)
        return;
    iCount = (pLCD->iCurrentHeight - iAmount) * iPitch;
    memmove(pLCD->pBackBuffer, &pLCD->pBackBuffer[iPitch*iAmount], iCount);
    d = (uint16_t *)&pLCD->pBackBuffer[iCount];
    if (!(pLCD->iLCDFlags & FLAGS_SWAP_COLOR)) {
        us = (pLCD->iBG >> 8) | (pLCD->iBG << 8);
    } else {
        us = pLCD->iBG;
    }
    for (i=0; i<iAmount * pLCD->iCurrentWidth; i++) {
        *d++ = us;
    }
} /* obdScroll1Line() */

#if defined( ARDUINO_M5Stick_C ) || defined (ARDUINO_M5STACK_CORE2) || defined (ARDUINO_M5STACK_CORES3)
void Write1Byte( uint8_t Addr ,  uint8_t Data )
{
    Wire1.beginTransmission(0x34);
    Wire1.write(Addr);
    Wire1.write(Data);
    Wire1.endTransmission();
}
uint8_t Read8bit( uint8_t Addr )
{
    Wire1.beginTransmission(0x34);
    Wire1.write(Addr);
    Wire1.endTransmission();
    Wire1.requestFrom(0x34, 1);
    return Wire1.read();
}
#endif // both Core2 + StickC+

#if defined (ARDUINO_M5STACK_CORES3)
void CoreS3AxpPowerUp(void)
{
uint8_t u8;
const uint8_t u8InitList[] = {
      0x90, 0xBF,  // LDOS ON/OFF control 0
      0x92, 18 -5, // ALDO1 set to 1.8v // for AW88298
      0x93, 33 -5, // ALDO2 set to 3.3v // for ES7210 (mic)
      0x94, 33 -5, // ALDO3 set to 3.3v // for camera
      0x95, 33 -5, // ALDO4 set to 3.3v // for TF card slot
      0x27, 0x00, // PowerKey Hold=1sec / PowerOff=4sec
      0x69, 0x11, // CHGLED setting
      0x10, 0x30, // PMU common config
};
    Wire1.begin(12, 11);
    Wire1.setClock(400000);

    // turn on power boost of SY7088
    Wire1.beginTransmission(0x58); // AW9523B I/O expander
    Wire1.write(0x3); // P1
    Wire1.endTransmission();
    Wire1.requestFrom(0x58, 1);
    u8 = Wire1.read(); // current value
    u8 |= 0x83; // BOOST_EN (0x80) and take LCD+camera out of RESET (P1_0,1 = 3)
    Wire1.beginTransmission(0x58);
    Wire1.write(0x3);
    Wire1.write(u8);
    Wire1.endTransmission();

    // Take Microphone and Touch controller out of reset
    Wire1.beginTransmission(0x58); // AW9523B I/O expander
    Wire1.write(0x2); // P0 
    Wire1.endTransmission();
    Wire1.requestFrom(0x58, 1);
    u8 = Wire1.read(); // current value
    u8 |= 0x5; // MIC out of RST (P0_2 = 4) and touch out of RST (P0_0 = 1)
    Wire1.beginTransmission(0x58);
    Wire1.write(0x2); // P0
    Wire1.write(u8);
    Wire1.endTransmission();

    //AXP2101 34H
    for (int i=0; i<sizeof(u8InitList); i+=2) {
        Write1Byte(u8InitList[i], u8InitList[i+1]);
    }
    // enable LCD backlight and set brightness
    Wire1.beginTransmission(0x34); // AXP2101
    Wire1.write(0x90); // LDOS ON/OFF control
    Wire1.endTransmission();
    Wire1.requestFrom(0x34, 1);
    u8 = Wire1.read(); // current value
    u8 |= 0x80; // LCD_BL LDO enable
    Wire1.beginTransmission(0x34);
    Wire1.write(0x90);
    Wire1.write(u8);
    Wire1.endTransmission();
    Write1Byte(0x99, 26); // set brightness PWM (20=dim, 29=bright)
} /* CoreS3AxpPowerUp() */

#endif // CORES3

#if defined (ARDUINO_M5STACK_CORE2)
typedef enum
{
    kMBusModeOutput = 0,  // powered by USB or Battery
    kMBusModeInput = 1  // powered by outside input
} mbus_mode_t;

enum CHGCurrent{
    kCHG_100mA = 0,
    kCHG_190mA,
    kCHG_280mA,
    kCHG_360mA,
    kCHG_450mA,
    kCHG_550mA,
    kCHG_630mA,
    kCHG_700mA,
    kCHG_780mA,
    kCHG_880mA,
    kCHG_960mA,
    kCHG_1000mA,
    kCHG_1080mA,
    kCHG_1160mA,
    kCHG_1240mA,
    kCHG_1320mA,
};
void SetLed(uint8_t state)
{
    uint8_t reg_addr=0x94;
    uint8_t data;
    data=Read8bit(reg_addr);

    if(state)
    {
      data=data&0XFD;
    }
    else
    {
      data|=0X02;
    }

    Write1Byte(reg_addr,data);
}

void SetDCVoltage(uint8_t number, uint16_t voltage)
{
    uint8_t addr;
    if (number > 2)
        return;
    voltage = (voltage < 700) ? 0 : (voltage - 700) / 25;
    switch (number)
    {
    case 0:
        addr = 0x26;
        break;
    case 1:
        addr = 0x25;
        break;
    case 2:
        addr = 0x27;
        break;
    }
    Write1Byte(addr, (Read8bit(addr) & 0X80) | (voltage & 0X7F));
}
void SetESPVoltage(uint16_t voltage)
{
    if (voltage >= 3000 && voltage <= 3400)
    {
        SetDCVoltage(0, voltage);
    }
}
void SetLDOVoltage(uint8_t number, uint16_t voltage)
{
    voltage = (voltage > 3300) ? 15 : (voltage / 100) - 18;
    switch (number)
    {
    //uint8_t reg, data;
    case 2:
        Write1Byte(0x28, (Read8bit(0x28) & 0X0F) | (voltage << 4));
        break;
    case 3:
        Write1Byte(0x28, (Read8bit(0x28) & 0XF0) | voltage);
        break;
    }
}

void SetLcdVoltage(uint16_t voltage)
{
    if (voltage >= 2500 && voltage <= 3300)
    {
        SetDCVoltage(2, voltage);
    }
}
void SetLCDRSet(bool state)
{
    uint8_t reg_addr = 0x96;
    uint8_t gpio_bit = 0x02;
    uint8_t data;
    data = Read8bit(reg_addr);

    if (state)
    {
        data |= gpio_bit;
    }
    else
    {
        data &= ~gpio_bit;
    }

    Write1Byte(reg_addr, data);
}

void SetLDOEnable(uint8_t number, bool state)
{
    uint8_t mark = 0x01;
    if ((number < 2) || (number > 3))
        return;

    mark <<= number;
    if (state)
    {
        Write1Byte(0x12, (Read8bit(0x12) | mark));
    }
    else
    {
        Write1Byte(0x12, (Read8bit(0x12) & (~mark)));
    }
}
void SetBusPowerMode(uint8_t state)
{
    uint8_t data;
    if (state == 0)
    {
        data = Read8bit(0x91);
        Write1Byte(0x91, (data & 0X0F) | 0XF0);

        data = Read8bit(0x90);
        Write1Byte(0x90, (data & 0XF8) | 0X02); //set GPIO0 to LDO OUTPUT , pullup N_VBUSEN to disable supply from BUS_5V

        data = Read8bit(0x91);

        data = Read8bit(0x12);         //read reg 0x12
        Write1Byte(0x12, data | 0x40); //set EXTEN to enable 5v boost
    }
    else
    {
        data = Read8bit(0x12);         //read reg 0x10
        Write1Byte(0x12, data & 0XBF); //set EXTEN to disable 5v boost
        //delay(2000);

        data = Read8bit(0x90);
        Write1Byte(0x90, (data & 0xF8) | 0X01); //set GPIO0 to float , using enternal pulldown resistor to enable supply from BUS_5VS
    }
}
void SetCHGCurrent(uint8_t state)
{
    uint8_t data = Read8bit(0x33);
    data &= 0xf0;
    data = data | ( state & 0x0f );
    Write1Byte(0x33,data);
}

// Backlight control
void SetDCDC3(bool bPower)
{
    uint8_t buf = Read8bit(0x12);
    if (bPower == true)
        buf = (1 << 1) | buf;
    else
        buf = ~(1 << 1) & buf;
    Write1Byte(0x12, buf);
}
void Core2AxpPowerUp()
{
    Wire1.end();
    Wire1.begin(21, 22);
    Wire1.setClock(400000);

    //AXP192 30H
    Write1Byte(0x30, (Read8bit(0x30) & 0x04) | 0X02);
    //Serial.printf("axp: vbus limit off\n");

    //AXP192 GPIO1:OD OUTPUT
    Write1Byte(0x92, Read8bit(0x92) & 0xf8);
    //Serial.printf("axp: gpio1 init\n");

    //AXP192 GPIO2:OD OUTPUT
    Write1Byte(0x93, Read8bit(0x93) & 0xf8);
    //Serial.printf("axp: gpio2 init\n");

    //AXP192 RTC CHG
    Write1Byte(0x35, (Read8bit(0x35) & 0x1c) | 0xa2);
    //Serial.printf("axp: rtc battery charging enabled\n");
    
    SetESPVoltage(3350);
    //Serial.printf("axp: esp32 power voltage was set to 3.35v\n");

    SetLcdVoltage(2800);
    //Serial.printf("axp: lcd backlight voltage was set to 2.80v\n");

    SetLDOVoltage(2, 3300); //Periph power voltage preset (LCD_logic, SD card)
    //Serial.printf("axp: lcd logic and sdcard voltage preset to 3.3v\n");

    SetLDOVoltage(3, 2000); //Vibrator power voltage preset
    //Serial.printf("axp: vibrator voltage preset to 2v\n");

    SetLDOEnable(2, true);
    SetDCDC3(true); // LCD backlight
    SetLed(true);

    SetCHGCurrent(kCHG_100mA);
    //SetAxpPriphPower(1);
    //Serial.printf("axp: lcd_logic and sdcard power enabled\n\n");

    //pinMode(39, INPUT_PULLUP);
    
    //AXP192 GPIO4
    Write1Byte(0X95, (Read8bit(0x95) & 0x72) | 0X84);

    Write1Byte(0X36, 0X4C);

    Write1Byte(0x82,0xff);

    SetLCDRSet(0);
    delay(100);
    SetLCDRSet(1);
    delay(100);
    // I2C_WriteByteDataAt(0X15,0XFE,0XFF);

    // axp: check v-bus status
    if(Read8bit(0x00) & 0x08) {
        Write1Byte(0x30, Read8bit(0x30) | 0x80);
        // if v-bus can use, disable M-Bus 5V output to input
        SetBusPowerMode(kMBusModeInput);
    }else{
        // if not, enable M-Bus 5V output
        SetBusPowerMode(kMBusModeOutput);
    }
} /* Core2AxpPowerUp() */
#endif // Core2

#if defined( ARDUINO_M5Stick_C )
void AxpBrightness(uint8_t brightness)
{
    if (brightness > 12)
    {
        brightness = 12;
    }
    uint8_t buf = Read8bit( 0x28 );
    Write1Byte( 0x28 , ((buf & 0x0f) | (brightness << 4)) );
}
void AxpPowerUp()
{
    Wire1.begin(21, 22);
    Wire1.setClock(400000);
    // Set LDO2 & LDO3(TFT_LED & TFT) 3.0V
    Write1Byte(0x28, 0xcc);

    // Set ADC sample rate to 200hz
    Write1Byte(0x84, 0b11110010);
   
    // Set ADC to All Enable
    Write1Byte(0x82, 0xff);

    // Bat charge voltage to 4.2, Current 100MA
    Write1Byte(0x33, 0xc0);

    // Depending on configuration enable LDO2, LDO3, DCDC1, DCDC3.
    byte buf = (Read8bit(0x12) & 0xef) | 0x4D;
//    if(disableLDO3) buf &= ~(1<<3);
//    if(disableLDO2) buf &= ~(1<<2);
//    if(disableDCDC3) buf &= ~(1<<1);
//    if(disableDCDC1) buf &= ~(1<<0);
    Write1Byte(0x12, buf);
     // 128ms power on, 4s power off
    Write1Byte(0x36, 0x0C);

    if (1) //if(!disableRTC)
    {
        // Set RTC voltage to 3.3V
        Write1Byte(0x91, 0xF0);

        // Set GPIO0 to LDO
        Write1Byte(0x90, 0x02);
    }

    // Disable vbus hold limit
    Write1Byte(0x30, 0x80);

    // Set temperature protection
    Write1Byte(0x39, 0xfc);

    // Enable RTC BAT charge
//    Write1Byte(0x35, 0xa2 & (disableRTC ? 0x7F : 0xFF));
    Write1Byte(0x35, 0xa2);
     // Enable bat detection
    Write1Byte(0x32, 0x46);

    // Set Power off voltage 3.0v
    Write1Byte(0x31 , (Read8bit(0x31) & 0xf8) | (1 << 2));

} /* AxpPowerUp() */
#endif // ARDUINO_M5Stick_C
//
// Full duplex SPI transfer for the touch controller
//
void rtSPIXfer(SPILCD *pLCD, uint8_t ucCMD, uint8_t *pRXBuf, int iLen)
{
#ifdef __LINUX__
	// not supported
    (void)pLCD;
    (void)ucCMD;
    (void)pRXBuf;
    (void)iLen;
#else
int i, j;
uint8_t ucIn, ucOut;
uint8_t ucTemp[4];

   ucTemp[0] = ucCMD;
   ucTemp[1] = ucTemp[2] = 0;

 // do simultaneous read+write on the SPI touch controller bus

   if (pLCD->iRTMOSI != 0xff) { // use bit bang
       for (i=0; i<iLen; i++) { // for each byte
           ucOut = ucTemp[i];
           ucIn = 0;
           for (j=0; j<8; j++) { // for each bit
               ucIn <<= 1;
               digitalWrite(pLCD->iRTMOSI, (ucOut & 0x80) >> 7);
               __asm__ __volatile__ ("nop");
               __asm__ __volatile__ ("nop");
//               delayMicroseconds(1);
               digitalWrite(pLCD->iRTCLK, 1);
               __asm__ __volatile__ ("nop");
               __asm__ __volatile__ ("nop");
//               delayMicroseconds(1);
               ucIn |= digitalRead(pLCD->iRTMISO);
               digitalWrite(pLCD->iRTCLK, 0);
               ucOut <<= 1;
           } // for each bit
        pRXBuf[i] = ucIn; // store the received data
        } // for each byte
    } else { // shared SPI bus
#ifdef ARDUINO_ARCH_ESP32
        spi_transaction_t t;
        memcpy(pRXBuf, ucTemp, iLen); // Arduino only allows duplex overwrite
        digitalWrite(pLCD->iRTCS, LOW);
        pLCD->pSPI->beginTransaction(SPISettings(2000000, MSBFIRST, 0));
        pLCD->pSPI->transfer(pRXBuf, iLen);
        pLCD->pSPI->endTransaction();
        digitalWrite(pLCD->iRTCS, HIGH);
#endif // ESP32
    }
#endif // !__LINUX__   
} /* rtSPIXfer() */

#ifdef __cplusplus
void BB_SPI_LCD::setScrollPosition(int iLines)
{
uint8_t ucTemp[4];

    ucTemp[0] = (uint8_t)(iLines >> 8);
    ucTemp[1] = (uint8_t)iLines;

    if (_lcd.iLCDType > LCD_QUAD_SPI) {
#ifdef ARDUINO_ARCH_ESP32
       qspiSendCMD(&_lcd, 0x37, ucTemp, 2);
#endif // ESP32
    } else { // SPI and parallel LCDs
        if (_lcd.iLCDType == LCD_SSD1351) {
            spilcdWriteCommand(&_lcd, 0xa1); // set scroll start line
            spilcdWriteData8(&_lcd, iLines);
            return;
        } else if (_lcd.iLCDType == LCD_SSD1331) {
            spilcdWriteCommand(&_lcd, 0xa1);
            spilcdWriteCommand(&_lcd, iLines);
            return;
        } else {
            if (_lcd.iLCDType == LCD_ILI9341 || _lcd.iLCDType == LCD_ILI9342 || _lcd.iLCDType == LCD_ST7735R || _lcd.iLCDType == LCD_ST7789 || _lcd.iLCDType == LCD_ST7789_135 || _lcd.iLCDType == LCD_ST7735S) {
               spilcdWriteCmdParams(&_lcd, 0x37, ucTemp, 2);
            } else {
               spilcdWriteData16(&_lcd, iLines >> 8, DRAW_TO_LCD);
               spilcdWriteData16(&_lcd, iLines & -1, DRAW_TO_LCD);
            }
        }
    } // spi / parallel
} /* setScrollPosition() */

int BB_SPI_LCD::rtInit(SPIClass &spi, uint8_t u8CS)
{
    if (u8CS != 0xff) _lcd.iRTCS = u8CS;
    _lcd.pSPI = &spi;
    _lcd.iRTMOSI = 0xff; // disable bit banging logic
    pinMode(_lcd.iRTCS, OUTPUT);
    return 1;
}
//
// C++ Class implementation
//
int BB_SPI_LCD::rtInit(uint8_t u8MOSI, uint8_t u8MISO, uint8_t u8CLK, uint8_t u8CS)
{
uint8_t ucTemp[4];

   if (u8MOSI != 255) {
      _lcd.iRTMOSI = u8MOSI;
   }
   if (u8MISO != 255) {
      _lcd.iRTMISO = u8MISO;
   }
   if (u8CLK != 255) {
      _lcd.iRTCLK = u8CLK;
   }
   if (u8CS != 255) {
      _lcd.iRTCS = u8CS;
   }
   //Serial.printf("rtInit on pins %d, %d, %d, %d\n", _lcd.iRTMOSI, _lcd.iRTMISO, _lcd.iRTCLK, _lcd.iRTCS);
   if (_lcd.iRTMOSI != _lcd.iMOSIPin) { // use bit bang for the touch controller
       //Serial.println("Resistive touch bit bang");
       pinMode(_lcd.iRTMOSI, OUTPUT);
       pinMode(_lcd.iRTMISO, INPUT);
       pinMode(_lcd.iRTCLK, OUTPUT);
       if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99)
           pinMode(_lcd.iRTCS, OUTPUT);
   } else if (_lcd.iRTCS != 255) { // CYD_24R
       pinMode(_lcd.iRTCS, OUTPUT);
   }
   return BB_ERROR_SUCCESS;
} /* rtInit() */

// Return the average of the closest 2 of 3 values
static int rtAVG(int *pI)
{
  int da, db, dc;
  int avg = 0;
  if ( pI[0] > pI[1] ) da = pI[0] - pI[1]; else da = pI[1] - pI[0];
  if ( pI[0] > pI[2] ) db = pI[0] - pI[2]; else db = pI[2] - pI[0];
  if ( pI[2] > pI[1] ) dc = pI[2] - pI[1]; else dc = pI[1] - pI[2]; 
        
  if ( da <= db && da <= dc ) avg = (pI[0] + pI[1]) >> 1;
  else if ( db <= da && db <= dc ) avg = (pI[0] + pI[2]) >> 1;
  else avg = (pI[1] + pI[2]) >> 1;

  return avg;
} /* rtAVG() */

//
// returns 1 for a touch position available
// or 0 for no touch or invalid parameter
//
int BB_SPI_LCD::rtReadTouch(TOUCHINFO *ti)
{
// commands and SPI transaction filler to read 3 byte response for x/y
uint8_t ucTemp[4];
int iOrient, x, y, xa[3], ya[3], x1, y1, z, z1, z2;
const int iOrients[4] = {0,90,180,270};

    iOrient = iOrients[_lcd.iOrientation];
    iOrient += _lcd.iRTOrientation; // touch sensor relative to default angle
    iOrient %= 360;

    if (ti == NULL)
        return 0;
    if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99)
        digitalWrite(_lcd.iRTCS, 0); // active CS
    // read the "pressure" value to see if there is a touch
    rtSPIXfer(&_lcd, 0xb1, ucTemp, 3);
    rtSPIXfer(&_lcd, 0xc1, ucTemp, 3);
    z1 = (int)((ucTemp[2] + (ucTemp[1]<<8)) >> 3);
    z = z1 + 4095;
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    z2 = (int)((ucTemp[2] + (ucTemp[1]<<8)) >> 3);
    z -= z2;
    if (z > _lcd.iRTThreshold) {
        ti->count = 0;
        if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99)
            digitalWrite(_lcd.iRTCS, 1); // inactive CS
        return 0; // not a valid pressure reading
    } else {
      //Serial.printf("pressure = %d\n", z);
    }
    ti->count = 1; // only 1 touch point possible
    z = (_lcd.iRTThreshold - z)/16;
    ti->pressure[0] = (uint8_t)z;

    // read the X and Y values 3 times because they jitter
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3); // first Y is always noisy
    rtSPIXfer(&_lcd, 0xd1, ucTemp, 3);
    xa[0] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    ya[0] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0xd1, ucTemp, 3);
    xa[1] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x91, ucTemp, 3);
    ya[1] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0xd0, ucTemp, 3); // last X, power down
    xa[2] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    rtSPIXfer(&_lcd, 0x00, ucTemp, 3); // last Y
    ya[2] = ((ucTemp[2] + (ucTemp[1]<<8)) >> 4);
    // take the average of the closest values
    x = rtAVG(xa);
    y = rtAVG(ya);
    // since we know the display size and orientation, scale the coordinates
    // to match. The 0 orientation corresponds to flipped X and correct Y
    switch (iOrient) {
        case 0:
        case 180:
            y1 = (1900 - y)*_lcd.iCurrentHeight;
            y1 /= 1780;
            if (y1 < 0) y1 = 0;
            else if (y1 >= _lcd.iCurrentHeight) y1 = _lcd.iCurrentHeight-1;
            x1 = (1950 - x) * _lcd.iCurrentWidth;
            x1 /= 1750;
            if (x1 < 0) x1 = 0;
            else if (x1 >= _lcd.iCurrentWidth) x1 = _lcd.iCurrentWidth-1;
            break;
        case 90:
        case 270:
            x1 = (1900 - y) * _lcd.iCurrentWidth;
            x1 /= 1780;
            if (x1 < 0) x1 = 0;
            else if (x1 >= _lcd.iCurrentWidth) x1 = _lcd.iCurrentWidth-1;
            y1 = (1950 - x) * _lcd.iCurrentHeight;
            y1 /= 1750;
            if (y1 < 0) y1 = 0;
            else if (y1 >= _lcd.iCurrentHeight) y1 = _lcd.iCurrentHeight-1;
            break;
    } // switch on orientation
    if (iOrient == 0 || iOrient == 90) {
        x1 = _lcd.iCurrentWidth - 1 - x1;
        y1 = _lcd.iCurrentHeight - 1 - y1;
    }
    ti->x[0] = x1;
    ti->y[0] = y1;
    if (_lcd.iRTCS >= 0 && _lcd.iRTCS < 99)
        digitalWrite(_lcd.iRTCS, 1); // inactive CS
    //delay(10); // don't let the user try to read samples too quickly
    return 1;
} /* rtReadTouch() */

void Write9Bits(const BB_RGB *pPanel, uint8_t dc, uint8_t *pData, int iCount)
{
int i, j;
uint8_t uc;

   for (i=0; i<iCount; i++) {
       uc = *pData++;
       digitalWrite(pPanel->sck, LOW);
       digitalWrite(pPanel->mosi, dc); // D/C bit
       delayMicroseconds(1);
       digitalWrite(pPanel->sck, HIGH); // clock D/C bit
       delayMicroseconds(1);
       for (int j=0; j<8; j++) {
          digitalWrite(pPanel->sck, LOW);
          digitalWrite(pPanel->mosi, (uc >> 7) & 1);
          delayMicroseconds(1);
          digitalWrite(pPanel->sck, HIGH); // each bit
          uc <<= 1;
          delayMicroseconds(1);
       } // for j
   } // for i
} /* Write9Bits() */

void spilcdWritePanelCommands(const BB_RGB *pPanel, const uint8_t *pCmdList, int iListLen)
{
uint8_t c, *s = (uint8_t *)pCmdList;
uint8_t ucTemp[16];

// 9-bit mode (no D/C pin)
   pinMode(pPanel->mosi, OUTPUT);
   pinMode(pPanel->sck, OUTPUT);
   pinMode(pPanel->cs, OUTPUT);
   digitalWrite(pPanel->cs, HIGH);
//   SPI.begin(pPanel->sck, -1/*MISO*/, pPanel->mosi, -1/*pPanel->cs*/);
   while (*s) {
      c = *s++; // count
      if (c == LCD_DELAY) { // pause
         c = *s++;
         delay(c);
      } else {
         //SPI.beginTransaction(SPISettings(2000000, MSBFIRST, SPI_MODE0));    
         //SPI.transferBytes(s, ucTemp, c);
         digitalWrite(pPanel->cs, LOW);
         Write9Bits(pPanel, 0, s, 1); // command
         if (c > 1) {
            Write9Bits(pPanel, 1, &s[1], c-1); // data
         }
         digitalWrite(pPanel->cs, HIGH);
         s += c;
         //SPI.endTransaction();
      }
   } // while
   //digitalWrite(pPanel->cs, HIGH);
} /* spilcdWritePanelCommands() */

int BB_SPI_LCD::beginQSPI(int iType, int iFlags, uint8_t CS_PIN, uint8_t CLK_PIN, uint8_t D0_PIN, uint8_t D1_PIN, uint8_t D2_PIN, uint8_t D3_PIN, uint8_t RST_PIN, uint32_t u32Freq)
{
    memset(&_lcd, 0, sizeof(_lcd));
    _lcd.bUseDMA = 1; // allows DMA access
#ifdef ARDUINO_ARCH_ESP32
    return qspiInit(&_lcd, iType, iFlags, u32Freq, CS_PIN, CLK_PIN, D0_PIN, D1_PIN, D2_PIN, D3_PIN, RST_PIN, -1);
#endif
    return 0;
} /* beginQSPI() */

int BB_SPI_LCD::beginParallel(int iType, int iFlags, uint8_t RST_PIN, uint8_t RD_PIN, uint8_t WR_PIN, uint8_t CS_PIN, uint8_t DC_PIN, int iBusWidth, uint8_t *data_pins, uint32_t u32Freq)
{
    memset(&_lcd, 0, sizeof(_lcd));
    if (RST_PIN != 0xff) {
        pinMode(RST_PIN, OUTPUT);
        digitalWrite(RST_PIN, LOW);
        delay(100);
        digitalWrite(RST_PIN, HIGH);
        delay(100);
    }
    ParallelDataInit(RD_PIN, WR_PIN, CS_PIN, DC_PIN, iBusWidth, data_pins, iFlags, u32Freq);
    spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
    return spilcdInit(&_lcd, iType, iFlags, 0,0,0,0,0,0,0,0,0);
} /* beginParallel() */

#ifdef ARDUINO_ESP32P4_DEV
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_lcd_panel_io.h"
#include "esp_ldo_regulator.h"
#if SOC_MIPI_DSI_SUPPORTED
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_mipi_dsi.h"
#include "esp_cache.h"
#endif
#include "esp_lcd_jd9165.h"
#define LCD_RST 27
#define LCD_LED 23
#define MIPI_DPI_PX_FORMAT (LCD_COLOR_PIXEL_FORMAT_RGB565)
#define LCD_BIT_PER_PIXEL (16)
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN 3 // LDO_VO3 连接至 VDD_MIPI_DPHY
#define EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV 2500
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_io_handle_t io_handle;

void enable_dsi_phy_power(void)
{
    // 打开 MIPI DSI PHY 的电源，使其从“无电”状态进入“关机”状态
    esp_ldo_channel_handle_t ldo_mipi_phy = NULL;
#ifdef EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_CHAN,
        .voltage_mv = EXAMPLE_MIPI_DSI_PHY_PWR_LDO_VOLTAGE_MV,
    };
    ESP_ERROR_CHECK(esp_ldo_acquire_channel(&ldo_mipi_phy_config, &ldo_mipi_phy));
    ESP_LOGI("bb_spi_lcd", "MIPI DSI PHY Powered on");
#endif
} /* enable_dsi_phy_power() */

uint16_t *jd9165_init(void)
{
    enable_dsi_phy_power();
    pinMode(LCD_LED, OUTPUT);
    digitalWrite(LCD_LED, HIGH); // turn on LCD backlight
    // 首先创建 MIPI DSI 总线，它还将初始化 DSI PHY
    esp_lcd_dsi_bus_handle_t mipi_dsi_bus;
    esp_lcd_dsi_bus_config_t bus_config = JD9165_PANEL_BUS_DSI_2CH_CONFIG();
    ESP_ERROR_CHECK(esp_lcd_new_dsi_bus(&bus_config, &mipi_dsi_bus));
    
    ESP_LOGI("bb_spi_lcd", "Install MIPI DSI LCD control panel");
    // 我们使用DBI接口发送LCD命令和参数
    esp_lcd_dbi_io_config_t dbi_config = JD9165_PANEL_IO_DBI_CONFIG();
    
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_dbi(mipi_dsi_bus, &dbi_config, &io_handle));
    // 创建JD9165控制面板
    esp_lcd_dpi_panel_config_t dpi_config = JD9165_1024_600_PANEL_60HZ_DPI_CONFIG(MIPI_DPI_PX_FORMAT);
    
    jd9165_vendor_config_t vendor_config = {
        .mipi_config = {
            .dsi_bus = mipi_dsi_bus,
            .dpi_config = &dpi_config,
        },
    };
    const esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = LCD_RST,
        .rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = LCD_BIT_PER_PIXEL,
        .vendor_config = &vendor_config,
    };
    ESP_LOGI("bb_spi_lcd", "Create new panel for jd9165");
    ESP_ERROR_CHECK(esp_lcd_new_panel_jd9165(io_handle, &panel_config, &panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
    ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
    uint16_t *pFrameBuffer;
    esp_lcd_dpi_panel_get_frame_buffer(panel_handle, 1, (void **)&pFrameBuffer, NULL, NULL);
    return pFrameBuffer;
} /* jd9165_init() */
#endif // ARDUINO_ESP32P4_DEV

int BB_SPI_LCD::begin(int iDisplayType)
{
    int iCS=0, iDC=0, iMOSI=0, iSCK=0; // swap pins around for the different TinyPico boards
    int iLED=0, iRST = -1;

    memset(&_lcd, 0, sizeof(_lcd));
#ifdef ARDUINO_TINYPICO
    iMOSI = 23;
    iSCK = 18;
    if (iDisplayType == DISPLAY_TINYPICO_IPS_SHIELD) {
        iCS = 5;
        iDC = 15;
        iLED = 14;
    } else { // Explorer Shield
        iCS = 14;
        iDC = 4;
        iLED = -1;
    }
#elif defined (ARDUINO_TINYS2)
    iMOSI = 35;
    iSCK = 37;
    if (iDisplayType == DISPLAY_TINYPICO_IPS_SHIELD) {
        iCS = 14;
        iDC = 6;
        iLED = 5;
    } else { // Explorer Shield
        iCS = 5;
        iDC = 4;
        iLED = -1;
    }
#elif defined (ARDUINO_TINYS3)
    iMOSI = 35;
    iSCK = 36;
    if (iDisplayType == DISPLAY_TINYPICO_IPS_SHIELD) {
        iCS = 34;
        iDC = 3;
        iLED = 2;
    } else { // Explorer Shield
        iCS = 2;
        iDC = 1;
        iLED = -1;
    }
#endif
    switch (iDisplayType)
    {
        case DISPLAY_TINYPICO_IPS_SHIELD:
            spilcdInit(&_lcd, LCD_ST7735S_B, FLAGS_SWAP_RB | FLAGS_INVERT, 40000000, iCS, iDC, iRST, iLED, -1, iMOSI, iSCK,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_TINYPICO_EXPLORER_SHIELD:
            spilcdInit(&_lcd, LCD_ST7789_240, FLAGS_NONE, 40000000, iCS, iDC, iRST, iLED, -1, iMOSI, iSCK,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_180);
            break;
        case DISPLAY_WIO_TERMINAL:
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 30000000, 69, 70, 71, 72, -1, -1, -1, 1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_TEENSY_ILI9341:
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 60000000, 10, 9, -1, -1, -1, -1, 13,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_RANKIN_SENSOR:
            spilcdInit(&_lcd, LCD_ST7789_135, FLAGS_NONE, 40000000, 4, 21, 22, 26, -1, 23, 18,1); // Mike's coin cell pin numbering
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_CYD_2USB:
            spilcdInit(&_lcd, LCD_ST7789, FLAGS_INVERT, 40000000, 15, 2, -1, 21, 12, 13, 14, 1); // Cheap Yellow Display 2 USB (2.8 w/resistive touch, 2 USB ports)
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            _lcd.pSPI = NULL;
            _lcd.iRTMOSI = 32;
            _lcd.iRTMISO = 39; // pre-configure resistive touch
            _lcd.iRTCLK = 25;
            _lcd.iRTCS = 33;
            _lcd.iRTOrientation = 0;
            _lcd.iRTThreshold = 6300;
            break;
        case DISPLAY_CYD_35:
            spilcdInit(&_lcd, LCD_ILI9488, FLAGS_FLIPX | FLAGS_SWAP_RB, 80000000, 15, 2, -1, 27, 12, 13, 14, 1); // Cheap Yellow Display (ESP32 3.5" 320x480 version)
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_CYD_35R:
            spilcdInit(&_lcd, LCD_ILI9488, FLAGS_FLIPX | FLAGS_SWAP_RB, 80000000, 15, 2, -1, 27, 12, 13, 14, 1); // Cheap Yellow Display (ESP32 3.5" 320x480 version) 
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            _lcd.pSPI = NULL;
            _lcd.iRTMOSI = 13;
            _lcd.iRTMISO = 12; // pre-configure resistive touch
            _lcd.iRTCLK = 14;
            _lcd.iRTCS = 33;
            _lcd.iRTOrientation = 0;
            _lcd.iRTThreshold = 6300; 
            break;
        case DISPLAY_WT32_SC01_PLUS: // 3.5" 480x320 ST7796 8-bit parallel
            memset(&_lcd, 0, sizeof(_lcd));
            pinMode(45, OUTPUT); // backlight
            digitalWrite(45, HIGH);
            _lcd.iLEDPin = 45; 
            beginParallel(LCD_ST7796, FLAGS_INVERT | FLAGS_FLIPX | FLAGS_SWAP_RB | FLAGS_MEM_RESTART, 4, -1, 47, -1, 0, 8, (uint8_t *)u8_WT32_Pins, 12000000);
            setRotation(270);
            break;
        case DISPLAY_CYD_22C: // 2.2" ST7789 8-bit parallel
            memset(&_lcd, 0, sizeof(_lcd));
            pinMode(0, OUTPUT); // backlight
            digitalWrite(0, HIGH);
            _lcd.iLEDPin = 0;
            beginParallel(LCD_ST7789, FLAGS_INVERT | FLAGS_MEM_RESTART, -1, 2, 4, 17, 16, 8, (uint8_t *)u8_22C_Pins, 12000000);
            setRotation(270);
            break;
        case DISPLAY_CYD_128:
            spilcdInit(&_lcd, LCD_GC9A01, FLAGS_NONE, 40000000, 10, 2, -1, 3, -1, 7, 6, 1); // Cheap Yellow Display (ESP32-C3 1.28" round version)
            break;
#if defined(D1) && defined(D3) && defined(D10) && defined(d8)
// These 'D' pins are not defined on all Arduino targets
        case DISPLAY_XIAO_ROUND:
            spilcdInit(&_lcd, LCD_GC9A01, FLAGS_NONE, 40000000, D1, D3, -1, -1, -1, D10, D8, 1); // Seeed Xiao 1.28" 240x240 round LCD
            break;
#endif
        case DISPLAY_CYD_24C:
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 40000000, 15, 2, -1, 27, 12, 13, 14, 0); // Cheap Yellow Display (2.4 w/capacitive touch)
            setRotation(270);
            break;
        case DISPLAY_CYD_24R:
            spilcdInit(&_lcd, LCD_ST7789, FLAGS_INVERT, 40000000, 15, 2, -1, 27, 12, 13, 14, 0); // Cheap Yellow Display (2.4 w/resistive touch)
            setRotation(270);
            _lcd.pSPI = &SPI; // shared SPI
            _lcd.iRTCS = 33;
            _lcd.iRTMOSI = 255;
            _lcd.iRTOrientation = 270;
            _lcd.iRTThreshold = 6300;
            break;
        case DISPLAY_CYD_28C:
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 40000000, 15, 2, -1, 27, 12, 13, 14, 1); // Cheap Yellow Display (2.4 and 2.8 w/cap touch)
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_CYD_4848: // 4.0" 480x480 ESP32-S3
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.iDCPin = _lcd.iCSPin = -1; // make sure we don't try to toggle these
            _lcd.iLEDPin = -1; 
            _lcd.iLCDType = LCD_VIRTUAL_MEM;
            _lcd.iWidth = _lcd.iCurrentWidth = 480;
            _lcd.iHeight = _lcd.iCurrentHeight = 480;
            spilcdWritePanelCommands(&rgbpanel_480x480, st7701list, sizeof(st7701list));
            spilcdSetBuffer(&_lcd, (uint8_t *)RGBInit((BB_RGB *)&rgbpanel_480x480));
            break;
        case DISPLAY_CYD_700: // 7" 800x480 ESP32-S3
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.iDCPin = _lcd.iCSPin = -1; // make sure we don't try to toggle these
            _lcd.iLEDPin = 2;
            pinMode(2, OUTPUT);
            digitalWrite(2, HIGH);
            _lcd.iLCDType = LCD_VIRTUAL_MEM;
            _lcd.iWidth = _lcd.iCurrentWidth = 800;
            _lcd.iHeight = _lcd.iCurrentHeight = 480;
            spilcdSetBuffer(&_lcd, (uint8_t *)RGBInit((BB_RGB *)&rgbpanel_800x480_7));
            break;

#ifdef ARDUINO_ESP32P4_DEV
        case DISPLAY_CYD_P4_1024x600:
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.iLCDFlags = FLAGS_SWAP_COLOR; // little endian byte order
            _lcd.iDCPin = _lcd.iCSPin = -1; // make sure we don't try to toggle these
            _lcd.iLEDPin = 23;
            _lcd.iLCDType = LCD_VIRTUAL_MEM;
            _lcd.iWidth = _lcd.iCurrentWidth = 1024;
            _lcd.iHeight = _lcd.iCurrentHeight = 600;
            spilcdSetBuffer(&_lcd, (uint8_t *)jd9165_init());
            break;
#endif // ESP32-P4
            
        case DISPLAY_CYD_8048: // 4.3" and 5.5" 800x480 ESP32-S3
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.iDCPin = _lcd.iCSPin = -1; // make sure we don't try to toggle these
            _lcd.iLEDPin = 2;
            pinMode(2, OUTPUT);
            digitalWrite(2, HIGH);
            _lcd.iLCDType = LCD_VIRTUAL_MEM;
            _lcd.iWidth = _lcd.iCurrentWidth = 800;
            _lcd.iHeight = _lcd.iCurrentHeight = 480;
            spilcdSetBuffer(&_lcd, (uint8_t *)RGBInit((BB_RGB *)&rgbpanel_800x480));
            break;
        case DISPLAY_CYD:
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 40000000, 15, 2, -1, 21, 12, 13, 14, 1); // Cheap Yellow Display (common versions w/resistive touch)
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            _lcd.pSPI = NULL;
            _lcd.iRTMOSI = 32;
            _lcd.iRTMISO = 39; // pre-configure resistive touch
            _lcd.iRTCLK = 25;
            _lcd.iRTCS = 33;
            _lcd.iRTOrientation = 0;
            _lcd.iRTThreshold = 6300;
            break;
        case DISPLAY_LOLIN_S3_MINI_PRO: // ESP32-S3 + 0.85" 128x128 LCD
// spilcdInit(&_lcd, LCD_ST7789_240, FLAGS_NONE, 40000000, iCS, iDC, iRST, iLED, -1, iMOSI, iSCK,1);
            spilcdInit(&_lcd, LCD_GC9107, FLAGS_NONE, 40000000, 35, 36, 34, 33, -1, 38, 40, 1);
            //qspiSetBrightness(&_lcd, 255); // something wrong with the backlight LED; needs to be set bright
            break;
        case DISPLAY_STAMPS3_8PIN:
            spilcdInit(&_lcd, LCD_ST7789_172, FLAGS_NONE, 40000000, 37, 34, 33, 38, -1, 35, 36, 1);
#ifdef ARDUINO_ARCH_ESP32
            qspiSetBrightness(&_lcd, 255); // needs to be set bright
#endif
            break;
        case DISPLAY_M5STACK_ATOMS3:
            spilcdInit(&_lcd, LCD_GC9107, FLAGS_NONE, 40000000, 15, 33, 34, 16, -1, 21, 17, 1);
#ifdef ARDUINO_ARCH_ESP32
            qspiSetBrightness(&_lcd, 255); // something wrong with the backlight LED; needs to be set bright
#endif // ESP32
            break;
#ifdef ARDUINO_M5Stick_C
        case DISPLAY_M5STACK_STICKC:
            AxpPowerUp();
            AxpBrightness(9);
            spilcdInit(&_lcd, LCD_ST7735S_B, FLAGS_SWAP_RB | FLAGS_INVERT, 24000000, 5, 23, 18, -1, -1, 15, 13,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_M5STACK_STICKCPLUS:
            AxpPowerUp();
            AxpBrightness(9); // turn on backlight (0-12)
            spilcdInit(&_lcd, LCD_ST7789_135, FLAGS_NONE, 40000000, 5, 23, 18, -1, -1, 15, 13,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
#endif // ARDUINO_M5Stick_C
#ifdef ARDUINO_M5STACK_CORES3
        case DISPLAY_M5STACK_CORES3:
            CoreS3AxpPowerUp(); // D/C is shared with MISO
            spilcdInit(&_lcd, LCD_ILI9342, FLAGS_NONE, 40000000, 3, 35, -1, -1, -1, 37, 36,1);
            break; 
#endif // ARDUINO_M5STACK_CORES3 
#ifdef ARDUINO_M5STACK_CORE2
        case DISPLAY_M5STACK_CORE2:
            Core2AxpPowerUp();
            spilcdInit(&_lcd, LCD_ILI9342, FLAGS_NONE, 40000000, 5, 15, -1, -1, 38, 23, 18,0);
            break;
#endif // ARDUINO_M5STACK_CORE2
        case DISPLAY_RANKIN_COLORCOIN:
            spilcdInit(&_lcd, LCD_ST7735S_B, FLAGS_SWAP_RB | FLAGS_INVERT, 24000000, 4, 21, 22, 26, -1, 23, 18,1); // Mike's coin cell pin numbering
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_RANKIN_POWER:
            spilcdInit(&_lcd, LCD_ST7735S_B, FLAGS_SWAP_RB | FLAGS_INVERT, 24000000, 4, 22, 5, 19, -1, 23, 18,1); // Mike's coin cell pin numbering
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_T_DISPLAY:
            spilcdInit(&_lcd, LCD_ST7789_135, 0, 40000000, 5, 16, 23, 4, -1, 19, 18,1);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
        case DISPLAY_T_TRACK: // ESP32-S3 + 126x294 AMOLED
            pinMode(4, OUTPUT); // MOSFET power control
            digitalWrite(4, 1); // turn on the AMOLED display
            spilcdInit(&_lcd, LCD_JD9613, FLAGS_FLIPX, 32000000, 9, 7, 8, 10, -1, 6, 5, 1);
            break;
        case DISPLAY_T_QT_C6:
            spilcdInit(&_lcd, LCD_GC9107, 0, 40000000, 14, 19, 20, 2, -1, 15, 18,1); 
            break;
        case DISPLAY_T_QT:
            spilcdInit(&_lcd, LCD_GC9107, 0, 50000000, 5, 6, 1, -1, -1, 2, 3,1);
            pinMode(10, OUTPUT);
            digitalWrite(10, LOW); // inverted backlight signal
            break;
        case DISPLAY_D1_R32_ILI9341:
            pinMode(32, OUTPUT); // reset
            digitalWrite(32, LOW);
            delay(100);
            digitalWrite(32, HIGH);
            delay(100);
            ParallelDataInit(2, 4, 33, 15, 8, u8D1R32DataPins, 0, 12000000);
            spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 0,0,0,0,0,0,0,0,0);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_90);
            break;
#ifdef ARDUINO_ARCH_RP2040
        case DISPLAY_KUMAN_35: // ILI9486 320x480 8-bit parallel
            // toggle reset
            pinMode(9, OUTPUT);
            digitalWrite(9, LOW);
            delay(100);
            digitalWrite(9, HIGH);
            delay(100);
            //ParallelDataInit(uint8_t RD_PIN, uint8_t WR_PIN, uint8_t CS_PIN, uint8_t DC_PIN, int iBusWidth, uint8_t *data_pins)
            ucRXBuf[0] = 14; // D0
            ParallelDataInit(13, 12, 10, 11, 8, ucRXBuf, 0, 12000000);
            spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
            spilcdInit(&_lcd, LCD_ILI9486, FLAGS_SWAP_RB, 0,0,0,0,0,0,0,0,0);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_KUMAN_24: // ILI9341 240x320 8-bit parallel
            // toggle reset
            pinMode(9, OUTPUT);
            digitalWrite(9, LOW);
            delay(100);
            digitalWrite(9, HIGH);
            delay(100);
            //ParallelDataInit(uint8_t RD_PIN, uint8_t WR_PIN, uint8_t CS_PIN, uint8_t DC_PIN, int iBusWidth, uint8_t *data_pins)
            ucRXBuf[0] = 14; // D0
            ParallelDataInit(13, 12, 10, 11, 8, ucRXBuf, 0, 12000000);
            spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
            spilcdInit(&_lcd, LCD_ILI9341, FLAGS_NONE, 0,0,0,0,0,0,0,0,0);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_RP2040_C3: // ST7789 172x320 8-bit parallel
            pinMode(28, OUTPUT); // backlight
            digitalWrite(28, HIGH);
            pinMode(23, OUTPUT); // Reset
            digitalWrite(23, LOW);
            delay(100);
            digitalWrite(23, HIGH);
            delay(100);
            ucRXBuf[0] = 0; // D0
            ParallelDataInit(8,10, -1, 9, 8, ucRXBuf, 0, 0);
            spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
            spilcdInit(&_lcd, LCD_ST7789_172, 0,0,0,0,0,0,0,0,0,0);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
        case DISPLAY_TUFTY2040: // ST7789 240x320 8-bit parallel
            pinMode(2, OUTPUT); // backlight
            digitalWrite(2, HIGH);
            ucRXBuf[0] = 14; // D0
            ParallelDataInit(13, 12, 10, 11, 8, ucRXBuf, 0, 0);
            spilcdSetCallbacks(&_lcd, ParallelReset, ParallelDataWrite);
            spilcdInit(&_lcd, LCD_ST7789, 0,0,0,0,0,0,0,0,0,0);
            spilcdSetOrientation(&_lcd, LCD_ORIENTATION_270);
            break;
#endif
#ifdef ARDUINO_ARCH_ESP32
	case DISPLAY_T_DONGLE_S3:
            pinMode(38, OUTPUT); // power enable
            digitalWrite(38, LOW); // turn on LCD
            begin(LCD_ST7735S_B, FLAGS_SWAP_RB | FLAGS_INVERT, 40000000, 4, 2, 1, -1, -1, 3, 5);
            break;
        case DISPLAY_T_DISPLAY_S3_PRO: // 222x480 ST7796
            memset(&_lcd, 0, sizeof(_lcd));
            // MISO=8, MOSI=17,CLK=18,CS=39,DC=9,RST=47,BL=48
            begin(LCD_ST7796_222, FLAGS_SWAP_RB | FLAGS_INVERT | FLAGS_FLIPX, 40000000, 39, 9, 47, 48, 8, 17, 18);
            _lcd.iLEDPin = 48;
            //setRotation(270);
            break;

        case DISPLAY_CYD_518: // 360x360  ST77916
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=10, SCK=9, D0=11, D1=12, D2=13, D3=14,7RST=46, BL=1
            qspiInit(&_lcd, LCD_ST77916, FLAGS_NONE, 40000000, 10,9,11,12,13,14,47,15);       
           break;

        case DISPLAY_WS_AMOLED_146: // Waveshare 1.46" 412x412 round AMOLED
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=21, SCK=40, D0=46, D1=45, D2=42, D3=41, RST=-1, BL=-1 
            qspiInit(&_lcd, LCD_SPD2010, FLAGS_NONE, 40000000, 21,40,46,45,42,41,-1,-1);
            break;
        case DISPLAY_WS_AMOLED_18: // Waveshare 1.8" 368x448 AMOLED
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=12, SCK=11, D0=4, D1=5, D2=6, D3=7, RST=-1, BL=-1
            qspiInit(&_lcd, LCD_SH8601, FLAGS_NONE, 32000000, 12,11,4,5,6,7,-1,-1);
            break;
        case DISPLAY_UM_AMOLED_18: // UnexpectedMaker 1.8 AMOLED
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=34, SCK=36, D0=35, D1=9, D2=8, D3=7, RST=6, BL=-1
            qspiInit(&_lcd, LCD_SH8601, FLAGS_NONE, 32000000, 34, 36, 35, 9, 8, 7, 6, -1);
            break;
        case DISPLAY_CYD_535: // 320x480 AXS15321
            // CS=45, SCK=47, D0=21, D1=48, D2=40, D3=39, RST=-1, BL=1
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1;
            qspiInit(&_lcd, LCD_AXS15231, FLAGS_NONE, 40000000, 45,47,21,48,40,39,-1,1);
            break;
        case DISPLAY_CYD_543: // 480x272  NV3041A
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.iRTMOSI = 11;
            _lcd.iRTMISO = 13; // pre-configure resistive touch
            _lcd.iRTCLK = 12;
            _lcd.iRTCS = 38;
            _lcd.iRTOrientation = 180;
            _lcd.iRTThreshold = 7500;
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=45, SCK=47, D0=21, D1=48, D2=40, D3=39, RST=-1, BL=1
            qspiInit(&_lcd, LCD_NV3041A, FLAGS_NONE, 32000000, 45,47,21,48,40,39,-1,1);       
           break;

        case DISPLAY_T_DISPLAY_S3_LONG: // 180x640 AXS15231B
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=12, SCK=17, D0=13, D1=18, D2=21, D3=14, RST=16, BL=1
            qspiInit(&_lcd, LCD_AXS15231B, FLAGS_NONE, 32000000, 12,17,13,18,21,14,16,1);
            break;

        case DISPLAY_T_DISPLAY_S3_AMOLED: // 240x536 RM67162
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            // CS=6, SCK=47, D0=18, D1=7, D2=48, D3=5, RST=17, BL=-1
            qspiInit(&_lcd, LCD_RM67162, FLAGS_NONE, 40000000, 6,47,18,7,48,5,17,-1);
            break;

        case DISPLAY_T_DISPLAY_S3_AMOLED_164: // 280x456 ICNA3311 
            memset(&_lcd, 0, sizeof(_lcd));
            _lcd.bUseDMA = 1; // allows DMA access
            pinMode(16, OUTPUT); // power enable for LCD
            digitalWrite(16, 1);
            // CS=10, SCK=12, D0=11, D1=13, D2=14, D3=15, RST=17, BL=-1
            qspiInit(&_lcd, LCD_ICNA3311, FLAGS_NONE, 40000000, 10,12,11,13,14,15,17,-1);
            break;

        case DISPLAY_T_DISPLAY_S3:
            pinMode(38, OUTPUT); // backlight
            digitalWrite(38, HIGH);
            pinMode(15, OUTPUT); // power enable
            digitalWrite(15, HIGH); // turn on LCD
            _lcd.iLEDPin = 38;
            {
            static const uint8_t u8Pins[8] = {39,40,41,42,45,46,47,48};
            beginParallel(LCD_ST7789_172, FLAGS_MEM_RESTART, 5, 9, 8, 6, 7, 8, (uint8_t *)u8Pins, 20000000);
                // Adjust the parameters for the 170x320 LCD
            _lcd.iCurrentWidth = _lcd.iWidth = 170;
            _lcd.iColStart = _lcd.iMemoryX = 35;
            setRotation(270);
            }
            break;
        case DISPLAY_MAKERFABS_S3:
            pinMode(45, OUTPUT); // backlight
            digitalWrite(45, HIGH);
            {
            static const uint8_t u8Pins[16] = {47,21,14,13,12,11,10,9,3,8,16,15,7,6,5,4};
            beginParallel(LCD_ILI9488, (FLAGS_SWAP_RB | FLAGS_SWAP_COLOR), -1, 48, 35, 37, 36, 16, (uint8_t *)u8Pins, 20000000);
            _lcd.iLEDPin = 45;
            setRotation(90);
            }
            break;
#endif // ARDUINO_ARCH_ESP32
        default:
            return -1;
    }
    return 0;
}
//
// Merge 2 class instances with transparence (must be the same size)
//
int BB_SPI_LCD::merge(uint16_t *s, uint16_t usTrans, int bSwap565)
{
    uint16_t *d = (uint16_t *)_lcd.pBackBuffer;
    int i;
    if (bSwap565) {
        for (i = 0; i<_lcd.iCurrentWidth * _lcd.iCurrentHeight; i++) {
            if (d[i] == usTrans) { // replace transparent pixels in current surface
                d[i] = __builtin_bswap16(s[i]);
            }
        }
    } else {
        for (i = 0; i<_lcd.iCurrentWidth * _lcd.iCurrentHeight; i++) {
            if (d[i] == usTrans) {
                d[i] = s[i];
            }
        }
    }
    return 1;
} /* merge() */
//
// Capture pixels being drawn into the current drawing surface
// onto a virtual surface at a specific position
// e.g. to capture the lower right corner of an image
// set dst_x = current_width/2, dst_y = current_height/2
//
int BB_SPI_LCD::captureArea(int dst_x, int dst_y, int src_x, int src_y, int src_w, int src_h, uint16_t *pPixels, int bSwap565)
{
uint16_t *s, *d;
int x, y, sx, sy, dx, dy, cx, cy;
    
    if (_lcd.pBackBuffer == 0) return 0; // no buffer
    // see if any overlap
    if (dst_x >= (src_x+src_w) || src_x >= (dst_x + _lcd.iCurrentWidth) || dst_y >= (src_y+src_h) || src_y >= (dst_y + _lcd.iCurrentHeight))
        return 0; // no intersection
    
    s = pPixels; d = (uint16_t *)_lcd.pBackBuffer;
    dx = dy = 0;
    cx = _lcd.iCurrentWidth; cy = _lcd.iCurrentHeight;
    sx = dst_x - src_x; // source starting point
    sy = dst_y - src_y;
    if (sx < 0) {
        dx -= sx;
        cx += sx;
        sx = 0;
    }
    if (sy < 0) {
        dy -= sy;
        cy += sy;
        sy = 0;
    }
    if (cx > src_w) cx = src_w;
    if (cy > src_h) cy = src_h;
    
    s += sx + (sy * src_w);
    d += dx + (dy * _lcd.iCurrentWidth);
    if (bSwap565) {
        for (y=0; y<cy; y++) {
            for (x=0; x<cx; x++) {
                d[x] = __builtin_bswap16(s[x]);
            } // for x
            s += src_w;
            d += _lcd.iCurrentWidth;
        } // for y
    } else { // no swap
        for (y=0; y<cy; y++) {
            memcpy(d, s, cx*2);
            s += src_w;
            d += _lcd.iCurrentWidth;
        }
    }
    return 1;
} /* captureArea() */

int BB_SPI_LCD::createVirtual(int iWidth, int iHeight, void *p)
{
    memset(&_lcd, 0, sizeof(_lcd));
    _lcd.iLCDType = LCD_VIRTUAL_MEM;
    _lcd.iDCPin = _lcd.iCSPin = -1; // make sure we don't try to toggle these
    if (p == NULL) {
        p = malloc(iWidth * iHeight * 2);
#ifdef ARDUINO_ARCH_ESP32
        // Try to use PSRAM
        p = heap_caps_aligned_alloc(16, (iWidth * iHeight * 2), MALLOC_CAP_8BIT);
        //p = ps_malloc(iWidth * iHeight * 2);
#endif
       if (!p) return 0;
    }
    _lcd.iCurrentWidth = _lcd.iWidth = iWidth;
    _lcd.iCurrentHeight = _lcd.iHeight = iHeight;
    spilcdSetBuffer(&_lcd, p);
    return 1;
} /* createVirtual() */

int BB_SPI_LCD::freeVirtual(void)
{
    if (_lcd.pBackBuffer == 0) return 0;
    free(_lcd.pBackBuffer);
    _lcd.pBackBuffer = 0;
    return 1;
} /* freeVirtual() */

uint16_t BB_SPI_LCD::color565(uint8_t r, uint8_t g, uint8_t b)
{
uint16_t u16;
   u16 = b >> 3;
   u16 |= ((g >> 2) << 5);
   u16 |= ((r >> 3) << 11);
   return u16;
} /* color565() */

void BB_SPI_LCD::setBrightness(uint8_t u8Brightness)
{
#ifdef ARDUINO_ARCH_ESP32
     qspiSetBrightness(&_lcd, u8Brightness);
#endif
} /* setBrightness() */

int BB_SPI_LCD::begin(int iType, int iFlags, int iFreq, int iCSPin, int iDCPin, int iResetPin, int iLEDPin, int iMISOPin, int iMOSIPin, int iCLKPin)
{
  if (iMISOPin == -1) iMISOPin = MISO;
  if (iMOSIPin == -1) iMOSIPin = MOSI;
  if (iCLKPin == -1) iCLKPin = SCK;
  return spilcdInit(&_lcd, iType, iFlags, iFreq, iCSPin, iDCPin, iResetPin, iLEDPin, iMISOPin, iMOSIPin, iCLKPin,1);
} /* begin() */

void BB_SPI_LCD::freeBuffer(void)
{
    spilcdFreeBackbuffer(&_lcd);
}

void BB_SPI_LCD::waitDMA(void)
{
    spilcdWaitDMA();
} /* waitDMA() */

uint8_t * BB_SPI_LCD::getDMABuffer(void)
{
    return (uint8_t *)pDMA;
}

void * BB_SPI_LCD::getBuffer(void)
{
    return (void *)_lcd.pBackBuffer;
}
SPILCD * BB_SPI_LCD::getLCDStruct(void)
{
    return &_lcd;
}
int BB_SPI_LCD::fontHeight(void)
{
int h;
    if (_lcd.pFont == NULL) { // built-in fonts
        if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
          h = 8;
        } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
          h = 16;
        } else {
          h = 32;
        }
    } else {
        int w, top, bottom;
        const char *string = "H";
        spilcdGetStringBox(_lcd.pFont, (char *)string, &w, &top, &bottom);
        h = bottom - top;
    }
    return h;
} /* fontHeight() */

void BB_SPI_LCD::getTextBounds(const char *string, int16_t x, int16_t y, int16_t *x1, int16_t *y1, uint16_t *w1, uint16_t *h1)
{
    if (_lcd.pFont == NULL) { // use built-in fonts
        int iLen = strlen(string);
        int h, w;
        if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
          h = 8;
          w = (_lcd.iFont == FONT_8x8) ? 8 : 6;
        } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
          h = 16;
          w = (_lcd.iFont == FONT_12x16) ? 12 : 16;
        } else { w = 16; h = 32; }
        *x1 = x; *y1 = y; // starts drawing downward
        *w1 = w * iLen;
        *h1 = h;
    } else { // custom fonts
        int w, top, bottom;
        spilcdGetStringBox(_lcd.pFont, (char *)string, &w, &top, &bottom);
        *w1 = w;
        *h1 = bottom - top;
        *y1 = y + top;
        *x1 = x;
    }
} /* getTextBounds() */

bool BB_SPI_LCD::allocBuffer(void)
{
    int rc = spilcdAllocBackbuffer(&_lcd);
    return (rc == 0);
}
void BB_SPI_LCD::setScroll(bool bScroll)
{
    _lcd.bScroll = bScroll;
}

void BB_SPI_LCD::drawPattern(uint8_t *pPattern, int iSrcPitch, int iDestX, int iDestY, int iCX, int iCY, uint16_t usColor, int iTranslucency)
{
  spilcdDrawPattern(&_lcd, pPattern, iSrcPitch, iDestX, iDestY, iCX, iCY, usColor, iTranslucency);
} /* drawPattern() */

void BB_SPI_LCD::pushPixels(uint16_t *pixels, int count, int flags)
{
   spilcdWriteDataBlock(&_lcd, (uint8_t *)pixels, count * 2, flags);
} /* pushPixels() */

void BB_SPI_LCD::setAddrWindow(int x, int y, int w, int h)
{
   spilcdSetPosition(&_lcd, x, y, w, h, DRAW_TO_LCD);
} /* setAddrWindow() */

void BB_SPI_LCD::setRotation(int iAngle)
{
int i;

  if (_lcd.iLCDType == LCD_VIRTUAL_MEM) return; // not supported

  switch (iAngle) {
    default: return;
    case 0:
      i = LCD_ORIENTATION_0;
      break;
    case 90:
    case 1: // allow Adafruit way or angle
      i = LCD_ORIENTATION_90;
      break;
    case 180:
    case 2:
      i = LCD_ORIENTATION_180;
      break;
    case 270:
    case 3:
      i = LCD_ORIENTATION_270;
      break; 
  }
  spilcdSetOrientation(&_lcd, i);
} /* setRotation() */

void BB_SPI_LCD::fillScreen(int iColor, int iFlags)
{
    spilcdFill(&_lcd, iColor, iFlags);
    _lcd.iCursorX = 0;
    _lcd.iCursorY = 0;
} /* fillScreen() */

void BB_SPI_LCD::drawRect(int16_t x, int16_t y, int16_t w, int16_t h, uint16_t color, int iFlags)
{
  spilcdRectangle(&_lcd, x, y, w, h, color, color, 0, iFlags);
} /* drawRect() */

void BB_SPI_LCD::fillRoundRect(int16_t x, int16_t y, int16_t w, int16_t h, int16_t r, uint16_t color, int iFlags)
{
  spilcdRectangle(&_lcd, x+r, y, w-(2*r), r, color, color, 1, iFlags);
  spilcdRectangle(&_lcd, x, y+r, w, h-(2*r), color, color, 1, iFlags);
  spilcdRectangle(&_lcd, x+r, y+h-r, w-(2*r), r, color, color, 1, iFlags);

  // draw four corners
  spilcdEllipse(&_lcd, x+w-r-1, y+r, r, r, 1, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+r, y+r, r, r, 2, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+w-r-1, y+h-r, r, r, 1, color, 1, iFlags);
  spilcdEllipse(&_lcd, x+r, y+h-r, r, r, 2, color, 1, iFlags);

} /* fillRoundRect() */

void BB_SPI_LCD::drawRoundRect(int16_t x, int16_t y, int16_t w, int16_t h,
                                 int16_t r, uint16_t color, int iFlags)
{
    spilcdDrawLine(&_lcd, x+r, y, x+w-r, y, color, iFlags); // top
    spilcdDrawLine(&_lcd, x+r, y+h-1, x+w-r, y+h-1, color, iFlags); // bottom
    spilcdDrawLine(&_lcd, x, y+r, x, y+h-r, color, iFlags); // left
    spilcdDrawLine(&_lcd, x+w-1, y+r, x+w-1, y+h-r, color, iFlags); // right
    // four corners
    spilcdEllipse(&_lcd, x+r, y+r, r, r, 1, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+w-r-1, y+r, r, r, 2, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+w-r-1, y+h-r-1, r, r, 4, color, 0, iFlags);
    spilcdEllipse(&_lcd, x+r, y+h-r-1, r, r, 8, color, 0, iFlags);
} /* drawRoundRect() */

void BB_SPI_LCD::fillRect(int x, int y, int w, int h, int iColor, int iFlags)
{
  spilcdRectangle(&_lcd, x, y, w, h, iColor, iColor, 1, iFlags);
} /* fillRect() */

void BB_SPI_LCD::setTextColor(int iFG, int iBG)
{
  _lcd.iFG = iFG;
  _lcd.iBG = (iBG == -1) ? iFG : iBG;
} /* setTextColor() */

void BB_SPI_LCD::setCursor(int x, int y)
{
  _lcd.iCursorX = x;
  _lcd.iCursorY = y;
} /* setCursor() */

void BB_SPI_LCD::setFont(int iFont)
{
  _lcd.iFont = iFont;
  _lcd.pFont = NULL;
} /* setFont() */

void BB_SPI_LCD::backlight(bool bOn)
{
   if (_lcd.iLEDPin != -1)
      myPinWrite(_lcd.iLEDPin, (int)bOn); 
} /* backlight() */

void BB_SPI_LCD::setFreeFont(const GFXfont *pFont)
{
  _lcd.pFont = (GFXfont *)pFont;
} /* setFreeFont() */

int BB_SPI_LCD::drawBMP(const uint8_t *pBMP, int iDestX, int iDestY, int bStretch, int iTransparent, int iFlags)
{
    return spilcdDrawBMP(&_lcd, (uint8_t *)pBMP, iDestX, iDestY, bStretch, iTransparent, iFlags);
} /* drawBMP() */

void BB_SPI_LCD::drawStringFast(const char *szText, int x, int y, int size, int iFlags)
{
    _lcd.iWriteFlags = iFlags;
    if (size == -1) size = _lcd.iFont;
    spilcdWriteStringFast(&_lcd, x, y, (char *)szText, _lcd.iFG, _lcd.iBG, size, iFlags);
} /* drawStringFast() */

void BB_SPI_LCD::setPrintFlags(int iFlags)
{
    _lcd.iWriteFlags = iFlags;
}
void BB_SPI_LCD::drawString(const char *pText, int x, int y, int size, int iFlags)
{
    _lcd.iWriteFlags = iFlags;
   if (size == 1) setFont(FONT_6x8);
   else if (size == 2) setFont(FONT_12x16);
   setCursor(x,y);
   for (int i=0; i<strlen(pText); i++) {
      write(pText[i]);
   } 
} /* drawString() */
void BB_SPI_LCD::drawString(String text, int x, int y, int size, int iFlags)
{
    drawString(text.c_str(), x, y, size, iFlags);
} /* drawString() */

void BB_SPI_LCD::drawLine(int x1, int y1, int x2, int y2, int iColor, int iFlags)
{
  spilcdDrawLine(&_lcd, x1, y1, x2, y2, iColor, iFlags);
} /* drawLine() */
inline GFXglyph *pgm_read_glyph_ptr(const GFXfont *gfxFont, uint8_t c) {
#ifdef __AVR__
  return &(((GFXglyph *)pgm_read_pointer(&gfxFont->glyph))[c]);
#else
  // expression in __AVR__ section may generate "dereferencing type-punned
  // pointer will break strict-aliasing rules" warning In fact, on other
  // platforms (such as STM32) there is no need to do this pointer magic as
  // program memory may be read in a usual way So expression may be simplified
  return gfxFont->glyph + c;
#endif //__AVR__
}
#ifdef ARDUINO_ESP32S3_DEV
    const uint16_t u16RGBMasks[] = {0x001f, 0x07e0, 0x07c0, 0xf800}; // B, G, R bitmasks for SIMD code 
#endif
//
// Use a mask to alpha blend a tint color
// color 0 = don't affect, color 0xffff = affect
// Source sprite is blended with the tint color and the given alpha for mask pixels that are 0xFFFF (WHITE)
// Destination pixels are written starting at the x,y of the instance calling this method
//
void BB_SPI_LCD::maskedTint(BB_SPI_LCD *pSrc, BB_SPI_LCD *pMask, int x, int y, uint16_t u16Tint, uint8_t u8Alpha)
{
    int w, h, tx, ty, iPitch;
    uint16_t u16, *s, *d, *m;
    uint32_t u32Tint, u32Dest;
    uint32_t u8BGAlpha = 32 - u8Alpha;
    const uint32_t u32Mask = 0x07e0f81f;
    
    if (!_lcd.pBackBuffer || !pSrc || !pMask || !pSrc->_lcd.pBackBuffer || !pMask->_lcd.pBackBuffer) return; // no valid memory planes

#ifdef ARDUINO_ESP32S3_DEV
// If the source and destination are the same size, use SIMD code
    if (_lcd.iCurrentWidth == pSrc->_lcd.iCurrentWidth && _lcd.iCurrentHeight == pSrc->_lcd.iCurrentHeight) { // use SIMD code
        s3_masked_tint_be((uint16_t *)_lcd.pBackBuffer, (uint16_t *)pSrc->_lcd.pBackBuffer, (uint16_t *)pMask->_lcd.pBackBuffer, u16Tint, pSrc->_lcd.iCurrentWidth * pSrc->_lcd.iCurrentHeight, u8Alpha, u16RGBMasks);
        return;
    }
#endif

    w = pMask->_lcd.iCurrentWidth;
    h = pMask->_lcd.iCurrentHeight;
    iPitch = _lcd.iScreenPitch / 2; // in terms of u16's
    u32Tint = u16Tint | (u16Tint << 16); // prepare tint color
    u32Tint &= u32Mask;
    d = (uint16_t *)&_lcd.pBackBuffer[(y * _lcd.iScreenPitch)  + (x * 2)];
    s = (uint16_t *)pSrc->_lcd.pBackBuffer;
    m = (uint16_t *)pMask->_lcd.pBackBuffer;
    for (ty = 0; ty < h; ty++) {
        for (tx = 0; tx < w; tx++) {
            if (m[0]) { // N.B. Should check for 0xffff instead of just non-zero
                u16 = __builtin_bswap16(s[0]);
                u32Dest = u16 | (u16 << 16);
                u32Dest &= u32Mask;
                u32Dest = (u32Dest * u8BGAlpha) + (u32Tint * u8Alpha);
                u32Dest = (u32Dest >> 5) & u32Mask;
                u32Dest |= (u32Dest >> 16);
                d[tx] = __builtin_bswap16((uint16_t)u32Dest);
            } else { // copy source to dest
                d[tx] = s[0];
            }
            s++; m++;
        } // for tx
        d += iPitch;
    } // for ty
} /* maskedTint() */
//
// Alpha blend a foreground and background sprite and store the result
// in a destination sprite
//
void BB_SPI_LCD::blendSprite(BB_SPI_LCD *pFGSprite, BB_SPI_LCD *pBGSprite, BB_SPI_LCD *pDestSprite, uint8_t u8Alpha)
{
#ifdef ARDUINO_ESP32S3_DEV
    s3_alpha_blend_be((uint16_t *)pFGSprite->_lcd.pBackBuffer, (uint16_t *)pBGSprite->_lcd.pBackBuffer, (uint16_t *)pDestSprite->_lcd.pBackBuffer, pFGSprite->_lcd.iCurrentWidth * pFGSprite->_lcd.iCurrentHeight, u8Alpha, u16RGBMasks);
#else
    int i, iCount = pFGSprite->_lcd.iCurrentWidth * pFGSprite->_lcd.iCurrentHeight;
    uint16_t u16, *pFG, *pBG, *pD;
    uint32_t u32FG, u32BG, u32D;
    uint32_t u8BGAlpha = 32 - u8Alpha;
    const uint32_t u32Mask = 0x07e0f81f;
    pFG = (uint16_t *)pFGSprite->_lcd.pBackBuffer;
    pBG = (uint16_t *)pBGSprite->_lcd.pBackBuffer;
    pD = (uint16_t *)pDestSprite->_lcd.pBackBuffer;
    for (i = 0; i < iCount; i++) {
        u16 = __builtin_bswap16(*pFG++);
        u32FG = u16 | (u16 << 16);
        u16 = __builtin_bswap16(*pBG++);
        u32BG = u16 | (u16 << 16);
        u32FG &= u32Mask;
        u32BG &= u32Mask;
        u32D = (u32FG * u8Alpha) + (u32BG * u8BGAlpha);
        u32D = (u32D >> 5) & u32Mask;
        u32D |= (u32D >> 16);
        *pD++ = __builtin_bswap16((uint16_t)u32D);
    }
#endif
} /* blendSprite() */
//
// Draw a sprite (another instance of the BB_SPI_LCD class) with scaling and optional transparency
// allows negative offsets, the sprite will be clipped properly
//
int BB_SPI_LCD::drawSprite(int x, int y, BB_SPI_LCD *pSprite, float fScale, int iTransparentColor, int iFlags)
{
    int tx, ty, cx, cy;
    uint16_t u16, *s, *d, *s16;
    uint32_t u32Frac, u32XAcc, u32YAcc; // integer fraction vars

    if (pSprite == NULL || pSprite->_lcd.iLCDType != LCD_VIRTUAL_MEM) return BB_ERROR_INV_PARAM; // invalid parameters
    // Calculate scaled destination size
    cx = (int)(fScale * (float)pSprite->_lcd.iCurrentWidth);
    cy = (int)(fScale * (float)pSprite->_lcd.iCurrentHeight);
    s = (uint16_t *)pSprite->_lcd.pBackBuffer;
    if (x <= -cx || y <= - cy) return BB_ERROR_SUCCESS; // nothing to draw
    if (x >= _lcd.iCurrentWidth || y >= _lcd.iCurrentHeight) return BB_ERROR_SUCCESS; // nothing to draw
    if (x < 0) {
        cx += x; // reduce width because of clipping
        s -= x; // start at the visible part
    }
    if (y < 0) {
        cy += y; // reduce height because of clipping
        s -= (y * pSprite->_lcd.iCurrentWidth); // start at the visible part
    }
    if (x + cx > _lcd.iCurrentWidth) { // clipped at right edge?
        cx = _lcd.iCurrentWidth - x;
        if (cx < 1) return BB_ERROR_INV_PARAM;
    }
    if (y + cy > _lcd.iCurrentHeight) { // clipped at bottom edge?
        cy = _lcd.iCurrentHeight - y;
        if (cy < 1) return BB_ERROR_INV_PARAM;
    }
    u32Frac = (uint32_t)(65536.0f / fScale); // calculate the fraction to advance the destination x/y

    if (_lcd.iLCDType == LCD_VIRTUAL_MEM) iFlags = DRAW_TO_RAM; // only one possibility (no physical LCD)
    if (iFlags & DRAW_TO_LCD) {
        // send the sprite to the physical display; transparency is not possible
        spilcdSetPosition(&_lcd, x, y, cx, cy, DRAW_TO_LCD);
        u32YAcc = 0;
        for (ty = 0; ty < cy; ty++) { // write it a line at a time
            u32XAcc = 0;
            s16 = &s[(u32YAcc >> 16) * pSprite->_lcd.iWidth];
            d = (uint16_t *)ucTXBuf;
            for (tx = 0; tx < cx; tx++) {
                // scale the source pixels to the destination (cx) width
                *d++ = s16[(u32XAcc >> 16)];
                u32XAcc += u32Frac;
            }
            myspiWrite(&_lcd, ucTXBuf, cx*2, MODE_DATA, iFlags);
            u32YAcc += u32Frac;
        }
    } else if (iFlags & DRAW_TO_RAM) { // only RAM, it can have a transparent color
        uint16_t u16Trans = (uint16_t)iTransparentColor;
        d = (uint16_t *)&_lcd.pBackBuffer[(x*2) + (y*2*_lcd.iCurrentWidth)];
        u32YAcc = 0;
        for (ty = 0; ty < cy; ty++) {
            u32XAcc = 0;
            s16 = &s[(u32YAcc >> 16) * pSprite->_lcd.iWidth];
            if (iTransparentColor != -1) { // slower to check each pixel
                for (tx = 0; tx<cx; tx++) {
                    u16 = s16[(u32XAcc >> 16)];
                    if (u16 != u16Trans) d[tx] = u16;
                    u32XAcc += u32Frac;
                }
            } else { // overwrite
                for (tx = 0; tx<cx; tx++) {
                    d[tx] = s16[(u32XAcc >> 16)];
                    u32XAcc += u32Frac;
                }
            }
            u32YAcc += u32Frac;
            d += _lcd.iCurrentWidth;
        }
    }
    return BB_ERROR_SUCCESS;
} /* drawSprite() */

//
// Draw a sprite (another instance of the BB_SPI_LCD class) with optional transparency
// allows negative offsets, the sprite will be clipped properly
//
int BB_SPI_LCD::drawSprite(int x, int y, BB_SPI_LCD *pSprite, int iTransparentColor, int iFlags)
{
    int tx, ty, cx, cy;
    uint16_t *s, *d;
    if (pSprite == NULL || pSprite->_lcd.iLCDType != LCD_VIRTUAL_MEM) return BB_ERROR_INV_PARAM; // invalid parameters
    cx = pSprite->_lcd.iCurrentWidth;
    cy = pSprite->_lcd.iCurrentHeight;
    s = (uint16_t *)pSprite->_lcd.pBackBuffer;
    if (x <= -cx || y <= - cy) return BB_ERROR_SUCCESS; // nothing to draw
    if (x >= _lcd.iCurrentWidth || y >= _lcd.iCurrentHeight) return BB_ERROR_SUCCESS; // nothing to draw
    if (x < 0) {
        cx += x; // reduce width because of clipping
        s -= x; // start at the visible part
    }
    if (y < 0) {
        cy += y; // reduce height because of clipping
        s -= (y * pSprite->_lcd.iCurrentWidth); // start at the visible part
    }
    if (x + pSprite->_lcd.iCurrentWidth > _lcd.iCurrentWidth) { // clipped at right edge?
        cx = _lcd.iCurrentWidth - x;
        if (cx < 1) return BB_ERROR_INV_PARAM;
    }
    if (y + pSprite->_lcd.iCurrentHeight > _lcd.iCurrentHeight) { // clipped at bottom edge?
        cy = _lcd.iCurrentHeight - y;
        if (cy < 1) return BB_ERROR_INV_PARAM;
    }
    if (_lcd.iLCDType == LCD_VIRTUAL_MEM) iFlags = DRAW_TO_RAM; // only one possibility (no physical LCD)
    if (iFlags & DRAW_TO_LCD) {
        // send the sprite to the physical display; transparency is not possible
        spilcdSetPosition(&_lcd, x, y, cx, cy, DRAW_TO_LCD);
        for (ty = 0; ty < cy; ty++) { // write it a line at a time
            myspiWrite(&_lcd, (uint8_t *)s, cx*2, MODE_DATA, iFlags);
            s += pSprite->_lcd.iCurrentWidth; // source pitch can be different from amount written to the LCD
        }
    } else if (iFlags & DRAW_TO_RAM) { // only RAM, it can have a transparent color
        uint16_t u16Trans = (uint16_t)iTransparentColor;
        d = (uint16_t *)&_lcd.pBackBuffer[(x*2) + (y*2*_lcd.iCurrentWidth)];
        for (ty = 0; ty < cy; ty++) {
            if (iTransparentColor != -1) { // slower to check each pixel
                for (tx = 0; tx<cx; tx++) {
                    if (s[tx] != u16Trans) d[tx] = s[tx];
                }
            } else { // overwrite
                memcpy(d, s, cx*2);
            }
            s += pSprite->_lcd.iCurrentWidth;
            d += _lcd.iCurrentWidth;
        }
    }
    return BB_ERROR_SUCCESS;
} /* drawSprite() */
//
// write (Print friend class)
//
size_t BB_SPI_LCD::write(uint8_t c) {
char szTemp[2]; // used to draw 1 character at a time to the C methods
int w, h;

  if (_lcd.iWriteFlags == 0) _lcd.iWriteFlags = DRAW_TO_LCD; // default write mode
  szTemp[0] = c; szTemp[1] = 0;
  if (_lcd.pFont == NULL) { // use built-in fonts
      if (_lcd.iFont == FONT_8x8 || _lcd.iFont == FONT_6x8) {
        h = 8;
        w = (_lcd.iFont == FONT_8x8) ? 8 : 6;
      } else if (_lcd.iFont == FONT_12x16 || _lcd.iFont == FONT_16x16) {
        h = 16;
        w = (_lcd.iFont == FONT_12x16) ? 12 : 16;
      } else { w = 16; h = 32; }

    if (c == '\n') {              // Newline?
      _lcd.iCursorX = 0;          // Reset x to zero,
      _lcd.iCursorY += h; // advance y one line
        // should we scroll the screen up 1 line?
        if ((_lcd.iCursorY + (h-1)) >= _lcd.iCurrentHeight && _lcd.pBackBuffer && _lcd.bScroll) {
            spilcdScroll1Line(&_lcd, h);
            spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, _lcd.iWriteFlags);
            _lcd.iCursorY -= h;
        }
    } else if (c != '\r') {       // Ignore carriage returns
      if (_lcd.iWrap && ((_lcd.iCursorX + w) > _lcd.iCurrentWidth)) { // Off right?
        _lcd.iCursorX = 0;               // Reset x to zero,
        _lcd.iCursorY += h; // advance y one line
          // should we scroll the screen up 1 line?
          if ((_lcd.iCursorY + (h-1)) >= _lcd.iCurrentHeight && _lcd.pBackBuffer && _lcd.bScroll) {
              spilcdScroll1Line(&_lcd, h);
              spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, _lcd.iWriteFlags);
              _lcd.iCursorY -= h;
          }
      }
      spilcdWriteString(&_lcd, -1, -1, szTemp, _lcd.iFG, _lcd.iBG, _lcd.iFont, _lcd.iWriteFlags);
    }
  } else { // Custom font
    if (c == '\n') {
      _lcd.iCursorX = 0;
      _lcd.iCursorY += (uint8_t)pgm_read_byte(&_lcd.pFont->yAdvance);
    } else if (c != '\r') {
      uint8_t first = pgm_read_byte(&_lcd.pFont->first);
      if ((c >= first) && (c <= (uint8_t)pgm_read_byte(&_lcd.pFont->last))) {
        GFXglyph *glyph = pgm_read_glyph_ptr(_lcd.pFont, c - first);
        w = pgm_read_byte(&glyph->width);
        h = pgm_read_byte(&glyph->height);
        if ((w > 0) && (h > 0)) { // Is there an associated bitmap?
          int16_t xo = (int8_t)pgm_read_byte(&glyph->xOffset);
          w += xo; // xadvance
          h = (uint8_t)pgm_read_byte(&_lcd.pFont->yAdvance);
          if (_lcd.iAntialias) { w /= 2; h /= 2; }
          if (_lcd.iWrap && ((_lcd.iCursorX + w) > _lcd.iCurrentWidth)) {
            _lcd.iCursorX = 0;
            _lcd.iCursorY += h;
          }
          if (_lcd.iAntialias)
            spilcdWriteStringAntialias(&_lcd, _lcd.pFont, -1, -1, szTemp, _lcd.iFG, 0, _lcd.iWriteFlags);
          else
            spilcdWriteStringCustom(&_lcd, _lcd.pFont, -1, -1, szTemp, _lcd.iFG, _lcd.iBG, 0, _lcd.iWriteFlags);
        }
      }
    }
  }
  return 1;
} /* write() */

void BB_SPI_LCD::display(void)
{
    spilcdShowBuffer(&_lcd, 0, 0, _lcd.iCurrentWidth, _lcd.iCurrentHeight, DRAW_TO_LCD);
}
void BB_SPI_LCD::display(int x, int y, int w, int h)
{
    spilcdShowBuffer(&_lcd, x, y, w, h, DRAW_TO_LCD);
}

int BB_SPI_LCD::rotateSprite(BB_SPI_LCD *pDstSprite, int iCenterX, int iCenterY, int iAngle)
{
    if (!pDstSprite || iAngle < 0 || iAngle > 359) return BB_ERROR_INV_PARAM;
    if (!_lcd.pBackBuffer || !pDstSprite->_lcd.pBackBuffer) return BB_ERROR_INV_PARAM;

    spilcdRotateBitmap(_lcd.pBackBuffer, pDstSprite->_lcd.pBackBuffer, 16,
       _lcd.iWidth, _lcd.iHeight, _lcd.iWidth*2, iCenterX, iCenterY, iAngle);
    return BB_ERROR_SUCCESS;
} /* rotateSprite() */

void BB_SPI_LCD::drawPixel(int16_t x, int16_t y, uint16_t color, int iFlags)
{
  spilcdSetPosition(&_lcd, x, y, 1, 1, iFlags);
  if (!(_lcd.iLCDFlags & FLAGS_SWAP_COLOR)) {
  	color = __builtin_bswap16(color);
  }
  spilcdWriteDataBlock(&_lcd, (uint8_t *)&color, 2, iFlags);
}

void BB_SPI_LCD::setWordwrap(int bWrap)
{
  _lcd.iWrap = bWrap;
}
void BB_SPI_LCD::setAntialias(bool bAntialias)
{
  _lcd.iAntialias = (int)bAntialias;
}
int16_t BB_SPI_LCD::getCursorX(void)
{
  return _lcd.iCursorX;
}
int16_t BB_SPI_LCD::getCursorY(void)
{
  return _lcd.iCursorY;
}
uint8_t BB_SPI_LCD::getRotation(void)
{
  return _lcd.iOrientation;
}
int16_t BB_SPI_LCD::width(void)
{
   return _lcd.iCurrentWidth;
}
int16_t BB_SPI_LCD::height(void)
{
   return _lcd.iCurrentHeight;
}
void BB_SPI_LCD::drawCircle(int32_t x, int32_t y, int32_t r, uint32_t color, int iFlags)
{
  spilcdEllipse(&_lcd, x, y, r, r, 0xf, (uint16_t)color, 0, iFlags);
}
void BB_SPI_LCD::fillCircle(int32_t x, int32_t y, int32_t r, uint32_t color, int iFlags)
{
  spilcdEllipse(&_lcd, x, y, r, r, 0xf, (uint16_t)color, 1, iFlags);
}
void BB_SPI_LCD::drawEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color, int iFlags)
{
  spilcdEllipse(&_lcd, x, y, rx, ry, 0xf, (uint16_t)color, 0, iFlags);
}
void BB_SPI_LCD::fillEllipse(int16_t x, int16_t y, int32_t rx, int32_t ry, uint16_t color, int iFlags)
{
  spilcdEllipse(&_lcd, x, y, rx, ry, 0xf, (uint16_t)color, 1, iFlags);
}

void BB_SPI_LCD::pushImage(int x, int y, int w, int h, uint16_t *pixels, int iFlags)
{
  spilcdSetPosition(&_lcd, x, y, w, h, DRAW_TO_LCD);
  spilcdWriteDataBlock(&_lcd, (uint8_t *)pixels, w*h*2, iFlags);
}
void BB_SPI_LCD::readImage(int x, int y, int w, int h, uint16_t *pixels)
{
uint16_t *s;
int ty;

  if (!pixels || !_lcd.pBackBuffer || x < 0 || y < 0 || w < 0 || h < 0) return;
  if (x+w > _lcd.iCurrentWidth || y+h > _lcd.iCurrentHeight) return;
  s = (uint16_t *)&_lcd.pBackBuffer[(y * _lcd.iScreenPitch) + (x*2)];
  for (ty=0; ty < h; ty++) {
      memcpy(pixels, s, w*2);
      pixels += w;
      s += _lcd.iScreenPitch / 2;
  } // for ty   
}
#endif // __cplusplus
