/*
  Adapted from the Adafruit and Xark's PDQ graphicstest sketch.

  See end of file for original header text and MIT license info.
*/

/*******************************************************************************
 * Start of Arduino_GFX setting
 ******************************************************************************/
/* Arduino_GFX try to find the settings depends on selected board in Arduino IDE */
/* Or you can define the dev kit cannot find in board list */
#include <Arduino_GFX_Library.h>

// #define ESP32_2432S028
// #define ESP32_3248S035
// #define ESP32_4827S043
// #define ESP32_8048S070
// #define ESP32_LCDKIT_SPI
// #define ESP32_LCDKIT_PAR8A
// #define ESP32_LCDKIT_PAR8B
// #define ESP32_LCDKIT_PAR16
// #define ESP32_S3_EYE
// #define ESP32_S3_RGB
// #define ESP32_S3_RPI_DPI
// #define MAKERFABS_TFT_TOUCH_3_5
// #define TTGO_T_DISPLAY
// #define TTGO_T_DISPLAY_S3
// #define TTGO_T_QT
// #define WT32_SC01
// #define ZX3D50CE02S
// #define ZX3D95CE01S_AR
#if defined(ESP32_2432S028)
#define TFT_BL 21
Arduino_DataBus *bus = new Arduino_ESP32SPI(2 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, 12 /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, -1 /* RST */, 0 /* rotation */);

#elif defined(ESP32_3248S035)
#define TFT_BL 27
Arduino_DataBus *bus = new Arduino_ESP32SPI(2 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, 12 /* MISO */, HSPI /* spi_num */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, -1 /* RST */, 0 /* rotation */);

#elif defined(ESP32_4827S043)
#define TFT_BL 2
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    40 /* DE */, 41 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    45 /* R0 */, 48 /* R1 */, 47 /* R2 */, 21 /* R3 */, 14 /* R4 */,
    5 /* G0 */, 6 /* G1 */, 7 /* G2 */, 15 /* G3 */, 16 /* G4 */, 4 /* G5 */,
    8 /* B0 */, 3 /* B1 */, 46 /* B2 */, 9 /* B3 */, 1 /* B4 */
);
// option 1:
// ILI6485 LCD 480x272
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    480 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 43 /* hsync_back_porch */,
    272 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 12 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 9000000 /* prefer_speed */, true /* auto_flush */);
// option 2:
// ST7262 IPS LCD 800x480
// Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
//   bus,
//   800 /* width */, 0 /* hsync_polarity */, 8 /* hsync_front_porch */, 4 /* hsync_pulse_width */, 8 /* hsync_back_porch */,
//   480 /* height */, 0 /* vsync_polarity */, 8 /* vsync_front_porch */, 4 /* vsync_pulse_width */, 8 /* vsync_back_porch */,
//   1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);
// option 3:
// RPi DPI 1024x600
// Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
//   bus,
//   1024 /* width */, 1 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 128 /* hsync_back_porch */,
//   600 /* height */, 1 /* vsync_polarity */, 13 /* vsync_front_porch */, 3 /* vsync_pulse_width */, 45 /* vsync_back_porch */);

#elif defined(ESP32_8048S070)
#define TFT_BL 2
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    41 /* DE */, 40 /* VSYNC */, 39 /* HSYNC */, 42 /* PCLK */,
    14 /* R0 */, 21 /* R1 */, 47 /* R2 */, 48 /* R3 */, 45 /* R4 */,
    9 /* G0 */, 46 /* G1 */, 3 /* G2 */, 8 /* G3 */, 16 /* G4 */, 1 /* G5 */,
    15 /* B0 */, 7 /* B1 */, 6 /* B2 */, 5 /* B3 */, 4 /* B4 */
);
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    800 /* width */, 0 /* hsync_polarity */, 210 /* hsync_front_porch */, 30 /* hsync_pulse_width */, 16 /* hsync_back_porch */,
    480 /* height */, 0 /* vsync_polarity */, 22 /* vsync_front_porch */, 13 /* vsync_pulse_width */, 10 /* vsync_back_porch */,
    1 /* pclk_active_neg */, 16000000 /* prefer_speed */, true /* auto_flush */);

#elif defined(ESP32_LCDKIT_SPI)
#define TFT_BL 23
Arduino_DataBus *bus = new Arduino_ESP32SPI(19 /* DC */, 5 /* CS */, 22 /* SCK */, 21 /* MOSI */, 27 /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, 18 /* RST */, 1 /* rotation */);

#elif defined(ESP32_LCDKIT_PAR8A)
Arduino_DataBus *bus = new Arduino_ESP32PAR8(5 /* DC */, GFX_NOT_DEFINED /* CS */, 18 /* WR */, GFX_NOT_DEFINED /* RD */, 19 /* D0 */, 21 /* D1 */, 0 /* D2 */, 22 /* D3 */, 23 /* D4 */, 33 /* D5 */, 32 /* D6 */, 27 /* D7 */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */);

#elif defined(ESP32_LCDKIT_PAR8B)
Arduino_DataBus *bus = new Arduino_ESP32PAR8(5 /* DC */, GFX_NOT_DEFINED /* CS */, 18 /* WR */, GFX_NOT_DEFINED /* RD */, 25 /* D0 */, 26 /* D1 */, 12 /* D2 */, 13 /* D3 */, 14 /* D4 */, 15 /* D5 */, 2 /* D6 */, 4 /* D7 */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */);

#elif defined(ESP32_LCDKIT_PAR16)
Arduino_DataBus *bus = new Arduino_ESP32PAR16(
    5 /* DC */, GFX_NOT_DEFINED /* CS */, 18 /* WR */, GFX_NOT_DEFINED /* RD */,
    19 /* D0 */, 21 /* D1 */, 0 /* D2 */, 22 /* D3 */, 23 /* D4 */, 33 /* D5 */, 32 /* D6 */, 27 /* D7 */,
    25 /* D8 */, 26 /* D9 */, 12 /* D10 */, 13 /* D11 */, 14 /* D12 */, 15 /* D13 */, 2 /* D14 */, 4 /* D15 */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */);

#elif defined(ESP32_S3_EYE)
#define TFT_BL 48
Arduino_DataBus *bus = new Arduino_ESP32SPI(43 /* DC */, 44 /* CS */, 21 /* SCK */, 47 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */, 240 /* width */, 240 /* height */, 0 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col offset 2 */, 80 /* row offset 2 */);

#elif defined(ESP32_S3_RGB)
#define TFT_BL 38
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    39 /* CS */, 48 /* SCK */, 47 /* SDA */,
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */,
    4 /* R0 */, 3 /* R1 */, 2 /* R2 */, 1 /* R3 */, 0 /* R4 */,
    10 /* G0 */, 9 /* G1 */, 8 /* G2 */, 7 /* G3 */, 6 /* G4 */, 5 /* G5 */,
    15 /* B0 */, 14 /* B1 */, 13 /* B2 */, 12 /* B3 */, 11 /* B4 */
);
// Uncomment for 4" rect display
Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
    bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */,
    true /* IPS */, 480 /* width */, 480 /* height */,
    st7701_type1_init_operations, sizeof(st7701_type1_init_operations),
     true /* BGR */,
     10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
     10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);
// Uncomment for 2.1" round display
// Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
//     bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */,
//     false /* IPS */, 480 /* width */, 480 /* height */,
//     st7701_type5_init_operations, sizeof(st7701_type5_init_operations),
//     true /* BGR */,
//     10 /* hsync_front_porch */, 8 /* hsync_pulse_width */, 50 /* hsync_back_porch */,
//     10 /* vsync_front_porch */, 8 /* vsync_pulse_width */, 20 /* vsync_back_porch */);
// Uncomment for 2.8" round display
// Arduino_ST7701_RGBPanel *gfx = new Arduino_ST7701_RGBPanel(
//     bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */,
//     false /* IPS */, 480 /* width */, 480 /* height */,
//     st7701_type6_init_operations, sizeof(st7701_type6_init_operations),
//     true /* BGR */,
//     22 /* hsync_front_porch */, 3 /* hsync_pulse_width */, 45 /* hsync_back_porch */,
//     4 /* vsync_front_porch */, 12 /* vsync_pulse_width */, 40 /* vsync_back_porch */);

#elif defined(ESP32_S3_RPI_DPI)
// #define TFT_BL 38
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    GFX_NOT_DEFINED /* CS */, GFX_NOT_DEFINED /* SCK */, GFX_NOT_DEFINED /* SDA */,
    18 /* DE */, 17 /* VSYNC */, 16 /* HSYNC */, 21 /* PCLK */,
    4 /* R0 */, 3 /* R1 */, 2 /* R2 */, 1 /* R3 */, 0 /* R4 */,
    10 /* G0 */, 9 /* G1 */, 8 /* G2 */, 7 /* G3 */, 6 /* G4 */, 5 /* G5 */,
    15 /* B0 */, 14 /* B1 */, 13 /* B2 */, 12 /* B3 */, 11 /* B4 */
);
// e.g. Waveshare 7" RPi DPI LCD: https://www.waveshare.com/wiki/7inch_LCD_for_Pi
// dpi_timings=1024 1 40 48 128 600 1 13 3 45 0 0 0 60 0 37000000 6
Arduino_RPi_DPI_RGBPanel *gfx = new Arduino_RPi_DPI_RGBPanel(
    bus,
    1024 /* width */, 1 /* hsync_polarity */, 40 /* hsync_front_porch */, 48 /* hsync_pulse_width */, 128 /* hsync_back_porch */,
    600 /* height */, 1 /* vsync_polarity */, 13 /* vsync_front_porch */, 3 /* vsync_pulse_width */, 45 /* vsync_back_porch */);

#elif defined(MAKERFABS_TFT_TOUCH_3_5)
Arduino_DataBus *bus = new Arduino_ESP32SPI(33 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, 12 /* MISO */);
Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */, false /* IPS */);

#elif defined(TTGO_T_DISPLAY)
#define TFT_BL 4
Arduino_DataBus *bus = new Arduino_ESP32SPI(16 /* DC */, 5 /* CS */, 18 /* SCK */, 19 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 23 /* RST */, 0 /* rotation */, true /* IPS */, 135 /* width */, 240 /* height */, 52 /* col offset 1 */, 40 /* row offset 1 */, 53 /* col offset 2 */, 40 /* row offset 2 */);

#elif defined(TTGO_T_DISPLAY_S3)
#define TFT_PWD 15
#define TFT_BL 38
Arduino_DataBus *bus = new Arduino_ESP32LCD8(
    7 /* DC */, 6 /* CS */, 8 /* WR */, 9 /* RD */,
    39 /* D0 */, 40 /* D1 */, 41 /* D2 */, 42 /* D3 */, 45 /* D4 */, 46 /* D5 */, 47 /* D6 */, 48 /* D7 */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, 5 /* RST */, 0 /* rotation */, true /* IPS */, 170 /* width */, 320 /* height */, 35 /* col offset 1 */, 0 /* row offset 1 */, 35 /* col offset 2 */, 0 /* row offset 2 */);

#elif defined(TTGO_T_QT)
#define TFT_BL 10
Arduino_DataBus *bus = new Arduino_ESP32SPI(6 /* DC */, 5 /* CS */, 3 /* SCK */, 2 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_GC9107(bus, 1 /* RST */, 0 /* rotation */, true /* IPS */);

#elif defined(WT32_SC01)
#define TFT_BL 23
Arduino_DataBus *bus = new Arduino_ESP32SPI(21 /* DC */, 15 /* CS */, 14 /* SCK */, 13 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 22 /* RST */, 3 /* rotation */);

/* Wio Terminal */
#elif defined(ARDUINO_ARCH_SAMD) && defined(SEEED_GROVE_UI_WIRELESS)
// #define TFT_BL LCD_BACKLIGHT
Arduino_DataBus *bus = new Arduino_HWSPI(LCD_DC /* DC */, LCD_SS_PIN /* CS */);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */);

/* ESP32-S3-BOX */
#elif defined(ARDUINO_ESP32_S3_BOX)
#define TFT_BL 45
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, TFT_CLK, TFT_MOSI, TFT_MISO);
Arduino_GFX *gfx = new Arduino_ILI9342(bus, TFT_RST, 0 /* rotation */);

/* M5Stack */
#elif defined(ARDUINO_M5Stack_Core_ESP32) || defined(ARDUINO_M5STACK_FIRE)
// #define TFT_BL 32
Arduino_DataBus *bus = new Arduino_ESP32SPI(27 /* DC */, 14 /* CS */, SCK, MOSI, MISO);
Arduino_GFX *gfx = new Arduino_ILI9342(bus, 33 /* RST */, 2 /* rotation */);

/* Odroid-Go */
#elif defined(ARDUINO_ODROID_ESP32)
// #define TFT_BL 14
Arduino_DataBus *bus = new Arduino_ESP32SPI(21 /* DC */, 5 /* CS */, SCK, MOSI, MISO);
Arduino_GFX *gfx = new Arduino_ILI9341(bus, GFX_NOT_DEFINED /* RST */, 3 /* rotation */);
// Arduino_ST7789 *gfx = new Arduino_ST7789(bus, GFX_NOT_DEFINED /* RST */, 3 /* rotation */, true /* IPS */);

/* TTGO T-Watch */
#elif defined(ARDUINO_T) || defined(ARDUINO_TWATCH_BASE) || defined(ARDUINO_TWATCH_2020_V1) || defined(ARDUINO_TWATCH_2020_V2)
// #define TFT_BL 12
Arduino_DataBus *bus = new Arduino_ESP32SPI(27 /* DC */, 5 /* CS */, 18 /* SCK */, 19 /* MOSI */, GFX_NOT_DEFINED /* MISO */);
Arduino_GFX *gfx = new Arduino_ST7789(bus, GFX_NOT_DEFINED /* RST */, 0 /* rotation */, true /* IPS */, 240, 240, 0, 80);

#elif defined(ZX3D50CE02S)
#define TFT_BL 45
Arduino_DataBus *bus = new Arduino_ESP32LCD8(
    0 /* DC */, GFX_NOT_DEFINED /* CS */, 47 /* WR */, GFX_NOT_DEFINED /* RD */,
    9 /* D0 */, 46 /* D1 */, 3 /* D2 */, 8 /* D3 */, 18 /* D4 */, 17 /* D5 */, 16 /* D6 */, 15 /* D7 */);
Arduino_GFX *gfx = new Arduino_ST7796(bus, 4 /* RST */, 0 /* rotation */, true /* IPS */);

#elif defined(ZX3D95CE01S_AR)
#define TFT_BL 45
Arduino_ESP32RGBPanel *bus = new Arduino_ESP32RGBPanel(
    0 /* CS */, 10 /* SCK */, 9 /* SDA */,
    13 /* DE */, 12 /* VSYNC */, 11 /* HSYNC */, 14 /* PCLK */,
    2 /* R0 */, 17 /* R1 */, 16 /* R2 */, 1 /* R3 */, 15 /* R4 */,
    41 /* G0 */, 46 /* G1 */, 3 /* G2 */, 42 /* G3 */, 8 /* G4 */, 18 /* G5 */,
    10 /* B0 */, 9 /* B1 */, 40 /* B2 */, 20 /* B3 */, 19 /* B4 */
);
Arduino_GC9503V_RGBPanel *gfx = new Arduino_GC9503V_RGBPanel(bus, GFX_NOT_DEFINED /* RST */, 480, 480);

#else /* not selected specific hardware */

#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
// PJRC Teensy 4.x
#define TFT_CS 39 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 41
#define TFT_RST 40
#define TFT_BL 22
#elif defined(ARDUINO_BLACKPILL_F411CE)
#define TFT_CS 4 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 3
#define TFT_RST 2
#define TFT_BL 1
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
#define TFT_CS 17 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 27
#define TFT_RST 26
#define TFT_BL 28
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)
#define TFT_CS 5  // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 27 // GFX_NOT_DEFINED for display without DC pin (9-bit SPI)
#define TFT_RST 33
#define TFT_BL 22
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S2)
#define TFT_CS 34 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 35
#define TFT_RST 33
#define TFT_BL 21
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
#define TFT_CS 40 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 41
#define TFT_RST 42
#define TFT_BL 48
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32C3)
#define TFT_CS 7 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 2
#define TFT_RST 1
#define TFT_BL 3
#elif defined(ESP8266)
#define TFT_CS 15 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 4
#define TFT_RST 2
#define TFT_BL 5
#elif defined(RTL8722DM)
#if defined(BOARD_RTL8720DN_BW16)
#define TFT_CS 9
#define TFT_DC 8
#define TFT_RST 6
#define TFT_BL 3
#elif defined(BOARD_RTL8722DM)
#define TFT_CS 18
#define TFT_DC 17
#define TFT_RST 22
#define TFT_BL 23
#elif defined(BOARD_RTL8722DM_MINI)
#define TFT_CS 12
#define TFT_DC 14
#define TFT_RST 15
#define TFT_BL 13
#else             // old version
#define TFT_CS 18 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 17
#define TFT_RST 2
#define TFT_BL 23
#endif
#elif defined(SEEED_XIAO_M0)
#define TFT_CS 3 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 2
#define TFT_RST 1
#define TFT_BL 0
#else
#define TFT_CS 9 // GFX_NOT_DEFINED for display without CS pin
#define TFT_DC 8
#define TFT_RST 7
#define TFT_BL 6
#endif

/*
 * Step 1: Initize one databus for your display
 */

// General software SPI
// Arduino_DataBus *bus = new Arduino_SWSPI(TFT_DC, TFT_CS, 18 /* SCK */, 23 /* MOSI */, GFX_NOT_DEFINED /* MISO */);

// hardware SPI
#if defined(ARDUINO_ARCH_NRF52840)
// Arduino_DataBus *bus = new Arduino_mbedSPI(TFT_DC, TFT_CS);
Arduino_DataBus *bus = new Arduino_NRFXSPI(TFT_DC, TFT_CS, 13 /* SCK */, 11 /* MOSI */, 12 /* MISO */);
#elif defined(ARDUINO_RASPBERRY_PI_PICO) || defined(ARDUINO_RASPBERRY_PI_PICO_W)
Arduino_DataBus *bus = new Arduino_RPiPicoSPI(TFT_DC /* DC */, TFT_CS /* CS */, 18 /* SCK */, 19 /* MOSI */, 16 /* MISO */, spi0 /* spi */);
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32)
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, 18 /* SCK */, 23 /* MOSI */, GFX_NOT_DEFINED /* MISO */, VSPI /* spi_num */);
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3)
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, 36 /* SCK */, 35 /* MOSI */, GFX_NOT_DEFINED /* MISO */, HSPI /* spi_num */);
#elif defined(ESP32) && (CONFIG_IDF_TARGET_ESP32C3)
Arduino_DataBus *bus = new Arduino_ESP32SPI(TFT_DC, TFT_CS, 4 /* SCK */, 6 /* MOSI */, GFX_NOT_DEFINED /* MISO */, FSPI /* spi_num */);
#elif defined(ESP8266)
Arduino_DataBus *bus = new Arduino_ESP8266SPI(TFT_DC, TFT_CS);
#else
// General hardware SPI
Arduino_DataBus *bus = new Arduino_HWSPI(TFT_DC, TFT_CS);
#endif

// General Software parallel 8-bit
// Arduino_DataBus *bus = new Arduino_SWPAR8(TFT_DC, TFT_CS, 25 /* WR */, 32 /* RD */, 23 /* D0 */, 19 /* D1 */, 18 /* D2 */, 26 /* D3 */, 21 /* D4 */, 4 /* D5 */, 0 /* D6 */, 2 /* D7 */);

// AVR parallel 8-bit
// Arduino Pro Micro port 2(PB): 17, 15, 16, 14, 8, 9, 10, 11
// Arduino_DataBus *bus = new Arduino_AVRPAR8(4 /* DC */, 5 /* CS */, 18 /* WR */, 19 /* RD */, 2 /* PORT */);

// ESP32 parallel 8-bit
// Arduino_DataBus *bus = new Arduino_ESP32PAR8(TFT_DC, TFT_CS, 25 /* WR */, 32 /* RD */, 23 /* D0 */, 19 /* D1 */, 18 /* D2 */, 26 /* D3 */, 21 /* D4 */, 4 /* D5 */, 0 /* D6 */, 2 /* D7 */);

// ESP32 parallel 16-bit
// Almost all GPIO 0-31 used up for 16-bit and WR, disable PSRAM to gain 16 and 17 but still no GPIOs remain for CS and RD.
// CS connect to GND (enable); RD connect to Vcc (disable).
// Arduino_DataBus *bus = new Arduino_ESP32PAR16(
//     32 /* DC */, GFX_NOT_DEFINED /* CS */, 21 /* WR */, GFX_NOT_DEFINED /* RD */,
//     19 /* D0 */, 23 /* D1 */, 18 /* D2 */, 5 /* D3 */, 17 /* D4 */, 16 /* D5 */, 25 /* D6 */, 26 /* D7 */,
//     27 /* D8 */, 14 /* D9 */, 12 /* D10 */, 13 /* D11 */, 15 /* D12 */, 2 /* D13 */, 0 /* D14 */, 4 /* D15 */);

// ESP32S2 parallel 8-bit
// Display D0-D7 connect to GPIO 0-7
// Arduino_DataBus *bus = new Arduino_ESP32S2PAR8(TFT_DC, TFT_CS, 16 /* WR */, 17 /* RD */);

// ESP32S2 parallel 16-bit
// Display D0-D15 connect to GPIO 0-15
// Arduino_DataBus *bus = new Arduino_ESP32S2PAR16(TFT_DC, TFT_CS, 16 /* WR */, 17 /* RD */);

// ESP32S3 i80 LCD parallel 8-bit
// Arduino_DataBus *bus = new Arduino_ESP32LCD8(
//     TFT_DC, TFT_CS, 16 /* WR */, 17 /* RD */,
//     0 /* D0 */, 1 /* D1 */, 2 /* D2 */, 3 /* D3 */, 4 /* D4 */, 5 /* D5 */, 6 /* D6 */, 7 /* D7 */);

// ESP32S3 i80 LCD parallel 16-bit
// Arduino_DataBus *bus = new Arduino_ESP32LCD16(
//     TFT_DC, TFT_CS, 16 /* WR */, 17 /* RD */,
//     0 /* D0 */, 1 /* D1 */, 2 /* D2 */, 3 /* D3 */, 4 /* D4 */, 5 /* D5 */, 6 /* D6 */, 7 /* D7 */,
//     8 /* D8 */, 9 /* D9 */, 10 /* D10 */, 11 /* D11 */, 12 /* D12 */, 13 /* D13 */, 14 /* D14 */, 15 /* D15 */);

// Raspberry Pi Pico parallel 8-bit
// Display D0-D7 connect to GPIO 0-7
// Arduino_DataBus *bus = new Arduino_RPiPicoPAR8(TFT_DC, TFT_CS, 18 /* WR */, 19 /* RD */);

// Raspberry Pi Pico parallel 16-bit
// Display D0-D15 connect to GPIO 0-15
// Arduino_DataBus *bus = new Arduino_RPiPicoPAR16(TFT_DC, TFT_CS, 18 /* WR */, 19 /* RD */);

// RTL8722 parallel 8-bit
// Reduce GPIO usage: CS connect to GND (enable); RD connect to Vcc (disable); No Backlight pins.
// Arduino_DataBus *bus = new Arduino_RTLPAR8(0 /* DC */, GFX_NOT_DEFINED /* CS */, 1 /* WR */, GFX_NOT_DEFINED /* RD */, 18 /* D0 */, 22 /* D1 */, 17 /* D2 */, 20 /* D3 */, 19 /* D4 */, 23 /* D5 */, 21 /* D6 */, 16 /* D7 */);

/*
 * Step 2: Initize one driver for your display
 */

/***************************************
 * Start of Canvas (framebuffer)
 **************************************/
// 16-bit color Canvas (240x320 resolution only works for ESP32 with PSRAM)
// Arduino_G *output_display = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */);
// Arduino_GFX *gfx = new Arduino_Canvas(240 /* width */, 320 /* height */, output_display);

// Indexed color Canvas, mask_level: 0-2, larger mask level mean less color variation but can have faster index mapping
// Arduino_G *output_display = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */);
// Arduino_GFX *gfx = new Arduino_Canvas_Indexed(240 /* width */, 320 /* height */, output_display, 0 /* output_x */, 0 /* output_y */, MAXMASKLEVEL /* mask_level */);

// 3-bit color Canvas, R1G1B1, 8 colors
// Arduino_G *output_display = new Arduino_ILI9488_3bit(bus, GFX_NOT_DEFINED /* RST */, 1 /* rotation */, false /* IPS */);
// Arduino_GFX *gfx = new Arduino_Canvas_3bit(480 /* width */, 320 /* height */, output_display, 0 /* output_x */, 0 /* output_y */);

// Mono color Canvas
// Arduino_G *output_display = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */);
// Arduino_GFX *gfx = new Arduino_Canvas_Mono(240 /* width */, 320 /* height */, output_display, 0 /* output_x */, 0 /* output_y */);
/***************************************
 * End of Canvas (framebuffer)
 **************************************/

// GC9A01 IPS LCD 240x240
// Arduino_GFX *gfx = new Arduino_GC9A01(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// GC9106 IPS LCD 80x160
// Arduino_GFX *gfx = new Arduino_GC9106(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// GC9107 IPS LCD 128x128
// Arduino_GFX *gfx = new Arduino_GC9107(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// HX8347C IPS LCD 240x320
// Arduino_GFX *gfx = new Arduino_HX8347C(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// HX8347D IPS LCD 240x320
// Arduino_GFX *gfx = new Arduino_HX8347D(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// HX8352C IPS LCD 240x400
// Arduino_GFX *gfx = new Arduino_HX8352C(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// HX8357A IPS LCD 320x480 (currently only portrait works, i.e. rotation 0 and 2)
// Arduino_GFX *gfx = new Arduino_HX8357A(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// HX8357B IPS LCD 320x480
// Arduino_GFX *gfx = new Arduino_HX8357B(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// ILI9225 LCD 176x220
// Arduino_GFX *gfx = new Arduino_ILI9225(bus, TFT_RST);

// ILI9341 LCD 240x320
Arduino_GFX *gfx = new Arduino_ILI9341(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9342 LCD 320x240
// Arduino_GFX *gfx = new Arduino_ILI9342(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9481 parallel 16-bit LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9481(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9481 SPI LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9481_18bit(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9486 parallel 16-bit LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9486(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9486 SPI parallel LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9486_18bit(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9488 parallel 16-bit LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9488(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9488 SPI LCD 320x480
// Arduino_GFX *gfx = new Arduino_ILI9488_18bit(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// ILI9806 LCD 480x854
// Arduino_GFX *gfx = new Arduino_ILI9806(bus, TFT_RST, 0 /* rotation */, false /* IPS */);

// JBT6K71 LCD 240x320
// Arduino_GFX *gfx = new Arduino_JBT6K71(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 240, 320, 0, 0, 16, 0);

// NT35310 LCD 320x480
// Arduino_GFX *gfx = new Arduino_NT35310(bus, TFT_RST, 0 /* rotation */);

// NT35510 LCD 480x800
// Arduino_GFX *gfx = new Arduino_NT35510(bus, TFT_RST, 0 /* rotation */);

// NT39125 LCD 240x376
// Arduino_GFX *gfx = new Arduino_NT39125(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 240, 376, 0, 0, 0, 56);

// R61529 IPS LCD 320x480
// Arduino_GFX *gfx = new Arduino_R61529(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

// SEPS525 OLED 160x128
// Arduino_GFX *gfx = new Arduino_SEPS525(bus, TFT_RST, 0 /* rotation */);

// SSD1283A OLED 130x130
// Arduino_GFX *gfx = new Arduino_SSD1283A(bus, TFT_RST, 0 /* rotation */);

// SSD1331 OLED 96x64
// Arduino_GFX *gfx = new Arduino_SSD1331(bus, TFT_RST, 0 /* rotation */);

// SSD1351 OLED 128x128
// Arduino_GFX *gfx = new Arduino_SSD1351(bus, TFT_RST, 0 /* rotation */);

// ST7735 LCD
// 1.8" REDTAB 128x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */);
// 1.8" BLACKTAB 128x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 160 /* height */, 2 /* col offset 1 */, 1 /* row offset 1 */, 2 /* col offset 2 */, 1 /* row offset 2 */, false /* BGR */);
// 1.8" GREENTAB A 128x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 160 /* height */, 2 /* col offset 1 */, 1 /* row offset 1 */, 2 /* col offset 2 */, 1 /* row offset 2 */);
// 1.8" GREENTAB B 128x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 160 /* height */, 2 /* col offset 1 */, 3 /* row offset 1 */, 2 /* col offset 2 */, 1 /* row offset 2 */);
// 1.8" Wide angle LCD 128x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 160 /* height */, 0 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col offset 2 */, 0 /* row offset 2 */, false /* BGR */);
// 1.5" GREENTAB B 128x128
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 128 /* height */, 2 /* col offset 1 */, 3 /* row offset 1 */, 2 /* col offset 2 */, 1 /* row offset 2 */);
// 1.5" GREENTAB C 128x128
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, false /* IPS */, 128 /* width */, 128 /* height */, 0 /* col offset 1 */, 32 /* row offset 1 */);
// 0.96" IPS LCD 80x160
// Arduino_GFX *gfx = new Arduino_ST7735(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 80 /* width */, 160 /* height */, 26 /* col offset 1 */, 1 /* row offset 1 */, 26 /* col offset 2 */, 1 /* row offset 2 */);

// ST7789 LCD
// 2.4" LCD 240x320
// Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */);
// 2.4" IPS LCD 240x320
// Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */);
// 1.69" IPS round corner LCD 240x280
// Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 240 /* width */, 280 /* height */, 0 /* col offset 1 */, 20 /* row offset 1 */, 0 /* col offset 2 */, 20 /* row offset 2 */);
// 1.3"/1.5" square IPS LCD 240x240
// Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 240 /* width */, 240 /* height */, 0 /* col offset 1 */, 0 /* row offset 1 */, 0 /* col offset 2 */, 80 /* row offset 2 */);
// 1.14" IPS LCD 135x240 TTGO T-Display
// Arduino_GFX *gfx = new Arduino_ST7789(bus, TFT_RST, 0 /* rotation */, true /* IPS */, 135 /* width */, 240 /* height */, 52 /* col offset 1 */, 40 /* row offset 1 */, 53 /* col offset 2 */, 40 /* row offset 2 */);

// ST7796 LCD
// 4" LCD 320x480
// Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 0 /* rotation */);
// 4" IPS LCD 320x480
// Arduino_GFX *gfx = new Arduino_ST7796(bus, TFT_RST, 0 /* rotation */, true /* IPS */);

#endif /* not selected specific hardware */
/*******************************************************************************
 * End of Arduino_GFX setting
 ******************************************************************************/

int32_t w, h, n, n1, cx, cy, cx1, cy1, cn, cn1;
uint8_t tsa, tsb, tsc, ds;

void setup()
{
  Serial.begin(115200);
  // while(!Serial);
  Serial.println("Arduino_GFX library Test!");

#ifdef TFT_PWD
  pinMode(TFT_PWD, OUTPUT);
  digitalWrite(TFT_PWD, HIGH);
#endif

  gfx->begin();
  // gfx->begin(80000000); /* specify data bus speed */

  w = gfx->width();
  h = gfx->height();
  n = min(w, h);
  n1 = n - 1;
  cx = w / 2;
  cy = h / 2;
  cx1 = cx - 1;
  cy1 = cy - 1;
  cn = min(cx1, cy1);
  cn1 = cn - 1;
  tsa = ((w <= 176) || (h <= 160)) ? 1 : (((w <= 240) || (h <= 240)) ? 2 : 3); // text size A
  tsb = ((w <= 240) || (h <= 220)) ? 1 : 2;                                    // text size B
  tsc = ((w <= 220) || (h <= 220)) ? 1 : 2;                                    // text size C
  ds = (w <= 160) ? 9 : 12;                                                    // digit size

#ifdef TFT_BL
  pinMode(TFT_BL, OUTPUT);
  digitalWrite(TFT_BL, HIGH);
#endif
}

static inline uint32_t micros_start() __attribute__((always_inline));
static inline uint32_t micros_start()
{
  uint8_t oms = millis();
  while ((uint8_t)millis() == oms)
    ;
  return micros();
}

void loop(void)
{
  Serial.println(F("Benchmark\tmicro-secs"));

  int32_t usecFillScreen = testFillScreen();
  serialOut(F("Screen fill\t"), usecFillScreen, 100, true);

  int32_t usecText = testText();
  serialOut(F("Text\t"), usecText, 3000, true);

  int32_t usecPixels = testPixels();
  serialOut(F("Pixels\t"), usecPixels, 100, true);

  int32_t usecLines = testLines();
  serialOut(F("Lines\t"), usecLines, 100, true);

  int32_t usecFastLines = testFastLines();
  serialOut(F("Horiz/Vert Lines\t"), usecFastLines, 100, true);

  int32_t usecFilledRects = testFilledRects();
  serialOut(F("Rectangles (filled)\t"), usecFilledRects, 100, false);

  int32_t usecRects = testRects();
  serialOut(F("Rectangles (outline)\t"), usecRects, 100, true);

  int32_t usecFilledTrangles = testFilledTriangles();
  serialOut(F("Triangles (filled)\t"), usecFilledTrangles, 100, false);

  int32_t usecTriangles = testTriangles();
  serialOut(F("Triangles (outline)\t"), usecTriangles, 100, true);

  int32_t usecFilledCircles = testFilledCircles(10);
  serialOut(F("Circles (filled)\t"), usecFilledCircles, 100, false);

  int32_t usecCircles = testCircles(10);
  serialOut(F("Circles (outline)\t"), usecCircles, 100, true);

  int32_t usecFilledArcs = testFillArcs();
  serialOut(F("Arcs (filled)\t"), usecFilledArcs, 100, false);

  int32_t usecArcs = testArcs();
  serialOut(F("Arcs (outline)\t"), usecArcs, 100, true);

  int32_t usecFilledRoundRects = testFilledRoundRects();
  serialOut(F("Rounded rects (filled)\t"), usecFilledRoundRects, 100, false);

  int32_t usecRoundRects = testRoundRects();
  serialOut(F("Rounded rects (outline)\t"), usecRoundRects, 100, true);

#ifdef CANVAS
  uint32_t start = micros_start();
  gfx->flush();
  int32_t usecFlush = micros() - start;
  serialOut(F("flush (Canvas only)\t"), usecFlush, 0, false);
#endif

  Serial.println(F("Done!"));

  uint16_t c = 4;
  int8_t d = 1;
  for (int32_t i = 0; i < h; i++)
  {
    gfx->drawFastHLine(0, i, w, c);
    c += d;
    if (c <= 4 || c >= 11)
    {
      d = -d;
    }
  }

  gfx->setCursor(0, 0);

  gfx->setTextSize(tsa);
  gfx->setTextColor(MAGENTA);
  gfx->println(F("Arduino GFX PDQ"));

  if (h > w)
  {
    gfx->setTextSize(tsb);
    gfx->setTextColor(GREEN);
    gfx->print(F("\nBenchmark "));
    gfx->setTextSize(tsc);
    if (ds == 12)
    {
      gfx->print(F("   "));
    }
    gfx->println(F("micro-secs"));
  }

  gfx->setTextSize(1);
  printnice(F("Screen fill "), usecFillScreen);
  printnice(F("Text        "), usecText);
  printnice(F("Pixels      "), usecPixels);
  printnice(F("Lines       "), usecLines);
  printnice(F("H/V Lines   "), usecFastLines);
  printnice(F("Rectangles F"), usecFilledRects);
  printnice(F("Rectangles  "), usecRects);
  printnice(F("Triangles F "), usecFilledTrangles);
  printnice(F("Triangles   "), usecTriangles);
  printnice(F("Circles F   "), usecFilledCircles);
  printnice(F("Circles     "), usecCircles);
  printnice(F("Arcs F      "), usecFilledArcs);
  printnice(F("Arcs        "), usecArcs);
  printnice(F("RoundRects F"), usecFilledRoundRects);
  printnice(F("RoundRects  "), usecRoundRects);

  if ((h > w) || (h > 240))
  {
    gfx->setTextSize(tsc);
    gfx->setTextColor(GREEN);
    gfx->print(F("\nBenchmark Complete!"));
  }

#ifdef CANVAS
  gfx->flush();
#endif

  delay(60 * 1000L);
}

void serialOut(const __FlashStringHelper *item, int32_t v, uint32_t d, bool clear)
{
#ifdef CANVAS
  gfx->flush();
#endif
  Serial.print(item);
  if (v < 0)
  {
    Serial.println(F("N/A"));
  }
  else
  {
    Serial.println(v);
  }
  delay(d);
  if (clear)
  {
    gfx->fillScreen(BLACK);
  }
}

void printnice(const __FlashStringHelper *item, long int v)
{
  gfx->setTextSize(tsb);
  gfx->setTextColor(CYAN);
  gfx->print(item);

  gfx->setTextSize(tsc);
  gfx->setTextColor(YELLOW);
  if (v < 0)
  {
    gfx->println(F("      N / A"));
  }
  else
  {
    char str[32] = {0};
#ifdef RTL8722DM
    sprintf(str, "%d", (int)v);
#else
    sprintf(str, "%ld", v);
#endif
    for (char *p = (str + strlen(str)) - 3; p > str; p -= 3)
    {
      memmove(p + 1, p, strlen(p) + 1);
      *p = ',';
    }
    while (strlen(str) < ds)
    {
      memmove(str + 1, str, strlen(str) + 1);
      *str = ' ';
    }
    gfx->println(str);
  }
}

int32_t testFillScreen()
{
  uint32_t start = micros_start();
  // Shortened this tedious test!
  gfx->fillScreen(WHITE);
  gfx->fillScreen(RED);
  gfx->fillScreen(GREEN);
  gfx->fillScreen(BLUE);
  gfx->fillScreen(BLACK);

  return micros() - start;
}

int32_t testText()
{
  uint32_t start = micros_start();
  gfx->setCursor(0, 0);

  gfx->setTextSize(1);
  gfx->setTextColor(WHITE, BLACK);
  gfx->println(F("Hello World!"));

  gfx->setTextSize(2);
  gfx->setTextColor(gfx->color565(0xff, 0x00, 0x00));
  gfx->print(F("RED "));
  gfx->setTextColor(gfx->color565(0x00, 0xff, 0x00));
  gfx->print(F("GREEN "));
  gfx->setTextColor(gfx->color565(0x00, 0x00, 0xff));
  gfx->println(F("BLUE"));

  gfx->setTextSize(tsa);
  gfx->setTextColor(YELLOW);
  gfx->println(1234.56);

  gfx->setTextColor(WHITE);
  gfx->println((w > 128) ? 0xDEADBEEF : 0xDEADBEE, HEX);

  gfx->setTextColor(CYAN, WHITE);
  gfx->println(F("Groop,"));

  gfx->setTextSize(tsc);
  gfx->setTextColor(MAGENTA, WHITE);
  gfx->println(F("I implore thee,"));

  gfx->setTextSize(1);
  gfx->setTextColor(NAVY, WHITE);
  gfx->println(F("my foonting turlingdromes."));

  gfx->setTextColor(DARKGREEN, WHITE);
  gfx->println(F("And hooptiously drangle me"));

  gfx->setTextColor(DARKCYAN, WHITE);
  gfx->println(F("with crinkly bindlewurdles,"));

  gfx->setTextColor(MAROON, WHITE);
  gfx->println(F("Or I will rend thee"));

  gfx->setTextColor(PURPLE, WHITE);
  gfx->println(F("in the gobberwartsb"));

  gfx->setTextColor(OLIVE, WHITE);
  gfx->println(F("with my blurglecruncheon,"));

  gfx->setTextColor(DARKGREY, WHITE);
  gfx->println(F("see if I don't!"));

  gfx->setTextSize(2);
  gfx->setTextColor(RED);
  gfx->println(F("Size 2"));

  gfx->setTextSize(3);
  gfx->setTextColor(ORANGE);
  gfx->println(F("Size 3"));

  gfx->setTextSize(4);
  gfx->setTextColor(YELLOW);
  gfx->println(F("Size 4"));

  gfx->setTextSize(5);
  gfx->setTextColor(GREENYELLOW);
  gfx->println(F("Size 5"));

  gfx->setTextSize(6);
  gfx->setTextColor(GREEN);
  gfx->println(F("Size 6"));

  gfx->setTextSize(7);
  gfx->setTextColor(BLUE);
  gfx->println(F("Size 7"));

  gfx->setTextSize(8);
  gfx->setTextColor(PURPLE);
  gfx->println(F("Size 8"));

  gfx->setTextSize(9);
  gfx->setTextColor(PINK);
  gfx->println(F("Size 9"));

  return micros() - start;
}

int32_t testPixels()
{
  uint32_t start = micros_start();

  for (int16_t y = 0; y < h; y++)
  {
    for (int16_t x = 0; x < w; x++)
    {
      gfx->drawPixel(x, y, gfx->color565(x << 3, y << 3, x * y));
    }
#ifdef ESP8266
    yield(); // avoid long run triggered ESP8266 WDT restart
#endif
  }

  return micros() - start;
}

int32_t testLines()
{
  uint32_t start;
  int32_t x1, y1, x2, y2;

  start = micros_start();

  x1 = y1 = 0;
  y2 = h - 1;
  for (x2 = 0; x2 < w; x2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x1 = w - 1;
  y1 = 0;
  y2 = h - 1;
  for (x2 = 0; x2 < w; x2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x1 = 0;
  y1 = h - 1;
  y2 = 0;
  for (x2 = 0; x2 < w; x2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x2 = w - 1;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x1 = w - 1;
  y1 = h - 1;
  y2 = 0;
  for (x2 = 0; x2 < w; x2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  x2 = 0;
  for (y2 = 0; y2 < h; y2 += 6)
  {
    gfx->drawLine(x1, y1, x2, y2, BLUE);
  }
#ifdef ESP8266
  yield(); // avoid long run triggered ESP8266 WDT restart
#endif

  return micros() - start;
}

int32_t testFastLines()
{
  uint32_t start;
  int32_t x, y;

  start = micros_start();

  for (y = 0; y < h; y += 5)
  {
    gfx->drawFastHLine(0, y, w, RED);
  }
  for (x = 0; x < w; x += 5)
  {
    gfx->drawFastVLine(x, 0, h, BLUE);
  }

  return micros() - start;
}

int32_t testFilledRects()
{
  uint32_t start;
  int32_t i, i2;

  start = micros_start();

  for (i = n; i > 0; i -= 6)
  {
    i2 = i / 2;

    gfx->fillRect(cx - i2, cy - i2, i, i, gfx->color565(i, i, 0));
  }

  return micros() - start;
}

int32_t testRects()
{
  uint32_t start;
  int32_t i, i2;

  start = micros_start();
  for (i = 2; i < n; i += 6)
  {
    i2 = i / 2;
    gfx->drawRect(cx - i2, cy - i2, i, i, GREEN);
  }

  return micros() - start;
}

int32_t testFilledCircles(uint8_t radius)
{
  uint32_t start;
  int32_t x, y, r2 = radius * 2;

  start = micros_start();

  for (x = radius; x < w; x += r2)
  {
    for (y = radius; y < h; y += r2)
    {
      gfx->fillCircle(x, y, radius, MAGENTA);
    }
  }

  return micros() - start;
}

int32_t testCircles(uint8_t radius)
{
  uint32_t start;
  int32_t x, y, r2 = radius * 2;
  int32_t w1 = w + radius;
  int32_t h1 = h + radius;

  // Screen is not cleared for this one -- this is
  // intentional and does not affect the reported time.
  start = micros_start();

  for (x = 0; x < w1; x += r2)
  {
    for (y = 0; y < h1; y += r2)
    {
      gfx->drawCircle(x, y, radius, WHITE);
    }
  }

  return micros() - start;
}

int32_t testFillArcs()
{
  int16_t i, r = 360 / cn;
  uint32_t start = micros_start();

  for (i = 6; i < cn; i += 6)
  {
    gfx->fillArc(cx1, cy1, i, i - 3, 0, i * r, RED);
  }

  return micros() - start;
}

int32_t testArcs()
{
  int16_t i, r = 360 / cn;
  uint32_t start = micros_start();

  for (i = 6; i < cn; i += 6)
  {
    gfx->drawArc(cx1, cy1, i, i - 3, 0, i * r, WHITE);
  }

  return micros() - start;
}

int32_t testFilledTriangles()
{
  uint32_t start;
  int32_t i;

  start = micros_start();

  for (i = cn1; i > 10; i -= 5)
  {
    gfx->fillTriangle(cx1, cy1 - i, cx1 - i, cy1 + i, cx1 + i, cy1 + i,
                      gfx->color565(0, i, i));
  }

  return micros() - start;
}

int32_t testTriangles()
{
  uint32_t start;
  int32_t i;

  start = micros_start();

  for (i = 0; i < cn; i += 5)
  {
    gfx->drawTriangle(
        cx1, cy1 - i,     // peak
        cx1 - i, cy1 + i, // bottom left
        cx1 + i, cy1 + i, // bottom right
        gfx->color565(0, 0, i));
  }

  return micros() - start;
}

int32_t testFilledRoundRects()
{
  uint32_t start;
  int32_t i, i2;

  start = micros_start();

  for (i = n1; i > 20; i -= 6)
  {
    i2 = i / 2;
    gfx->fillRoundRect(cx - i2, cy - i2, i, i, i / 8, gfx->color565(0, i, 0));
  }

  return micros() - start;
}

int32_t testRoundRects()
{
  uint32_t start;
  int32_t i, i2;

  start = micros_start();

  for (i = 20; i < n1; i += 6)
  {
    i2 = i / 2;
    gfx->drawRoundRect(cx - i2, cy - i2, i, i, i / 8, gfx->color565(i, 0, 0));
  }

  return micros() - start;
}

/***************************************************
  Original sketch text:

  This is an example sketch for the Adafruit 2.2" SPI display.
  This library works with the Adafruit 2.2" TFT Breakout w/SD card
  ----> http://www.adafruit.com/products/1480

  Check out the links above for our tutorials and wiring diagrams
  These displays use SPI to communicate, 4 or 5 pins are required to
  interface (RST is optional)
  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/