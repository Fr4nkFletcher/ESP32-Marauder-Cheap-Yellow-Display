// Setup for the ESP32 S3 with ILI9341 display
// Note SPI DMA with ESP32 S3 is not currently supported
//#define USER_SETUP_ID 1000
#define USER_SETUP_INFO "User_Setup"
// See SetupX_Template.h for all options available
#define ST7789_DRIVER

#define TFT_WIDTH  240
#define TFT_HEIGHT 320
#define TFT_RGB_ORDER TFT_BGR 

//14 /* DC */, 15 /* CS */, 2 /* SCK */, 13 /* MOSI */, GFX_NOT_DEFINED /* MISO */

                    // Typical board default pins - change to match your board
#define TFT_CS   15 //     10 or 34 (FSPI CS0) 
#define TFT_MOSI 13 //     11 or 35 (FSPI D)
#define TFT_SCLK 14 //     12 or 36 (FSPI CLK)
#define TFT_MISO 12 //     13 or 37 (FSPI Q)

// Use pins in range 0-31
#define TFT_DC    2
#define TFT_RST   35

#define TFT_BL    27            // LED back-light control pin
#define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

//#define TOUCH_CS 33 // Optional for touch screen

#define LOAD_GLCD
#define LOAD_FONT2
#define LOAD_FONT4
#define LOAD_FONT6
#define LOAD_FONT7
#define LOAD_FONT8
#define LOAD_GFXFF

#define SMOOTH_FONT

// FSPI port (SPI2) used unless following defined. HSPI port (SPI3) NOT TESTED YET
#define USE_HSPI_PORT

//#define SPI_FREQUENCY  27000000
#define SPI_FREQUENCY  80000000   // Maximum for ILI9341

#define SPI_READ_FREQUENCY  10000000 // 6 MHz is the maximum SPI read speed for the ST7789V

#define SPI_TOUCH_FREQUENCY 2500000
