#pragma once

#ifndef configs_h

  #define configs_h

  #define POLISH_POTATO

  //// BOARD TARGETS
  //#define MARAUDER_M5STICKC
  //#define MARAUDER_MINI
  #define MARAUDER_V4
  //#define MARAUDER_V6
  //#define MARAUDER_V6_1
  //#define MARAUDER_V7
  //#define MARAUDER_KIT
  //#define GENERIC_ESP32
  //#define MARAUDER_FLIPPER
  //#define ESP32_LDDB
  //#define MARAUDER_DEV_BOARD_PRO
  //#define XIAO_ESP32_S3
  //#define MARAUDER_REV_FEATHER
  //// END BOARD TARGETS

  #define MARAUDER_VERSION "v1.4.3"

  #define GRAPH_REFRESH 100

  //// HARDWARE NAMES
  #ifdef MARAUDER_M5STICKC
    #define HARDWARE_NAME "M5Stick-C Plus"
  #elif defined(MARAUDER_MINI)
    #define HARDWARE_NAME "Marauder Mini"
  #elif defined(MARAUDER_V7)
    #define HARDWARE_NAME "Marauder v7"
  #elif defined(MARAUDER_REV_FEATHER)
    #define HARDWARE_NAME "Adafruit Feather ESP32-S2 Reverse TFT"
  #elif defined(MARAUDER_V4)
    #define HARDWARE_NAME "Marauder vCYD"
  #elif defined(MARAUDER_V6)
    #define HARDWARE_NAME "Marauder v6"
  #elif defined(MARAUDER_V6_1)
    #define HARDWARE_NAME "Marauder v6.1"
  #elif defined(MARAUDER_KIT)
    #define HARDWARE_NAME "Marauder Kit"
  #elif defined(MARAUDER_FLIPPER)
    #define HARDWARE_NAME "Flipper Zero Dev Board"
  #elif defined(ESP32_LDDB)
    #define HARDWARE_NAME "ESP32 LDDB"
  #elif defined(MARAUDER_DEV_BOARD_PRO)
    #define HARDWARE_NAME "Flipper Zero Dev Board Pro"
  #elif defined(XIAO_ESP32_S3)
    #define HARDWARE_NAME "XIAO ESP32 S3"
  #else
    #define HARDWARE_NAME "ESP32"
  #endif

  //// END HARDWARE NAMES

 //// BOARD FEATURES
  #ifdef MARAUDER_M5STICKC
    //#define FLIPPER_ZERO_HAT
    #define HAS_BATTERY
    #define HAS_BT
    #define HAS_BUTTONS
    //#define HAS_NEOPIXEL_LED
    #define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_MINI_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef MARAUDER_MINI
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    #define HAS_BT
    #define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_MINI_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef MARAUDER_V7
    //#define FLIPPER_ZERO_HAT
    #define HAS_BATTERY
    #define HAS_BT
    #define HAS_BT_REMOTE
    #define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_FULL_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef MARAUDER_REV_FEATHER
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    //#define HAS_BT
    #define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_MINI_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef MARAUDER_V4
    #define HAS_BATTERY
    #define HAS_BT
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_FULL_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_GPS
    //#define CYD_22CAP
    //#define CYD_24
    //#define CYD_24G
    //#define CYD_24CAP
    #define CYD_28
    //#define CYD_32
    //#define CYD_32CAP
    //#define CYD_35
    //#define CYD_35CAP
  #endif

  #if defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
    //#define FLIPPER_ZERO_HAT
    #define HAS_BATTERY
    #define HAS_BT
    #define HAS_BT_REMOTE
    #define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_FULL_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef MARAUDER_KIT
    //#define FLIPPER_ZERO_HAT
    #define HAS_BATTERY
    #define HAS_BT
    //#define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    #define HAS_SCREEN
    #define HAS_FULL_SCREEN
    #define HAS_SD
    #define USE_SD
    #define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef GENERIC_ESP32
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    #define HAS_BT
    //#define HAS_BUTTONS
    //#define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    //#define HAS_SCREEN
    //#define HAS_SD
    //#define HAS_TEMP_SENSOR
    //#define HAS_GPS
  #endif

  #ifdef MARAUDER_FLIPPER
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    //#define HAS_BT
    //#define HAS_BUTTONS
    //#define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    //#define HAS_SCREEN
    #define HAS_GPS
    #define HAS_SD
    #define USE_SD
    //#define HAS_TEMP_SENSOR
  #endif

  #ifdef ESP32_LDDB
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    #define HAS_BT
    //#define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    //#define HAS_SCREEN
    #define HAS_SD
    #define USE_SD
    //#define HAS_TEMP_SENSOR
    //#define HAS_GPS
  #endif

  #ifdef MARAUDER_DEV_BOARD_PRO
    //#define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    #define HAS_BT
    //#define HAS_BUTTONS
    #define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    //#define HAS_SCREEN
    #define HAS_SD
    #define USE_SD
    //#define HAS_TEMP_SENSOR
    #define HAS_GPS
  #endif

  #ifdef XIAO_ESP32_S3
    #define FLIPPER_ZERO_HAT
    //#define HAS_BATTERY
    #define HAS_BT
    //#define HAS_BUTTONS
    //#define HAS_NEOPIXEL_LED
    //#define HAS_PWR_MGMT
    //#define HAS_SCREEN
    //#define HAS_SD
    //#define HAS_TEMP_SENSOR
    //#define HAS_GPS
  #endif
  //// END BOARD FEATURES

  //// POWER MANAGEMENT
  #ifdef HAS_PWR_MGMT
    #ifdef MARAUDER_M5STICKC
      #include "AXP192.h"
    #endif
  #endif
  //// END POWER MANAGEMENT

  //// BUTTON DEFINITIONS
  #ifdef HAS_BUTTONS

    #ifdef MARAUDER_REV_FEATHER
      #define L_BTN -1
      #define C_BTN 1
      #define U_BTN 0
      #define R_BTN -1
      #define D_BTN 2

      //#define HAS_L
      //#define HAS_R
      #define HAS_U
      #define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL false
      #define U_PULL true
      #define R_PULL true
      #define D_PULL false
    #endif

    #ifdef MARAUDER_MINI
      #define L_BTN 13
      #define C_BTN 34
      #define U_BTN 36
      #define R_BTN 39
      #define D_BTN 35

      #define HAS_L
      #define HAS_R
      #define HAS_U
      #define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL true
      #define U_PULL true
      #define R_PULL true
      #define D_PULL true
    #endif

    #ifdef MARAUDER_V7
      #define L_BTN 13
      #define C_BTN 34
      #define U_BTN 36
      #define R_BTN 39
      #define D_BTN 35

      #define HAS_L
      #define HAS_R
      #define HAS_U
      #define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL true
      #define U_PULL true
      #define R_PULL true
      #define D_PULL true
    #endif

    #ifdef MARAUDER_M5STICKC
      #define L_BTN -1
      #define C_BTN 37
      #define U_BTN -1
      #define R_BTN -1
      #define D_BTN 39

      //#define HAS_L
      //#define HAS_R
      //#define HAS_U
      #define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL true
      #define U_PULL true
      #define R_PULL true
      #define D_PULL true
    #endif

    #ifdef MARAUDER_V6
      #define L_BTN -1
      #define C_BTN 0
      #define U_BTN -1
      #define R_BTN -1
      #define D_BTN -1

      //#define HAS_L
      //#define HAS_R
      //#define HAS_U
      //#define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL true
      #define U_PULL true
      #define R_PULL true
      #define D_PULL true
    #endif

    #ifdef MARAUDER_V6_1
      #define L_BTN -1
      #define C_BTN 0
      #define U_BTN -1
      #define R_BTN -1
      #define D_BTN -1

      //#define HAS_L
      //#define HAS_R
      //#define HAS_U
      //#define HAS_D
      #define HAS_C

      #define L_PULL true
      #define C_PULL true
      #define U_PULL true
      #define R_PULL true
      #define D_PULL true
    #endif  

  #endif
  //// END BUTTON DEFINITIONS

  //// DISPLAY DEFINITIONS
  #ifdef HAS_SCREEN

    #ifdef MARAUDER_M5STICKC
      #define SCREEN_CHAR_WIDTH 40
      //#define TFT_MISO 19
      #define TFT_MOSI 15
      #define TFT_SCLK 13
      #define TFT_CS 5
      #define TFT_DC 23
      #define TFT_RST 18
      #define TFT_BL -1
      #define TOUCH_CS -1
      //#define SD_CS 1

      #define SCREEN_BUFFER

      #define MAX_SCREEN_BUFFER 9

      #define BANNER_TEXT_SIZE 1

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 135
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 240
      #endif

      #define CHAR_WIDTH 6
      #define SCREEN_WIDTH TFT_HEIGHT // Originally 240
      #define SCREEN_HEIGHT TFT_WIDTH // Originally 320
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_WIDTH
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT (TFT_HEIGHT/10) // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX TFT_HEIGHT // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 6
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH (TFT_HEIGHT/16)
      #define LVGL_TICK_PERIOD 6
    
      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50
    
      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H
    
      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49

    #endif

    #ifdef MARAUDER_V4
      // 2.2 capacitive
      #ifdef CYD_22CAP
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ST7789
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif

        #define TOUCH_SDA  21
        #define TOUCH_SCL  22
        #define TOUCH_INT -1
        #define TOUCH_RST -1
        //#define TOUCH_CS -1

        //#define TFT_MISO 12 // or SDO
        //#define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" or SDI
        //#define TFT_SCLK 14
        //#define TFT_CS   15  // Chip select control pin
        //#define TFT_DC   2   // Data Command control pin (OR RS)
        //#define TFT_RST  -1  // Reset pin (could connect to Arduino RESET pin)
        #define TFT_BL   27  // LED back-light
        #define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

        #define TOUCH_WIDTH  240
        #define TOUCH_HEIGHT 320

        // Generic commands used by TFT_eSPI.cpp
        #define TFT_NOP     0x00
        #define TFT_SWRST   0x01
        #define TFT_INVOFF  0x20
        #define TFT_INVON   0x21
        #define TFT_DISPOFF 0x28
        #define TFT_DISPON  0x29
        #define TFT_CASET   0x2A
        #define TFT_PASET   0x2B
        #define TFT_RAMWR   0x2C
        #define TFT_RAMRD   0x2E
        #define TFT_MADCTL  0x36
        #define TFT_MAD_MY  0x80
        #define TFT_MAD_MX  0x40
        #define TFT_MAD_MV  0x20
        #define TFT_MAD_ML  0x10
        #define TFT_MAD_BGR 0x08
        #define TFT_MAD_MH  0x10
        #define TFT_MAD_RGB 0x00

        #ifdef TFT_RGB_ORDER
          #if (TFT_RGB_ORDER == 1)
            #define TFT_MAD_COLOR_ORDER TFT_MAD_RGB
          #else
            #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
          #endif
        #else
          #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
        #endif

        
        #define ST7789_NOP     0x00
        #define ST7789_SWRESET 0x01
        #define ST7789_RDDID   0x04
        #define ST7789_RDDST   0x09
        #define ST7789_SLPIN   0x10
        #define ST7789_SLPOUT  0x11
        #define ST7789_PTLON   0x12
        #define ST7789_NORON   0x13
        #define ST7789_INVOFF  0x20
        #define ST7789_INVON   0x21
        #define ST7789_DISPOFF 0x28
        #define ST7789_DISPON  0x29
        #define ST7789_CASET   0x2A
        #define ST7789_PASET   0x2B
        #define ST7789_RAMWR   0x2C
        #define ST7789_RAMRD   0x2E
        #define ST7789_PTLAR   0x30
        #define ST7789_MADCTL  0x36
        #define ST7789_COLMOD  0x3A
        #define ST7789_FRMCTR1 0xB1
        #define ST7789_FRMCTR2 0xB2
        #define ST7789_FRMCTR3 0xB3
        #define ST7789_INVCTR  0xB4
        #define ST7789_DISSET5 0xB6
        #define ST7789_PWCTR1  0xC0
        #define ST7789_PWCTR2  0xC1
        #define ST7789_PWCTR3  0xC2
        #define ST7789_VMCTR1  0xC5
        #define ST7789_RDID1   0xDA
        #define ST7789_RDID2   0xDB
        #define ST7789_RDID3   0xDC
        #define ST7789_RDID4   0xDD
        #define ST7789_GMCTRP1 0xE0
        #define ST7789_GMCTRN1 0xE1
      #endif
      
      #ifdef CYD_24
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ILI9341
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif
      #endif // CYD_24

      #ifdef CYD_24G
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ILI9341
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif
      #endif // CYD_24

      #ifdef CYD_24CAP
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ILI9341
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
            #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
            #define TFT_HEIGHT 320
        #endif

        //#define THROW_AWAY_TOUCH_COUNT 31
        // Touchscreen pins (already in your config, kept as-is)
        #define TOUCH_SDA  33
        #define TOUCH_SCL  32
        #define TOUCH_INT 21
        #define TOUCH_RST 25

        // Additional capacitive touch definitions
        #define TFT_MISO 12 // or SDO
        #define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" or SDI
        #define TFT_SCLK 14
        #define TFT_CS   15  // Chip select control pin
        #define TFT_DC   2   // Data Command control pin (OR RS)
        #define TFT_RST  -1  // Reset pin (could connect to Arduino RESET pin)
        #define TFT_BL   27  // LED back-light
        #define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

        #define TOUCH_WIDTH  240
        #define TOUCH_HEIGHT 320

        //#define SPI_FREQUENCY  55000000
        //#define SPI_READ_FREQUENCY  20000000
        //#define SPI_TOUCH_FREQUENCY  2500000

        // Generic commands used by TFT_eSPI.cpp
        #define TFT_NOP     0x00
        #define TFT_SWRST   0x01
        #define TFT_INVOFF  0x20
        #define TFT_INVON   0x21
        #define TFT_DISPOFF 0x28
        #define TFT_DISPON  0x29
        #define TFT_CASET   0x2A
        #define TFT_PASET   0x2B
        #define TFT_RAMWR   0x2C
        #define TFT_RAMRD   0x2E
        #define TFT_MADCTL  0x36
        #define TFT_MAD_MY  0x80
        #define TFT_MAD_MX  0x40
        #define TFT_MAD_MV  0x20
        #define TFT_MAD_ML  0x10
        #define TFT_MAD_BGR 0x08
        #define TFT_MAD_MH  0x10
        #define TFT_MAD_RGB 0x00

        #ifdef TFT_RGB_ORDER
            #if (TFT_RGB_ORDER == 1)
                #define TFT_MAD_COLOR_ORDER TFT_MAD_RGB
            #else
                #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
            #endif
        #else
            #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
        #endif

        // ILI9341 specific commands (replacing ST7789 commands)
        #define ILI9341_NOP     0x00
        #define ILI9341_SWRESET 0x01
        #define ILI9341_RDDID   0x04
        #define ILI9341_RDDST   0x09
        #define ILI9341_SLPIN   0x10
        #define ILI9341_SLPOUT  0x11
        #define ILI9341_PTLON   0x12
        #define ILI9341_NORON   0x13
        #define ILI9341_INVOFF  0x20
        #define ILI9341_INVON   0x21
        #define ILI9341_DISPOFF 0x28
        #define ILI9341_DISPON  0x29
        #define ILI9341_CASET   0x2A
        #define ILI9341_PASET   0x2B
        #define ILI9341_RAMWR   0x2C
        #define ILI9341_RAMRD   0x2E
        #define ILI9341_PTLAR   0x30
        #define ILI9341_MADCTL  0x36
        #define ILI9341_COLMOD  0x3A
        #define ILI9341_FRMCTR1 0xB1
        #define ILI9341_FRMCTR2 0xB2
        #define ILI9341_FRMCTR3 0xB3
        #define ILI9341_INVCTR  0xB4
        #define ILI9341_DISSET5 0xB6
        #define ILI9341_PWCTR1  0xC0
        #define ILI9341_PWCTR2  0xC1
        #define ILI9341_PWCTR3  0xC2
        #define ILI9341_VMCTR1  0xC5
        #define ILI9341_RDID1   0xDA
        #define ILI9341_RDID2   0xDB
        #define ILI9341_RDID3   0xDC
        #define ILI9341_RDID4   0xDD
        #define ILI9341_GMCTRP1 0xE0
        #define ILI9341_GMCTRN1 0xE1
      #endif // CYD_24CAP

      #ifdef CYD_28
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ILI9341
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif
      #endif // CYD_28

      #ifdef CYD_32
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ST7789
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif
      #endif // CYD_32

      #ifdef CYD_32CAP
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ST7789
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 240
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 320
        #endif

        #define THROW_AWAY_TOUCH_COUNT 31
        // Touchscreen pins (already in your config, kept as-is)
        #define TOUCH_SDA  33
        #define TOUCH_SCL  32
        #define TOUCH_INT 21
        #define TOUCH_RST 25

        // Additional capacitive touch definitions
        #define TFT_MISO 12 // or SDO
        #define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" or SDI
        #define TFT_SCLK 14
        #define TFT_CS   15  // Chip select control pin
        #define TFT_DC   2   // Data Command control pin (OR RS)
        #define TFT_RST  -1  // Reset pin (could connect to Arduino RESET pin)
        #define TFT_BL   27  // LED back-light
        #define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

        #define TOUCH_WIDTH  240
        #define TOUCH_HEIGHT 320

        #define SPI_FREQUENCY  65000000
        #define SPI_READ_FREQUENCY  20000000
        #define SPI_TOUCH_FREQUENCY  2500000

        // Generic commands used by TFT_eSPI.cpp
        #define TFT_NOP     0x00
        #define TFT_SWRST   0x01
        #define TFT_INVOFF  0x20
        #define TFT_INVON   0x21
        #define TFT_DISPOFF 0x28
        #define TFT_DISPON  0x29
        #define TFT_CASET   0x2A
        #define TFT_PASET   0x2B
        #define TFT_RAMWR   0x2C
        #define TFT_RAMRD   0x2E
        #define TFT_MADCTL  0x36
        #define TFT_MAD_MY  0x80
        #define TFT_MAD_MX  0x40
        #define TFT_MAD_MV  0x20
        #define TFT_MAD_ML  0x10
        #define TFT_MAD_BGR 0x08
        #define TFT_MAD_MH  0x10
        #define TFT_MAD_RGB 0x00

        #ifdef TFT_RGB_ORDER
          #if (TFT_RGB_ORDER == 1)
            #define TFT_MAD_COLOR_ORDER TFT_MAD_RGB
          #else
            #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
          #endif
        #else
          #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
        #endif

        
        #define ST7789_NOP     0x00
        #define ST7789_SWRESET 0x01
        #define ST7789_RDDID   0x04
        #define ST7789_RDDST   0x09
        #define ST7789_SLPIN   0x10
        #define ST7789_SLPOUT  0x11
        #define ST7789_PTLON   0x12
        #define ST7789_NORON   0x13
        #define ST7789_INVOFF  0x20
        #define ST7789_INVON   0x21
        #define ST7789_DISPOFF 0x28
        #define ST7789_DISPON  0x29
        #define ST7789_CASET   0x2A
        #define ST7789_PASET   0x2B
        #define ST7789_RAMWR   0x2C
        #define ST7789_RAMRD   0x2E
        #define ST7789_PTLAR   0x30
        #define ST7789_MADCTL  0x36
        #define ST7789_COLMOD  0x3A
        #define ST7789_FRMCTR1 0xB1
        #define ST7789_FRMCTR2 0xB2
        #define ST7789_FRMCTR3 0xB3
        #define ST7789_INVCTR  0xB4
        #define ST7789_DISSET5 0xB6
        #define ST7789_PWCTR1  0xC0
        #define ST7789_PWCTR2  0xC1
        #define ST7789_PWCTR3  0xC2
        #define ST7789_VMCTR1  0xC5
        #define ST7789_RDID1   0xDA
        #define ST7789_RDID2   0xDB
        #define ST7789_RDID3   0xDC
        #define ST7789_RDID4   0xDD
        #define ST7789_GMCTRP1 0xE0
        #define ST7789_GMCTRN1 0xE1
      #endif

      #ifdef CYD_35
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ST7796
        #define BANNER_TEXT_SIZE 2

        #ifndef TFT_WIDTH
          #define TFT_WIDTH 320
        #endif

        #ifndef TFT_HEIGHT
          #define TFT_HEIGHT 480
        #endif
      #endif // CYD_35

      #ifdef CYD_35CAP
        #define SCREEN_CHAR_WIDTH 40
        #define HAS_ST7796
        #define BANNER_TEXT_SIZE 2

        #define TFT_BL   27            // LED back-light control pin
        #define TFT_BACKLIGHT_ON HIGH  // Level to turn ON back-light (HIGH or LOW)

        #define TFT_MISO 12 // or SDO
        #define TFT_MOSI 13 // In some display driver board, it might be written as "SDA" or SDI
        #define TFT_SCLK 14
        #define TFT_CS   15  // Chip select control pin
        #define TFT_DC   2  // Data Command control pin (OR RS)
        #define TFT_RST  -1  // Reset pin 
        #define TFT_BL   27  // LED back-light
        #define THROW_AWAY_TOUCH_COUNT 31
        
        #define SPI_FREQUENCY  65000000
        #define SPI_READ_FREQUENCY  20000000
        #define SPI_TOUCH_FREQUENCY  2500000  //2500000

        // #define TOUCH_CS 33     // Chip select pin (T_CS) of touch screen
        
        #define TOUCH_SDA  33
        #define TOUCH_SCL  32
        #define TOUCH_INT 21
        #define TOUCH_RST 25

        #define TOUCH_WIDTH  320
        #define TOUCH_HEIGHT 480

        // Change the width and height if required (defined in portrait mode)
        // or use the constructor to over-ride defaults
    
        #define TFT_WIDTH  320
        #define TFT_HEIGHT 480

        #define TFT_NOP     0x00
        #define TFT_SWRST   0x01

        #define TFT_INVOFF  0x20
        #define TFT_INVON   0x21

        #define TFT_DISPOFF 0x28
        #define TFT_DISPON  0x29

        #define TFT_CASET   0x2A
        #define TFT_PASET   0x2B
        #define TFT_RAMWR   0x2C
        #define TFT_RAMRD   0x2E

        #define TFT_MADCTL  0x36
        #define TFT_MAD_MY  0x80
        #define TFT_MAD_MX  0x40
        #define TFT_MAD_MV  0x20
        #define TFT_MAD_ML  0x10
        #define TFT_MAD_BGR 0x08
        #define TFT_MAD_MH  0x10
        #define TFT_MAD_RGB 0x00

        #ifdef TFT_RGB_ORDER
          #if (TFT_RGB_ORDER == 1)
            #define TFT_MAD_COLOR_ORDER TFT_MAD_RGB
          #else
            #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
          #endif
        #else
          #define TFT_MAD_COLOR_ORDER TFT_MAD_BGR
        #endif

        // ST7796 specific commands
        #define ST7796_NOP     0x00
        #define ST7796_SWRESET 0x01
        #define ST7796_RDDID   0x04
        #define ST7796_RDDST   0x09

        #define ST7796_SLPIN   0x10
        #define ST7796_SLPOUT  0x11
        #define ST7796_PTLON   0x12
        #define ST7796_NORON   0x13

        #define ST7796_RDMODE  0x0A
        #define ST7796_RDMADCTL  0x0B
        #define ST7796_RDPIXFMT  0x0C
        #define ST7796_RDIMGFMT  0x0A
        #define ST7796_RDSELFDIAG  0x0F

        #define ST7796_INVOFF  0x20
        #define ST7796_INVON   0x21

        #define ST7796_DISPOFF 0x28
        #define ST7796_DISPON  0x29

        #define ST7796_CASET   0x2A
        #define ST7796_PASET   0x2B
        #define ST7796_RAMWR   0x2C
        #define ST7796_RAMRD   0x2E

        #define ST7796_PTLAR   0x30
        #define ST7796_VSCRDEF 0x33
        #define ST7796_MADCTL  0x36
        #define ST7796_VSCRSADD 0x37
        #define ST7796_PIXFMT  0x3A

        #define ST7796_WRDISBV  0x51
        #define ST7796_RDDISBV  0x52
        #define ST7796_WRCTRLD  0x53

        #define ST7796_FRMCTR1 0xB1
        #define ST7796_FRMCTR2 0xB2
        #define ST7796_FRMCTR3 0xB3
        #define ST7796_INVCTR  0xB4
        #define ST7796_DFUNCTR 0xB6

        #define ST7796_PWCTR1  0xC0
        #define ST7796_PWCTR2  0xC1
        #define ST7796_PWCTR3  0xC2

        #define ST7796_VMCTR1  0xC5
        #define ST7796_VMCOFF  0xC6

        #define ST7796_RDID4   0xD3

        #define ST7796_GMCTRP1 0xE0
        #define ST7796_GMCTRN1 0xE1

        #define ST7796_MADCTL_MY  0x80
        #define ST7796_MADCTL_MX  0x40
        #define ST7796_MADCTL_MV  0x20
        #define ST7796_MADCTL_ML  0x10
        #define ST7796_MADCTL_RGB 0x00
        #define ST7796_MADCTL_BGR 0x08
        #define ST7796_MADCTL_MH  0x04
      #endif // CYD_35CAP

      #define SCREEN_BUFFER

      #if defined(CYD_35CAP) || defined(CYD_35)
        #define MAX_SCREEN_BUFFER 33
      #else
        #define MAX_SCREEN_BUFFER 22
      #endif

      #define GRAPH_VERT_LIM TFT_HEIGHT/2

      #define EXT_BUTTON_WIDTH 20

    //#if defined(CYD_35) || defined(CYD_35CAP)
      //#define TEXT_HEIGHT 24 // Height of text to be printed and scrolled
    //#else
      #define TEXT_HEIGHT 16 // Height of text to be printed and scrolled
    //#endif
      #define CHAR_WIDTH 12
      #define SCREEN_WIDTH TFT_WIDTH
      #define SCREEN_HEIGHT TFT_HEIGHT
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_HEIGHT
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH / 6) // number of characters on a single line with normal font
      
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX TFT_HEIGHT // Dynamically set based on the display height
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 12
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH 16
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50

      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W / 2)
      #define REDBUTTON_H FRAME_H

      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W / 2)
      #define GREENBUTTON_H FRAME_H

      #define STATUSBAR_COLOR 0x18ED

      #define KIT_LED_BUILTIN 4
    #endif
    

    #if defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
      #define SCREEN_CHAR_WIDTH 40
      #define HAS_ILI9341
    
      #define BANNER_TEXT_SIZE 2

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 240
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 320
      #endif

      #define TFT_DIY
    
      #define GRAPH_VERT_LIM TFT_HEIGHT/2

      #define EXT_BUTTON_WIDTH 20

      #define SCREEN_BUFFER

      #define MAX_SCREEN_BUFFER 22

      #define CHAR_WIDTH 12
      #define SCREEN_WIDTH TFT_WIDTH
      #define SCREEN_HEIGHT TFT_HEIGHT
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_HEIGHT
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT 16 // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX 320 // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 12
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH 16
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50
    
      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H
    
      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49
    
      #define KIT_LED_BUILTIN 13
    #endif 

    #ifdef MARAUDER_V7
      #define SCREEN_CHAR_WIDTH 40
      //#define HAS_ILI9341
    
      #define BANNER_TEXT_SIZE 2

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 240
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 320
      #endif

      #define TFT_DIY

      #define SCREEN_BUFFER

      #define MAX_SCREEN_BUFFER 22

      #define EXT_BUTTON_WIDTH 0


      #define CHAR_WIDTH 12
      #define SCREEN_WIDTH TFT_WIDTH
      #define SCREEN_HEIGHT TFT_HEIGHT
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_HEIGHT
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT 16 // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX 320 // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 12
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH 16
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50
    
      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H
    
      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49
    
      #define KIT_LED_BUILTIN 13
    #endif

    #ifdef MARAUDER_KIT
      #define SCREEN_CHAR_WIDTH 40
      #define HAS_ILI9341
    
      #define BANNER_TEXT_SIZE 2

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 240
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 320
      #endif

      #define TFT_DIY
      #define KIT


      #define CHAR_WIDTH 12
      #define SCREEN_WIDTH TFT_WIDTH
      #define SCREEN_HEIGHT TFT_HEIGHT
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_HEIGHT
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT 16 // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX 320 // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 12
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH 16
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50

      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H

      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49
    
      #define KIT_LED_BUILTIN 13
    #endif
  
    #ifdef MARAUDER_MINI
      #define SCREEN_CHAR_WIDTH 40
      #define TFT_MISO 19
      #define TFT_MOSI 23
      #define TFT_SCLK 18
      #define TFT_CS 27
      #define TFT_DC 26
      #define TFT_RST 5
      #define TFT_BL 32
      #define TOUCH_CS 21
      #define SD_CS 4

      #define SCREEN_BUFFER

      #define MAX_SCREEN_BUFFER 9

      #define BANNER_TEXT_SIZE 1

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 128
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 128
      #endif

      #define CHAR_WIDTH 6
      #define SCREEN_WIDTH TFT_WIDTH // Originally 240
      #define SCREEN_HEIGHT TFT_HEIGHT // Originally 320
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_WIDTH
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT (TFT_HEIGHT/10) // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX TFT_HEIGHT // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 10
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH (TFT_HEIGHT/16)
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50

      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H

      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49
    #endif

    #ifdef MARAUDER_REV_FEATHER
      #define SCREEN_CHAR_WIDTH 40
      //#define TFT_MISO 37
      //#define TFT_MOSI 35
      //#define TFT_SCLK 36
      #define TFT_CS 42
      #define TFT_DC 40
      #define TFT_RST 41
      #define TFT_BL 45
      //#define TOUCH_CS 21
      #define SD_CS 4

      #define SCREEN_BUFFER

      #define MAX_SCREEN_BUFFER 9

      #define BANNER_TEXT_SIZE 1

      #ifndef TFT_WIDTH
        #define TFT_WIDTH 240
      #endif

      #ifndef TFT_HEIGHT
        #define TFT_HEIGHT 135
      #endif

      #define CHAR_WIDTH 6
      #define SCREEN_WIDTH TFT_WIDTH // Originally 240
      #define SCREEN_HEIGHT TFT_HEIGHT // Originally 320
      #define HEIGHT_1 TFT_WIDTH
      #define WIDTH_1 TFT_WIDTH
      #define STANDARD_FONT_CHAR_LIMIT (TFT_WIDTH/6) // number of characters on a single line with normal font
      #define TEXT_HEIGHT (TFT_HEIGHT/10) // Height of text to be printed and scrolled
      #define BOT_FIXED_AREA 0 // Number of lines in bottom fixed area (lines counted from bottom of screen)
      #define TOP_FIXED_AREA 48 // Number of lines in top fixed area (lines counted from top of screen)
      #define YMAX TFT_HEIGHT // Bottom of screen area
      #define minimum(a,b)     (((a) < (b)) ? (a) : (b))
      //#define MENU_FONT NULL
      #define MENU_FONT &FreeMono9pt7b // Winner
      //#define MENU_FONT &FreeMonoBold9pt7b
      //#define MENU_FONT &FreeSans9pt7b
      //#define MENU_FONT &FreeSansBold9pt7b
      #define BUTTON_SCREEN_LIMIT 5
      #define BUTTON_ARRAY_LEN BUTTON_SCREEN_LIMIT
      #define STATUS_BAR_WIDTH (TFT_HEIGHT/16)
      #define LVGL_TICK_PERIOD 6

      #define FRAME_X 100
      #define FRAME_Y 64
      #define FRAME_W 120
      #define FRAME_H 50

      // Red zone size
      #define REDBUTTON_X FRAME_X
      #define REDBUTTON_Y FRAME_Y
      #define REDBUTTON_W (FRAME_W/2)
      #define REDBUTTON_H FRAME_H

      // Green zone size
      #define GREENBUTTON_X (REDBUTTON_X + REDBUTTON_W)
      #define GREENBUTTON_Y FRAME_Y
      #define GREENBUTTON_W (FRAME_W/2)
      #define GREENBUTTON_H FRAME_H
    
      #define STATUSBAR_COLOR 0x4A49
    #endif

  #endif
  //// END DISPLAY DEFINITIONS

  //// MENU DEFINITIONS
  #if defined(MARAUDER_V4)
    #define BANNER_TIME 100
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X 120 // Centre of key
    #define KEY_Y 50
    #define KEY_W 240 // Width and height
    #define KEY_H 22
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 22
    //#define BUTTON_ARRAY_LEN 5
  #endif

  #if defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
    #define BANNER_TIME 100
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X 120 // Centre of key
    #define KEY_Y 50
    #define KEY_W 240 // Width and height
    #define KEY_H 22
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 22
    //#define BUTTON_ARRAY_LEN 5
  #endif

  #ifdef MARAUDER_V7
    #define BANNER_TIME 100
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X 120 // Centre of key
    #define KEY_Y 50
    #define KEY_W 240 // Width and height
    #define KEY_H 22
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 22
    //#define BUTTON_ARRAY_LEN 5
  #endif

  #ifdef MARAUDER_KIT
    #define BANNER_TIME 100
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X 120 // Centre of key
    #define KEY_Y 50
    #define KEY_W 240 // Width and height
    #define KEY_H 22
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 22
    //#define BUTTON_ARRAY_LEN 5
  #endif
  
  #ifdef MARAUDER_MINI
    #define BANNER_TIME 50
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X (TFT_WIDTH/2) // Centre of key
    #define KEY_Y (TFT_HEIGHT/4.5)
    #define KEY_W TFT_WIDTH // Width and height
    #define KEY_H (TFT_HEIGHT/12.8)
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 10
  #endif

  #ifdef MARAUDER_REV_FEATHER
    #define BANNER_TIME 50
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X (TFT_WIDTH/2) // Centre of key
    #define KEY_Y (TFT_HEIGHT/4.5)
    #define KEY_W TFT_WIDTH // Width and height
    #define KEY_H (TFT_HEIGHT/12.8)
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 10
  #endif

  #ifdef MARAUDER_M5STICKC
    #define BANNER_TIME 50
    
    #define COMMAND_PREFIX "!"
    
    // Keypad start position, key sizes and spacing
    #define KEY_X (TFT_WIDTH/2) // Centre of key
    #define KEY_Y (TFT_HEIGHT/5)
    #define KEY_W TFT_HEIGHT // Width and height
    #define KEY_H (TFT_HEIGHT/17)
    #define KEY_SPACING_X 0 // X and Y gap
    #define KEY_SPACING_Y 1
    #define KEY_TEXTSIZE 1   // Font size multiplier
    #define ICON_W 22
    #define ICON_H 22
    #define BUTTON_PADDING 60
  #endif
  //// END MENU DEFINITIONS

  //// SD DEFINITIONS
  #if defined(USE_SD)

    #ifdef MARAUDER_V4
      #define SD_CS 5
    #endif

    #ifdef MARAUDER_V6
      #define SD_CS 12
    #endif

    #ifdef MARAUDER_V6_1
      #define SD_CS 14
    #endif

    #ifdef MARAUDER_KIT
      #define SD_CS 12
    #endif

    #ifdef MARAUDER_MINI
      #define SD_CS 4
    #endif

    #ifdef MARAUDER_V7
      #define SD_CS 4
    #endif

    #ifdef MARAUDER_REV_FEATHER
      #define SD_CS 5
    #endif

    #ifdef MARAUDER_M5STICKC
      #define SD_CS -1
    #endif

    #ifdef MARAUDER_FLIPPER
      #define SD_CS 10
    #endif

    #ifdef ESP32_LDDB
      #define SD_CS 4
    #endif

    #ifdef MARAUDER_DEV_BOARD_PRO
      #define SD_CS 4
    #endif

    #ifdef XIAO_ESP32_S3
      #define SD_CS 3
    #endif

  #endif
  //// END SD DEFINITIONS

  //// SPACE SAVING COLORS
  #define TFTWHITE     1
  #define TFTCYAN      2
  #define TFTBLUE      3
  #define TFTRED       4
  #define TFTGREEN     5
  #define TFTGREY      6
  #define TFTGRAY      7
  #define TFTMAGENTA   8
  #define TFTVIOLET    9
  #define TFTORANGE    10
  #define TFTYELLOW    11
  #define TFTLIGHTGREY 12
  #define TFTPURPLE    13
  #define TFTNAVY      14
  #define TFTSILVER    15
  #define TFTDARKGREY  16
  #define TFTSKYBLUE   17
  #define TFTLIME      18
  //// END SPACE SAVING COLORS
  
  //// SCREEN STUFF
  #ifndef HAS_SCREEN

    #define BANNER_TIME GRAPH_REFRESH
    
    #define TFT_WHITE 0
    #define TFT_CYAN 0
    #define TFT_BLUE 0
    #define TFT_RED 0
    #define TFT_GREEN 0
    #define TFT_GREY 0
    #define TFT_GRAY 0
    #define TFT_MAGENTA 0
    #define TFT_VIOLET 0
    #define TFT_ORANGE 0
    #define TFT_YELLOW 0
    #define STANDARD_FONT_CHAR_LIMIT 40
    #define FLASH_BUTTON -1

    #include <FS.h>
    #include <functional>
    #include <LinkedList.h>
    #include "SPIFFS.h"
    #include "Assets.h"

  #endif
  //// END SCREEN STUFF

  //// MEMORY LOWER LIMIT STUFF
  // These values are in bytes
  #ifdef MARAUDER_M5STICKC
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_MINI)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_V7)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_REV_FEATHER)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_V4)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_KIT)
    #define MEM_LOWER_LIM 10000
  #elif defined(GENERIC_ESP32)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_FLIPPER)
    #define MEM_LOWER_LIM 10000
  #elif defined(ESP32_LDDB)
    #define MEM_LOWER_LIM 10000
  #elif defined(MARAUDER_DEV_BOARD_PRO)
    #define MEM_LOWER_LIM 10000
  #elif defined(XIAO_ESP32_S3)
    #define MEM_LOWER_LIM 10000
  #endif
  //// END MEMORY LOWER LIMIT STUFF

  //// NEOPIXEL STUFF  
  #ifdef HAS_NEOPIXEL_LED
    
    #if defined(ESP32_LDDB)
      #define PIN 17
    #elif defined(MARAUDER_DEV_BOARD_PRO)
      #define PIN 16
    #elif defined(MARAUDER_REV_FEATHER)
      #define PIN 33
    #else
      #define PIN 4
    #endif
  
  #endif
  //// END NEOPIXEL STUFF

  //// EVIL PORTAL STUFF
  #ifdef MARAUDER_M5STICKC
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_MINI)
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_V7)
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_REV_FEATHER)
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_V4)
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
    #define MAX_HTML_SIZE 11400
  #elif defined(MARAUDER_KIT)
    #define MAX_HTML_SIZE 11400
  #elif defined(GENERIC_ESP32)
    #define MAX_HTML_SIZE 20000
  #elif defined(MARAUDER_FLIPPER)
    #define MAX_HTML_SIZE 20000
  #elif defined(ESP32_LDDB)
    #define MAX_HTML_SIZE 20000
  #elif defined(MARAUDER_DEV_BOARD_PRO)
    #define MAX_HTML_SIZE 20000
  #elif defined(XIAO_ESP32_S3)
    #define MAX_HTML_SIZE 20000
  #else
    #define MAX_HTML_SIZE 20000
  #endif
  //// END EVIL PORTAL STUFF

  //// GPS STUFF
  #ifdef HAS_GPS
    #if defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 4
      #define GPS_RX 13
      #define mac_history_len 100
    #elif defined(MARAUDER_V4)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 1
      #define GPS_RX 3
      #define mac_history_len 100
    #elif defined(MARAUDER_KIT)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 4
      #define GPS_RX 13
      #define mac_history_len 100
    #elif defined(MARAUDER_DEV_BOARD_PRO)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 21
      #define GPS_RX 17
      #define mac_history_len 100
    #elif defined(MARAUDER_MINI)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 21
      #define GPS_RX 22
      #define mac_history_len 100
    #elif defined(MARAUDER_V7)
      #define GPS_SERIAL_INDEX 2
      #define GPS_TX 21
      #define GPS_RX 22
      #define mac_history_len 100
    #elif defined(MARAUDER_FLIPPER)
      #define GPS_SERIAL_INDEX 1
      #define GPS_TX 9
      #define GPS_RX 21
      #define mac_history_len 100
    #elif defined(MARAUDER_M5STICKC)
      #define GPS_SERIAL_INDEX 1
      #define GPS_TX 33
      #define GPS_RX 32
      #define mac_history_len 100
    #elif defined(MARAUDER_REV_FEATHER)
      #define GPS_SERIAL_INDEX 1
      #define GPS_TX 6
      #define GPS_RX 9
      #define mac_history_len 100
    #endif
  #else
    #define mac_history_len 100
  #endif
  //// END GPS STUFF

  //// BATTERY STUFF
  #ifdef HAS_BATTERY
    #ifdef MARAUDER_V4
      #if defined(CYD_35) || defined(CYD_24) || defined(CYD_32) || defined(CYD_32CAP) || defined(CYD_35CAP)
        #define I2C_SDA 21
        #define I2C_SCL 22
      #elif defined(CYD_28) || defined(CYD_24G) || defined(CYD_24CAP)
        #define I2C_SDA 22
        #define I2C_SCL 27
      #endif
    #endif

    #ifdef MARAUDER_V6
      #define I2C_SDA 33
      #define I2C_SCL 22
    #endif

    #ifdef MARAUDER_V6_1
      #define I2C_SDA 33
      #define I2C_SCL 22
    #endif

    #ifdef MARAUDER_M5STICKC
      #define I2C_SDA 33
      #define I2C_SCL 22
    #endif

    #ifdef MARAUDER_KIT
      #define I2C_SDA 33
      #define I2C_SCL 22
    #endif

    #ifdef MARAUDER_MINI
      #define I2C_SDA 33
      #define I2C_SCL 26
    #endif

    #ifdef MARAUDER_V7
      #define I2C_SDA 33
      #define I2C_SCL 16
    #endif

  #endif

  //// MARAUDER TITLE STUFF
  #ifdef MARAUDER_V4
    #define MARAUDER_TITLE_BYTES 13578
  #elif defined(MARAUDER_V6) || defined(MARAUDER_V6_1)
    #define MARAUDER_TITLE_BYTES 13578
  #elif defined(MARAUDER_KIT)
    #define MARAUDER_TITLE_BYTES 13578
  #elif defined(MARAUDER_MINI)
    #define MARAUDER_TITLE_BYTES 13578
  #elif defined(MARAUDER_V7)
    #define MARAUDER_TITLE_BYTES 13578
  #elif defined(MARAUDER_REV_FEATHER)
    #define MARAUDER_TITLE_BYTES 13578
  #else
    #define MARAUDER_TITLE_BYTES 13578
  #endif
  //// END MARAUDER TITLE STUFF

  //// PCAP BUFFER STUFF
  #ifdef MARAUDER_V7
    #define BUF_SIZE 8 * 1024 // Had to reduce buffer size to save RAM. GG @spacehuhn
    #define SNAP_LEN 4096 // max len of each recieved packet
  #elif defined(MARAUDER_MINI)
    #define BUF_SIZE 8 * 1024 // Had to reduce buffer size to save RAM. GG @spacehuhn
    #define SNAP_LEN 4096 // max len of each recieved packet
  #elif defined(MARAUDER_REV_FEATHER)
    #define BUF_SIZE 8 * 1024 // Had to reduce buffer size to save RAM. GG @spacehuhn
    #define SNAP_LEN 4096 // max len of each recieved packet
  #else
    #define BUF_SIZE 3 * 1024 // Had to reduce buffer size to save RAM. GG @spacehuhn
    #define SNAP_LEN 2324 // max len of each recieved packet
  #endif
  //// PCAP BUFFER STUFF
#endif
