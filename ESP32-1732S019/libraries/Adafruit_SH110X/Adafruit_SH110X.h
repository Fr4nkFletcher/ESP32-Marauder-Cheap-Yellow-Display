/*!
 * @file Adafruit_SH110X.h
 *
 * This is part of for Adafruit's SH110X library for monochrome
 * OLED displays: http://www.adafruit.com/category/63_98
 *
 * These displays use I2C or SPI to communicate. I2C requires 2 pins
 * (SCL+SDA) and optionally a RESET pin. SPI requires 4 pins (MOSI, SCK,
 * select, data/command) and optionally a reset pin. Hardware SPI or
 * 'bitbang' software SPI are both supported.
 *
 * Adafruit invests time and resources providing this open source code,
 * please support Adafruit and open-source hardware by purchasing
 * products from Adafruit!
 *
 * Written by Limor Fried/Ladyada for Adafruit Industries, with
 * contributions from the open source community.
 *
 * BSD license, all text above, and the splash screen header file,
 * must be included in any redistribution.
 *
 */

#ifndef _Adafruit_SH110X_H_
#define _Adafruit_SH110X_H_

#include <Adafruit_GrayOLED.h>

/// fit into the SH110X_ naming scheme
#define SH110X_BLACK 0   ///< Draw 'off' pixels
#define SH110X_WHITE 1   ///< Draw 'on' pixels
#define SH110X_INVERSE 2 ///< Invert pixels

// Uncomment to disable Adafruit splash logo
//#define SH110X_NO_SPLASH

#define SH110X_MEMORYMODE 0x20          ///< See datasheet
#define SH110X_COLUMNADDR 0x21          ///< See datasheet
#define SH110X_PAGEADDR 0x22            ///< See datasheet
#define SH110X_SETCONTRAST 0x81         ///< See datasheet
#define SH110X_CHARGEPUMP 0x8D          ///< See datasheet
#define SH110X_SEGREMAP 0xA0            ///< See datasheet
#define SH110X_DISPLAYALLON_RESUME 0xA4 ///< See datasheet
#define SH110X_DISPLAYALLON 0xA5        ///< Not currently used
#define SH110X_NORMALDISPLAY 0xA6       ///< See datasheet
#define SH110X_INVERTDISPLAY 0xA7       ///< See datasheet
#define SH110X_SETMULTIPLEX 0xA8        ///< See datasheet
#define SH110X_DCDC 0xAD                ///< See datasheet
#define SH110X_DISPLAYOFF 0xAE          ///< See datasheet
#define SH110X_DISPLAYON 0xAF           ///< See datasheet
#define SH110X_SETPAGEADDR                                                     \
  0xB0 ///< Specify page address to load display RAM data to page address
       ///< register
#define SH110X_COMSCANINC 0xC0         ///< Not currently used
#define SH110X_COMSCANDEC 0xC8         ///< See datasheet
#define SH110X_SETDISPLAYOFFSET 0xD3   ///< See datasheet
#define SH110X_SETDISPLAYCLOCKDIV 0xD5 ///< See datasheet
#define SH110X_SETPRECHARGE 0xD9       ///< See datasheet
#define SH110X_SETCOMPINS 0xDA         ///< See datasheet
#define SH110X_SETVCOMDETECT 0xDB      ///< See datasheet
#define SH110X_SETDISPSTARTLINE                                                \
  0xDC ///< Specify Column address to determine the initial display line or
       ///< COM0.

#define SH110X_SETLOWCOLUMN 0x00  ///< Not currently used
#define SH110X_SETHIGHCOLUMN 0x10 ///< Not currently used
#define SH110X_SETSTARTLINE 0x40  ///< See datasheet

/*!
    @brief  Class that stores state and functions for interacting with
            SH110X OLED displays. Not instantiatable - use a subclass!
*/
class Adafruit_SH110X : public Adafruit_GrayOLED {
public:
  // NEW CONSTRUCTORS -- recommended for new projects
  Adafruit_SH110X(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                  int8_t rst_pin = -1, uint32_t preclk = 400000,
                  uint32_t postclk = 100000);
  Adafruit_SH110X(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                  int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  Adafruit_SH110X(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                  int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);

  virtual ~Adafruit_SH110X(void) = 0;

  void display(void);

protected:
  /*! some displays are 'inset' in memory, so we have to skip some memory to
   * display */
  uint8_t _page_start_offset = 0;

private:
};

/*!
    @brief  Class that stores state and functions for interacting with
            SH1106G OLED displays.
*/
class Adafruit_SH1106G : public Adafruit_SH110X {
public:
  Adafruit_SH1106G(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                   int8_t rst_pin = -1, uint32_t preclk = 400000,
                   uint32_t postclk = 100000);
  Adafruit_SH1106G(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                   int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  Adafruit_SH1106G(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                   int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);

  ~Adafruit_SH1106G(void);

  bool begin(uint8_t i2caddr = 0x3C, bool reset = true);
};

/*!
    @brief  Class that stores state and functions for interacting with
            SH1107 OLED displays.
*/
class Adafruit_SH1107 : public Adafruit_SH110X {
public:
  Adafruit_SH1107(uint16_t w, uint16_t h, TwoWire *twi = &Wire,
                  int8_t rst_pin = -1, uint32_t preclk = 400000,
                  uint32_t postclk = 100000);
  Adafruit_SH1107(uint16_t w, uint16_t h, int8_t mosi_pin, int8_t sclk_pin,
                  int8_t dc_pin, int8_t rst_pin, int8_t cs_pin);
  Adafruit_SH1107(uint16_t w, uint16_t h, SPIClass *spi, int8_t dc_pin,
                  int8_t rst_pin, int8_t cs_pin, uint32_t bitrate = 8000000UL);

  ~Adafruit_SH1107(void);

  bool begin(uint8_t i2caddr = 0x3C, bool reset = true);
};
#endif // _Adafruit_SH110X_H_
