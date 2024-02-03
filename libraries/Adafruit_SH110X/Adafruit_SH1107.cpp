/*!
 * @file Adafruit_SH1107.cpp
 *
 */

#include "Adafruit_SH110X.h"
#include "splash.h"

// CONSTRUCTORS, DESTRUCTOR ------------------------------------------------

/*!
    @brief  Constructor for I2C-interfaced SH1107 displays.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  twi
            Pointer to an existing TwoWire instance (e.g. &Wire, the
            microcontroller's primary I2C bus).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  clkDuring
            Speed (in Hz) for Wire transmissions in SH110X library calls.
            Defaults to 400000 (400 KHz), a known 'safe' value for most
            microcontrollers, and meets the SH110X datasheet spec.
            Some systems can operate I2C faster (800 KHz for ESP32, 1 MHz
            for many other 32-bit MCUs), and some (perhaps not all)
            SH110X's can work with this -- so it's optionally be specified
            here and is not a default behavior. (Ignored if using pre-1.5.7
            Arduino software, which operates I2C at a fixed 100 KHz.)
    @param  clkAfter
            Speed (in Hz) for Wire transmissions following SH110X library
            calls. Defaults to 100000 (100 KHz), the default Arduino Wire
            speed. This is done rather than leaving it at the 'during' speed
            because other devices on the I2C bus might not be compatible
            with the faster rate. (Ignored if using pre-1.5.7 Arduino
            software, which operates I2C at a fixed 100 KHz.)
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_SH1107::Adafruit_SH1107(uint16_t w, uint16_t h, TwoWire *twi,
                                 int8_t rst_pin, uint32_t clkDuring,
                                 uint32_t clkAfter)
    : Adafruit_SH110X(w, h, twi, rst_pin, clkDuring, clkAfter) {}

/*!
    @brief  Constructor for SPI SH1107 displays, using software (bitbang)
            SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  mosi_pin
            MOSI (master out, slave in) pin (using Arduino pin numbering).
            This transfers serial data from microcontroller to display.
    @param  sclk_pin
            SCLK (serial clock) pin (using Arduino pin numbering).
            This clocks each bit from MOSI.
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_SH1107::Adafruit_SH1107(uint16_t w, uint16_t h, int8_t mosi_pin,
                                 int8_t sclk_pin, int8_t dc_pin, int8_t rst_pin,
                                 int8_t cs_pin)
    : Adafruit_SH110X(w, h, mosi_pin, sclk_pin, dc_pin, rst_pin, cs_pin) {}

/*!
    @brief  Constructor for SPI SH1107 displays, using native hardware SPI.
    @param  w
            Display width in pixels
    @param  h
            Display height in pixels
    @param  spi
            Pointer to an existing SPIClass instance (e.g. &SPI, the
            microcontroller's primary SPI bus).
    @param  dc_pin
            Data/command pin (using Arduino pin numbering), selects whether
            display is receiving commands (low) or data (high).
    @param  rst_pin
            Reset pin (using Arduino pin numbering), or -1 if not used
            (some displays might be wired to share the microcontroller's
            reset pin).
    @param  cs_pin
            Chip-select pin (using Arduino pin numbering) for sharing the
            bus with other devices. Active low.
    @param  bitrate
            SPI clock rate for transfers to this display. Default if
            unspecified is 8000000UL (8 MHz).
    @note   Call the object's begin() function before use -- buffer
            allocation is performed there!
*/
Adafruit_SH1107::Adafruit_SH1107(uint16_t w, uint16_t h, SPIClass *spi,
                                 int8_t dc_pin, int8_t rst_pin, int8_t cs_pin,
                                 uint32_t bitrate)
    : Adafruit_SH110X(w, h, spi, dc_pin, rst_pin, cs_pin, bitrate) {}

/*!
    @brief  Destructor for Adafruit_SH1107 object.
*/
Adafruit_SH1107::~Adafruit_SH1107(void) {}

/*!
    @brief  Allocate RAM for image buffer, initialize peripherals and pins.
    @param  addr
            I2C address of corresponding SH110X display (or pass 0 to use
            default of 0x3C for 128x32 display, 0x3D for all others).
            SPI displays (hardware or software) do not use addresses, but
            this argument is still required (pass 0 or any value really,
            it will simply be ignored). Default if unspecified is 0.
    @param  reset
            If true, and if the reset pin passed to the constructor is
            valid, a hard reset will be performed before initializing the
            display. If using multiple SH110X displays on the same bus, and
            if they all share the same reset pin, you should only pass true
            on the first display being initialized, false on all others,
            else the already-initialized displays would be reset. Default if
            unspecified is true.
    @return true on successful allocation/init, false otherwise.
            Well-behaved code should check the return value before
            proceeding.
    @note   MUST call this function before any drawing or updates!
*/
bool Adafruit_SH1107::begin(uint8_t addr, bool reset) {

  Adafruit_GrayOLED::_init(addr, reset);

  setContrast(0x2F);

#ifndef SH110X_NO_SPLASH
  // the featherwing with 128x64 oled is 'rotated' so to make the splash right,
  // rotate!
  if (WIDTH == 64 && HEIGHT == 128) {
    setRotation(1);
    drawBitmap((HEIGHT - splash2_width) / 2, (WIDTH - splash2_height) / 2,
               splash2_data, splash2_width, splash2_height, 1);
    setRotation(0);
  }
  if (WIDTH == 128 && HEIGHT == 128) {
    drawBitmap((HEIGHT - splash2_width) / 2, (WIDTH - splash2_height) / 2,
               splash2_data, splash2_width, splash2_height, 1);
  }
#endif

  // Init sequence, make sure its under 32 bytes, or split into multiples!
  // clang-format off
  static const uint8_t init[] = {
      SH110X_DISPLAYOFF,               // 0xAE
      SH110X_SETDISPLAYCLOCKDIV, 0x51, // 0xd5, 0x51,
      SH110X_MEMORYMODE,               // 0x20
      SH110X_SETCONTRAST, 0x4F,        // 0x81, 0x4F
      SH110X_DCDC, 0x8A,               // 0xAD, 0x8A
      SH110X_SEGREMAP,                 // 0xA0
      SH110X_COMSCANINC,               // 0xC0
      SH110X_SETDISPSTARTLINE, 0x0,    // 0xDC 0x00
      SH110X_SETDISPLAYOFFSET, 0x60,   // 0xd3, 0x60,
      SH110X_SETPRECHARGE, 0x22,       // 0xd9, 0x22,
      SH110X_SETVCOMDETECT, 0x35,      // 0xdb, 0x35,
      SH110X_SETMULTIPLEX, 0x3F,       // 0xa8, 0x3f,
      // SH110X_SETPAGEADDR,                  // 0xb0
      // SH110X_SETCOMPINS, 0x12,             // 0xda, 0x12,
      SH110X_DISPLAYALLON_RESUME, // 0xa4
      SH110X_NORMALDISPLAY,       // 0xa6
  };
  // clang-format on

  if (!oled_commandList(init, sizeof(init))) {
    return false;
  }

  if (WIDTH == 128 && HEIGHT == 128) {
    static const uint8_t init_128x128[] = {
        SH110X_SETDISPLAYOFFSET, 0x00, SH110X_SETMULTIPLEX, 0x7F, // 0xa8, 0x3f,
    };
    if (!oled_commandList(init_128x128, sizeof(init_128x128))) {
      return false;
    }
  }

  delay(100);                     // 100ms delay recommended
  oled_command(SH110X_DISPLAYON); // 0xaf

  return true; // Success
}
