// SPDX-FileCopyrightText: 2023 P Burgess for Adafruit Industries
//
// SPDX-License-Identifier: BSD-3-Clause

/*!
 * @file PicoDVI.h
 *
 * Arduino-and-Adafruit-GFX wrapper around Luke Wren's PicoDVI library.
 */

#pragma once

#include "../software/include/common_dvi_pin_configs.h"
#include "hardware/vreg.h" // In Pico SDK
#include "libdvi/dvi.h"
#include "pico/stdlib.h" // In Pico SDK
#include <Adafruit_GFX.h>

/** A set list of resolutions keeps things manageable for users,
    avoids some possibly-incompatible argument combos to constructors. */
enum DVIresolution {
  DVI_RES_320x240p60 = 0,
  DVI_RES_400x240p60,
  DVI_RES_400x240p30, // Reduced refresh rate, less overclock required
  DVI_RES_640x480p60,
  DVI_RES_800x480p60,
  DVI_RES_800x480p30, // Reduced refresh rate, less overclock required
  DVI_RES_640x240p60, // "Tall" pixels, e.g. for 80-column text mode
  DVI_RES_800x240p60, // Same, 100-column
  DVI_RES_800x240p30, // Reduced refresh rate, less overclock required
  DVI_RES_1280x720p30 // Experimenting, not working, plz don't use
};

extern uint8_t dvi_vertical_repeat; ///< In libdvi/dvi.c
extern bool dvi_monochrome_tmds;    ///< In libdvi/dvi.c

/*!
  @brief  A base class used by some of the framebuffer classes that follow.
          Wraps essential functionality of the underlying PicoDVI library.
          Maybe only rarely (if ever) needed in Arduino sketch code, but
          it's here if a project requires working at a lower level to
          provide features not present in the framebuffer modes.
*/
class PicoDVI {
public:
  /*!
    @brief  PicoDVI constructor. Instantiates a new PicoDVI object (must
            follow with a begin() call to alloc buffers and init hardware).
    @param  t  Pointer to dvi_timing struct, one of the supported
               resolutions declared in libdvi/dvi_timing.c. Default if not
               specified is 640x480p60.
    @param  c  Pointer to dvi_serialiser_cfg struct, which defines pinout.
               These are in ../software/include/common_dvi_pin_configs.h.
               Default if not specified is adafruit_feather_dvi_cfg.
    @param  v  Core voltage; higher resolution modes require faster
               overclocking and potentially higher voltages. This is an
               enumeration declared in Pico SDK hardware/vreg.h. Normal
               system operation would be 1.1V, default here if not
               specified is 1.2V.
  */
  PicoDVI(const struct dvi_timing &t = dvi_timing_640x480p_60hz,
          const struct dvi_serialiser_cfg &c = adafruit_feather_dvi_cfg,
          vreg_voltage v = VREG_VOLTAGE_1_20);
  ~PicoDVI(void);
  /*!
    @brief  Internal function used during DVI initialization. User code
            should not touch this, but it does need to remain public for
            C init code to access protected elements.
  */
  void _setup(void);

protected:
  /*!
    @brief  Initialize hardware and start video thread.
  */
  void begin(void);
  vreg_voltage voltage;                ///< Core voltage
  struct dvi_inst dvi0;                ///< DVI instance
  void (*mainloop)(dvi_inst *) = NULL; ///< Video mode loop handler
  volatile bool wait_begin = true;     ///< For synchronizing cores
};

/*!
  @brief  DVIGFX16 provides a 16-bit color (RGB565) Adafruit_GFX-compatible
          framebuffer. Double-buffering is NOT provided even as an option,
          there simply isn't adequate RAM.
*/
class DVIGFX16 : public PicoDVI, public GFXcanvas16 {
public:
  /*!
    @brief  DVIGFX16 constructor.
    @param  res   Video mode, from the DVIresolution enumeration. This will
                  usually be 320x240 or 400x240, there just isn't RAM to
                  handle anything larger as a pixel-accessible framebuffer.
    @param  c     Pointer to dvi_serialiser_cfg struct, which defines pinout.
                  These are in ../software/include/common_dvi_pin_configs.h.
                  Default if not specified is adafruit_feather_dvi_cfg.
    @param  v     Core voltage; higher resolution modes require faster
                  overclocking and potentially higher voltages. This is an
                  enumeration declared in Pico SDK hardware/vreg.h. Normal
                  system operation would be 1.1V, default here if not
                  specified is 1.2V.
  */
  DVIGFX16(const DVIresolution res = DVI_RES_320x240p60,
           const struct dvi_serialiser_cfg &c = adafruit_feather_dvi_cfg,
           vreg_voltage v = VREG_VOLTAGE_1_20);
  ~DVIGFX16(void);
  /*!
    @brief   Initialize DVI output for 16-bit color display.
    @return  true on success, false otherwise (buffer allocation usually).
  */
  bool begin(void);
  /*!
    @brief   Convert a 24-bit RGB color (as three 8-bit values) to a packed
             16-bit "RGB565" color, as native to PicoDVI and Adafruit_GFX.
    @param   red    Red component, 0-255.
    @param   green  Green component, 0-255.
    @param   blue   Blue component, 0-255.
    @return  Nearest 16-bit RGB565 equivalent.
  */
  uint16_t color565(uint8_t red, uint8_t green, uint8_t blue) {
    return ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
  }
  /*!
    @brief  Scanline-processing function used internally during video
            generation. User sketch code should never call this, but it
            needs to be public so that C code can access it.
  */
  void _scanline_callback(void);

protected:
  uint16_t scanline = 2; ///< First 2 scanlines are set up before DVI start
};

/*!
  @brief  DVIGFX8 provides a 8-bit color-mapped Adafruit_GFX-compatible
          framebuffer, with optional double-buffering.
*/
class DVIGFX8 : public PicoDVI, public GFXcanvas8 {
public:
  /*!
    @brief  DVIGFX8 constructor.
    @param  res   Video mode, from the DVIresolution enumeration.
    @param  dbuf  Double-buffered flag. If true, two 8-bit framebuffers
                  (each with associated color palette) are allocated, code
                  can alternate between these for flicker-free animation.
    @param  c     Pointer to dvi_serialiser_cfg struct, which defines pinout.
                  These are in ../software/include/common_dvi_pin_configs.h.
                  Default if not specified is adafruit_feather_dvi_cfg.
    @param  v     Core voltage; higher resolution modes require faster
                  overclocking and potentially higher voltages. This is an
                  enumeration declared in Pico SDK hardware/vreg.h. Normal
                  system operation would be 1.1V, default here if not
                  specified is 1.2V.
  */
  DVIGFX8(const DVIresolution res = DVI_RES_320x240p60, const bool dbuf = false,
          const struct dvi_serialiser_cfg &c = adafruit_feather_dvi_cfg,
          vreg_voltage v = VREG_VOLTAGE_1_20);
  ~DVIGFX8(void);
  /*!
    @brief   Initialize DVI output for 8-bit colormapped display.
    @return  true on success, false otherwise (buffer allocation usually).
  */
  bool begin(void);
  /*!
    @brief   Get base address of 'back' palette (or single palette in
             single-buffered mode), for code that wants to access this.
             directly rather than through setColor() and getColor().
    @return  uint16_t* pointer to 256-entry palette (RGB565 colors).
  */
  uint16_t *getPalette(void) { return palette[back_index]; }
  /*!
    @brief   Set one color in the 'back' color palette (or single palette in
             single-buffered mode).
    @param   idx    Color index to set, 0-255.
    @param   color  16-bit "RGB565" color.
  */
  void setColor(uint8_t idx, uint16_t color) {
    palette[back_index][idx] = color;
  }
  /*!
    @brief   Set one color in the 'back' color palette (or single palette in
             single-buffered mode). Accepts three 8-bit values, but COLOR
             WILL BE QUANTIZED TO 16-BIT RGB565 because that's what PicoDVI
             uses. Original 24-bit value is LOST and cannot be accurately
             read back by getColor().
    @param   idx    Color index to set, 0-255.
    @param   red    Red component, 0-255.
    @param   green  Green component, 0-255.
    @param   blue   Blue component, 0-255.
  */
  void setColor(uint8_t idx, uint8_t red, uint8_t green, uint8_t blue) {
    palette[back_index][idx] =
        ((red & 0xF8) << 8) | ((green & 0xFC) << 3) | (blue >> 3);
  }
  /*!
    @brief   Read one color from the 'back' color palette (or single palette
             in single-buffered mode).
    @param   idx  Color index to read, 0-255.
    @return  16-bit "RGB565" color.
  */
  uint16_t getColor(uint8_t idx) { return palette[back_index][idx]; }
  /*!
    @brief  Swap front/back framebuffers on next vertical sync.
    @param  copy_framebuffer  If true, contents of new 'front' framebuffer
                              will be duplicated into new 'back' buffer for
                              updates (useful for code that modifies screen
                              frame-to-frame, rather than complete regen).
    @param  copy_palette      If true, copies 'front' palette to 'back'.
                              This is obscure; most code will set up a global
                              palette once, swap-with-copy once (so front
                              and back are both using same palette), and not
                              touch this...but it's there if palette-
                              animating code needs it.
  */
  void swap(bool copy_framebuffer = false, bool copy_palette = false);

  /*!
    @brief  Scanline-processing function used internally during video
            generation. User sketch code should never call this, but it
            needs to be public so that C code can access it.
  */
  void _scanline_callback(void);

protected:
  uint16_t palette[2][256];    ///< [2] for double-buffering
  uint16_t *row565[2];         ///< 2 scanlines of 16-bit RGB565 data
  uint16_t scanline = 2;       ///< 1st 2 scanlines loaded before DVI start
  uint8_t rowidx = 1;          ///< Alternate 0/1 for which row565[] is active
  bool dbuf = false;           ///< True if double-buffered
  uint8_t *buffer_save;        ///< Original canvas buffer pointer
  uint8_t back_index = 0;      ///< Which of 2 buffers receives draw ops
  volatile bool swap_wait = 0; ///< For synchronizing fromt/back buffer swap
};

/*!
  @brief  DVIGFX1 provides a 1-bit monochrome (black, white) Adafruit_GFX-
          compatible framebuffer, with optional double-buffering.
*/
class DVIGFX1 : public PicoDVI, public GFXcanvas1 {
public:
  /*!
    @brief  DVIGFX1 constructor.
    @param  res   Video mode, from the DVIresolution enumeration. This will
                  usually be 640x480 or 800x480. Higher settings are not
                  currently supported.
    @param  dbuf  Double-buffered flag. If true, two 1-bit framebuffers are
                  allocated, code can alternate between these for flicker-
                  free animation.
    @param  c     Pointer to dvi_serialiser_cfg struct, which defines pinout.
                  These are in ../software/include/common_dvi_pin_configs.h.
                  Default if not specified is adafruit_feather_dvi_cfg.
    @param  v     Core voltage; higher resolution modes require faster
                  overclocking and potentially higher voltages. This is an
                  enumeration declared in Pico SDK hardware/vreg.h. Normal
                  system operation would be 1.1V, default here if not
                  specified is 1.2V.
  */
  DVIGFX1(const DVIresolution res = DVI_RES_640x480p60, const bool dbuf = false,
          const struct dvi_serialiser_cfg &c = adafruit_feather_dvi_cfg,
          vreg_voltage v = VREG_VOLTAGE_1_20);
  ~DVIGFX1(void);
  /*!
    @brief   Initialize DVI output for 1-bit monochrome graphics display.
    @return  true on success, false otherwise (buffer allocation usually).
  */
  bool begin(void);
  /*!
    @brief  Swap front/back framebuffers on next vertical sync.
    @param  copy_framebuffer  If true, contents of new 'front' framebuffer
                              will be duplicated into new 'back' buffer for
                              updates (useful for code that modifies screen
                              frame-to-frame, rather than complete regen).
  */
  void swap(bool copy_framebuffer = false);

  /*!
    @brief  Video main loop handler invoked by underlying PicoDVI code.
            User sketch code should never call this, but it needs to be
            public so that C code can access it.
  */
  void _mainloop(void);

private:
  bool dbuf = false;           // True if double-buffered
  uint8_t *buffer_save;        // Original canvas buffer pointer
  uint8_t back_index = 0;      // Which of 2 buffers receives draw ops
  volatile bool swap_wait = 0; // For syncronizing front/back buffer swap
};

/*!
  @brief  DVItext1 provides a 1-bit monochrome (black, white) text-only
          display. print() and related calls to this display will vertical
          scroll when needed. It's very basic and does not provide any ANSI-
          like terminal functionality.
*/
class DVItext1 : public PicoDVI, public GFXcanvas16 {
public:
  /*!
    @brief  DVItext1 constructor.
    @param  res  Video mode, from the DVIresolution enumeration. This is the
                 pixel resolution; text cells are 8x8 pixels and thus the
                 number of rows and columns will be less (e.g. 640x240 mode
                 yields 80x30 characters).
    @param  c    Pointer to dvi_serialiser_cfg struct, which defines pinout.
                 These are in ../software/include/common_dvi_pin_configs.h.
                 Default if not specified is adafruit_feather_dvi_cfg.
    @param  v    Core voltage; higher resolution modes require faster
                 overclocking and potentially higher voltages. This is an
                 enumeration declared in Pico SDK hardware/vreg.h. Normal
                 system operation would be 1.1V, default here if not
                 specified is 1.2V.
  */
  DVItext1(const DVIresolution res = DVI_RES_640x240p60,
           const struct dvi_serialiser_cfg &c = adafruit_feather_dvi_cfg,
           vreg_voltage v = VREG_VOLTAGE_1_20);
  ~DVItext1(void);
  /*!
    @brief   Initialize DVI output for text display.
    @return  true on success, false otherwise (buffer allocation usually).
  */
  bool begin(void);
  /*!
    @brief   Base character-writing function, allows Arduino print() and
             println() calls to work with display. Text will scroll upward
             as needed.
    @param   c  ASCII character to write.
    @return  Number of characters written; always 1.
  */
  size_t write(uint8_t c);
  /*!
    @brief  Scanline-processing function used internally during video
            generation. User sketch code should never call this, but it
            needs to be public so that C code can access it.
    @param  y  Scanline index; 0 to height-1.
  */
  void _prepare_scanline(uint16_t y);
  /*!
    @brief  Video main loop handler invoked by underlying PicoDVI code.
            User sketch code should never call this, but it needs to be
            public so that C code can access it.
  */
  void _mainloop(void);

protected:
};
