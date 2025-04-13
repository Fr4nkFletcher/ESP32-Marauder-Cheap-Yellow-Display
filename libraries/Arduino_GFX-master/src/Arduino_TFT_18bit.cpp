/*
 * start rewrite from:
 * https://github.com/adafruit/Adafruit-GFX-Library.git
 */
#include "Arduino_DataBus.h"
#include "Arduino_GFX.h"
#include "Arduino_TFT_18bit.h"

Arduino_TFT_18bit::Arduino_TFT_18bit(
    Arduino_DataBus *bus, int8_t rst, uint8_t r,
    bool ips, int16_t w, int16_t h,
    uint8_t col_offset1, uint8_t row_offset1, uint8_t col_offset2, uint8_t row_offset2)
    : Arduino_TFT(bus, rst, r, ips, w, h, col_offset1, row_offset1, col_offset2, row_offset2)
{
}

void Arduino_TFT_18bit::writeColor(uint16_t color)
{
  _bus->write((color & 0xF800) >> 8);
  _bus->write((color & 0x07E0) >> 3);
  _bus->write(color << 3);
}

void Arduino_TFT_18bit::writePixelPreclipped(int16_t x, int16_t y, uint16_t color)
{
  writeAddrWindow(x, y, 1, 1);
  _bus->write((color & 0xF800) >> 8);
  _bus->write((color & 0x07E0) >> 3);
  _bus->write(color << 3);
}

void Arduino_TFT_18bit::writeRepeat(uint16_t color, uint32_t len)
{
#if defined(ESP8266) || defined(ESP32)
  uint8_t c[3] = {(uint8_t)((color & 0xF800) >> 8), (uint8_t)((color & 0x07E0) >> 3), (uint8_t)((color & 0x001F) << 3)};
  _bus->writePattern(c, 3, len);
#else
  uint8_t c1 = (uint8_t)((color & 0xF800) >> 8);
  uint8_t c2 = (uint8_t)((color & 0x07E0) >> 3);
  uint8_t c3 = (uint8_t)((color & 0x001F) << 3);
  while (len--)
  {
    _bus->write(c1);
    _bus->write(c2);
    _bus->write(c3);
  }
#endif
}

// TFT optimization code, too big for ATMEL family
#if !defined(LITTLE_FOOT_PRINT)

void Arduino_TFT_18bit::writePixels(uint16_t *data, uint32_t len)
{
  uint16_t d;
  while (len--)
  {
    d = *data++;
    _bus->write((d & 0xF800) >> 8);
    _bus->write((d & 0x07E0) >> 3);
    _bus->write(d << 3);
  }
}

// TFT tuned BITMAP / XBITMAP / GRAYSCALE / RGB BITMAP FUNCTIONS ---------------------

/**************************************************************************/
/*!
    @brief  Draw a Indexed 16-bit image (RGB 5/6/5) at the specified (x,y) position.
    @param  bitmap      byte array of Indexed color bitmap
    @param  color_index byte array of 16-bit color index
    @param  w           Width of bitmap in pixels
    @param  h           Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::writeIndexedPixels(uint8_t *bitmap, uint16_t *color_index, uint32_t len)
{
  uint16_t d;
  while (len--)
  {
    d = color_index[*(bitmap++)];
    _bus->write((d & 0xF800) >> 8);
    _bus->write((d & 0x07E0) >> 3);
    _bus->write(d << 3);
  }
}

/**************************************************************************/
/*!
    @brief  Draw a Indexed 16-bit image (RGB 5/6/5) at the specified (x,y) position.
    @param  bitmap      byte array of Indexed color bitmap
    @param  color_index byte array of 16-bit color index
    @param  w           Width of bitmap in pixels
    @param  h           Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::writeIndexedPixelsDouble(uint8_t *bitmap, uint16_t *color_index, uint32_t len)
{
  uint8_t r, g, b;
  uint16_t d;
  while (len--)
  {
    d = color_index[*(bitmap++)];
    r = (d & 0xF800) >> 8;
    g = (d & 0x07E0) >> 3;
    b = d << 3;
    _bus->write(r);
    _bus->write(g);
    _bus->write(b);
    _bus->write(r);
    _bus->write(g);
    _bus->write(b);
  }
}

/**************************************************************************/
/*!
   @brief      Draw a PROGMEM-resident 1-bit image at the specified (x,y) position, using the specified foreground (for set bits) and background (unset bits) colors.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
    @param    bg 16-bit 5-6-5 Color to draw background with
*/
/**************************************************************************/
void Arduino_TFT_18bit::drawBitmap(int16_t x, int16_t y,
                                   const uint8_t bitmap[], int16_t w, int16_t h,
                                   uint16_t color, uint16_t bg)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::drawBitmap(x, y, bitmap, w, h, color, bg);
  }
  else
  {
    uint16_t c;
    int32_t pixels = w * h;
    uint8_t byte = 0;
    uint16_t idx = 0;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int32_t i = 0; i < pixels; i++)
    {
      if (i & 7)
      {
        byte <<= 1;
      }
      else
      {
        byte = pgm_read_byte(&bitmap[idx++]);
      }
      c = (byte & 0x80) ? color : bg;
      _bus->write((c & 0xF800) >> 8);
      _bus->write((c & 0x07E0) >> 3);
      _bus->write((c & 0x001F) << 3);
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief      Draw a RAM-resident 1-bit image at the specified (x,y) position, using the specified foreground (for set bits) and background (unset bits) colors.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with monochrome bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
    @param    color 16-bit 5-6-5 Color to draw pixels with
    @param    bg 16-bit 5-6-5 Color to draw background with
*/
/**************************************************************************/
void Arduino_TFT_18bit::drawBitmap(int16_t x, int16_t y,
                                   uint8_t *bitmap, int16_t w, int16_t h, uint16_t color, uint16_t bg)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::drawBitmap(x, y, bitmap, w, h, color, bg);
  }
  else
  {
    uint16_t c;
    int32_t pixels = w * h;
    uint8_t byte = 0;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int32_t i = 0; i < pixels; i++)
    {
      if (i & 7)
      {
        byte <<= 1;
      }
      else
      {
        byte = *(bitmap++);
      }
      c = (byte & 0x80) ? color : bg;
      _bus->write((c & 0xF800) >> 8);
      _bus->write((c & 0x07E0) >> 3);
      _bus->write((c & 0x001F) << 3);
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 8-bit image (grayscale) at the specified (x,y) pos.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::drawGrayscaleBitmap(int16_t x, int16_t y,
                                            const uint8_t bitmap[], int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::drawGrayscaleBitmap(x, y, bitmap, w, h);
  }
  else
  {
    uint8_t v;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        v = (uint8_t)pgm_read_byte(&bitmap[j * w + i]);
        _bus->write(v);
        _bus->write(v);
        _bus->write(v);
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 8-bit image (grayscale) at the specified (x,y) pos.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with grayscale bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::drawGrayscaleBitmap(int16_t x, int16_t y,
                                            uint8_t *bitmap, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::drawGrayscaleBitmap(x, y, bitmap, w, h);
  }
  else
  {
    uint8_t v;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        v = bitmap[j * w + i];
        _bus->write(v);
        _bus->write(v);
        _bus->write(v);
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a Indexed 16-bit image (RGB 5/6/5) at the specified (x,y) position.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::drawIndexedBitmap(int16_t x, int16_t y,
                                          uint8_t *bitmap, uint16_t *color_index, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::drawIndexedBitmap(x, y, bitmap, color_index, w, h);
  }
  else
  {
    startWrite();
    writeAddrWindow(x, y, w, h);
    writeIndexedPixels(bitmap, color_index, w * h);
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 16-bit image (RGB 5/6/5) at the specified (x,y) position.
   For 16-bit display devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw16bitRGBBitmap(int16_t x, int16_t y,
                                           const uint16_t bitmap[], int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw16bitRGBBitmap(x, y, bitmap, w, h);
  }
  else
  {
    uint16_t d;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        d = pgm_read_word(&bitmap[j * w + i]);
        _bus->write((d & 0xF800) >> 8);
        _bus->write((d & 0x07E0) >> 3);
        _bus->write(d << 3);
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
    @brief  Draw a RAM-resident 16-bit image (RGB 5/6/5) with a 1-bit mask
            (set bits = opaque, unset bits = clear) at the specified (x,y) position.
            BOTH buffers (color and mask) must be RAM-resident.
    @param  x       Top left corner x coordinate
    @param  y       Top left corner y coordinate
    @param  bitmap  byte array with 16-bit color bitmap
    @param  mask    byte array with monochrome mask bitmap
    @param  w       Width of bitmap in pixels
    @param  h       Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw16bitRGBBitmap(int16_t x, int16_t y,
                                           uint16_t *bitmap, uint8_t *mask, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw16bitRGBBitmap(x, y, bitmap, mask, w, h);
  }
  else
  {
    uint16_t d;
    int32_t offset = 0, maskIdx = 0, len = 0;
    uint8_t byte = 0;
    startWrite();
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        if (i & 7)
        {
          byte <<= 1;
        }
        else
        {
          byte = mask[maskIdx++];
        }
        if (byte & 0x80)
        {
          len++;
        }
        else
        {
          if (len)
          {
            writeAddrWindow(x + i - len, y, len, 1);
            for (int16_t i = len; i > 0; i--)
            {
              d = bitmap[offset - i];
              _bus->write((d & 0xF800) >> 8);
              _bus->write((d & 0x07E0) >> 3);
              _bus->write(d << 3);
            }
            len = 0;
          }
        }
        offset++;
      }
      if (len)
      {
        writeAddrWindow(x + w - 1 - len, y, len, 1);
        for (int16_t i = len; i > 0; i--)
        {
          d = bitmap[offset - i];
          _bus->write((d & 0xF800) >> 8);
          _bus->write((d & 0x07E0) >> 3);
          _bus->write(d << 3);
        }
        len = 0;
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit image (RGB 5/6/5) at the specified (x,y) position.
   For 16-bit display devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw16bitRGBBitmap(int16_t x, int16_t y,
                                           uint16_t *bitmap, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw16bitRGBBitmap(x, y, bitmap, w, h);
  }
  else
  {
    uint16_t d;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        d = bitmap[j * w + i];
        _bus->write((d & 0xF800) >> 8);
        _bus->write((d & 0x07E0) >> 3);
        _bus->write(d << 3);
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 16-bit Big Endian image (RGB 5/6/5) at the specified (x,y) position.
   For 16-bit display devices; no color reduction performed.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw16bitBeRGBBitmap(int16_t x, int16_t y,
                                             uint16_t *bitmap, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw16bitBeRGBBitmap(x, y, bitmap, w, h);
  }
  else
  {
    uint16_t d;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        d = bitmap[j * w + i];
        _bus->write((d & 0x00F8));
        _bus->write(((d & 0xE000) >> 11) | (d & 0x0007) << 5);
        _bus->write((d & 0x1F00) >> 5);
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a PROGMEM-resident 24-bit image (RGB 5/6/5) at the specified (x,y) position.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw24bitRGBBitmap(int16_t x, int16_t y,
                                           const uint8_t bitmap[], int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw24bitRGBBitmap(x, y, bitmap, w, h);
  }
  else
  {
    int16_t offset = 0;
    startWrite();
    writeAddrWindow(x, y, w, h);
    for (int16_t j = 0; j < h; j++, y++)
    {
      for (int16_t i = 0; i < w; i++)
      {
        _bus->write(pgm_read_byte(&bitmap[offset++]));
        _bus->write(pgm_read_byte(&bitmap[offset++]));
        _bus->write(pgm_read_byte(&bitmap[offset++]));
      }
    }
    endWrite();
  }
}

/**************************************************************************/
/*!
   @brief   Draw a RAM-resident 24-bit image (RGB 5/6/5) at the specified (x,y) position.
    @param    x   Top left corner x coordinate
    @param    y   Top left corner y coordinate
    @param    bitmap  byte array with 16-bit color bitmap
    @param    w   Width of bitmap in pixels
    @param    h   Height of bitmap in pixels
*/
/**************************************************************************/
void Arduino_TFT_18bit::draw24bitRGBBitmap(int16_t x, int16_t y,
                                           uint8_t *bitmap, int16_t w, int16_t h)
{
  if (
      ((x + w - 1) < 0) || // Outside left
      ((y + h - 1) < 0) || // Outside top
      (x > _max_x) ||      // Outside right
      (y > _max_y)         // Outside bottom
  )
  {
    return;
  }
  else if (
      (x < 0) ||                // Clip left
      (y < 0) ||                // Clip top
      ((x + w - 1) > _max_x) || // Clip right
      ((y + h - 1) > _max_y)    // Clip bottom
  )
  {
    Arduino_GFX::draw24bitRGBBitmap(x, y, bitmap, w, h);
  }
  else
  {
    startWrite();
    writeAddrWindow(x, y, w, h);
    _bus->writeBytes(bitmap, w * h * 3);
    endWrite();
  }
}

#endif // !defined(LITTLE_FOOT_PRINT)
