#include "../Arduino_DataBus.h"

#if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)

#include "../Arduino_GFX.h"
#include "Arduino_RPi_DPI_RGBPanel.h"

Arduino_RPi_DPI_RGBPanel::Arduino_RPi_DPI_RGBPanel(
    Arduino_ESP32RGBPanel *bus,
    int16_t w, uint16_t hsync_polarity, uint16_t hsync_front_porch, uint16_t hsync_pulse_width, uint16_t hsync_back_porch,
    int16_t h, uint16_t vsync_polarity, uint16_t vsync_front_porch, uint16_t vsync_pulse_width, uint16_t vsync_back_porch,
    uint16_t pclk_active_neg, int32_t prefer_speed, bool auto_flush)
    : Arduino_GFX(w, h), _bus(bus),
      _hsync_polarity(hsync_polarity), _hsync_front_porch(hsync_front_porch), _hsync_pulse_width(hsync_pulse_width), _hsync_back_porch(hsync_back_porch),
      _vsync_polarity(vsync_polarity), _vsync_front_porch(vsync_front_porch), _vsync_pulse_width(vsync_pulse_width), _vsync_back_porch(vsync_back_porch),
      _pclk_active_neg(pclk_active_neg), _prefer_speed(prefer_speed), _auto_flush(auto_flush)
{
  _framebuffer_size = w * h * 2;
}

void Arduino_RPi_DPI_RGBPanel::begin(int32_t speed)
{
  _bus->begin(speed);

  _framebuffer = _bus->getFrameBuffer(
      _width, _height,
      _hsync_pulse_width, _hsync_back_porch, _hsync_front_porch, _hsync_polarity,
      _vsync_pulse_width, _vsync_back_porch, _vsync_front_porch, _vsync_polarity,
      _pclk_active_neg, _prefer_speed);
}

void Arduino_RPi_DPI_RGBPanel::writePixelPreclipped(int16_t x, int16_t y, uint16_t color)
{
  uint16_t *fb = _framebuffer;
  fb += (int32_t)y * _width;
  fb += x;
  *fb = color;
  if (_auto_flush)
  {
    Cache_WriteBack_Addr((uint32_t)fb, 2);
  }
}

void Arduino_RPi_DPI_RGBPanel::writeFastVLine(int16_t x, int16_t y,
                                              int16_t h, uint16_t color)
{
  if (_ordered_in_range(x, 0, _max_x) && h)
  { // X on screen, nonzero height
    if (h < 0)
    {             // If negative height...
      y += h + 1; //   Move Y to top edge
      h = -h;     //   Use positive height
    }
    if (y <= _max_y)
    { // Not off bottom
      int16_t y2 = y + h - 1;
      if (y2 >= 0)
      { // Not off top
        // Line partly or fully overlaps screen
        if (y < 0)
        {
          y = 0;
          h = y2 + 1;
        } // Clip top
        if (y2 > _max_y)
        {
          h = _max_y - y + 1;
        } // Clip bottom

        uint16_t *fb = _framebuffer + ((int32_t)y * _width) + x;
        if (_auto_flush)
        {
          while (h--)
          {
            *fb = color;
            Cache_WriteBack_Addr((uint32_t)fb, 2);
            fb += _width;
          }
        }
        else
        {
          while (h--)
          {
            *fb = color;
            fb += _width;
          }
        }
      }
    }
  }
}

void Arduino_RPi_DPI_RGBPanel::writeFastHLine(int16_t x, int16_t y,
                                              int16_t w, uint16_t color)
{
  if (_ordered_in_range(y, 0, _max_y) && w)
  { // Y on screen, nonzero width
    if (w < 0)
    {             // If negative width...
      x += w + 1; //   Move X to left edge
      w = -w;     //   Use positive width
    }
    if (x <= _max_x)
    { // Not off right
      int16_t x2 = x + w - 1;
      if (x2 >= 0)
      { // Not off left
        // Line partly or fully overlaps screen
        if (x < 0)
        {
          x = 0;
          w = x2 + 1;
        } // Clip left
        if (x2 > _max_x)
        {
          w = _max_x - x + 1;
        } // Clip right

        uint16_t *fb = _framebuffer + ((int32_t)y * _width) + x;
        uint32_t cachePos = (uint32_t)fb;
        int16_t writeSize = w * 2;
        while (w--)
        {
          *(fb++) = color;
        }
        if (_auto_flush)
        {
          Cache_WriteBack_Addr(cachePos, writeSize);
        }
      }
    }
  }
}

void Arduino_RPi_DPI_RGBPanel::writeFillRectPreclipped(int16_t x, int16_t y,
                                                       int16_t w, int16_t h, uint16_t color)
{
  uint16_t *row = _framebuffer;
  row += y * _width;
  uint32_t cachePos = (uint32_t)row;
  row += x;
  for (int j = 0; j < h; j++)
  {
    for (int i = 0; i < w; i++)
    {
      row[i] = color;
    }
    row += _width;
  }
  if (_auto_flush)
  {
    Cache_WriteBack_Addr(cachePos, _width * h * 2);
  }
}

void Arduino_RPi_DPI_RGBPanel::draw16bitRGBBitmap(int16_t x, int16_t y,
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
  else
  {
    int16_t xskip = 0;
    if ((y + h - 1) > _max_y)
    {
      h -= (y + h - 1) - _max_y;
    }
    if (y < 0)
    {
      bitmap -= y * w;
      h += y;
      y = 0;
    }
    if ((x + w - 1) > _max_x)
    {
      xskip = (x + w - 1) - _max_x;
      w -= xskip;
    }
    if (x < 0)
    {
      bitmap -= x;
      xskip -= x;
      w += x;
      x = 0;
    }
    uint16_t *row = _framebuffer;
    row += y * _width;
    uint32_t cachePos = (uint32_t)row;
    row += x;
    if (((_width & 1) == 0) && ((xskip & 1) == 0) && ((w & 1) == 0))
    {
      uint32_t *row2 = (uint32_t *)row;
      uint32_t *bitmap2 = (uint32_t *)bitmap;
      int16_t _width2 = _width >> 1;
      int16_t xskip2 = xskip >> 1;
      int16_t w2 = w >> 1;

      for (int16_t j = 0; j < h; j++)
      {
        for (int16_t i = 0; i < w2; i++)
        {
          row2[i] = *bitmap2++;
        }
        bitmap2 += xskip2;
        row2 += _width2;
      }
    }
    else
    {
      for (int j = 0; j < h; j++)
      {
        for (int i = 0; i < w; i++)
        {
          row[i] = *bitmap++;
        }
        bitmap += xskip;
        row += _width;
      }
    }
    if (_auto_flush)
    {
      Cache_WriteBack_Addr(cachePos, _width * h * 2);
    }
  }
}

void Arduino_RPi_DPI_RGBPanel::draw16bitBeRGBBitmap(int16_t x, int16_t y,
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
  else
  {
    int16_t xskip = 0;
    if ((y + h - 1) > _max_y)
    {
      h -= (y + h - 1) - _max_y;
    }
    if (y < 0)
    {
      bitmap -= y * w;
      h += y;
      y = 0;
    }
    if ((x + w - 1) > _max_x)
    {
      xskip = (x + w - 1) - _max_x;
      w -= xskip;
    }
    if (x < 0)
    {
      bitmap -= x;
      xskip -= x;
      w += x;
      x = 0;
    }
    uint16_t *row = _framebuffer;
    row += y * _width;
    uint32_t cachePos = (uint32_t)row;
    row += x;
    uint16_t color;
    for (int j = 0; j < h; j++)
    {
      for (int i = 0; i < w; i++)
      {
        color = *bitmap++;
        MSB_16_SET(row[i], color);
      }
      bitmap += xskip;
      row += _width;
    }
    if (_auto_flush)
    {
      Cache_WriteBack_Addr(cachePos, _width * h * 2);
    }
  }
}

void Arduino_RPi_DPI_RGBPanel::flush(void)
{
  if (!_auto_flush)
  {
    Cache_WriteBack_Addr((uint32_t)_framebuffer, _framebuffer_size);
  }
}

uint16_t *Arduino_RPi_DPI_RGBPanel::getFramebuffer()
{
  return _framebuffer;
}

#endif // #if defined(ESP32) && (CONFIG_IDF_TARGET_ESP32S3)
