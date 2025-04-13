#include "../Arduino_DataBus.h"
#if !defined(LITTLE_FOOT_PRINT)

#include "../Arduino_GFX.h"
#include "Arduino_Canvas.h"

Arduino_Canvas::Arduino_Canvas(
    int16_t w, int16_t h, Arduino_G *output, int16_t output_x, int16_t output_y)
    : Arduino_GFX(w, h), _output(output), _output_x(output_x), _output_y(output_y)
{
}

void Arduino_Canvas::begin(int32_t speed)
{
    _output->begin(speed);

    size_t s = _width * _height * 2;
#if defined(ESP32)
    if (psramFound())
    {
        _framebuffer = (uint16_t *)ps_malloc(s);
    }
    else
    {
        _framebuffer = (uint16_t *)malloc(s);
        if (!_framebuffer)
        {
            // hack for allocate memory over 63,360 pixels
            s /= 2;
            _framebuffer = (uint16_t *)malloc(s);
            uint16_t *tmp = (uint16_t *)malloc(s);
            UNUSED(tmp);
            log_v("_framebuffer delta: %d", tmp - _framebuffer);
        }
    }
#else
    _framebuffer = (uint16_t *)malloc(s);
#endif
    if (!_framebuffer)
    {
        Serial.println(F("_framebuffer allocation failed."));
    }
}

void Arduino_Canvas::writePixelPreclipped(int16_t x, int16_t y, uint16_t color)
{
    _framebuffer[((int32_t)y * _width) + x] = color;
}

void Arduino_Canvas::writeFastVLine(int16_t x, int16_t y,
                                    int16_t h, uint16_t color)
{
    if (_ordered_in_range(x, 0, _max_x) && h)
    { // X on screen, nonzero height
        if (h < 0)
        {               // If negative height...
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
                while (h--)
                {
                    *fb = color;
                    fb += _width;
                }
            }
        }
    }
}

void Arduino_Canvas::writeFastHLine(int16_t x, int16_t y,
                                    int16_t w, uint16_t color)
{
    if (_ordered_in_range(y, 0, _max_y) && w)
    { // Y on screen, nonzero width
        if (w < 0)
        {               // If negative width...
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
                while (w--)
                {
                    *(fb++) = color;
                }
            }
        }
    }
}

void Arduino_Canvas::writeFillRectPreclipped(int16_t x, int16_t y,
                                             int16_t w, int16_t h, uint16_t color)
{
    uint16_t *row = _framebuffer;
    row += y * _width;
    row += x;
    for (int j = 0; j < h; j++)
    {
        for (int i = 0; i < w; i++)
        {
            row[i] = color;
        }
        row += _width;
    }
}

void Arduino_Canvas::draw16bitRGBBitmap(int16_t x, int16_t y,
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
        row += x;
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
}

void Arduino_Canvas::draw16bitBeRGBBitmap(int16_t x, int16_t y,
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
    }
}

void Arduino_Canvas::flush()
{
    _output->draw16bitRGBBitmap(_output_x, _output_y, _framebuffer, _width, _height);
}

#endif // !defined(LITTLE_FOOT_PRINT)
