#include "../Arduino_DataBus.h"
#if !defined(LITTLE_FOOT_PRINT)

#include "../Arduino_GFX.h"
#include "Arduino_Canvas_Indexed.h"

Arduino_Canvas_Indexed::Arduino_Canvas_Indexed(int16_t w, int16_t h, Arduino_G *output, int16_t output_x, int16_t output_y, uint8_t mask_level)
    : Arduino_GFX(w, h), _output(output), _output_x(output_x), _output_y(output_y)
{
    if (mask_level >= MAXMASKLEVEL)
    {
        mask_level = MAXMASKLEVEL - 1;
    }
    _current_mask_level = mask_level;
    _color_mask = mask_level_list[_current_mask_level];
}

void Arduino_Canvas_Indexed::begin(int32_t speed)
{
    _output->begin(speed);

    size_t s = _width * _height;
#if defined(ESP32)
    if (psramFound())
    {
        _framebuffer = (uint8_t *)ps_malloc(s);
    }
    else
    {
        _framebuffer = (uint8_t *)malloc(s);
    }
#else
    _framebuffer = (uint8_t *)malloc(s);
#endif
    if (!_framebuffer)
    {
        Serial.println(F("_framebuffer allocation failed."));
    }
}

void Arduino_Canvas_Indexed::writePixelPreclipped(int16_t x, int16_t y, uint16_t color)
{
    _framebuffer[((int32_t)y * _width) + x] = get_color_index(color);
}

void Arduino_Canvas_Indexed::writeFastVLine(int16_t x, int16_t y,
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

                uint8_t idx = get_color_index(color);

                uint8_t *fb = _framebuffer + ((int32_t)y * _width) + x;
                while (h--)
                {
                    *fb = idx;
                    fb += _width;
                }
            }
        }
    }
}

void Arduino_Canvas_Indexed::writeFastHLine(int16_t x, int16_t y,
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

                uint8_t idx = get_color_index(color);

                uint8_t *fb = _framebuffer + ((int32_t)y * _width) + x;
                while (w--)
                {
                    *(fb++) = idx;
                }
            }
        }
    }
}

void Arduino_Canvas_Indexed::flush()
{
    _output->drawIndexedBitmap(_output_x, _output_y, _framebuffer, _color_index, _width, _height);
}

uint8_t Arduino_Canvas_Indexed::get_color_index(uint16_t color)
{
    color &= _color_mask;
    for (uint8_t i = 0; i < _indexed_size; i++)
    {
        if (_color_index[i] == color)
        {
            return i;
        }
    }
    if (_indexed_size == (COLOR_IDX_SIZE - 1)) // overflowed
    {
        raise_mask_level();
    }
    _color_index[_indexed_size] = color;
    // Serial.print("color_index[");
    // Serial.print(_indexed_size);
    // Serial.print("] = ");
    // Serial.println(color);
    return _indexed_size++;
}

uint16_t Arduino_Canvas_Indexed::get_index_color(uint8_t idx)
{
    return _color_index[idx];
}

void Arduino_Canvas_Indexed::raise_mask_level()
{
    if ((_current_mask_level + 1) < MAXMASKLEVEL)
    {
        int32_t buffer_size = _width * _height;
        uint8_t old_indexed_size = _indexed_size;
        uint8_t new_color;
        _indexed_size = 0;
        _color_mask = mask_level_list[++_current_mask_level];
        Serial.print("Raised mask level: ");
        Serial.println(_current_mask_level);

        // update _framebuffer color index, it is a time consuming job
        for (uint8_t old_color = 0; old_color < old_indexed_size; old_color++)
        {
            new_color = get_color_index(_color_index[old_color]);
            for (int32_t i = 0; i < buffer_size; i++)
            {
                if (_framebuffer[i] == old_color)
                {
                    _framebuffer[i] = new_color;
                }
            }
        }
    }
}

#endif // !defined(LITTLE_FOOT_PRINT)
