#include "../Arduino_DataBus.h"
#if !defined(LITTLE_FOOT_PRINT)

#ifndef _ARDUINO_CANVAS_MONO_H_
#define _ARDUINO_CANVAS_MONO_H_

#include "../Arduino_GFX.h"

class Arduino_Canvas_Mono : public Arduino_GFX
{
public:
  Arduino_Canvas_Mono(int16_t w, int16_t h, Arduino_G *output, int16_t output_x = 0, int16_t output_y = 0);

  void begin(int32_t speed = GFX_NOT_DEFINED) override;
  void writePixelPreclipped(int16_t x, int16_t y, uint16_t color) override;
  void flush(void) override;

protected:
  uint8_t *_framebuffer;
  Arduino_G *_output;
  int16_t _output_x, _output_y;

private:
};

#endif // _ARDUINO_CANVAS_MONO_H_

#endif // !defined(LITTLE_FOOT_PRINT)
