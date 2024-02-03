#ifndef WAVEFORM_H__
#define WAVEFORM_H__

#include <Arduino.h>
#include <math.h>
#include "ILI9341_t3.h"
#include "BaseAnimation.h"


class Waveform : public BaseAnimation {
public:
	Waveform() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  uint_fast16_t _step = 0;
  uint_fast8_t _colorPhase = 0;
  uint_fast16_t _bgColor;
};

void Waveform::init( ILI9341_t3 tft ) {
  _bgColor = tft.color565( 0, 0, 0x55 );
}

uint_fast16_t Waveform::bgColor( void ) {
	return _bgColor;
}

String Waveform::title() {
	return "Waveform";
}

void Waveform::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = tft.width();
  uint_fast16_t h = tft.height();

  // fillRect: flickers pretty bad
  //tft.fillRect( 0, 0, w, h, LV_RED );

  // Prepare body color
  uint_fast8_t bright = (frameParams.audioPeak >> 1); // 0..512 -> 0..255
  uint_fast16_t bodyColor = tft.color565( bright, bright, 0 );  // yellow

  // Prepare body height
  uint_fast16_t h_2 = (h >> 1);
  uint_fast16_t bodyHeight = frameParams.audioMean * h;
  tft.drawFastVLine( _step, h_2-(bodyHeight>>1), bodyHeight, bodyColor );

  // Clear this column. Background should have a triangle wave effect
  uint_fast8_t bgBright = _colorPhase + _step;
  if( bgBright & 0x80 ) {
    bgBright = (0xff-bgBright) << 1;
  } else {
    bgBright <<= 1;
  }
  uint_fast16_t bgColor = tft.color565( 0, 0, bgBright );

  // Draw background color in the other pixels
  uint_fast16_t margin = (h - bodyHeight) >> 1;
  tft.drawFastVLine( _step, 0,          margin, bgColor );
  tft.drawFastVLine( _step, h - margin, margin, bgColor );

  // Advance to next line
  if( _step == 0 ) {
    _step = w - 1; // Restart drawing on right-hand size

    // After the screen is covered: Advance the background color
    _colorPhase += 3;

  } else {
    _step--;
  }

}

#endif
