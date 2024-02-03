#ifndef TRANSITION_DITHER_H__
#define TRANSITION_DITHER_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseTransition.h"


const float TRANSITION_DITHER_SPEED = 0.015f;


class TransitionDither : public BaseTransition {
public:
	TransitionDither() : BaseTransition() {};

  void init( ILI9341_t3 tft );
	void restart( ILI9341_t3 tft, uint_fast16_t color );
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );
  boolean isComplete();

private:
	float _phase = 0;
	uint_fast16_t _color;
	uint_fast8_t _step;
};

void TransitionDither::init( ILI9341_t3 tft ) {

}

void TransitionDither::restart( ILI9341_t3 tft, uint_fast16_t inColor ) {
  //uint_fast16_t w = tft.width();
  //uint_fast16_t h = tft.height();
	_phase = 0;
	_color = inColor;
	_step = 0;
}

void TransitionDither::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();

	_phase += frameParams.timeMult * TRANSITION_DITHER_SPEED;

	// Apply some easing. Linear speed feels dull
	float easeOutQuad = 1.0f - (1.0f-_phase)*(1.0f-_phase);

	// Calculate destination
	float dest_f = min( easeOutQuad, 1.0f ) * 0xff;
	uint_fast8_t dest = floor(dest_f);

	// Draw dither dots until _step reaches the destination
	while( _step <= dest ) {

		// Bayer matrix. See: https://en.wikipedia.org/wiki/Ordered_dithering
		// Recreate Bayer matrix pattern, basically a recursive sequence of 2D moves:  [ 0, 3,
		//                                                                               2, 1 ]
		uint_fast16_t start_i = 0;
		uint_fast16_t start_j = 0;

		uint_fast8_t move = (1<<3);
		uint_fast8_t s = _step;
		while( s > 0 ) {
			if( s&0b01 ) start_i += move;	// 1 & 3: move horizontal
			if( (s&0b01) ^ ((s&0b10)>>1) ) start_j += move;	// 1 & 2: move vertical

			move >>= 1;
			s >>= 2;
		}

		for( uint_fast16_t i=start_i; i<w; i+=0x10 ) {
			for( uint_fast16_t j=start_j; j<h; j+=0x10 ) {
				tft.drawPixel( i, j, _color );
			}
		}

		_step++;

	}

}

boolean TransitionDither::isComplete() {
  return _phase >= 1.0f;
}

#endif
