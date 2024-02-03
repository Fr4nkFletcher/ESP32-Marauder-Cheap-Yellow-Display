#ifndef TRANSITION_HALFTONE_H__
#define TRANSITION_HALFTONE_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseTransition.h"


const float HALFTONE_SPEED = 0.06f;
const float HALFTONE_SLOPE = 0.5f;
const int_fast8_t HALFTONE_CIRC_SIZE = 15;
const uint_fast8_t HALFTONE_CIRC_SPACING = (uint_fast8_t)(HALFTONE_CIRC_SIZE * cos(M_PI/6.0f)) * 2 - 1;


class TransitionHalftone : public BaseTransition {
public:
	TransitionHalftone() : BaseTransition() {};

  void init( ILI9341_t3 tft );
	void restart( ILI9341_t3 tft, uint_fast16_t color );
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );
  boolean isComplete();

private:
	float _phase = 0;
	uint_fast16_t _color;
  boolean _isComplete = false;
};

void TransitionHalftone::init( ILI9341_t3 tft ) {

}

void TransitionHalftone::restart( ILI9341_t3 tft, uint_fast16_t inColor ) {
  //uint_fast16_t w = tft.width();
  //uint_fast16_t h = tft.height();
	_phase = 0;
	_color = inColor;
  _isComplete = false;
}

void TransitionHalftone::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();
	uint_fast16_t w_2 = (w>>1);
	uint_fast16_t h_2 = (h>>1);

	float _prevPhase = _phase;
	_phase += frameParams.timeMult * HALFTONE_SPEED;

  boolean anySmallCircles = false;

	// Draw circes, offset so circles grow out from center
  int_fast8_t across = (w / (HALFTONE_CIRC_SPACING*cos(M_PI/6.0f))) + 1;
	int_fast8_t across_2 = (across>>1);
  int_fast8_t down = h / HALFTONE_CIRC_SPACING + 1;
	int_fast8_t down_2 = (down>>1)+1;

  for( int_fast8_t i=-across_2; i<=across_2; i++ ) {
    for( int_fast8_t j=-down_2; j<=down_2; j++ ) {

			float i_f = (float)(i);
			float j_f = (float)(j);
			if( i & 0x1 ) j_f -= 0.5f;	// Stagger odd columns, so a triangular lattice is formed

			float distance = sqrtf( i_f*i_f + j_f*j_f );

			int_fast8_t prevSize = HALFTONE_CIRC_SIZE * constrain( _prevPhase - distance*HALFTONE_SLOPE, 0.0f, 1.0f );

      int_fast8_t size = HALFTONE_CIRC_SIZE * constrain( _phase - distance*HALFTONE_SLOPE, 0.0f, 1.0f );
      if( size < HALFTONE_CIRC_SIZE ) anySmallCircles = true;

			// Only redraw circles that change size
			if( size > prevSize ) {
				int_fast16_t x = w_2 + i_f*HALFTONE_CIRC_SPACING;
				int_fast16_t y = h_2 + j_f*HALFTONE_CIRC_SPACING;
				tft.fillCircle( x, y, size, _color );

				// Circle drawing issue: Circles at the top of the screen aren't filling correctly,
				// so just draw a box there :P
				if( j == -down_2 ) {
					tft.fillRect( max( x-size, 0 ), max( y-size, 0 ), size*2, size*2, _color );
				}
			}

    }
  }

  // When all circles are full-size, the transition is done!
	_isComplete = !anySmallCircles;
}

boolean TransitionHalftone::isComplete() {
  return _isComplete;
}

#endif
