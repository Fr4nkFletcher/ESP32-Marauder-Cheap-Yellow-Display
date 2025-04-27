#ifndef TRANSITION_SQUARES_H__
#define TRANSITION_SQUARES_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseTransition.h"


const float TRANSITION_SQUARES_SPEED = 0.06f;
const float TRANSITION_SQUARES_SLOPE = 0.08f;
const uint_fast8_t TRANSITION_SQ_SIZE = 30; // multiple of 2, please


class TransitionSquares : public BaseTransition {
public:
	TransitionSquares() : BaseTransition() {};

  void init( ILI9341_t3 tft );
	void restart( ILI9341_t3 tft, uint_fast16_t color );
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );
  boolean isComplete();

private:
	float _phase = 0;
	uint_fast16_t _color;
  boolean _isComplete = false;
};

void TransitionSquares::init( ILI9341_t3 tft ) {

}

void TransitionSquares::restart( ILI9341_t3 tft, uint_fast16_t inColor ) {
  //uint_fast16_t w = tft.width();
  //uint_fast16_t h = tft.height();
	_phase = 0;
	_color = inColor;
  _isComplete = false;
}

float easeInOutSine( float p ) {
  if( p < 0.5f ) return (1.0f - cos( p * M_PI )) * 0.5f;
  return 0.5f + ( sin( ( p - 0.5f ) * M_PI ) * 0.5f );
}

void TransitionSquares::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();

  float prevPhase = _phase;
	_phase += frameParams.timeMult * TRANSITION_SQUARES_SPEED;

  boolean anySmallSquares = false;

	// Draw squares, offset so squares grow out from center
  uint_fast8_t across = w / TRANSITION_SQ_SIZE + 1;
  uint_fast8_t down = h / TRANSITION_SQ_SIZE + 1;
  for( uint_fast8_t i=0; i<across; i++ ) {
    for( uint_fast8_t j=0; j<down; j++ ) {

      uint_fast8_t prevSize = TRANSITION_SQ_SIZE * easeInOutSine( constrain( prevPhase - (i+j)*TRANSITION_SQUARES_SLOPE, 0.0f, 1.0f ) );
      prevSize = (prevSize>>1) << 1;  // Round down to even number

      uint_fast8_t size = TRANSITION_SQ_SIZE * easeInOutSine( constrain( _phase - (i+j)*TRANSITION_SQUARES_SLOPE, 0.0f, 1.0f ) );
      size = (size>>1) << 1;  // Round down to even number
      if( size < TRANSITION_SQ_SIZE ) anySmallSquares = true;

      // Squares should grow out from center.
      // Draw hollow squares, so we only redraw pixels that change.
      for( uint_fast8_t s=prevSize+2; s<=size; s+=2 ) {
        tft.drawRect( (i+0.5f)*TRANSITION_SQ_SIZE - (s>>1), (j+0.5f)*TRANSITION_SQ_SIZE - (s>>1), s, s, _color );
      }

      //tft.fillRect( (i+0.5f)*TRANSITION_SQ_SIZE - (size>>1), (j+0.5f)*TRANSITION_SQ_SIZE - (size>>1), size, size, _color );
    }
  }

  // When all squares are full-size, the transition is done!
	_isComplete = !anySmallSquares;
}

boolean TransitionSquares::isComplete() {
  return _isComplete;
}

#endif
