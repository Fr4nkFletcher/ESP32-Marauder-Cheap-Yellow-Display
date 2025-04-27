#ifndef MAGENTA_SQUARES_H__
#define MAGENTA_SQUARES_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const uint_fast8_t SQ_SPEED = 0x06;
const uint_fast8_t SQ_SIZE = 16;
const uint_fast8_t RIPPLE_SPEED_MIN = 0x05;
const uint_fast8_t RIPPLE_SPEED_MAX = 0x0b;
const uint_fast8_t RIPPLE_STROKE = 0x20;
const float RIPPLE_AMP_LATCH = 1.5f;
const float RIPPLE_AMP_DECAY = 0.01f;


class MagentaSquares : public BaseAnimation {
public:
	MagentaSquares() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  inline float distance2D( float w, float h );

  uint_fast8_t _step = 0;
  float _audioPeak = 0;
  uint_fast8_t _ripplePos = 0;
  uint_fast8_t _rippleSpeed = 0;
  uint_fast16_t _bgColor;

  uint_fast8_t sqAcross;
  uint_fast8_t sqDown;
  uint_fast8_t * radii;
};

inline float MagentaSquares::distance2D( float w, float h ) {
  return sqrt( w*w + h*h );
}

void MagentaSquares::init( ILI9341_t3 tft ) {
  uint_fast16_t w = tft.width();
  uint_fast16_t h = tft.height();

  sqAcross = w / SQ_SIZE;
  sqDown = h / SQ_SIZE;

  radii = (uint_fast8_t*)malloc( sqAcross * sqDown * sizeof(uint_fast8_t) );

  float w_2 = w / 2;
  float h_2 = h / 2;
  float distanceMult = (255.0-RIPPLE_STROKE) / distance2D( w_2, h_2 );

  for( uint_fast8_t i=0; i<sqAcross; i++ ) {
    for( uint_fast8_t j=0; j<sqDown; j++ ) {
      uint_fast8_t idx = j*sqAcross + i;

      radii[idx] = distanceMult * distance2D( ((float)i-sqAcross/2)*SQ_SIZE, ((float)j-sqDown/2)*SQ_SIZE );
    }
  }

  _bgColor = tft.color565( 0x88, 0, 0x88 );
}

uint_fast16_t MagentaSquares::bgColor( void ) {
	return _bgColor;
}

String MagentaSquares::title() {
	return "MagentaSquares";
}

void MagentaSquares::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  //uint_fast16_t w = tft.width();
  //uint_fast16_t h = tft.height();

  // Trigger a new audio ripple?
  float amp_f = frameParams.audioMean;  // range [0..1]
  if( amp_f > _audioPeak ) {
    _ripplePos = 0.0; // reset the ripple

    _rippleSpeed = lerp( RIPPLE_SPEED_MIN, RIPPLE_SPEED_MAX, min( 1.0, amp_f/_audioPeak - 1.0 ) );

    // The next peak must be ~1.5x louder than this one to trigger a ripple
    _audioPeak = amp_f * RIPPLE_AMP_LATCH;
  }

  for( uint_fast8_t i=0; i<sqAcross; i++ ) {
    for( uint_fast8_t j=0; j<sqDown; j++ ) {

      uint_fast16_t x = i * SQ_SIZE;
      uint_fast16_t y = j * SQ_SIZE;

      // Redraw black borders at the edge of each square.
      //tft.drawRect( x, y, SQ_SIZE, SQ_SIZE, 0x0 );

      uint_fast16_t clr;

      // If this square is within the ripple area, draw using white
      uint_fast8_t idx = j*sqAcross + i;
      uint_fast8_t radius = radii[idx];
      if( (radius < _ripplePos) && (_ripplePos < radius+RIPPLE_STROKE) ) {
        clr = tft.color565( 0xff, random(0x99,0xff), 0xff );

      } else {
        uint_fast8_t bright = (x+y+_step) & 0xff;
        clr = tft.color565( bright, 0, bright );
      }

      uint_fast8_t rando = random(SQ_SIZE>>1);
      tft.drawRect( x+rando, y+rando, SQ_SIZE-2*rando, SQ_SIZE-2*rando, clr );
    }
  }

  //delay(10);

  _step += SQ_SPEED * frameParams.timeMult;
  _audioPeak *= (1.0 - RIPPLE_AMP_DECAY * frameParams.timeMult);
  _ripplePos = min( 0xff, (uint_fast16_t)_ripplePos + _rippleSpeed * frameParams.timeMult );
}

#endif
