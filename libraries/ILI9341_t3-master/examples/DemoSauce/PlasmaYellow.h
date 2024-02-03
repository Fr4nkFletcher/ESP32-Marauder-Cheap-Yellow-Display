#ifndef PLASMA_YELLOW_H__
#define PLASMA_YELLOW_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const float PLASMA_YELLOW_SPEED = 0.02;
const uint_fast8_t PLASMA_YELLOW_DITHER = 8;
const uint_fast16_t PLASMA_YELLOW_MARGIN = 25;


class PlasmaYellow : public BaseAnimation {
public:
	PlasmaYellow() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  float _phase = 0;
	uint_fast8_t _ditherY = 0;
  uint_fast16_t _bgColor;
};

void PlasmaYellow::init( ILI9341_t3 tft ) {
  _bgColor = tft.color565( 0xff, 0xff, 0 );
}

uint_fast16_t PlasmaYellow::bgColor(){
	return _bgColor;
}

String PlasmaYellow::title() {
	return "PlasmaYellow";
}

void PlasmaYellow::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  int_fast16_t w = (int_fast16_t)tft.width();
  int_fast16_t h = (int_fast16_t)tft.height();

	_phase += frameParams.timeMult * PLASMA_YELLOW_SPEED;

	_ditherY = (_ditherY + 1) % PLASMA_YELLOW_DITHER;

	Point16 p0 = (Point16){
		(int_fast16_t)(w/2 + (sin(_phase*0.57f)*(w/2-PLASMA_YELLOW_MARGIN) )),
		(int_fast16_t)(h/2 + (sin(_phase*0.23f)*(h/2-PLASMA_YELLOW_MARGIN) ))
	};
	Point16 p1 = (Point16){
		(int_fast16_t)(w/2 + (cos(_phase*0.78f)*(w/2-PLASMA_YELLOW_MARGIN) )),
		(int_fast16_t)(h/2 + (cos(_phase*0.42f)*(h/2-PLASMA_YELLOW_MARGIN) ))
	};

	float audioPower = frameParams.audioMean;

	for( int_fast16_t x=0; x<w; x+=PLASMA_YELLOW_DITHER ) {
		for( int_fast16_t y=_ditherY; y<h; y+=PLASMA_YELLOW_DITHER ) {
			Point16 d0 = (Point16){ p0.x - x, p0.y - y };
			Point16 d1 = (Point16){ p1.x - x, p1.y - y };

			//uint_fast16_t distance = sqrt( d0.x*d0.x + d0.y*d0.y );// * sqrt( d1.x*d1.x + d1.y*d1.y );	// SLOW
			//uint_fast16_t distance = abs( d0.x*d0.y );	// pretty good
			uint_fast16_t distance = abs( d0.x*d0.y + d1.x*d1.y );

			uint_fast8_t bright = lerp8( (uint_fast16_t)(distance >> 5) & 0xff, 0xff, audioPower );
			if( bright > 0x7f ) bright = 0xff - bright;
			bright <<= 2;

			uint_fast16_t color = tft.color565( bright, bright, 0 );

			tft.drawFastHLine( x, y, PLASMA_YELLOW_DITHER, color );
		}
	}
}

#endif
