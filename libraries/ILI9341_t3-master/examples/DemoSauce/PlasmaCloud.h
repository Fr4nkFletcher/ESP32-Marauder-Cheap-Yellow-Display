#ifndef PLASMA_CLOUD_H__
#define PLASMA_CLOUD_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const float PLASMA_CLOUD_SPEED = 0.02;
const uint_fast16_t PLASMA_CLOUD_MARGIN = 25;

const uint_fast8_t PLASMA_CLOUD_LINE_WIDTH = 8;
const uint_fast8_t PLASMA_CLOUD_STEP_Y = 4;

const float PLASMA_COLOR_CYCLE_SPEED = 0.003;

const uint_fast8_t SQRT_TABLE_LENGTH = 255;
uint_fast8_t sqrtTable[ SQRT_TABLE_LENGTH ];
uint_fast8_t sqrtBitShift;	// Shift distances (integers) this many bits. This will give you a lookup index to get the square root.

class PlasmaCloud : public BaseAnimation {
public:
	PlasmaCloud() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
	float _phase = 0;
	float _colorCycle = 0;
	uint_fast8_t _ditherY = 0;
  uint_fast16_t _bgColor;
};

void PlasmaCloud::init( ILI9341_t3 tft ) {
	_bgColor = tft.color565( 0x77, 0, 0xcc );

  float w = (float)tft.width();
  float h = (float)tft.height();

	float maxScreenDistance = (w*w) + (h*h);
	uint_fast8_t nextPow2 = ceil( log2f( maxScreenDistance ) );
	float maxTableDistance = pow( 2.0f, nextPow2 );
	sqrtBitShift = nextPow2 - 8;

	float sqrtTableMult = (float)0xff / sqrt(maxScreenDistance);

	for( uint_fast8_t i=0; i<SQRT_TABLE_LENGTH; i++ ) {
		float useDistance = ((float)i / (float)SQRT_TABLE_LENGTH) * maxTableDistance;
		float aSqrt = sqrt( useDistance );
		sqrtTable[i] = (uint_fast8_t)(aSqrt * sqrtTableMult);
		//sqrtTable[i] = random(0xff);
	}
}

uint_fast16_t PlasmaCloud::bgColor( void ) {
	return _bgColor;
}

String PlasmaCloud::title() {
	return "PlasmaCloud";
}

void PlasmaCloud::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (int_fast16_t)tft.width();
  uint_fast16_t h = (int_fast16_t)tft.height();

	_phase += frameParams.timeMult * PLASMA_CLOUD_SPEED * (1.0f + frameParams.audioMean*5.0f);

	_colorCycle -= frameParams.timeMult * PLASMA_COLOR_CYCLE_SPEED * (1.0f + frameParams.audioMean*6.0f);
	if( _colorCycle <= 0.0f ) _colorCycle += 1.0f;
	uint_fast8_t colorAdd = _colorCycle * 0xff;

	//py._ditherY = (py._ditherY + 1) % PLASMA_YELLOW_DITHER;

	PointU16 p0 = (PointU16){
		(uint_fast16_t)(w/2 + (sinf(_phase*0.32f)*(w/2-PLASMA_CLOUD_MARGIN) )),
		(uint_fast16_t)(h/2 + (sinf(_phase*0.23f)*(h/2-PLASMA_CLOUD_MARGIN) ))
	};
	PointU16 p1 = (PointU16){
		(uint_fast16_t)(w/2 + (cosf(_phase*1.07f)*(w/2-PLASMA_CLOUD_MARGIN) )),
		(uint_fast16_t)(h/2 + (cosf(_phase*1.42f)*(h/2-PLASMA_CLOUD_MARGIN) ))
	};
	/*
	PointU16 p2 = (PointU16){
		(uint_fast16_t)(w/2 + (cos(_phase*0.57f)*(w/2-PLASMA_CLOUD_MARGIN) )),
		(uint_fast16_t)(h/2 + (cos(_phase*0.81f)*(h/2-PLASMA_CLOUD_MARGIN) ))
	};
	*/

	for( uint_fast16_t x=0; x<w; x+=PLASMA_CLOUD_LINE_WIDTH ) {
		for( uint_fast16_t y=_ditherY; y<h; y+=PLASMA_CLOUD_STEP_Y ) {
			//PointU8 d0 = (PointU8){ abs(p0.x - x), abs(p0.y - y) };
			//PointU8 d1 = (PointU8){ abs(p1.x - x), abs(p1.y - y) };
			//PointU8 d2 = (PointU8){ abs(p2.x - x), abs(p2.y - y) };
			// newer compiler gives abs ambiguous overload error
			// simply do this math as int to work around the issue
			int d0_x = (int)p0.x - (int)x;
			int d0_y = (int)p0.y - (int)y;
			int d1_x = (int)p1.x - (int)x;
			int d1_y = (int)p1.y - (int)y;

			uint_fast8_t lookup0 = (d0_x*d0_x + d0_y*d0_y) >> sqrtBitShift;
			uint_fast8_t lookup1 = (d1_x*d1_x + d1_y*d1_y) >> sqrtBitShift;
			//uint_fast8_t lookup2 = (d2.x*d2.x + d2.y*d2.y) >> sqrtBitShift;

			uint_fast8_t bright = (sqrtTable[ lookup0 ] * sqrtTable[ lookup1 ] ) >> 6;
			bright = (uint_fast16_t)(bright + colorAdd) & 0xff;

			uint_fast16_t color = tft.color565( ((bright&0xfb)<<2), 0, bright );
			tft.drawFastHLine( x, y, PLASMA_CLOUD_LINE_WIDTH, color );
		}
	}

	_ditherY = (_ditherY + 1) % PLASMA_CLOUD_STEP_Y;

}

#endif
