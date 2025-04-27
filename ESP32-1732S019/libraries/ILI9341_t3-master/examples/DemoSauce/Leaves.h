#ifndef LEAVES_H__
#define LEAVES_H__

#include <Arduino.h>
#include <math.h>
#include "ILI9341_t3.h"
#include "BaseAnimation.h"


const uint_fast8_t LV_SIZE = 5;
const float LV_SPREAD_RADIUS = 100.0f;
const float LV_ITERS = 2;
const float LV_SPEED = 5.2;
const uint_fast16_t LV_RED = 0xf800;
const uint_fast16_t FADE_RED = 0x1800;


class Leaves : public BaseAnimation {
public:
	Leaves() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  void _drawLeaves( ILI9341_t3 tft, boolean doErase, uint_fast8_t iter, float radius, float spin, float x, float y, uint_fast16_t solidColor, uint_fast16_t outlineColor );

  float _phase = 0;
  uint_fast16_t _bgColor;
};

void Leaves::init( ILI9341_t3 tft ) {
  _bgColor = 0x780f;
}

uint_fast16_t Leaves::bgColor(){
	return _bgColor;
}

String Leaves::title() {
	return "Leaves";
}

void Leaves::_drawLeaves( ILI9341_t3 tft, boolean doErase, uint_fast8_t iter, float radius, float spin, float x, float y, uint_fast16_t solidColor, uint_fast16_t outlineColor ) {
  float radius_2 = radius * 0.5;
  float angle = M_PI + (spin * (iter+0.7));
  for( uint_fast8_t i=0; i<3; i++ ) {
    if( iter==0 ) {
      tft.drawCircle( x + cos(angle)*radius, y + sin(angle)*radius, LV_SIZE+2, outlineColor );
      tft.drawCircle( x + cos(angle)*radius, y + sin(angle)*radius, LV_SIZE+1, outlineColor );
      tft.fillCircle( x + cos(angle)*radius, y + sin(angle)*radius, LV_SIZE, solidColor );

    } else {
      _drawLeaves( tft, doErase, iter-1, radius_2, angle + i*0.2, x + cos(angle)*radius, y + sin(angle)*radius, solidColor, outlineColor );
    }

    angle += M_PI * (2.0/7.0);
  }
}

void Leaves::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = tft.width();
  uint_fast16_t h = tft.height();

  // fillRect: flickers pretty bad
  //tft.fillRect( 0, 0, w, h, LV_RED );

  float amp_f = frameParams.audioMean;  // range [0..1]

  _phase += (amp_f*amp_f) * LV_SPEED;

  uint_fast8_t bright = (frameParams.audioPeak >> 1); // 0..512 -> 0..255
  uint_fast16_t solidColor = tft.color565( 0xff, bright, bright );  // red
  uint_fast16_t outlineColor = tft.color565( bright, bright, bright ); // grey
  _drawLeaves( tft, false, LV_ITERS, LV_SPREAD_RADIUS, _phase, w/2, h/2, solidColor, outlineColor );
}

#endif
