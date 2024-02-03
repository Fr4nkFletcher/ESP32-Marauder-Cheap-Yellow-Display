#ifndef TRIANGLE_WEB_H__
#define TRIANGLE_WEB_H__

#include <Arduino.h>
#include <math.h>
#include "ILI9341_t3.h"
#include "BaseAnimation.h"


const uint_fast8_t WEB_POINT_SPACING = 64;
const float WEB_POINT_RADIUS = 20.0f;


class TriangleWeb : public BaseAnimation {
public:
	TriangleWeb() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  Point getWebPoint( uint_fast8_t i, uint_fast8_t j, float phase );

  uint_fast8_t _ptsAcross = 0;
  uint_fast8_t _ptsDown = 0;
  float _phase = 0;
  uint_fast16_t _bgColor;
};

void TriangleWeb::init( ILI9341_t3 tft ) {
  uint_fast16_t w = tft.width();
  uint_fast16_t h = tft.height();
  //tft.fillRect( 0, 0, w, h, 0x0 );

  _ptsAcross = w / WEB_POINT_SPACING + 1;
  _ptsDown = h / WEB_POINT_SPACING + 1;

  _bgColor = tft.color565( 0x22, 0x22, 0x22 );
}

uint_fast16_t TriangleWeb::bgColor(){
	return _bgColor;
}

String TriangleWeb::title() {
	return "TriangleWeb";
}

Point TriangleWeb::getWebPoint( uint_fast8_t i, uint_fast8_t j, float phase ) {
  uint_fast8_t idx = j*_ptsAcross + i;
  uint_fast8_t rando = (idx ^ 37);
  float angle = rando * phase;
  return (Point){
    (uint_fast16_t)( (i*WEB_POINT_SPACING) + (cos(angle)*WEB_POINT_RADIUS) ),
    (uint_fast16_t)( (j*WEB_POINT_SPACING) + (sin(angle)*WEB_POINT_RADIUS) )
   };
}

void TriangleWeb::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  //uint_fast16_t w = tft.width();
  //uint_fast16_t h = tft.height();

  // fillRect: flickers pretty bad
  //tft.fillRect( 0, 0, w, h, LV_RED );

  _phase += frameParams.timeMult * 0.0005 * (frameParams.audioMean*3.0f + 1.0f);

  // Prepare web color
  uint_fast8_t bright = (frameParams.audioPeak >> 1); // 0..512 -> 0..255

	// Weight brighter
	uint_fast16_t invBrightSquared = (0xff-bright)*(0xff-bright);
	bright = lerp8( bright, 0xff - (invBrightSquared>>8), 0.55f );

  uint_fast16_t color = tft.color565( bright, bright, bright );

  // When dark: Always illuminate one "lucky" line at random
  uint_fast16_t luckyLine = random( _ptsAcross * _ptsDown * 3 );
  uint_fast8_t luckyBright = random( 0x11, 0x55 );
  uint_fast16_t luckyColor = tft.color565( luckyBright, luckyBright, luckyBright );

  // It is possible to re-use some Point's here. TODO?
  Point nw, ne, sw;
  uint_fast16_t luckyIdx = 0;
  for( uint_fast8_t i=0; i<_ptsAcross; i++ ) {
    nw = getWebPoint( i, 0, _phase );

    for( uint_fast8_t j=0; j<_ptsDown; j++ ) {
      ne = getWebPoint( i+1, j, _phase );
      sw = getWebPoint( i, j+1, _phase );

      uint_fast16_t useColor;

      useColor = (luckyIdx==luckyLine) ? luckyColor : color;
      tft.drawLine( nw.x, nw.y, ne.x, ne.y, useColor );
      luckyIdx++;

      useColor = (luckyIdx==luckyLine) ? luckyColor : color;
      tft.drawLine( nw.x, nw.y, sw.x, sw.y, useColor );
      luckyIdx++;

      useColor = (luckyIdx==luckyLine) ? luckyColor : color;
      tft.drawLine( ne.x, ne.y, sw.x, sw.y, useColor );
      luckyIdx++;

      // Re-use sw point as new nw
      nw = sw;
    }
  }

}

#endif
