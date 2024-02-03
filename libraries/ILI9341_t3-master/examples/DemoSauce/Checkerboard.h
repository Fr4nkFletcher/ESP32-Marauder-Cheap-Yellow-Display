#ifndef CHECKERBOARD_H__
#define CHECKERBOARD_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const uint_fast8_t CHECKERBOARD_SIZE_MIN = 20;
const uint_fast8_t CHECKERBOARD_SIZE_MAX = 60;

const float CHECKERBOARD_ZOOM_SPEED = 0.027;

const float CHECKERBOARD_STRAFE_DISTANCE = 5;  // in squares
const float CHECKERBOARD_STRAFE_SPEED_X = 0.015;
const float CHECKERBOARD_STRAFE_SPEED_Y = 0.0095;


class Checkerboard : public BaseAnimation {
public:
	Checkerboard() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  float _phase = 0;
  uint_fast16_t _bgColor;
};

void Checkerboard::init( ILI9341_t3 tft ) {
  _bgColor = tft.color565( 0xff, 0xbb, 0xbb );
}

uint_fast16_t Checkerboard::bgColor(){
	return _bgColor;
}

String Checkerboard::title() {
	return "Checkerboard";
}

void Checkerboard::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  int_fast16_t w = (int_fast16_t)tft.width();
  int_fast16_t h = (int_fast16_t)tft.height();

  _phase += frameParams.timeMult;

  float sizer = (cos(_phase*CHECKERBOARD_ZOOM_SPEED) + 1.0) * 0.5; // Range 0..1
  float size = lerp8( CHECKERBOARD_SIZE_MIN, CHECKERBOARD_SIZE_MAX, sizer );

  // Zoom in & out from center
  int_fast16_t strafeX = sin(_phase*CHECKERBOARD_STRAFE_SPEED_X) * CHECKERBOARD_STRAFE_DISTANCE * size;
  uint_fast8_t sqLeftOfCenter = ceil( (w/2 + strafeX) / (float)size );
  float startX = (w/2) - sqLeftOfCenter * size + strafeX;

  int_fast16_t strafeY = sin(_phase*CHECKERBOARD_STRAFE_SPEED_Y) * CHECKERBOARD_STRAFE_DISTANCE * size;
  uint_fast8_t sqAboveCenter = ceil( (h/2 + strafeY) / (float)size );
  float startY = (h/2) - sqAboveCenter * size + strafeY;

  // First square is light or 8dark?
  boolean startLight = (sqLeftOfCenter%2) ^ (sqAboveCenter%2);

	// Modulate colors based on audio input
	float ease = (1.0f-frameParams.audioMean);
	ease *= ease;	// Stronger reactions, please

	uint_fast8_t bright = (1.0f-ease) * 0xff;
	uint_fast16_t audioColor = tft.color565( bright, bright>>1, 0 );

  int_fast8_t sqX = -sqLeftOfCenter;
  for( float x=startX; x<w; x+=size ) {
    boolean isLight = startLight;

    int16_t drawX = max( x, 0 );
    int16_t drawW = (x >= 0) ? size : (size+x);

    int_fast8_t sqY = -sqAboveCenter;
    for( float y=startY; y<h; y+=size ) {
      int16_t drawY = max( y, 0 );
      int16_t drawH = (y >= 0) ? size : (size+y);

      uint_fast16_t color = tft.color565( (sqX * sqY * 57), (isLight ? 0xff : 0), (isLight ? 0xff : 0) );
			color ^= audioColor;

      tft.fillRect( drawX, drawY, drawW, drawH, color );
      isLight = !isLight;

      sqY++;
    }

    startLight = !startLight;
    sqX++;
  }

}

#endif
