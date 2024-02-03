#ifndef CUBE_3D_H__
#define CUBE_3D_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const float CUBE_3D_ROTATE_SPEED = 0.02f;


class Cube3D : public BaseAnimation {
public:
	Cube3D() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  float _phase = 0;
	float _audio = 0;
  uint_fast16_t _bgColor;
};

void Cube3D::init( ILI9341_t3 tft ) {
  _bgColor = tft.color565( 0, 0, 0 );
}

uint_fast16_t Cube3D::bgColor(){
	return _bgColor;
}

String Cube3D::title() {
	return "Cube3D";
}

void Cube3D::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();

	uint_fast16_t w_2 = (w>>1);
	uint_fast16_t h_2 = (h>>1);

	float oldPhase = _phase;
	float oldAudio = _audio;
	float oldCos = cos( oldPhase );
	float oldSin = sin( oldPhase );

  _phase += frameParams.timeMult * CUBE_3D_ROTATE_SPEED;
	_audio = frameParams.audioMean;
	float pCos = cos( _phase );
	float pSin = sin( _phase );

	//uint_fast16_t border = tft.color565( 0x08, 0, 0 );
	uint_fast16_t eraseColor = tft.color565( 0, 0, 0 );

	// Rotating cube yo
	for( float x=-1.0f; x<=1.0f; x+=0.333f ) {
		for( float y=-1.0f; y<=1.0f; y+=1.0f ) {
			for( float z=-1.0f; z<=1.0f; z+=1.0f ) {

				// Audio hash
				float hash = 3.7f*x + 5.1f*y + 9.8f*z;
				hash -= floor(hash);
				hash = (hash*2.0) - 1.0;	// bipolar signal, -1..1

				// Erase the old point
				float x3d = oldCos*x - oldSin*z;
				float y3d = y + hash*oldAudio;
				float z3d = 3.0 + oldCos*z + oldSin*x;

				Point16 pt = xyz2screen( x3d, y3d, z3d, w_2, h_2 );

				//tft.drawCircle( pt.x, pt.y, 5, border );
				if( pt.y >= 0 ) {	// sanity check
					tft.drawCircle( pt.x, pt.y, 7.0-z3d, eraseColor );
				}

				x3d = pCos*x - pSin*z;
				y3d = y + hash*_audio;
				z3d = 3.0 + pCos*z + pSin*x;

				pt = xyz2screen( x3d, y3d, z3d, w_2, h_2 );

				if( pt.y >= 0 ) {	// sanity check
					float bright = ((3.0f+M_SQRT2)-z3d) * (1.0f/(2*M_SQRT2));	// range 0..1, higher number == closer
					uint_fast16_t fillColor = tft.color565( 0x44, lerp8(0x66,0xff,bright), 0x44 );

					tft.drawCircle( pt.x, pt.y, 7.0-z3d, fillColor );
				}

			}
		}
	}

}

#endif
