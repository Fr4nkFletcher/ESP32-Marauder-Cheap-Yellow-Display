#ifndef SPHERE_3D_H__
#define SPHERE_3D_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "BaseAnimation.h"


const float SPHERE_3D_ROTATE_SPEED = 0.005f;
const float SPHERE_3D_TILT_SPEED = 2.0f;
const float SPHERE_3D_TILT_AMOUNT = 0.4f;

// 3D render
const float SPHERE_DISTANCE = 2.5f;
const float SPHERE_OUTER_MULT = 1.414f;  // spike size


class Sphere3D : public BaseAnimation {
public:
	Sphere3D() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
  void _drawLine( ILI9341_t3 tft, float cosTilt, float sinTilt, float x, float y, float z, uint_fast16_t w_2, uint_fast16_t h_2, uint_fast16_t color );

  float _rotatePhase = 0;
  uint_fast16_t _baseCircSize = 0;
  uint_fast16_t _circStep = 0;
  uint_fast8_t _sparkle = 0;

  uint_fast16_t _bgColor;
};

void Sphere3D::init( ILI9341_t3 tft ) {
  uint_fast16_t w = tft.width();
  uint_fast16_t h = tft.height();

  //_bgColor = tft.color565( 0, 0x44, 0x44 ); // Only looks good in low light :P
  _bgColor = tft.color565( 0, 0, 0 );

  Point16 sphereTopOnScreen = xyz2screen( 0, -1.0f, SPHERE_DISTANCE, (w>>1), (h>>1) );
  _baseCircSize = (h/2) - sphereTopOnScreen.y + 1;
}

uint_fast16_t Sphere3D::bgColor(){
	return _bgColor;
}

String Sphere3D::title() {
	return "Sphere3D";
}

void Sphere3D::_drawLine( ILI9341_t3 tft, float cosTilt, float sinTilt, float x, float y, float z, uint_fast16_t w_2, uint_fast16_t h_2, uint_fast16_t color ) {
  // Tilt!
  float tempY = y;
  y = tempY*cosTilt + z*sinTilt;
  z = z*cosTilt - tempY*sinTilt;

	Point16 innerPt = xyz2screen( x, y, z+SPHERE_DISTANCE, w_2, h_2 );
	Point16 outerPt = xyz2screen( x*SPHERE_OUTER_MULT, y*SPHERE_OUTER_MULT, z*SPHERE_OUTER_MULT + SPHERE_DISTANCE, w_2, h_2 );
	tft.drawLine( innerPt.x, innerPt.y, outerPt.x, outerPt.y, color );

	// Just for kicks... Let's draw some circumference lines
	const float T_BAR_RADIUS = 0.1f;
	Point16 p0 = xyz2screen( x-z*T_BAR_RADIUS, y, z+x*T_BAR_RADIUS + SPHERE_DISTANCE, w_2, h_2 );
	Point16 p1 = xyz2screen( x+z*T_BAR_RADIUS, y, z-x*T_BAR_RADIUS + SPHERE_DISTANCE, w_2, h_2 );
	tft.drawLine( p0.x, p0.y, p1.x, p1.y, color );

}

void Sphere3D::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
  uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();

	uint_fast16_t w_2 = (w>>1);
	uint_fast16_t h_2 = (h>>1);

	float oldPhase = _rotatePhase;
  _rotatePhase += frameParams.timeMult * SPHERE_3D_ROTATE_SPEED;

	_sparkle = max( (frameParams.audioPeak >> 1), _sparkle * (1.0f-(frameParams.timeMult*0.02f)) );
	uint_fast16_t erase = _bgColor;

	float x, y, z, sinLat;

  float oldTilt = -sin( oldPhase * SPHERE_3D_TILT_SPEED ) * SPHERE_3D_TILT_AMOUNT;
  float oldCosTilt = cos( oldTilt );
  float oldSinTilt = sin( oldTilt );

  float tilt = -sin( _rotatePhase * SPHERE_3D_TILT_SPEED ) * SPHERE_3D_TILT_AMOUNT;
  float cosTilt = cos( tilt );
  float sinTilt = sin( tilt );

	// Rotating sphere yo
	for( float lon=0.0f; lon<(M_PI*0.5f); lon+=(M_PI*0.125f) ) {	// longitude (around). Only 1/4 of sphere circumference.
		for( float lat=(M_PI*0.0625f); lat<(M_PI*0.5f); lat+=(M_PI*0.125f) ) {	// latitude (up & down). Only 1/2 of sphere height.

			// Erase the old line here
			x = cos( oldPhase + lon );
			z = sin( oldPhase + lon );

			y = cos( lat );
			sinLat = sin( lat );
			x *= sinLat;
			z *= sinLat;

			// We can swap & negate x,y,z to draw at least 8 lines without recomputing cos & sin values etc
			_drawLine( tft, oldCosTilt, oldSinTilt, x, y, z, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, -x, y, -z, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, -z, y, x, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, z, y, -x, w_2, h_2, erase );

			// Draw the other hemisphere
			_drawLine( tft, oldCosTilt, oldSinTilt, x, -y, z, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, -x, -y, -z, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, -z, -y, x, w_2, h_2, erase );
			_drawLine( tft, oldCosTilt, oldSinTilt, z, -y, -x, w_2, h_2, erase );

			// Now, draw the new lines

      uint_fast16_t color = tft.color565(
        random( _sparkle>>1, _sparkle ),
        random( _sparkle>>1, _sparkle ),
        0xff
      );

			x = cos( _rotatePhase + lon );
			z = sin( _rotatePhase + lon );

			// Now we need the y (up & down), then normalize x & z to create a normalized 3D vector (length == 1.0)
			y = cos( lat );
			sinLat = sin( lat );
			x *= sinLat;
			z *= sinLat;

			// We can swap & negate x,y,z to draw at least 8 lines without recomputing cos & sin values etc
			_drawLine( tft, cosTilt, sinTilt, x, y, z, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, -x, y, -z, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, -z, y, x, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, z, y, -x, w_2, h_2, color );

			// Draw the other hemisphere
			_drawLine( tft, cosTilt, sinTilt, x, -y, z, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, -x, -y, -z, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, -z, -y, x, w_2, h_2, color );
			_drawLine( tft, cosTilt, sinTilt, z, -y, -x, w_2, h_2, color );
		}
	}

	const uint_fast8_t RADIATION_PX = 12;

  _circStep += random(1,4);
  if( _circStep > RADIATION_PX ) _circStep -= RADIATION_PX;

	float brightAmt = ((frameParams.audioMean + 1.0f) * 0.5f);	// 0.5...1
	uint_fast8_t bright = (0xff/RADIATION_PX) * brightAmt;

	uint_fast16_t radiationColor = tft.color565( 0, random((RADIATION_PX-_circStep)*(bright>>1)), random((RADIATION_PX-_circStep)*bright) ) | _bgColor;
  tft.drawCircle( w_2, h_2, _baseCircSize + _circStep, radiationColor );
}

#endif
