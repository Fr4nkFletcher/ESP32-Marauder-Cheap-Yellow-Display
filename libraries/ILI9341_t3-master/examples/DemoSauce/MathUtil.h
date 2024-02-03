#ifndef MATH_UTIL_H__
#define MATH_UTIL_H__

struct Point16 {
	int_fast16_t x;
	int_fast16_t y;
};

struct PointU16 {
	uint_fast16_t x;
	uint_fast16_t y;
};

struct PointU8 {
	uint_fast8_t x;
	uint_fast8_t y;
};

struct PointF {
	float x;
	float y;
};

uint_fast8_t lerp8( uint_fast8_t a, uint_fast8_t b, float progress ){
	// Cast to int, avoid horrible values when b-a is less than zero
	return a + (int_fast8_t)((int_fast8_t)b-(int_fast8_t)a)*progress;
}

float lerp( float a, float b, float progress ){
	return a + (b-a)*progress;
}

// x=0,y=0 returns center of screen.
// z=1: Normalized coordinates with screen (x==-1 is left, x==1 is right, etc.)
// screenW_2 and screenH_2 are __half__ the screen size, and it's your job to pass in these numbers
// because you're probably re-using them for several points on the other end :P
// Assumes landscape orientation, so screenW_2 is greater than screenH_2.
Point16 xyz2screen( float x, float y, float z, uint_fast16_t screenW_2, uint_fast16_t screenH_2 ) {
	float invZ = 1.0f / z;

	return (Point16){
		(int_fast16_t)(screenW_2 + x*invZ*screenW_2),
		(int_fast16_t)(screenH_2 + y*invZ*screenW_2)
	};
}

#endif
