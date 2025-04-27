#ifndef BASE_ANIMATION_H__
#define BASE_ANIMATION_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"

class BaseAnimation {
public:
	BaseAnimation(){};

	virtual void init( ILI9341_t3 tft );
	virtual uint_fast16_t bgColor( void );
	virtual void reset( ILI9341_t3 tft );
	virtual String title();

	virtual boolean willForceTransition( void );
	virtual boolean forceTransitionNow( void );

	virtual void perFrame( ILI9341_t3 tft, FrameParams frameParams );
};

void BaseAnimation::init( ILI9341_t3 tft ) {
	// Extend me
}

uint_fast16_t BaseAnimation::bgColor( void ) {
	// Extend me
	return 0xf81f;	// Everyone loves magenta
}

void BaseAnimation::reset( ILI9341_t3 tft ) {
	// Extend me
}

String BaseAnimation::title() {
	return "BaseAnimation";
}

boolean BaseAnimation::willForceTransition( void ) {
	return false;	// Default: SuperTFT will transition animations automatically
}

boolean BaseAnimation::forceTransitionNow( void ) {
	// Extend me
	return false;	// Default: SuperTFT will transition animations automatically
}

void BaseAnimation::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
	// Extend me
}

#endif
