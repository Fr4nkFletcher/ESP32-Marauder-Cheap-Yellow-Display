#ifndef TWISTY_TEXT_H__
#define TWISTY_TEXT_H__

#include <Arduino.h>
#include "ILI9341_t3.h"
#include "MathUtil.h"
#include "TwistyTextFont.h"
#include "BaseAnimation.h"


const uint_fast8_t TEXT_PIXEL_WIDTH = 4;
const float TEXT_PIXEL_HEIGHT = 12.0f;
const float TEXT_3D_THICKNESS_MULT = 2.0f;

const float LINE_SCROLL_SPEED = 0.004f;
const float TWIST_AMOUNT = -0.0063f;

const float WOBBLE_FREQ = 2.5f;
const float WOBBLE_AMOUNT = 0.35f;
const float WOBBLE_TORQUE = -0.19f;

//                    0.........10........20........30........40........
const char LINES[] = "HYDRONICS + ZKARCHER PRESENT: DEMOSAUCE!(){}(){}()";
const uint_fast8_t LINE_COUNT = 5;
const uint_fast8_t CHARS_PER_LINE = 10;


class TwistyText : public BaseAnimation {
public:
	TwistyText() : BaseAnimation() {};

	void init( ILI9341_t3 tft );
	uint_fast16_t bgColor( void );
	String title();
	void reset( ILI9341_t3 tft );

	boolean willForceTransition( void );
	boolean forceTransitionNow( void );

	void perFrame( ILI9341_t3 tft, FrameParams frameParams );

private:
	float _initPhase = 0;
	float _phase = 0;
	uint_fast16_t _bgColor;
	uint_fast8_t _meters[12];
	boolean _drawnColumns[CHARS_PER_LINE*7];
};

void TwistyText::init( ILI9341_t3 tft ) {
	_bgColor = tft.color565( 0x0, 0x0, 0x33 );
}

uint_fast16_t TwistyText::bgColor( void ) {
	return _bgColor;
}

String TwistyText::title() {
	return "TwistyText";
}

void TwistyText::reset( ILI9341_t3 tft ) {
	_phase = _initPhase = LINE_COUNT * random(999);

	for( uint_fast8_t m=0; m<12; m++ ) {
		_meters[m] = 0;
	}

	for( uint_fast8_t c=0; c<(CHARS_PER_LINE*7); c++ ) {
		_drawnColumns[c] = false;
	}
}

boolean TwistyText::willForceTransition( void ) {
	return true;
}

boolean TwistyText::forceTransitionNow( void ) {
	return _phase > (_initPhase + LINE_COUNT);
}

void TwistyText::perFrame( ILI9341_t3 tft, FrameParams frameParams ) {
	uint_fast16_t w = (uint_fast16_t)tft.width();
  uint_fast16_t h = (uint_fast16_t)tft.height();
	uint_fast16_t w_2 = (w>>1);
	uint_fast16_t h_2 = (h>>1);

	uint_fast16_t paddingLeft = ((w - CHARS_PER_LINE * TEXT_PIXEL_WIDTH * 7) >> 1);
	paddingLeft += TEXT_PIXEL_WIDTH;	// Because each character has 2 empty cols on the right

	_phase += frameParams.timeMult * LINE_SCROLL_SPEED;

	for( uint_fast8_t c=0; c<(CHARS_PER_LINE*7); c++ ) {
		uint_fast8_t columnInChar = c % 7;
		//if( columnInChar >= 5 ) continue;	// Ignore spacing between characters

		uint_fast8_t charInLine = (c / 7);

		// Get the angle of rotation
		float angle = (_phase + c*TWIST_AMOUNT);

		// I want to pause so the user can read each line, so add some easing to hang out at
		// (angle%1.0)==0.5.
		float angleFloor = floor(angle);
		float decimal = angle - angleFloor;
		if( decimal < 0.5f ) {
			float inverse = (0.5f-decimal) * 2.0f;	// 1..0
			inverse *= (inverse*inverse);
			angle = angleFloor + (1.0f-inverse)*0.5f;

		} else {
			float ramp = (decimal*2.0f) - 1.0f;	// 0..1
			ramp *= (ramp*ramp);
			angle = angleFloor + 0.5f + ramp*0.5f;
		}

		// Hang on the final line, the hearts. Don't roll over to the first line.
		if( angle > _initPhase + LINE_COUNT - 0.5f ) {
			angle = _initPhase + LINE_COUNT - 0.5f;
		}

		angle *= M_PI;

		// Add wiggly wobble
		angle += sin( _phase*M_PI*WOBBLE_FREQ + (c*WOBBLE_TORQUE) ) * WOBBLE_AMOUNT;

		uint_fast16_t wrapAmt = floor( angle / M_PI );
		angle -= wrapAmt * M_PI;

		// Get the text for this rotation
		uint_fast8_t lineIdx = wrapAmt % LINE_COUNT;
		char asciiValue = LINES[ lineIdx*CHARS_PER_LINE + charInLine ];

		uint_fast8_t fontIdx;
		switch( asciiValue ) {
			case '+':			fontIdx = 26; break;
			case ':':     fontIdx = 27; break;
			case '-':     fontIdx = 28; break;
			case '!':     fontIdx = 29; break;
			case '(':     fontIdx = 30; break;	// heart (left)
			case ')':     fontIdx = 31; break;	// heart (right)
			case '{':     fontIdx = 32; break;	// skull (left)
			case '}':     fontIdx = 33; break;	// skull (right)
			default:      fontIdx = asciiValue - 'A'; break;
		}

		uint_fast16_t charStart = fontIdx * 7;	// 7 columns per character

		uint_fast8_t colByte = 0;
		if( (0 <= charStart) && (charStart < FONT_TABLE_LENGTH) ) {
			colByte = pgm_read_byte( &FONT_TABLE[ charStart + columnInChar ] );
		}

		uint_fast16_t left = paddingLeft + c * TEXT_PIXEL_WIDTH;

		if( colByte == 0 ) {
			// Only erase columns if there's rects to erase from the last frame.
			if( _drawnColumns[c] ) {
				// *1.6: Cheap hack. Ensure rects from the previous frame are erased.
				tft.fillRect( left, h_2 - TEXT_PIXEL_HEIGHT*1.6, TEXT_PIXEL_WIDTH, TEXT_PIXEL_HEIGHT*TEXT_3D_THICKNESS_MULT*1.6, _bgColor );
				_drawnColumns[c] = false;
			}
			continue;
		}

		_drawnColumns[c] = true;

		// Prepare to erase the background
		uint_fast16_t eraseTop = h_2 - 4.5*TEXT_PIXEL_HEIGHT;

		float cosAngle = cos( angle );
		float cosAngleAbs = abs( cosAngle );
		float sinAngle = sin( angle );

		uint_fast16_t baseColor;
		switch( lineIdx ) {
			case 0:    baseColor = 0x22ffff; break;	// "HYDRONICS"
			case 1:    baseColor = 0xffff44; break; // "+ ZKARCHER"
			case 2:    baseColor = 0xff22ff; break; // "PRESENT"
			case 3:    baseColor = 0x33ff88; break; // "SUPER-TFT!";
			default:   baseColor = 0xbb0000; break; // hearts
		}

		// Skull == grey
		if( (fontIdx==32) || (fontIdx==33) ) {
			baseColor = 0xaaaaaa;
		}

		// Material color:
		//    cosAngleAbs: gives a color channel a more metallic appearance.
		//    sinAngle: more traditional lighting.
		//    tri: Happy medium.
		float tri = angle * (2.0f/M_PI);	// 0..2
		if( tri > 1.0 ) tri = 2.0f - tri;	// 0..1..0
		float triShiny = lerp( tri, tri*tri, 0.65f );	// Weight more towards grey. Material surface looks shiny.

		uint_fast16_t color = tft.color565(
			lerp8( 0x33, (baseColor&0xff0000)>>16, triShiny ),
			lerp8( 0x44, (baseColor&0x00ff00)>>8,  triShiny ),
			lerp8( 0x44, (baseColor&0x0000ff),     triShiny )
		);

		uint_fast16_t sideColor = tft.color565( 0xff * cosAngleAbs, 0x88 * cosAngleAbs, 0 );

		uint_fast16_t sideHeight = abs( cosAngle ) * TEXT_PIXEL_HEIGHT * TEXT_3D_THICKNESS_MULT;

		// Draw a minimal number of rects. Advance from top to bottom. Track when rects start & end.
		boolean inRect = false;
		uint_fast8_t topBit = 0;
		for( uint_fast8_t bit=0; bit<=8; bit++ ) {

			boolean isSolid = (boolean)( colByte & (0x1 << bit) );

			if( !inRect && isSolid ) {
				inRect = true;
				topBit = bit;

			} else if( inRect && !isSolid ) {
				inRect = false;

				uint_fast16_t top = h_2 + ((int_fast8_t)(topBit-4) * TEXT_PIXEL_HEIGHT)*sinAngle - (TEXT_PIXEL_HEIGHT*cosAngle);
				uint_fast16_t height = ((bit-topBit) * TEXT_PIXEL_HEIGHT) * sinAngle;

				// Draw the pixel column
				tft.fillRect( left, top, TEXT_PIXEL_WIDTH, height, color );

				// Draw the "side" hanging below, or above, depending on the twist angle
				if( cosAngle > 0.0f ) {
					// Side protudes below: (text is facing up)
					tft.fillRect( left, top+height, TEXT_PIXEL_WIDTH, sideHeight, sideColor );

					// Erase above
					tft.fillRect( left, eraseTop, TEXT_PIXEL_WIDTH, top-eraseTop, _bgColor );
					eraseTop = top + height + sideHeight;

				} else {
					// Side protrudes above: (text is facing down)
					uint_fast16_t drawTop = max( top-sideHeight, eraseTop );	// Don't draw over the letter material
					int_fast16_t drawHeight = top - drawTop;
					if( drawHeight > 0 ) {
						tft.fillRect( left, drawTop, TEXT_PIXEL_WIDTH, drawHeight, sideColor );
					}

					// Erase above
					tft.fillRect( left, eraseTop, TEXT_PIXEL_WIDTH, (top-sideHeight)-eraseTop, _bgColor );
					eraseTop = top + height;
				}
			}
		}

		// Erase old graphics below the column
		tft.fillRect( left, eraseTop, TEXT_PIXEL_WIDTH, (h_2+4.5*TEXT_PIXEL_HEIGHT)-eraseTop, _bgColor );

	}	// end each column

	// Super-awesome fake EQ meters
	const uint_fast8_t MAX_EQ_BARS = 24;
	const uint_fast8_t AUDIO_POWER = 96;	// Seriously overdrive this for maximum meter awesomeness
	const uint_fast8_t EQ_BAR_WIDTH = 3;
	const uint_fast8_t EQ_BAR_HEIGHT = 5;
	const uint_fast8_t EQ_BAR_SPACING_X = 3;
	const uint_fast8_t EQ_BAR_SPACING_Y = 5;

	// Position EQ groups
	const uint_fast8_t EQ_GROUP_SEPARATE_X = 0;
	const uint_fast8_t EQ_GROUP_PADDING_Y = 13;

	uint_fast8_t baseEnergy = (uint_fast16_t)(frameParams.audioPeak * AUDIO_POWER) / 512;

	// Draw 4 groups: 2 on top, 2 on bottom, facing away from each other (out from center)
	uint_fast8_t meterOffset = 0;
	for( int_fast8_t eqX=-1; eqX<=1; eqX+=2 ) {

		uint_fast16_t baseX = w_2 + eqX*((EQ_GROUP_SEPARATE_X+EQ_BAR_WIDTH+EQ_BAR_SPACING_X)>>1);

		for( int_fast8_t eqY=-1; eqY<=1; eqY+=2 ) {	// -1==bottom of screen, 1==top

			uint_fast16_t baseY = ( (eqY==1) ? EQ_GROUP_PADDING_Y : (h-EQ_GROUP_PADDING_Y-(EQ_BAR_HEIGHT)) );
			int_fast8_t sideYOffset = (eqY==1) ? EQ_BAR_HEIGHT : -1;	// pseudo-3D sides

			uint_fast8_t energy = baseEnergy;

			// Each meter steals some energy from audioEnergy
			uint_fast8_t newMeters[3];
			newMeters[2] = random( (energy>>2), energy );
			energy -= newMeters[2];
			newMeters[1] = random( (energy>>1), energy );
			energy -= newMeters[1];
			newMeters[0] = energy;

			for( uint_fast8_t row=0; row<3; row++ ) {	// 3 meter rows
				uint_fast16_t sideColor = tft.color565( 0xaa + row*0x20, 0x55 + row*0x10, 0 );

				// Draw position
				uint_fast16_t drawY = baseY + row*eqY*(EQ_BAR_HEIGHT+EQ_BAR_SPACING_Y);

				uint_fast8_t prevMeter = _meters[meterOffset+row];

				// Clamp the newMeters values to a sane range, please.
				// Avoid flicker at low volume levels.
				newMeters[row] = constrain( (uint_fast8_t)newMeters[row], (uint_fast8_t)1, (uint_fast8_t)MAX_EQ_BARS );

				// Gently fade out over time
				newMeters[row] = max( _meters[meterOffset+row]>>1, newMeters[row] );

				// Either draw or erase bricks, depending on whether the meter level increased or decreased
				boolean isErase = true;
				uint_fast16_t color = _bgColor;	// default: erase
				uint_fast8_t drawHeight = EQ_BAR_HEIGHT;

				// If the energy of this row is higher: Set the draw color to something bright and colorful
				if( newMeters[row] > prevMeter ) {
					isErase = false;

					uint_fast8_t bright = 0x66 + row*0x33;
					color = tft.color565( 0x66, bright, bright );

				} else {
					// Be sure to erase the pseudo-3D sides
					drawHeight++;
					if( eqY==-1 ) drawY--;
				}

				uint_fast8_t minMeter = min( newMeters[row], prevMeter );
				uint_fast8_t maxMeter = max( newMeters[row], prevMeter );

				for( uint_fast8_t m=minMeter; m<maxMeter; m++ ) {
					uint_fast16_t drawX = baseX + m*eqX*(EQ_BAR_WIDTH+EQ_BAR_SPACING_X);
					tft.fillRect( drawX, drawY, EQ_BAR_WIDTH, drawHeight, color );

					// Draw fake 3D below each bar
					if( !isErase ) {
						tft.drawFastHLine( drawX, drawY+sideYOffset, EQ_BAR_WIDTH, sideColor );
					}
				}

				// Save this value for next draw cycle. Redraw minimum area.
				_meters[ meterOffset+row ] = newMeters[row];
			}

			// Advance _meters index for next batch of EQ meters
			meterOffset += 3;
		}
	}

}

#endif
