/***************************************************
                 **  DemoSauce!  **
  State-of-the-art graphics for your beautiful TFT display.

  Greetz to the Portland Dorkbot Crew!!!!

         Programmed by Zach Archer (@zkarcher)  ::  http://controlzinc.com/
  Using hardware by Thomas Hudson (@hydronics)  ::  http://thomashudson.org/

  Usage:
    * Connect a microphone to MIC_PIN.
    * Connect TFT backlight to BACKLIGHT_PIN.

  MIT license, all text above must be included in any redistribution
 ****************************************************/

// https://github.com/zkarcher/demosauce

#include "SPI.h"
#include "ILI9341_t3.h"
#include "font_Arial.h"

#include "FrameParams.h"

// Animations
#include "Checkerboard.h"
#include "Cube3D.h"
#include "Leaves.h"
#include "MagentaSquares.h"
//#include "MicCheck.h"
#include "PlasmaCloud.h"
#include "PlasmaYellow.h"
#include "Sphere3D.h"
#include "TriangleWeb.h"
#include "TwistyText.h"
#include "Waveform.h"

// Transitions
#include "TransitionDither.h"
#include "TransitionHalftone.h"
#include "TransitionScroll.h"
#include "TransitionSquares.h"

const boolean DO_BENCHMARKS = true;
const uint32_t SERIAL_BAUD_RATE = 9600;

const boolean DEBUG_ANIM = false; // dev: for hacking on one animation.
const uint_fast8_t DEBUG_ANIM_INDEX = 0;

const boolean DEBUG_TRANSITION = false;  // dev: set to true for short animation durations
const int_fast8_t DEBUG_TRANSITION_INDEX = -1;  // Supports -1: chooses a transition at random

const int_fast16_t DEFAULT_ANIM_TIME = 20.0f * 1000.0f;  // ms

// TFT pins
const uint8_t TFT_DC = 9;
const uint8_t TFT_CS = 10;
const uint8_t MIC_PIN = 14;
const uint8_t BACKLIGHT_PIN = 23;

// Use hardware SPI (#13, #12, #11) and the above for CS/DC
ILI9341_t3 tft = ILI9341_t3(TFT_CS, TFT_DC);
FrameParams frameParams;
long previousMillis = 0;

Checkerboard * _checkerboard       = new Checkerboard();
Cube3D * _cube3D                   = new Cube3D();
Leaves * _leaves                   = new Leaves();
MagentaSquares * _magentaSquares   = new MagentaSquares();
PlasmaCloud * _plasmaCloud         = new PlasmaCloud();
PlasmaYellow * _plasmaYellow       = new PlasmaYellow();
Sphere3D * _sphere3D               = new Sphere3D();
TriangleWeb * _triangleWeb         = new TriangleWeb();
TwistyText * _twistyText           = new TwistyText();
Waveform * _waveform               = new Waveform();

TransitionDither * _transDither  = new TransitionDither();
TransitionHalftone * _transHalftone  = new TransitionHalftone();
TransitionScroll * _transScroll  = new TransitionScroll();
TransitionSquares * _transSquares  = new TransitionSquares();

BaseAnimation **anims; // Array of pointers to BaseAnimation's. Initialized in setup() below.
int_fast8_t animCount;

BaseAnimation *activeAnim = 0;
int_fast16_t animTimeLeft = 0;
BaseAnimation *nextAnim;

BaseTransition **transitions;
int_fast8_t transCount;
boolean isTransition = true;
BaseTransition *activeTransition = 0;

// Benchmarks
uint32_t frameCount;

// Search the anims[] aray for the activeAnim pointer. If found, return the array index.
int_fast8_t getActiveAnimIndex() {
  for( int_fast8_t i=0; i<animCount; i++ ) {
    if( anims[i] == activeAnim ) return i;
  }
  return -1;  // not found
}

void setup() {
  // Backlight
  pinMode( BACKLIGHT_PIN, OUTPUT );
  analogWrite( BACKLIGHT_PIN, 1023 );

  // Microphone
  pinMode( MIC_PIN, INPUT );

  tft.begin();
  tft.setRotation( 3 );
  tft.fillScreen(ILI9341_BLACK);

  // Serial
  if( DO_BENCHMARKS ) {
    Serial.begin( SERIAL_BAUD_RATE );
    tft.setTextColor(ILI9341_YELLOW);
    tft.setFont(Arial_18);
    tft.setCursor(98, 42);
    tft.print("waiting for");
    tft.setFont(Arial_24);
    tft.setCursor(100, 80);
    tft.print("Arduino");
    tft.setCursor(60, 120);
    tft.print("Serial Monitor");
    tft.setTextColor(ILI9341_GREEN);
    tft.setFont(Arial_18);
    while (!Serial && millis() < 8000) { // wait for Arduino Serial Monitor
      tft.fillRect(118, 182, 42, 18, ILI9341_BLACK);
      tft.setCursor(118, 182);
      tft.print((8000.0 - (float)millis()) / 1000.0, 1);
      tft.print(" sec");
      delay(100);
    }
  }
  previousMillis = millis();

  // Clear
  uint16_t w = tft.width();
  uint16_t h = tft.height();
  tft.fillRect( 0, 0, w, h, 0x0 );

  tft.setRotation( 3 ); // ribbon cable on left
  tft.setScroll( 0 );

  // Populate anims in the order you want them to display.
  BaseAnimation* ANIMS_TEMP[] = {
    _twistyText,
    _plasmaCloud,
    _waveform,
    _magentaSquares,
    _sphere3D,
    _checkerboard,
    _leaves,
    _cube3D,
    _plasmaYellow,
    _triangleWeb
  };
  animCount = sizeof( ANIMS_TEMP ) / sizeof( BaseAnimation* );

  // Retain ANIMS_TEMP objects permanently
  anims = (BaseAnimation**)malloc( animCount * sizeof(BaseAnimation*) );
  for( int_fast8_t i=0; i<animCount; i++ ) {
    anims[i] = ANIMS_TEMP[i];
    anims[i]->init( tft );      // Initalize all animations
  }

  BaseTransition* TRANS_TEMP[] = {
    _transDither,
    _transHalftone,
    _transScroll,
    _transSquares
  };
  transCount = sizeof( TRANS_TEMP ) / sizeof( BaseTransition* );

  // Retain TRANS_TEMP objects permanently
  transitions = (BaseTransition**)malloc( transCount * sizeof(BaseTransition*) );
  for( int_fast8_t i=0; i<transCount; i++ ) {
    transitions[i] = TRANS_TEMP[i];
    transitions[i]->init( tft );
  }

  // Start!
  if( !activeAnim ) {
    if( DEBUG_ANIM ) {
      startAnimation( anims[DEBUG_ANIM_INDEX] );
    } else {
      startAnimation( anims[0] );
    }
  }
}

void startAnimation( BaseAnimation *newAnim ) {
  isTransition = false;

  activeAnim = newAnim;
  tft.fillScreen( activeAnim->bgColor() );
  tft.setScroll( 0 );
  activeAnim->reset( tft );

  animTimeLeft = DEFAULT_ANIM_TIME;

  if( DEBUG_TRANSITION ) animTimeLeft = 2000;

  if( DO_BENCHMARKS ) {
    Serial.println("---");
    Serial.print( activeAnim->title() );

    if( activeAnim->willForceTransition() ) {
      // TwistyText does not obey DEFAULT_ANIM_TIME
      Serial.println("");

    } else {
      Serial.print("  [");
      Serial.print( (uint8_t)(animTimeLeft / 1000.0f) );
      Serial.println(" secs]");
    }

    frameCount = 0;
  }
}

void loop() {
  // Frame multiplier
  long newMillis = millis();
  uint_fast8_t elapsed = newMillis - previousMillis;
  previousMillis = newMillis;
  frameParams.timeMult = elapsed * (60.0f / 1000);  // 1.0==exactly 60fps. 4.0==15fps, 4x slower

  // Get some audio input
  const uint_fast8_t SAMPLES_PER_FRAME = 1;
  frameParams.audioPeak = 0;
  uint_fast16_t sum = 0;

  for( uint_fast8_t s=0; s<SAMPLES_PER_FRAME; s++ ) {
    uint_fast16_t sample = abs( analogRead( MIC_PIN ) - 511 );
    frameParams.audioPeak = max( frameParams.audioPeak, sample );
    sum += sample;
  }
  frameParams.audioMean = sum * (1.0 / (512*SAMPLES_PER_FRAME));  // Range: 0..1

  frameParams.audioPeak = min( (uint_fast16_t)frameParams.audioPeak, (uint_fast16_t)511 );

  if( !isTransition ) {
    activeAnim->perFrame( tft, frameParams );
    animTimeLeft -= elapsed;

    if( DO_BENCHMARKS ) frameCount++;
  }

  // Has this animation expired?
  boolean willForceTransition = activeAnim->willForceTransition();
  boolean forceTransitionNow = activeAnim->forceTransitionNow();

  // Debugging transitions: Ignore animations hogging the screen
  if( DEBUG_TRANSITION ) willForceTransition = false;

  if( !DEBUG_ANIM ) {
    if( (!willForceTransition && (animTimeLeft <= 0)) || forceTransitionNow ) {

      // If the transition has not started yet, then start it.
      if( !isTransition ) {
        isTransition = true;

        nextAnim = anims[ (getActiveAnimIndex() + 1) % animCount ];

        /*
        // Print some debug stuff
        if( DO_BENCHMARKS ) {
          Serial.println("---------- Testing sinf() vs sin() performance");
          unsigned long then = micros();
          float z = 0;
          for( uint32_t i=0; i<50000; i++ ) {
            z += sinf( (float)i );
          }
          unsigned long now = micros();
          Serial.print("sinf:");
          Serial.println( now - then );
          Serial.print("   z:");
          Serial.println( z );

          then = micros();
          z = 0;
          for( uint32_t i=0; i<50000; i++ ) {
            z += sin( (float)i );
          }
          now = micros();
          Serial.print(" sin:");
          Serial.println( now - then );
          Serial.print("   z:");
          Serial.println( z );
        }
        */

        // When we loop back to the first animation, shuffle the other ones for variety.
        if( nextAnim == anims[0] ) {
          for( int_fast8_t i=1; i<animCount-1; i++ ) {
            uint_fast8_t shuffleIdx = random( i, animCount );

            BaseAnimation* temp = anims[i];
            anims[i] = anims[shuffleIdx];
            anims[shuffleIdx] = temp;
          }
        }

        // Choose a random transition
        activeTransition = transitions[ random(transCount) ];
        if( DEBUG_TRANSITION && (DEBUG_TRANSITION_INDEX >= 0) ) {
          activeTransition = transitions[ DEBUG_TRANSITION_INDEX ];
        }

        activeTransition->restart( tft, nextAnim->bgColor() );

        // Benchmark: show how many frames the animation completed while alive.
        if( DO_BENCHMARKS ) {
          Serial.print("Frame count (more is better):  ");
          Serial.print( frameCount );

          if( !activeAnim->willForceTransition() ) {
            Serial.print( "  (" );
            Serial.print( (float)frameCount / (DEFAULT_ANIM_TIME / 1000.0f) );
            Serial.println(" FPS)");
          } else {
            Serial.println("");
          }
        }
      }

      // After the transition ends, advance to the next animation
      activeTransition->perFrame( tft, frameParams );
      if( activeTransition->isComplete() ) {
        startAnimation( nextAnim );
      }

    }
  }

}
