/***************************************************
  This is our touchscreen painting example for the Adafruit TFT FeatherWing
  ----> http://www.adafruit.com/products/3315

  Check out the links above for our tutorials and wiring diagrams

  Adafruit invests time and resources providing this open source code,
  please support Adafruit and open-source hardware by purchasing
  products from Adafruit!

  Written by Limor Fried/Ladyada for Adafruit Industries.
  MIT license, all text above must be included in any redistribution
 ****************************************************/

#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ILI9341.h> // Hardware-specific library

// If using the rev 1 with STMPE resistive touch screen controller uncomment this line:
//#include <Adafruit_STMPE610.h>
// If using the rev 2 with TSC2007, uncomment this line:
#include <Adafruit_TSC2007.h>

#ifdef ESP8266
   #define STMPE_CS 16
   #define TFT_CS   0
   #define TFT_DC   15
   #define SD_CS    2
#elif defined(ESP32) && !defined(ARDUINO_ADAFRUIT_FEATHER_ESP32S2)
   #define STMPE_CS 32
   #define TFT_CS   15
   #define TFT_DC   33
   #define SD_CS    14
#elif defined(TEENSYDUINO)
   #define TFT_DC   10
   #define TFT_CS   4
   #define STMPE_CS 3
   #define SD_CS    8
#elif defined(ARDUINO_STM32_FEATHER)
   #define TFT_DC   PB4
   #define TFT_CS   PA15
   #define STMPE_CS PC7
   #define SD_CS    PC5
#elif defined(ARDUINO_NRF52832_FEATHER)  /* BSP 0.6.5 and higher! */
   #define TFT_DC   11
   #define TFT_CS   31
   #define STMPE_CS 30
   #define SD_CS    27
#elif defined(ARDUINO_MAX32620FTHR) || defined(ARDUINO_MAX32630FTHR)
   #define TFT_DC   P5_4
   #define TFT_CS   P5_3
   #define STMPE_CS P3_3
   #define SD_CS    P3_2
#else
    // Anything else, defaults!
   #define STMPE_CS 6
   #define TFT_CS   9
   #define TFT_DC   10
   #define SD_CS    5
#endif


Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC);

#if defined(_ADAFRUIT_STMPE610H_)
  Adafruit_STMPE610 ts = Adafruit_STMPE610(STMPE_CS);
#elif defined(_ADAFRUIT_TSC2007_H)
  // If you're using the TSC2007 there is no CS pin needed, so instead its an IRQ!
  #define TSC_IRQ STMPE_CS
  Adafruit_TSC2007 ts = Adafruit_TSC2007();             // newer rev 2 touch contoller
#else
  #error("You must have either STMPE or TSC2007 headers included!")
#endif

// This is calibration data for the raw touch data to the screen coordinates
// For STMPE811/STMPE610
#define STMPE_TS_MINX 3800
#define STMPE_TS_MAXX 100
#define STMPE_TS_MINY 100
#define STMPE_TS_MAXY 3750
// For TSC2007
#define TSC_TS_MINX 300
#define TSC_TS_MAXX 3800
#define TSC_TS_MINY 185
#define TSC_TS_MAXY 3700
// we will assign the calibration values on init
int16_t min_x, max_x, min_y, max_y;

// Size of the color selection boxes and the paintbrush size
#define BOXSIZE 40
#define PENRADIUS 3
int oldcolor, currentcolor;

void setup(void) {
  Serial.begin(115200);
  //while (!Serial);
  
  delay(10);
  Serial.println("FeatherWing TFT Demo sketch");
  
#if defined(_ADAFRUIT_STMPE610H_)
  if (!ts.begin()) {
    Serial.println("Couldn't start STMPE touchscreen controller");
    while (1) delay(100);
  }
  min_x = STMPE_TS_MINX; max_x = STMPE_TS_MAXX;
  min_y = STMPE_TS_MINY; max_y = STMPE_TS_MAXY;
#else
  if (! ts.begin(0x48, &Wire)) {
    Serial.println("Couldn't start TSC2007 touchscreen controller");
    while (1) delay(100);
  }
  min_x = TSC_TS_MINX; max_x = TSC_TS_MAXX;
  min_y = TSC_TS_MINY; max_y = TSC_TS_MAXY;
  pinMode(TSC_IRQ, INPUT);
#endif

  Serial.println("Touchscreen started");
  
  tft.begin();
  tft.fillScreen(ILI9341_BLACK);
  
  // make the color selection boxes
  tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
  tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
  tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
  tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
  tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
  tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);
 
  // select the current color 'red'
  tft.drawRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
  currentcolor = ILI9341_RED;
}

void loop() {
#if defined(TSC_IRQ)
  if (digitalRead(TSC_IRQ)) {
    // IRQ pin is high, nothing to read!
    return;
  }
#endif
  TS_Point p = ts.getPoint();

  Serial.print("X = "); Serial.print(p.x);
  Serial.print("\tY = "); Serial.print(p.y);
  Serial.print("\tPressure = "); Serial.print(p.z);
  if (((p.x == 0) && (p.y == 0)) || (p.z < 10)) return; // no pressure, no touch
 
  // Scale from ~0->4000 to tft.width using the calibration #'s
  p.x = map(p.x, min_x, max_x, 0, tft.width());
  p.y = map(p.y, min_y, max_y, 0, tft.height());
  Serial.print(" -> "); Serial.print(p.x); Serial.print(", "); Serial.println(p.y);

 

  if (p.y < BOXSIZE) {
     oldcolor = currentcolor;

     if (p.x < BOXSIZE) { 
       currentcolor = ILI9341_RED; 
       tft.drawRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     } else if (p.x < BOXSIZE*2) {
       currentcolor = ILI9341_YELLOW;
       tft.drawRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     } else if (p.x < BOXSIZE*3) {
       currentcolor = ILI9341_GREEN;
       tft.drawRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     } else if (p.x < BOXSIZE*4) {
       currentcolor = ILI9341_CYAN;
       tft.drawRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     } else if (p.x < BOXSIZE*5) {
       currentcolor = ILI9341_BLUE;
       tft.drawRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     } else if (p.x < BOXSIZE*6) {
       currentcolor = ILI9341_MAGENTA;
       tft.drawRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, ILI9341_WHITE);
     }

     if (oldcolor != currentcolor) {
        if (oldcolor == ILI9341_RED) 
          tft.fillRect(0, 0, BOXSIZE, BOXSIZE, ILI9341_RED);
        if (oldcolor == ILI9341_YELLOW) 
          tft.fillRect(BOXSIZE, 0, BOXSIZE, BOXSIZE, ILI9341_YELLOW);
        if (oldcolor == ILI9341_GREEN) 
          tft.fillRect(BOXSIZE*2, 0, BOXSIZE, BOXSIZE, ILI9341_GREEN);
        if (oldcolor == ILI9341_CYAN) 
          tft.fillRect(BOXSIZE*3, 0, BOXSIZE, BOXSIZE, ILI9341_CYAN);
        if (oldcolor == ILI9341_BLUE) 
          tft.fillRect(BOXSIZE*4, 0, BOXSIZE, BOXSIZE, ILI9341_BLUE);
        if (oldcolor == ILI9341_MAGENTA) 
          tft.fillRect(BOXSIZE*5, 0, BOXSIZE, BOXSIZE, ILI9341_MAGENTA);
     }
  }

  if (((p.y-PENRADIUS) > 0) && ((p.y+PENRADIUS) < tft.height())) {
    tft.fillCircle(p.x, p.y, PENRADIUS, currentcolor);
  }
}