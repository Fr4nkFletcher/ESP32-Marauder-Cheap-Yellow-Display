#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_IS31FL3731.h>

// If you're using the full breakout...
Adafruit_IS31FL3731 matrix = Adafruit_IS31FL3731();
// If you're using the FeatherWing version
//Adafruit_IS31FL3731_Wing matrix = Adafruit_IS31FL3731_Wing();

static const uint8_t PROGMEM
  smile_bmp[] =
  { 0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10100101,
    0b10011001,
    0b01000010,
    0b00111100 },
  neutral_bmp[] =
  { 0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10111101,
    0b10000001,
    0b01000010,
    0b00111100 },
  frown_bmp[] =
  { 0b00111100,
    0b01000010,
    0b10100101,
    0b10000001,
    0b10011001,
    0b10100101,
    0b01000010,
    0b00111100 };


void setup() {

  Serial.begin(9600);
  Serial.println("ISSI manual animation test");
  if (! matrix.begin()) {
    Serial.println("IS31 not found");
    while (1);
  }
  Serial.println("IS31 Found!");
  
}


void loop() {
  matrix.setRotation(0);

  matrix.clear();
  matrix.drawBitmap(3, 0, smile_bmp, 8, 8, 255);
  delay(500);
  
  matrix.clear();
  matrix.drawBitmap(3, 0, neutral_bmp, 8, 8, 64);
  delay(500);

  matrix.clear();
  matrix.drawBitmap(3, 0, frown_bmp, 8, 8, 32);
  delay(500);

  matrix.clear();
  matrix.drawPixel(0, 0, 255);  
  delay(500);

  matrix.clear();
  matrix.drawLine(0,0, matrix.width()-1, matrix.height()-1, 127);
  delay(500);

  matrix.clear();
  matrix.drawRect(0,0, matrix.width(), matrix.height(), 255);
  matrix.fillRect(2,2, matrix.width()-4, matrix.height()-4, 20);
  delay(500);

  matrix.clear();
  matrix.drawCircle(8,4, 4, 64);
  matrix.drawCircle(8,4, 2, 32);
  delay(500);


  matrix.setTextSize(1);
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextColor(100);
  for (int8_t x=0; x>=-32; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("Hello");
    delay(100);
  }

  matrix.setTextSize(2);
  matrix.setTextWrap(false);  // we dont want text to wrap so it scrolls nicely
  matrix.setTextColor(32);
  matrix.setRotation(1);
  for (int8_t x=7; x>=-64; x--) {
    matrix.clear();
    matrix.setCursor(x,0);
    matrix.print("World");
    delay(100);
  }
}