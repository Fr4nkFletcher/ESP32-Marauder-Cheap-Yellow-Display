//
// Example showing how to work with extra large framebuffers and 'sprites'
// This is mostly aimed at devices like the ESP32 since it has the
// ability to use external PSRAM with a L1 cache to simulate an
// extended address space
// This sketch is written specifically for the Waveshare ESP32-S3
// AMOLED touch 1.8" device. It has a 368x448 color OLED display.
// Allocating a framebuffer for it will require 328K of RAM. The
// ESP32 cannot allocate a block that large within its internal
// static RAM, so we must use PSRAM.
//
// Cause a compilation error if the target is an ESP32 and PSRAM is not enabled
#if defined(ARDUINO_ARCH_ESP32) && !defined(BOARD_HAS_PSRAM)
#error "Please enable PSRAM support"
#endif

#include <bb_spi_lcd.h>
#include "Roboto_Black_40.h"
#include "bart_head.h"
BB_SPI_LCD lcd, sprite, sprite2;
#define SPRITE_SIZE 128
#define BALL_COUNT 10
typedef struct ballstruct
{
  int16_t x, y;
  int16_t dx, dy;
} BALLSTRUCT;

BALLSTRUCT balls[BALL_COUNT];
const uint16_t colors[8] = {TFT_WHITE, TFT_YELLOW, TFT_RED, TFT_BLUE, TFT_GREEN, TFT_MAGENTA, TFT_CYAN, TFT_BLACK};
void setup()
{
  long lTime;
  int x, y;
  char szTemp[32];
  Serial.begin(115200);
  delay(3000);
  //
  // The default flags for all operations is DRAW_TO_LCD | DRAW_TO_RAM
  // This means that all drawing will be sent to the physical LCD and
  // to the framebuffer (if it is allocated)
  // All drawing methods are overloaded with the flags as the last parameter
  // so that the default value is what most people will want, but it can be
  // overridden by specifying a different value
  //
//  lcd.begin(LCD_ILI9341, 0, 40000000, 10, 9, -1, -1, -1, MOSI, SCK);
  lcd.begin(DISPLAY_CYD_4848/*DISPLAY_WS_AMOLED_18*/); // initialize the display
  lcd.fillScreen(TFT_BLACK);
//  if (!lcd.allocBuffer()) { // unable to allocate a buffer
//    lcd.setTextColor(TFT_RED);
//    lcd.print("allocBuffer() failed!");
//    while (1) {}; // stop
//  }
  lcd.setFont(FONT_12x16);
  // A good example of how direct memory manipulation can save time in preparing graphics
  // Let's do a bunch of lines to the display and then into memory
  lcd.setTextColor(TFT_GREEN);
  lcd.setCursor(0,lcd.height()/2);
  lcd.println("Direct Draw Test");
  lTime = millis();
  for (int x=0; x<lcd.width(); x+=2) {
    lcd.drawLine(x, 16, lcd.width() - 1 - x, lcd.height()-1, TFT_WHITE, DRAW_TO_LCD);
  }
  lTime = millis() - lTime;
  lcd.printf("time = %d ms\n", (int)lTime);
  delay(3000);
  // Now do the same thing by drawing in RAM and then copying it to the LCD
  lcd.fillScreen(TFT_BLACK, DRAW_TO_RAM);
  lcd.setCursor(0,lcd.height()/2);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setPrintFlags(DRAW_TO_RAM);
  lcd.println("Buffer Draw Test");
  lTime = millis();
  for (int x=0; x<lcd.width(); x+=2) {
    lcd.drawLine(x, 16, lcd.width() - 1 - x, lcd.height()-1, TFT_WHITE, DRAW_TO_RAM);
  }
  lTime = millis() - lTime;
  lcd.printf("time = %d ms\n", (int)lTime);
  lcd.display(); // copy the buffer to the LCD
  delay(3000);

  sprite.createVirtual(150, 150); // create a sprite
  sprite.drawBMP(bart_head, 0, 0, 0, -1, DRAW_TO_RAM);
  // draw it directly to the LCD with scaling
  lcd.fillScreen(TFT_BLACK);
  float f = 0.5f;
  for (int i=0; i<400; i++) {
    lcd.drawSprite(0, 0, &sprite, f, -1, DRAW_TO_LCD);
    f += 0.01f;
  }
  delay(3000);
  sprite.freeVirtual(); // start over
  // Create a sprite that's smaller than the display
  sprite.createVirtual(SPRITE_SIZE, SPRITE_SIZE);
  sprite.fillScreen(TFT_BLACK);
  sprite.fillCircle(SPRITE_SIZE/2, SPRITE_SIZE/2, SPRITE_SIZE/2 - 1, TFT_BLUE); // make a solid circle within a circle
  sprite.fillCircle(SPRITE_SIZE/2, SPRITE_SIZE/2, SPRITE_SIZE/4, TFT_RED);
  // How fast can this work for animation?
  for (int i=0; i<BALL_COUNT; i++) {
    // create random starting positions and directions
    balls[i].x = 1 + rand() % (lcd.width()-SPRITE_SIZE-2); // bounce when right edge of sprite hits right edge of LCD
    balls[i].y = 1 + rand() % (lcd.height()-SPRITE_SIZE-2);
    balls[i].dx = balls[i].dy = 0;
    while (balls[i].dx == 0 && balls[i].dy == 0) { // make sure all balls are moving
      balls[i].dx = (rand() % 3) - 1;
      balls[i].dy = (rand() % 3) - 1;
    }
  }
  for (int j=0; j<90; j++) {
    lcd.fillScreen(TFT_BLACK, DRAW_TO_RAM);
    for (int i=0; i<BALL_COUNT; i++) {
      lcd.drawSprite(balls[i].x, balls[i].y, &sprite, TFT_BLACK, DRAW_TO_RAM);
      // advance ball direction and bounce off walls
      balls[i].x += balls[i].dx;
      balls[i].y += balls[i].dy;
      if (balls[i].x == 0 && balls[i].dx == -1) balls[i].dx = 1;
      else if (balls[i].x == lcd.width()-1-SPRITE_SIZE && balls[i].dx == 1) balls[i].dx = -1;
      if (balls[i].y == 0 && balls[i].dy == -1) balls[i].dy = 1;
      else if (balls[i].y == lcd.height()-1-SPRITE_SIZE && balls[i].dy == 1) balls[i].dy = -1;
    }
    lcd.display();
  }
  delay(3000);
  // transparent text
  lcd.fillScreen(TFT_BLACK, DRAW_TO_RAM);
  lcd.setFreeFont(&Roboto_Black_40);
  for (int i=0; i<200; i+= 20) {
    lcd.setTextColor(colors[i/20], -1); // draw transparent text
    lcd.setCursor(i, i+60);
    lcd.print("Text");
  }
  lcd.display();
  delay(3000);
  // Sprite rotation example
  sprite.fillScreen(TFT_BLACK);
  sprite.setFreeFont(&Roboto_Black_40);
  sprite.setTextColor(TFT_GREEN, TFT_BLACK);
  sprite.setCursor(10, 56);
  sprite.print("A");
  sprite.setTextColor(TFT_RED, -1);
  sprite.setCursor(40, 76);
  sprite.print("B");
  sprite.setTextColor(TFT_BLUE, -1);
  sprite.setCursor(70, 96);
  sprite.print("C");
  sprite2.createVirtual(SPRITE_SIZE, SPRITE_SIZE);
  lcd.fillScreen(TFT_BLACK, DRAW_TO_LCD);
  for (int j=0; j<15; j++) {
    for (int iAngle=0; iAngle<360; iAngle++) {
      sprite2.fillScreen(TFT_BLACK); // erase any old edge pixels
      sprite.rotateSprite(&sprite2, SPRITE_SIZE/2, SPRITE_SIZE/2, iAngle);
      lcd.drawSprite((lcd.width()-SPRITE_SIZE)/2, (lcd.height() - SPRITE_SIZE)/2, &sprite2, -1);
    }
  }
} /* setup() */

void loop()
{
} /* loop() */
