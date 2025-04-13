#include <bb_spi_lcd.h>
#include "shiny_hand.h"
#include "flowers_96x96.h"

BB_SPI_LCD lcd;
BB_SPI_LCD fg_sprite, bg_sprite, dest_sprite;

void setup() {
  lcd.begin(DISPLAY_WS_AMOLED_18);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN);
  lcd.setFont(FONT_12x16);
  lcd.println("bb_spi_lcd ESP32-S3 SIMD Alpha Blending");
  fg_sprite.createVirtual(96, 96);
  bg_sprite.createVirtual(96, 96);
  dest_sprite.createVirtual(96, 96);
  fg_sprite.drawBMP(shiny_hand, 0, 0, 0, -1, DRAW_TO_RAM);
  bg_sprite.drawBMP(flowers_96x96, 0, 0, 0, -1, DRAW_TO_RAM);
}

void loop() {
  int x, y;
  long l;
  x = (lcd.width() - 96)/2;
  y = (lcd.height() - 96)/2;
  for (int i=0; i<3; i++) {
    for (uint8_t u8Alpha = 0; u8Alpha < 32; u8Alpha++) {
      l = micros();
      lcd.blendSprite(&fg_sprite, &bg_sprite, &dest_sprite, u8Alpha);
      l = micros() - l;
      delay(40);
      lcd.drawSprite(x, y, &dest_sprite, 1.0f, -1, DRAW_TO_LCD);
    }
    for (uint8_t u8Alpha = 31; u8Alpha >= 1; u8Alpha--) {
      lcd.blendSprite(&fg_sprite, &bg_sprite, &dest_sprite, u8Alpha);
      delay(40);
      lcd.drawSprite(x, y, &dest_sprite, 1.0f, -1, DRAW_TO_LCD);
    }
  } // for i
  lcd.setCursor(0, lcd.height() - 16);
  lcd.printf("96x96 blend - %d microseconds", (int)l);
  while (1) {};
}
