#include <bb_spi_lcd.h>
#include <PNGdec.h>
#include "pattern_368x448.h"
PNG png;
BB_SPI_LCD lcd;
BB_SPI_LCD bg_sprite, fg_sprite, mask_sprite;
int x, y;

void PNGDraw(PNGDRAW *pDraw)
{
  if (pDraw->y < lcd.height()) {
    uint16_t *d = (uint16_t *)lcd.getBuffer();
    d += pDraw->y * lcd.width(); // point to the correct line of the framebuffer
    png.getLineAsRGB565(pDraw, d, PNG_RGB565_BIG_ENDIAN, 0xffffffff);
  }
} /* PNGDraw() */

void setup() {
  int rc;
  long l;

  lcd.begin(DISPLAY_WS_AMOLED_18); // Waveshare 1.8" 368x448 AMOLED
  lcd.allocBuffer();
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN);
  lcd.println("Tint color example");

// Use our PNG as the display background image
  rc = png.openRAM((uint8_t *)pattern_368x448, sizeof(pattern_368x448), PNGDraw);
  if (rc == PNG_SUCCESS) {
      png.decode(NULL, 0); // simple decode, no options
      png.close();
      lcd.display(); // copy framebuffer to display
  } else {
    lcd.println("Error opening PNG");
    while (1) {};
  }
  // Grab a 128x128 slice of the display so that we can use it for pattern tinting
  bg_sprite.createVirtual(128, 128);
  x = (lcd.width() - 128)/2; // center it
  y = (lcd.height() - 128)/2;
  lcd.readImage(x, y, 128, 128, (uint16_t *)bg_sprite.getBuffer());
  // Create a sprite to hold the tinted results
  fg_sprite.createVirtual(128, 128);
  // Prepare a rounded rect as a pushbutton
  mask_sprite.createVirtual(128, 128);
  mask_sprite.fillScreen(TFT_BLACK);
  mask_sprite.fillRoundRect(0, 0, 128, 128, 12, TFT_WHITE); // white will be our mask
  mask_sprite.setTextColor(TFT_BLACK, TFT_WHITE);
  mask_sprite.setFont(FONT_12x16);
  mask_sprite.setCursor(40, 40);
  mask_sprite.print("Push");
  mask_sprite.setCursor(28, 72);
  mask_sprite.print("Button");
  for (int i = 0; i<2; i++) {
    // Draw a red tinted button
    uint16_t u16Tint = TFT_RED;
    uint8_t alpha;
    for (alpha = 0; alpha < 32; alpha++) {
      l = micros();
      fg_sprite.maskedTint(&bg_sprite, &mask_sprite, 0, 0, u16Tint, alpha);
      l = micros() - l;
      lcd.drawSprite(x, y, &fg_sprite, 1.0f, 0xffffffff, DRAW_TO_LCD); // draw on the display
      delay(50);
    } // for each alpha
    for (alpha = 32; alpha > 0; alpha--) {
      fg_sprite.maskedTint(&bg_sprite, &mask_sprite, 0, 0, u16Tint, alpha);
      lcd.drawSprite(x, y, &fg_sprite, 1.0f, 0xffffffff, DRAW_TO_LCD); // draw on the display
      delay(50);
    }
    u16Tint = TFT_GREEN; // green
    for (alpha = 0; alpha < 32; alpha++) {
      fg_sprite.maskedTint(&bg_sprite, &mask_sprite, 0, 0, u16Tint, alpha);
      lcd.drawSprite(x, y, &fg_sprite, 1.0f, 0xffffffff, DRAW_TO_LCD); // draw on the display
      delay(50);
    } // for each alpha
    for (alpha = 32; alpha > 0; alpha--) {
      fg_sprite.maskedTint(&bg_sprite, &mask_sprite, 0, 0, u16Tint, alpha);
      lcd.drawSprite(x, y, &fg_sprite, 1.0f, 0xffffffff, DRAW_TO_LCD); // draw on the display
      delay(50);
    }
  } // for i
  lcd.setFont(FONT_12x16);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setCursor(0, lcd.height() - 16);
  lcd.printf("tint time = %d microseconds", (int)l);
} /* setup() */

void loop() {
}
