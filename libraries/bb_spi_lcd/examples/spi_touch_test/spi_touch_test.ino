#include <bb_spi_lcd.h>

BB_SPI_LCD lcd;
TOUCHINFO ti;

void setup()
{
  Serial.begin(115200);
  Serial.println("Starting...");
  lcd.begin(DISPLAY_CYD_543);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setFont(FONT_12x16);
  lcd.println("SPI Resistive Touch Test");
  lcd.rtInit(); // the resistive touch configuration is already linked to the display type
}
void loop()
{
  while (1) {
    if (lcd.rtReadTouch(&ti)) {
      lcd.drawPixel(ti.x[0], ti.y[0], TFT_WHITE);
      //Serial.printf("Touch x: %d y: %d\n", ti.x[0], ti.y[0]);;
    }
  }
}
