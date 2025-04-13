#include <bb_captouch.h>
#include <bb_spi_lcd.h>
#include <Wire.h>

// Define your display and touch configurations
#define CYD_24C

#ifdef CYD_24C
#define LCD DISPLAY_CYD_543
#define TOUCH_SDA 33
#define TOUCH_SCL 32
#define TOUCH_INT 21
#define TOUCH_RST 25
#endif

BBCapTouch bbct;
BB_SPI_LCD lcd;
int iWidth, iHeight;

const char *szNames[] = {"Unknown", "FT6x36", "GT911", "CST820", "CST226", "MXT144", "AXS15231"};

void setup() {
  Serial.begin(115200);
  while (!Serial) {};
  lcd.begin(LCD);
  Serial.println("Starting...");
  iWidth = lcd.width();
  iHeight = lcd.height();
  Serial.printf("LCD size = %dx%d\n", iWidth, iHeight);
  lcd.fillScreen(TFT_BLACK);
  lcd.setTextColor(TFT_GREEN, TFT_BLACK);
  lcd.setFont(FONT_8x8);
  lcd.setCursor(0, 0);
  lcd.println("CYD Touch Test");
  delay(1000);
  Wire.end();
  bbct.init(TOUCH_SDA, TOUCH_SCL, TOUCH_RST, TOUCH_INT);
  int iType = bbct.sensorType();
  Serial.printf("Sensor type = %s\n", szNames[iType]);
}

void loop() {
  TOUCHINFO ti;
  while (1) {
    if (bbct.getSamples(&ti)) {
      for (int i = 0; i < ti.count; i++) {
        Serial.print("Touch "); Serial.print(i + 1); Serial.print(": ");
        Serial.print("  x: "); Serial.print(ti.x[i]);
        Serial.print("  y: "); Serial.print(ti.y[i]);
        Serial.print("  size: "); Serial.println(ti.area[i]);
        Serial.println(' ');
        lcd.fillCircle(ti.x[i], ti.y[i], 3, (i == 0) ? TFT_BLUE : TFT_RED);
      }
    }
  }
}
