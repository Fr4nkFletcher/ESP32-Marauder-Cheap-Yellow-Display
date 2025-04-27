#include <Adafruit_SH110X.h>
#include <Fonts/FreeSans9pt7b.h>
#include "Adafruit_TSC2007.h"

Adafruit_TSC2007 touch;

Adafruit_SH1107 display = Adafruit_SH1107(64, 128, &Wire);

void setup() {
  Serial.begin(115200);
  //while (!Serial);
  
  Serial.println("128x64 OLED FeatherWing TSC2007");
  display.begin(0x3C, true); // Address 0x3C default

  Serial.println("OLED begun");

  // Show image buffer on the display hardware.
  // Since the buffer is intialized with an Adafruit splashscreen
  // internally, this will display the splashscreen.
  display.display();
  delay(1000);

  // Clear the buffer.
  display.clearDisplay();
  display.display();
  
  if (!touch.begin()) {
    Serial.println("Failed to find Adafruit TSC chip");
    while (1) { delay(10); }
  }
  
  Serial.println("TSC Found!");
  
  display.setRotation(1);
  display.setFont(&FreeSans9pt7b);
  display.setTextColor(SH110X_WHITE);  
}

void loop() {
  display.clearDisplay();
  display.setCursor(0, 15);
  display.println("~TSC2007 QT~");
  
  uint16_t x, y, z1, z2;
  if (touch.read_touch(&x, &y, &z1, &z2) && (z1 > 100)) {
    Serial.print("Touch point: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z1); Serial.print(" / ");
    Serial.print(z2); Serial.println(")");
 
    display.print("X: ");
    display.println(x); 
    display.print("Y: ");
    display.println(y); 
  }
  display.display();
  yield();
  delay(100);
}
