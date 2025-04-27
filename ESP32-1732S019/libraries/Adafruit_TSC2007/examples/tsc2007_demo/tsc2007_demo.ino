#include "Adafruit_TSC2007.h"

Adafruit_TSC2007 touch;

void setup() {
  Serial.begin(115200);
  while (!Serial) delay(10);

  if (!touch.begin()) {
    Serial.println("Couldn't find touch controller");
    while (1) delay(10);
  }
  Serial.println("Found touch controller");
}

void loop() {
  uint16_t x, y, z1, z2;
  if (touch.read_touch(&x, &y, &z1, &z2)) {
    Serial.print("Touch point: (");
    Serial.print(x); Serial.print(", ");
    Serial.print(y); Serial.print(", ");
    Serial.print(z1); Serial.print(" / ");
    Serial.print(z2); Serial.println(")");
  }

  delay(100);
}
