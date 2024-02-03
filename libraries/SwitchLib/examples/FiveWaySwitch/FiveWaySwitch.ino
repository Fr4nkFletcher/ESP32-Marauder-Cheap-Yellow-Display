#include <SwitchLib.h>

#define BUTTON_PIN 13

SwitchLib btn = SwitchLib(BUTTON_PIN, 1000, true);

void setup() {
  Serial.begin(115200);

  delay(1000);

  Serial.println("\n\nSwitchLib Test\n\n");
}

void loop() {
  if (btn.justPressed())
    Serial.println("Button just pressed");

  if (btn.isHeld())
    Serial.println("Button is held");
}
