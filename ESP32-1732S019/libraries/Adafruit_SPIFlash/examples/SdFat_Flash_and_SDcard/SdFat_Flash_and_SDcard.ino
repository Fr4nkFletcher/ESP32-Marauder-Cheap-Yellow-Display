// Adafruit Grand Central M4 QSPI Flash and SD Card Setup Example
// Author: Joshua Scoggins
//
// This is an example of how to bring up both the QSPI Flash and SD Card found
// on the Adafruit Grand Central M4. This example will setup both the QSPI
// Flash and SD card (if present) and display information about the QSPI flash.
//

#include <SPI.h>
#include <SdFat.h>

#include <Adafruit_SPIFlash.h>

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash onboardFlash(&flashTransport);
SdFat onboardSdCard;

constexpr int getSDCardPin() noexcept {
#ifdef SDCARD_SS_PIN
  return SDCARD_SS_PIN;
#else
  // modify to fit your needs
  // by default, pin 4 is the SD_CS pin used by the Adafruit 1.8" TFT SD Shield
  return 4;
#endif
}
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    // wait for native usb
    delay(100);
  }
  Serial.print("Starting up onboard QSPI Flash...");
  onboardFlash.begin();
  Serial.println("Done");
  Serial.println("Onboard Flash information");
  Serial.print("JEDEC ID: 0x");
  Serial.println(onboardFlash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.print(onboardFlash.size() / 1024);
  Serial.println(" KB");
  Serial.print("Starting up SD Card...");
  if (!onboardSdCard.begin(getSDCardPin())) {
    Serial.println("No card found (is one inserted?)");
  } else {
    Serial.println("Card found!");
  }
}

void loop() {
  // nothing to do
}
