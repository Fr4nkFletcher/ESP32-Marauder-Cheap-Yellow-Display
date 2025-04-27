// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries

#include "SPI.h"
#include "SdFat.h"

#include "Adafruit_SPIFlash.h"

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(10); // wait for native usb
  }

  flash.begin();

  Serial.println("Adafruit Serial Flash Sector Dump example");
  Serial.print("JEDEC ID: ");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.println(flash.size());
}

void dump_sector(uint32_t sector) {
  uint8_t buf[4096];
  memset(buf, 0xff, sizeof(buf));

  flash.readBuffer(sector * 4096, buf, 4096);

  for (uint32_t row = 0; row < sizeof(buf) / 16; row++) {
    if (row == 0)
      Serial.print("0");
    if (row < 16)
      Serial.print("0");
    Serial.print(row * 16, HEX);
    Serial.print(" : ");

    for (uint32_t col = 0; col < 16; col++) {
      uint8_t val = buf[row * 16 + col];

      if (val < 16)
        Serial.print("0");
      Serial.print(val, HEX);

      Serial.print(" ");
    }

    Serial.println();
  }
}

void loop() {
  Serial.print("Enter the sector number to dump: ");
  while (!Serial.available()) {
    delay(10);
  }

  int sector = Serial.parseInt();
  int sector_max = (int)flash.size() / 4096;

  Serial.println(sector); // echo

  if (sector < sector_max) {
    dump_sector(sector);
  } else {
    Serial.println("Invalid sector number");
  }

  Serial.println();
  delay(10); // a bit of delay
}
