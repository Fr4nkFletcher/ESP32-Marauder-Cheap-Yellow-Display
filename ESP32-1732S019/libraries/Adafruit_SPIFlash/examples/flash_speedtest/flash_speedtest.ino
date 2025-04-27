// The MIT License (MIT)
// Copyright (c) 2019 Ha Thach for Adafruit Industries

#include <SPI.h>
#include <SdFat.h>

#include "Adafruit_SPIFlash.h"

// for flashTransport definition
#include "flash_config.h"

Adafruit_SPIFlash flash(&flashTransport);

#ifdef __AVR__
#define BUFSIZE 512
#else
#define BUFSIZE 4096
#endif

#define TEST_WHOLE_CHIP 1

#ifdef LED_BUILTIN
uint8_t led_pin = LED_BUILTIN;
#else
uint8_t led_pin = 0;
#endif

// 4 byte aligned buffer has best result with nRF QSPI
uint8_t bufwrite[BUFSIZE] __attribute__((aligned(4)));
uint8_t bufread[BUFSIZE] __attribute__((aligned(4)));

// the setup function runs once when you press reset or power the board
void setup() {
  Serial.begin(115200);
  while (!Serial) {
    delay(100); // wait for native usb
  }
  flash.begin();

  pinMode(led_pin, OUTPUT);
  flash.setIndicator(led_pin, true);

  Serial.println("Adafruit Serial Flash Speed Test example");
  Serial.print("JEDEC ID: ");
  Serial.println(flash.getJEDECID(), HEX);
  Serial.print("Flash size: ");
  Serial.println(flash.size());
  Serial.flush();

  write_and_compare(0xAA);
  write_and_compare(0x55);

  Serial.println("Speed test is completed.");
  Serial.flush();
}

void print_speed(const char *text, uint32_t count, uint32_t ms) {
  Serial.print(text);
  Serial.print(count);
  Serial.print(" bytes in ");
  Serial.print(ms / 1000.0F, 2);
  Serial.println(" seconds.");

  Serial.print("Speed: ");
  Serial.print((count / 1000.0F) / (ms / 1000.0F), 2);
  Serial.println(" KB/s.\r\n");
}

bool write_and_compare(uint8_t pattern) {
  uint32_t ms;

  Serial.println("Erase chip");
  Serial.flush();

#if TEST_WHOLE_CHIP
  uint32_t const flash_sz = flash.size();
  flash.eraseChip();
#else
  uint32_t const flash_sz = 4096;
  flash.eraseSector(0);
#endif

  flash.waitUntilReady();

  // write all
  memset(bufwrite, (int)pattern, sizeof(bufwrite));
  Serial.print("Write flash with 0x");
  Serial.println(pattern, HEX);
  Serial.flush();
  ms = millis();

  for (uint32_t addr = 0; addr < flash_sz; addr += sizeof(bufwrite)) {
    flash.writeBuffer(addr, bufwrite, sizeof(bufwrite));
  }

  uint32_t ms_write = millis() - ms;
  print_speed("Write ", flash_sz, ms_write);
  Serial.flush();

  // read and compare
  Serial.println("Read flash and compare");
  Serial.flush();
  uint32_t ms_read = 0;
  for (uint32_t addr = 0; addr < flash_sz; addr += sizeof(bufread)) {
    memset(bufread, 0, sizeof(bufread));

    ms = millis();
    flash.readBuffer(addr, bufread, sizeof(bufread));
    ms_read += millis() - ms;

    if (memcmp(bufwrite, bufread, BUFSIZE)) {
      Serial.print("Error: flash contents mismatched at address 0x");
      Serial.println(addr, HEX);
      for (uint32_t i = 0; i < sizeof(bufread); i++) {
        if (i != 0)
          Serial.print(' ');
        if ((i % 16 == 0)) {
          Serial.println();
          if (i < 0x100)
            Serial.print('0');
          if (i < 0x010)
            Serial.print('0');
          Serial.print(i, HEX);
          Serial.print(": ");
        }

        if (bufread[i] < 0x10)
          Serial.print('0');
        Serial.print(bufread[i], HEX);
      }

      Serial.println();
      return false;
    }
  }

  print_speed("Read  ", flash_sz, ms_read);
  Serial.flush();

  return true;
}

void loop() {
  // nothing to do
}
