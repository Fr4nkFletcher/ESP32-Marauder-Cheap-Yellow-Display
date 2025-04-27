/*
  Demonstrate how to use FlashStorage with an API that is similar to the EEPROM library.

  This example code is in the public domain.

  Written by A. Christian
  Edited 14 Oct 2016 by Cristian Maglie
*/

// Include EEPROM-like API for FlashStorage
#include <FlashAsEEPROM.h>

void setup() {
  Serial.begin(9600);

  // If the EEPROM is empty then isValid() is false
  if (!EEPROM.isValid()) {

    Serial.println("EEPROM is empty, writing some example data:");
    Serial.print("->");
    for (int i=0; i<20; i++) {
      EEPROM.write(i, 100+i);
      Serial.print(" ");
      Serial.print(100+i);
    }
    Serial.println();

    // commit() saves all the changes to EEPROM, it must be called
    // every time the content of virtual EEPROM is changed to make
    // the change permanent.
    // This operation burns Flash write cycles and should not be
    // done too often. See readme for details:
    // https://github.com/cmaglie/FlashStorage#limited-number-of-writes
    EEPROM.commit();
    Serial.println("Done!");

    Serial.print("After commit, calling isValid() returns ");
    Serial.println(EEPROM.isValid());

  } else {

    Serial.println("EEPROM has been written.");
    Serial.println("Here is the content of the first 20 bytes:");

    Serial.print("->");
    for (int i=0; i<20; i++) {
      Serial.print(" ");
      Serial.print(EEPROM.read(i));
    }
    Serial.println();

  }
}

void loop() {
}
