# FlashStorage library for Arduino

The FlashStorage library aims to provide a convenient way to store and retrieve
user's data using the non-volatile flash memory of microcontrollers.

The flash memory, due to it's properties, is generally used to store the firmware
code, but it can also be used to store user data.

## Supported hardware

Currently, only ATSAMD21 cpu is supported (and consequently every board based
on this cpu like the Arduino Zero or Aduino MKR1000).

## Limited number of writes

Flash memory has a limited amount of write cycles. Typical flash
memory can perform about 10000 writes cycles to the same flash block
before starting to "wear out" and begin to lose the ability to retain data.

So **BEWARE: IMPROPER USE OF THIS LIBRARY CAN QUICKLY AND PERMANENTLY
DESTROY THE FLASH MEMORY OF YOUR MICRO**, in particular you should avoid to
call the `write()` function too often and make sure that in the entire life
of the micro the number of calls to `write` stay well below the above limit
of 10000 (it's a good rule-of-thumb to keep that number in mind even if the
manufacturer of the micro guarantees a bigger number of cycles).

The same caution must be taken if you're using the EEPROM API emulation (see
below) with the `EEPROM.commit()` function.

## Usage

First of all you must declare a global `FlashStorage` object for each piece of
data you intend to store in the flash memory.
For example if you want to store the age of a person you must declare an
`age_storage` like this:

```c++
FlashStorage(age_storage, int);
```

this instruction means "create a `FlashStorage` to store an `int` variable and call
it `age_storage`". Now you can use `age_storage` as a place to safely store an integer:

```c++
void readAndStoreUserAge() {
  Serial.println("Please enter your age:");
  String age = Serial.readStringUntil('\n');

  age_storage.write(age.toInt());  // <-- save the age
}
```

after a reset of the microcontroller you can retrieve the stored age by using:

```c++
int user_age = age_storage.read();
```

### Using the alternative EEPROM-like API

If you include `FlashAsEEPROM.h` you'll get an EEPROM emulation with the internal flash memory.
See [EmulateEEPROM](https://github.com/cmaglie/FlashStorage/tree/master/examples/EmulateEEPROM/EmulateEEPROM.ino) sketch for an example.

The API is very similar to the well known Arduino EEPROM.h API but with two additional functions:

* `EEPROM.isValid()` returns `true` if data in the EEPROM is valid or, in other words, if the data has been written at least once, otherwise EEPROM data is "undefined" and the function returns `false`.
* `EEPROM.commit()` store the EEPROM data in flash. Use this with care: Every call writes the complete EEPROM data to flash. This will reduce the remaining flash-write-cycles. Don't call this method in a loop or [you will kill your flash soon](https://github.com/cmaglie/FlashStorage#limited-number-of-writes).

## License

This library is released under LGPL-2.1.

## FAQ

### Can I use a single FlashStorage object to store more stuff?

Yes, you can declare a `struct` with more fields and create a `FlashStorage` object to
store the entire structure. See the [StoreNameAndSurname](https://github.com/cmaglie/FlashStorage/tree/master/examples/StoreNameAndSurname/StoreNameAndSurname.ino)
sketch for an example on how to do it.

### The content of the FlashStorage is erased each time a new sketch is uploaded?

Yes, every time you upload a new sketch, the previous content of the FlashStorage is erased.

### Do you recommend to use FLASH instead of EEPROM?

No. If your micro provides an EEPROM it's almost always better to use that because
it's a kind of memory designed with the specific purpose to store user data (it has a
longer lifetime, number of write cycles, etc...).

In the absence of an EEPROM you can use this library to use a piece of the flash memory
as an alternative to EEPROM. However, you must always keep in mind of it's limits.

