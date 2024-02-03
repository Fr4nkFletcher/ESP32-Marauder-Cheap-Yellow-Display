/*!
 *  @file Adafruit_TSC2007.cpp
 *
 *  @mainpage Adafruit TSC2007 Resistive Touch Panel library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit TSC2007 Resistive Touch Panel Sensor library
 * for Arduino
 *
 * 	This is a library for the Adafruit TSC2007 breakout:
 * 	https://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *  @section dependencies Dependencies
 *  This library depends on the Adafruit BusIO library
 *
 *  @section author Author
 *
 *  Limor Fried (Adafruit Industries)
 *
 * 	@section license License
 *
 * 	BSD (see license.txt)
 *
 * 	@section  HISTORY
 *
 *     v1.0 - First release
 */

#include "Arduino.h"

#include "Adafruit_TSC2007.h"

/*!
 *    @brief  Instantiates a new TSC2007 class
 */
Adafruit_TSC2007::Adafruit_TSC2007(void) {}

Adafruit_TSC2007::~Adafruit_TSC2007(void) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }
}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  address
 *            The I2C address to use, defaults to 0x48
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_TSC2007::begin(uint8_t address, TwoWire *wire) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
  }

  i2c_dev = new Adafruit_I2CDevice(address, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  /*
  uint8_t setupcmd = 0b10110000;
  if (!i2c_dev->write(&setupcmd, 1)) {
    return false;
  }
  */

  command(MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT);
  return true;
}

/*!
 *    @brief  Send a command and read 2 bytes from TSC
 *    @param  func The command function to make
 *    @param  pwr  The power mode to enter after command
 *    @param  res  The ADC resolution
 *    @return 12 bits of data shifted from the 16-bit read value
 */
uint16_t Adafruit_TSC2007::command(adafruit_tsc2007_function func,
                                   adafruit_tsc2007_power pwr,
                                   adafruit_tsc2007_resolution res) {
  uint8_t cmd = (uint8_t)func << 4;
  cmd |= (uint8_t)pwr << 2;
  cmd |= (uint8_t)res << 1;

  uint8_t reply[2];

  if (!i2c_dev->write(&cmd, 1)) {
    return 0;
  }

  // Wait 1/2ms for conversion
  delayMicroseconds(500);

  if (!i2c_dev->read(reply, 2)) {
    return 0;
  }

  return ((uint16_t)reply[0] << 4) | (reply[1] >> 4); // 12 bits
}

/*!
 *    @brief  Read touch data from the TSC and then power down
 *    @param  x Pointer to 16-bit value we will store x reading
 *    @param  y Pointer to 16-bit value we will store y reading
 *    @param  z1 Pointer to 16-bit value we will store z1 pressure reading
 *    @param  z2 Pointer to 16-bit value we will store z2 pressure reading
 *    @return True if ADC was able to read the x & y values
 */
bool Adafruit_TSC2007::read_touch(uint16_t *x, uint16_t *y, uint16_t *z1,
                                  uint16_t *z2) {
  *z1 = command(MEASURE_Z1, ADON_IRQOFF, ADC_12BIT);
  *z2 = command(MEASURE_Z2, ADON_IRQOFF, ADC_12BIT);
  // take two measurements since there can be a 'flicker' on pen up
  uint16_t x1, y1, x2, y2;
  x1 = command(MEASURE_X, ADON_IRQOFF, ADC_12BIT);
  y1 = command(MEASURE_Y, ADON_IRQOFF, ADC_12BIT);
  x2 = command(MEASURE_X, ADON_IRQOFF, ADC_12BIT);
  y2 = command(MEASURE_Y, ADON_IRQOFF, ADC_12BIT);

  command(MEASURE_TEMP0, POWERDOWN_IRQON, ADC_12BIT);

  if (abs((int32_t)x1 - (int32_t)x2) > 100)
    return false;
  if (abs((int32_t)y1 - (int32_t)y2) > 100)
    return false;

  *x = x1;
  *y = y1;
  return (*x != 4095) && (*y != 4095);
}

/*!
 *  @brief  Function to get a point object rather than passing in pointers
 *  @returns A TS_Point, all values will be 0 if touchscreen read failed
 */
TS_Point Adafruit_TSC2007::getPoint(void) {
  uint16_t x, y, z1, z2;

  if (!this->read_touch(&x, &y, &z1, &z2)) {
    return TS_Point(0, 0, 0);
  }
  return TS_Point(x, y, z1);
}

/*!
 *  @brief  TS_Point constructor
 */
TS_Point::TS_Point() { x = y = 0; }

/*!
 *  @brief  TS_Point constructor
 *  @param  x0
 *          Initial x
 *  @param  y0
 *          Initial y
 *  @param  z0
 *          Initial z
 */
TS_Point::TS_Point(int16_t x0, int16_t y0, int16_t z0) {
  x = x0;
  y = y0;
  z = z0;
}

/*!
 *  @brief  Equality operator for TS_Point
 *  @return True if points are equal
 */
bool TS_Point::operator==(TS_Point p1) {
  return ((p1.x == x) && (p1.y == y) && (p1.z == z));
}

/*!
 *  @brief  Non-equality operator for TS_Point
 *  @return True if points are not equal
 */
bool TS_Point::operator!=(TS_Point p1) {
  return ((p1.x != x) || (p1.y != y) || (p1.z != z));
}
