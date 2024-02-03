/*!
 *  @file Adafruit_TSC2007.h
 *
 * 	I2C Driver for the Adafruit TSC2007 Resistive Touch Panel Sensor library
 *for Arduino
 *
 * 	This is a library for the Adafruit TSC2007 breakout:
 * 	https://www.adafruit.com/
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_TSC2007_H
#define _ADAFRUIT_TSC2007_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>
#include <Wire.h>

#define TSC2007_I2CADDR_DEFAULT 0x48 ///< TSC2007 default i2c address

/*!
 *  @brief  Class for working with points
 */
class TS_Point {
public:
  TS_Point();
  TS_Point(int16_t x, int16_t y, int16_t z);

  bool operator==(TS_Point);
  bool operator!=(TS_Point);

  int16_t x; /**< x coordinate **/
  int16_t y; /**< y coordinate **/
  int16_t z; /**< z coordinate **/
};

/*!
 *    @brief  Different function commands
 */
typedef enum {
  MEASURE_TEMP0 = 0,
  MEASURE_AUX = 2,
  MEASURE_TEMP1 = 4,
  ACTIVATE_X = 8,
  ACTIVATE_Y = 9,
  ACTIVATE_YPLUS_X = 10,
  SETUP_COMMAND = 11,
  MEASURE_X = 12,
  MEASURE_Y = 13,
  MEASURE_Z1 = 14,
  MEASURE_Z2 = 15
} adafruit_tsc2007_function;

/*!
 *    @brief  Power and IRQ modes
 */
typedef enum {
  POWERDOWN_IRQON = 0,
  ADON_IRQOFF = 1,
  ADOFF_IRQON = 2,
} adafruit_tsc2007_power;

/*!
 *    @brief  ADC resolution
 */
typedef enum {
  ADC_12BIT = 0,
  ADC_8BIT = 1,
} adafruit_tsc2007_resolution;

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the TSC2007 driver
 */
class Adafruit_TSC2007 {
public:
  Adafruit_TSC2007();
  ~Adafruit_TSC2007();

  bool begin(uint8_t address = TSC2007_I2CADDR_DEFAULT, TwoWire *wire = &Wire);

  uint16_t command(adafruit_tsc2007_function func, adafruit_tsc2007_power pwr,
                   adafruit_tsc2007_resolution res);
  bool read_touch(uint16_t *x, uint16_t *y, uint16_t *z1, uint16_t *z2);

  TS_Point getPoint();

private:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface
};

#endif
