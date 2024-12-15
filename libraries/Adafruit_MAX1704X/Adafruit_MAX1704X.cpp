/*!
 *  @file Adafruit_MAX1704X.cpp
 *
 *  @mainpage Adafruit MAX17048 Battery Monitor library
 *
 *  @section intro_sec Introduction
 *
 * 	I2C Driver for the Adafruit MAX17048 Battery Monitor library
 *
 * 	This is a library for the Adafruit MAX17048 breakout:
 * 	https://www.adafruit.com/product/5580
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

#include "Adafruit_MAX1704X.h"

/*!
 *    @brief  Instantiates a new MAX17048 class
 */
Adafruit_MAX17048::Adafruit_MAX17048(void) {}

Adafruit_MAX17048::~Adafruit_MAX17048(void) {}

/*!
 *    @brief  Sets up the hardware and initializes I2C
 *    @param  wire
 *            The Wire object to be used for I2C connections.
 *    @return True if initialization was successful, otherwise false.
 */
bool Adafruit_MAX17048::begin(TwoWire *wire) {
  if (i2c_dev) {
    delete i2c_dev; // remove old interface
    delete status_reg;
  }

  i2c_dev = new Adafruit_I2CDevice(MAX17048_I2CADDR_DEFAULT, wire);

  if (!i2c_dev->begin()) {
    return false;
  }

  if (!isDeviceReady()) { // couldnt find the chip - check battery!
    return false;
  }

  status_reg = new Adafruit_BusIO_Register(i2c_dev, MAX1704X_STATUS_REG);

  if (!reset()) {
    return false;
  }

  enableSleep(false);
  sleep(false);

  return true;
}

/*!
 *    @brief  Get IC LSI version
 *    @return 16-bit value read from MAX1704X_VERSION_REG register
 */
uint16_t Adafruit_MAX17048::getICversion(void) {
  Adafruit_BusIO_Register ic_vers =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VERSION_REG, 2, MSBFIRST);
  return ic_vers.read();
}

/*!
 *    @brief  Get semi-unique chip ID
 *    @return 8-bit value read from MAX1704X_VERSION_REG register
 */
uint8_t Adafruit_MAX17048::getChipID(void) {
  Adafruit_BusIO_Register ic_vers =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_CHIPID_REG);
  return ic_vers.read();
}

/*!
 *    @brief  Check if the MAX1704x is ready to be read from.
 *    Chip ID = 0xFF and Version = 0xFFFF if no battery is attached
 *    @return True if the MAX1704x is ready to be read from
 */
bool Adafruit_MAX17048::isDeviceReady(void) {
  return (getICversion() & 0xFFF0) == 0x0010;
}

/*!
 *    @brief  Soft reset the MAX1704x
 *    @return True on reset success
 */
bool Adafruit_MAX17048::reset(void) {
  Adafruit_BusIO_Register cmd =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_CMD_REG, 2, MSBFIRST);

  // send reset command, the MAX1704 will reset before ACKing,
  // so I2C xfer is expected to *fail* with a NACK
  if (cmd.write(0x5400)) {
    return false;
  }

  // loop and attempt to clear alert until success
  for (uint8_t retries = 0; retries < 3; retries++) {
    if (clearAlertFlag(MAX1704X_ALERTFLAG_RESET_INDICATOR)) {
      return true;
    }
  }

  // something didn't work :(
  return false;
}

/*!
 *    @brief  Function for clearing an alert flag once it has been handled.
 *    @param flags A byte that can have any number of OR'ed alert flags:
 *    MAX1704X_ALERTFLAG_SOC_CHANGE, MAX1704X_ALERTFLAG_SOC_LOW,
 *    MAX1704X_ALERTFLAG_VOLTAGE_RESET, MAX1704X_ALERTFLAG_VOLTAGE_LOW
 *    MAX1704X_ALERTFLAG_VOLTAGE_HIGH, or MAX1704X_ALERTFLAG_RESET_INDICATOR
 *    @return True if the status register write succeeded
 */
bool Adafruit_MAX17048::clearAlertFlag(uint8_t flags) {
  return status_reg->write(status_reg->read() & ~flags);
}

/*!
 *    @brief  Get battery voltage
 *    @return Floating point value read in Volts
 */
float Adafruit_MAX17048::cellVoltage(void) {
  if (!isDeviceReady())
    return NAN;
  Adafruit_BusIO_Register vcell =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VCELL_REG, 2, MSBFIRST);
  float voltage = vcell.read();
  return voltage * 78.125 / 1000000;
}

/*!
 *    @brief  Get battery state in percent (0-100%)
 *    @return Floating point value from 0 to 100.0
 */
float Adafruit_MAX17048::cellPercent(void) {
  if (!isDeviceReady())
    return NAN;
  Adafruit_BusIO_Register vperc =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_SOC_REG, 2, MSBFIRST);

  float percent = vperc.read();
  return percent / 256.0;
}

/*!
 *    @brief  Charge or discharge rate of the battery in percent/hour
 *    @return Floating point value from 0 to 100.0% per hour
 */
float Adafruit_MAX17048::chargeRate(void) {
  if (!isDeviceReady())
    return NAN;
  Adafruit_BusIO_Register crate =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_CRATE_REG, 2, MSBFIRST);

  float percenthr = (int16_t)crate.read();
  return percenthr * 0.208;
}

/*!
 *    @brief Setter function for the voltage that the IC considers 'resetting'
 *    @param reset_v Floating point voltage that, when we go below, should be
 * considered a reset
 */

void Adafruit_MAX17048::setResetVoltage(float reset_v) {
  Adafruit_BusIO_Register vreset_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VRESET_REG);
  Adafruit_BusIO_RegisterBits vreset_bits =
      Adafruit_BusIO_RegisterBits(&vreset_reg, 7, 0);

  reset_v /= 0.04; // 40mV / LSB

  vreset_bits.write(max(min(int(reset_v), 127), 0));
}

/*!
 *    @brief Getter function for the voltage that the IC considers 'resetting'
 *    @returns Floating point voltage that, when we go below, should be
 * considered a reset
 */
float Adafruit_MAX17048::getResetVoltage(void) {
  Adafruit_BusIO_Register vreset_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VRESET_REG);
  Adafruit_BusIO_RegisterBits vreset_bits =
      Adafruit_BusIO_RegisterBits(&vreset_reg, 7, 0);

  float val = vreset_bits.read();
  val *= 0.04; // 40mV / LSB
  return val;
}

/*!
 *    @brief Setter function for the voltage alert min/max settings
 *    @param minv The minimum voltage: alert if we go below
 *    @param maxv The maximum voltage: alert if we go above
 */
void Adafruit_MAX17048::setAlertVoltages(float minv, float maxv) {
  Adafruit_BusIO_Register valert_min_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VALERT_REG);
  Adafruit_BusIO_Register valert_max_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VALERT_REG + 1);

  uint8_t minv_int = min(255, max(0, (int)(minv / 0.02)));
  uint8_t maxv_int = min(255, max(0, (int)(maxv / 0.02)));
  valert_min_reg.write(minv_int);
  valert_max_reg.write(maxv_int);
}

/*!
 *    @brief Getter function for the voltage alert min/max settings
 *    @param minv The minimum voltage: alert if we go below
 *    @param maxv The maximum voltage: alert if we go above
 */
void Adafruit_MAX17048::getAlertVoltages(float &minv, float &maxv) {
  Adafruit_BusIO_Register valert_min_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VALERT_REG);
  Adafruit_BusIO_Register valert_max_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_VALERT_REG + 1);

  minv = valert_min_reg.read() * 0.02; // 20mV / LSB
  maxv = valert_max_reg.read() * 0.02; // 20mV / LSB
}

/*!
 *    @brief A check to determine if there is an unhandled alert
 *    @returns True if there is an alert status flag
 */
bool Adafruit_MAX17048::isActiveAlert(void) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_CONFIG_REG, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits alert_bit =
      Adafruit_BusIO_RegisterBits(&config_reg, 1, 5);
  return alert_bit.read();
}

/*!
 *    @brief Get all 7 alert flags from the status register in a uint8_t
 *    @returns A byte that has all 7 alert flags. You can then check the flags
 *    MAX1704X_ALERTFLAG_SOC_CHANGE, MAX1704X_ALERTFLAG_SOC_LOW,
 *    MAX1704X_ALERTFLAG_VOLTAGE_RESET, MAX1704X_ALERTFLAG_VOLTAGE_LOW
 *    MAX1704X_ALERTFLAG_VOLTAGE_HIGH, or MAX1704X_ALERTFLAG_RESET_INDICATOR
 */
uint8_t Adafruit_MAX17048::getAlertStatus(void) {
  return status_reg->read() & 0x7F;
}

/*!
 *    @brief The voltage change that will trigger exiting hibernation mode.
 *    If at any ADC sample abs(OCVCELL) is greater than ActThr, the IC exits
 *    hibernate mode.
 *    @returns The threshold, from 0-0.31874 V that will be used to determine
 *    whether its time to exit hibernation.
 */
float Adafruit_MAX17048::getActivityThreshold(void) {
  Adafruit_BusIO_Register actthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG + 1);
  return (float)actthr_reg.read() * 0.00125; // 1.25mV per LSB
}

/*!
 *    @brief Set the voltage change that will trigger exiting hibernation mode.
 *    If at any ADC sample abs(OCVCELL) is greater than ActThr, the IC exits
 *    hibernate mode.
 *    @param actthresh The threshold voltage, from 0-0.31874 V that will be
 *    used to determine whether its time to exit hibernation.
 */
void Adafruit_MAX17048::setActivityThreshold(float actthresh) {
  Adafruit_BusIO_Register actthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG + 1);
  actthr_reg.write(
      min(255, max(0, (int)(actthresh / 0.00125)))); // 1.25mV per LSB
}

/*!
 *    @brief The %/hour change that will trigger hibernation mode. If the
 *    absolute value of CRATE is less than HibThr for longer than 6min,
 *    the IC enters hibernate mode
 *    @returns The threshold, from 0-53% that will be used to determine
 *    whether its time to hibernate.
 */
float Adafruit_MAX17048::getHibernationThreshold(void) {
  Adafruit_BusIO_Register hibthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG);
  return (float)hibthr_reg.read() * 0.208; // 0.208% per hour
}

/*!
 *    @brief Determine the %/hour change that will trigger hibernation mode
 *    If the absolute value of CRATE is less than HibThr for longer than 6min,
 *    the IC enters hibernate mode
 *    @param hibthresh The threshold, from 0-53% that will be used to determine
 *    whether its time to hibernate.
 */
void Adafruit_MAX17048::setHibernationThreshold(float hibthresh) {
  Adafruit_BusIO_Register hibthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG);
  hibthr_reg.write(
      min(255, max(0, (int)(hibthresh / 0.208)))); // 0.208% per hour
}

/*!
 *    @brief Query whether the chip is hibernating now
 *    @returns True if hibernating
 */
bool Adafruit_MAX17048::isHibernating(void) {
  Adafruit_BusIO_Register mode_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_MODE_REG);
  Adafruit_BusIO_RegisterBits hib_bit =
      Adafruit_BusIO_RegisterBits(&mode_reg, 1, 4);
  return hib_bit.read();
}

/*!
 *    @brief Enter hibernation mode.
 */
void Adafruit_MAX17048::hibernate(void) {
  Adafruit_BusIO_Register actthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG + 1);
  Adafruit_BusIO_Register hibthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG);
  actthr_reg.write(0xFF);
  hibthr_reg.write(0xFF);
}

/*!
 *    @brief Wake up from hibernation mode.
 */
void Adafruit_MAX17048::wake(void) {
  Adafruit_BusIO_Register actthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG + 1);
  Adafruit_BusIO_Register hibthr_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_HIBRT_REG);
  actthr_reg.write(0x0);
  hibthr_reg.write(0x0);
}

/*!
 *    @brief Enter ultra-low-power sleep mode (1uA draw)
 *    @param s True to force-enter sleep mode, False to leave sleep
 */
void Adafruit_MAX17048::sleep(bool s) {
  Adafruit_BusIO_Register config_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_CONFIG_REG, 2, MSBFIRST);
  Adafruit_BusIO_RegisterBits sleep_bit =
      Adafruit_BusIO_RegisterBits(&config_reg, 1, 7);
  sleep_bit.write(s);
}

/*!
 *    @brief Enable the ability to enter ultra-low-power sleep mode (1uA draw)
 *    @param en True to enable sleep mode, False to only allow hibernation
 */
void Adafruit_MAX17048::enableSleep(bool en) {
  Adafruit_BusIO_Register mode_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_MODE_REG);
  Adafruit_BusIO_RegisterBits sleepen_bit =
      Adafruit_BusIO_RegisterBits(&mode_reg, 1, 5);
  sleepen_bit.write(en);
}

/*!
 *    @brief  Quick starting allows an instant 'auto-calibration' of the
 battery. However, its a bad idea to do this right when the battery is first
 plugged in or if there's a lot of load on the battery so uncomment only if
 you're sure you want to 'reset' the chips charge calculator.
 */
void Adafruit_MAX17048::quickStart(void) {
  Adafruit_BusIO_Register mode_reg =
      Adafruit_BusIO_Register(i2c_dev, MAX1704X_MODE_REG);
  Adafruit_BusIO_RegisterBits quick_bit =
      Adafruit_BusIO_RegisterBits(&mode_reg, 1, 6);
  quick_bit.write(true);
  // bit is cleared immediately
}
