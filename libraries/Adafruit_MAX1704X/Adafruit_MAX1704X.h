/*!
 *  @file Adafruit_MAX1704X.h
 *
 * 	I2C Driver for the Adafruit MAX17048 Battery Monitor
 *
 * 	This is a library for the Adafruit MAX17048 breakout:
 * 	https://www.adafruit.com/products/5580
 *
 * 	Adafruit invests time and resources providing this open source code,
 *  please support Adafruit and open-source hardware by purchasing products from
 * 	Adafruit!
 *
 *
 *	BSD license (see license.txt)
 */

#ifndef _ADAFRUIT_MAX1704X_H
#define _ADAFRUIT_MAX1704X_H

#include "Arduino.h"
#include <Adafruit_BusIO_Register.h>
#include <Adafruit_I2CDevice.h>

#define MAX17048_I2CADDR_DEFAULT 0x36 ///< MAX17048 default i2c address

#define MAX1704X_VCELL_REG 0x02   ///< Register that holds cell voltage
#define MAX1704X_SOC_REG 0x04     ///< Register that holds cell state of charge
#define MAX1704X_MODE_REG 0x06    ///< Register that manages mode
#define MAX1704X_VERSION_REG 0x08 ///< Register that has IC version
#define MAX1704X_HIBRT_REG 0x0A   ///< Register that manages hibernation
#define MAX1704X_CONFIG_REG 0x0C  ///< Register that manages configuration
#define MAX1704X_VALERT_REG 0x14  ///< Register that holds voltage alert values
#define MAX1704X_CRATE_REG 0x16   ///< Register that holds cell charge rate
#define MAX1704X_VRESET_REG 0x18  ///< Register that holds reset voltage setting
#define MAX1704X_CHIPID_REG 0x19  ///< Register that holds semi-unique chip ID
#define MAX1704X_STATUS_REG 0x1A  ///< Register that holds current alert/status
#define MAX1704X_CMD_REG                                                       \
  0xFE ///< Register that can be written for special commands

#define MAX1704X_ALERTFLAG_SOC_CHANGE                                          \
  0x20 ///< Alert flag for state-of-charge change
#define MAX1704X_ALERTFLAG_SOC_LOW 0x10 ///< Alert flag for state-of-charge low
#define MAX1704X_ALERTFLAG_VOLTAGE_RESET                                       \
  0x08 ///< Alert flag for voltage reset dip
#define MAX1704X_ALERTFLAG_VOLTAGE_LOW 0x04 ///< Alert flag for cell voltage low
#define MAX1704X_ALERTFLAG_VOLTAGE_HIGH                                        \
  0x02 ///< Alert flag for cell voltage high
#define MAX1704X_ALERTFLAG_RESET_INDICATOR                                     \
  0x01 ///< Alert flag for IC reset notification

/*!
 *    @brief  Class that stores state and functions for interacting with
 *            the MAX17048 I2C battery monitor
 */
class Adafruit_MAX17048 {
public:
  Adafruit_MAX17048();
  ~Adafruit_MAX17048();

  bool begin(TwoWire *wire = &Wire);
  bool isDeviceReady(void);

  uint16_t getICversion(void);
  uint8_t getChipID(void);

  bool reset(void);
  bool clearAlertFlag(uint8_t flag);

  float cellVoltage(void);
  float cellPercent(void);
  float chargeRate(void);

  void setResetVoltage(float reset_v);
  float getResetVoltage(void);

  void setAlertVoltages(float minv, float maxv);
  void getAlertVoltages(float &minv, float &maxv);

  bool isActiveAlert(void);
  uint8_t getAlertStatus(void);

  void setActivityThreshold(float actthresh);
  float getActivityThreshold(void);
  void setHibernationThreshold(float hibthresh);
  float getHibernationThreshold(void);

  void hibernate(void);
  void wake(void);
  bool isHibernating(void);
  void sleep(bool s);
  void enableSleep(bool en);

  void quickStart(void);

protected:
  Adafruit_I2CDevice *i2c_dev = NULL; ///< Pointer to I2C bus interface

  Adafruit_BusIO_Register *status_reg = NULL; ///< Status indicator register
};

#endif
