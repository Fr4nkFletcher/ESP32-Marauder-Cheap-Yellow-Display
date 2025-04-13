#! /usr/bin/env python3
#
# before running, install required libraries:
#     pip3 install adafruit-extended-bus
#     pip3 install adafruit-circuitpython-bme280

import time
import adafruit_extended_bus
from adafruit_bme280 import basic as adafruit_bme280

# Run "i2cdetect -l" to find your bus number
i2c_usb = adafruit_extended_bus.ExtendedI2C(8)
bme280 = adafruit_bme280.Adafruit_BME280_I2C(i2c_usb)
while 1:
    print("\nTemperature: %0.1f C" % bme280.temperature)
    print("Humidity: %0.1f %%" % bme280.humidity)
    print("Pressure: %0.1f hPa" % bme280.pressure)
    time.sleep(1)