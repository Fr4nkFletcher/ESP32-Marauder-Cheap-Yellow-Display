/**
 *
 * @license MIT License
 *
 * Copyright (c) 2022 lewis he
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * @file      bosch_interfaces.cpp
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2023-10-09
 *
 */
#include "bosch_interfaces.h"

#if defined(ARDUINO)

#if defined(ARDUINO_ARCH_RP2040)
SPISettings  SensorInterfaces::__spiSetting = SPISettings();
#else
SPISettings  SensorInterfaces::__spiSetting = SPISettings(4000000, SPI_DATA_ORDER, SPI_MODE0);
#endif

void SensorInterfaces::close_interfaces(SensorLibConfigure config)
{

}

bool SensorInterfaces::setup_interfaces(SensorLibConfigure config)
{
    if (config.irq != SENSOR_PIN_NONE) {
        pinMode(config.irq, INPUT_PULLDOWN);
    }
    switch (config.intf) {
    case SENSORLIB_I2C_INTERFACE:
#if defined(ARDUINO_ARCH_RP2040)
        config.u.i2c_dev.wire->end();
        config.u.i2c_dev.wire->setSDA(config.u.i2c_dev.sda);
        config.u.i2c_dev.wire->setSCL(config.u.i2c_dev.scl);
        config.u.i2c_dev.wire->begin();
#elif defined(NRF52840_XXAA) || defined(NRF52832_XXAA)
        config.u.i2c_dev.wire->end();
        config.u.i2c_dev.wire->setPins(config.u.i2c_dev.sda, config.u.i2c_dev.scl);
        config.u.i2c_dev.wire->begin();
#else
        config.u.i2c_dev.wire->begin(config.u.i2c_dev.sda, config.u.i2c_dev.scl);
#endif
        config.u.i2c_dev.wire->beginTransmission(config.u.i2c_dev.addr);
        if (config.u.i2c_dev.wire->endTransmission() != 0) {
            return false;
        }
        break;
    case SENSORLIB_SPI_INTERFACE:
        pinMode(config.u.spi_dev.cs, OUTPUT);
        digitalWrite(config.u.spi_dev.cs, HIGH);
#if defined(ARDUINO_ARCH_RP2040)
        config.u.spi_dev.spi->setSCK(config.u.spi_dev.sck);
        config.u.spi_dev.spi->setRX(config.u.spi_dev.miso);
        config.u.spi_dev.spi->setTX(config.u.spi_dev.mosi);
        config.u.spi_dev.spi->begin();
#elif defined(ARDUINO_ARCH_NRF52)
        config.u.spi_dev.spi->setPins(config.u.spi_dev.miso, config.u.spi_dev.sck, config.u.spi_dev.mosi);
        config.u.spi_dev.spi->begin();
#elif defined(ARDUINO_ARCH_STM32)
        config.u.spi_dev.spi->setSCLK(config.u.spi_dev.sck);
        config.u.spi_dev.spi->setMISO(config.u.spi_dev.miso);
        config.u.spi_dev.spi->setMOSI(config.u.spi_dev.mosi);
        config.u.spi_dev.spi->begin();
#else
config.u.spi_dev.spi->begin(config.u.spi_dev.sck, config.u.spi_dev.miso, config.u.spi_dev.mosi);
#endif
        break;
    default:
        break;
    }
    return true;
}




int8_t SensorInterfaces::bhy2_spi_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SensorLibConfigure *pConfig  =  (SensorLibConfigure *)intf_ptr;
    if (!pConfig) {
        return DEV_WIRE_ERR;
    }
    digitalWrite(pConfig->u.spi_dev.cs, LOW);
    pConfig->u.spi_dev.spi->beginTransaction(__spiSetting);
    pConfig->u.spi_dev.spi->transfer((reg_addr));
    for (size_t i = 0; i < length; i++) {
        reg_data[i] = pConfig->u.spi_dev.spi->transfer(0x00);
    }
    pConfig->u.spi_dev.spi->endTransaction();
    digitalWrite(pConfig->u.spi_dev.cs, HIGH);

    return DEV_WIRE_NONE;
}

int8_t SensorInterfaces::bhy2_spi_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SensorLibConfigure *pConfig  =  (SensorLibConfigure *)intf_ptr;
    if (!pConfig) {
        return DEV_WIRE_ERR;
    }
    digitalWrite(pConfig->u.spi_dev.cs, LOW);
    pConfig->u.spi_dev.spi->beginTransaction(__spiSetting);
    pConfig->u.spi_dev.spi->transfer(reg_addr);
    pConfig->u.spi_dev.spi->transfer((uint8_t *)reg_data, length);
    pConfig->u.spi_dev.spi->endTransaction();
    digitalWrite(pConfig->u.spi_dev.cs, HIGH);
    return DEV_WIRE_NONE;
}

int8_t SensorInterfaces::bhy2_i2c_read(uint8_t reg_addr, uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SensorLibConfigure *pConfig  =  (SensorLibConfigure *)intf_ptr;
    if (!pConfig) {
        return DEV_WIRE_ERR;
    }
    pConfig->u.i2c_dev.wire->beginTransmission(pConfig->u.i2c_dev.addr);
    pConfig->u.i2c_dev.wire->write(reg_addr);
    if (pConfig->u.i2c_dev.wire->endTransmission() != 0) {
        return DEV_WIRE_ERR;
    }
    pConfig->u.i2c_dev.wire->requestFrom(pConfig->u.i2c_dev.addr, length);
    return pConfig->u.i2c_dev.wire->readBytes(reg_data, length) == length ? DEV_WIRE_NONE : DEV_WIRE_ERR;
}

int8_t SensorInterfaces::bhy2_i2c_write(uint8_t reg_addr, const uint8_t *reg_data, uint32_t length, void *intf_ptr)
{
    SensorLibConfigure *pConfig  =  (SensorLibConfigure *)intf_ptr;
    if (!pConfig) {
        return DEV_WIRE_ERR;
    }
    pConfig->u.i2c_dev.wire->beginTransmission(pConfig->u.i2c_dev.addr);
    pConfig->u.i2c_dev.wire->write(reg_addr);
    pConfig->u.i2c_dev.wire->write(reg_data, length);
    return (pConfig->u.i2c_dev.wire->endTransmission() == 0) ? 0 : DEV_WIRE_ERR;

}


void SensorInterfaces::bhy2_delay_us(uint32_t us, void *private_data)
{
    (void)private_data;
    delayMicroseconds(us);
}

#endif /*Arduino */