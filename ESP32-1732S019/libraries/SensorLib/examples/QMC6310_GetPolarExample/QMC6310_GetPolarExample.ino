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
 * @file      QMC6310_GetPolarExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#include "SensorQMC6310.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  -1
#endif

SensorQMC6310 qmc;

void setup()
{
    Serial.begin(115200);
    while (!Serial);



    if (!qmc.begin(Wire, QMC6310_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find QMC6310 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    /* Get Magnetometer chip id*/
    Serial.print("Device ID:");
    Serial.println(qmc.getChipID(), HEX);

    /* Config Magnetometer */
    qmc.configMagnetometer(
        /*
        * Run Mode
        * MODE_SUSPEND
        * MODE_NORMAL
        * MODE_SINGLE
        * MODE_CONTINUOUS
        * * */
        SensorQMC6310::MODE_NORMAL,
        /*
        * Full Range
        * RANGE_30G
        * RANGE_12G
        * RANGE_8G
        * RANGE_2G
        * * */
        SensorQMC6310::RANGE_8G,
        /*
        * Output data rate
        * DATARATE_10HZ
        * DATARATE_50HZ
        * DATARATE_100HZ
        * DATARATE_200HZ
        * * */
        SensorQMC6310::DATARATE_200HZ,
        /*
        * Over sample Ratio1
        * OSR_8
        * OSR_4
        * OSR_2
        * OSR_1
        * * * */
        SensorQMC6310::OSR_8,

        /*
        * Down sample Ratio1
        * DSR_8
        * DSR_4
        * DSR_2
        * DSR_1
        * * */
        SensorQMC6310::DSR_1);

    qmc.dumpCtrlRegister();

    // Declination is the difference between magnetic-north and true-north ("heading") and depends on location
    qmc.setDeclination(-2.77); // Found with: https://www.magnetic-declination.com/CHINA/SHENZHEN/475119.html

    Serial.println("Read data now...");
}

void loop()
{
    Polar data;
    // Wait for data ready
    if (qmc.readPolar(data)) {
        Serial.print(" polar:"); Serial.print(data.polar); Serial.print("°");
        Serial.print(" Gauss:"); Serial.print(data.Gauss);
        Serial.print(" uT:"); Serial.println(data.uT);
    }

    delay(100);
}



