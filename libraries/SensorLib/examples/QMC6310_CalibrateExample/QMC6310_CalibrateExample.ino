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
 * @file      QMC6310_CalibrateExample.ino
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


void calibrate()
{
    qmc.setDataOutputRate(SensorQMC6310::DATARATE_200HZ);

    int32_t x_min = 65535;
    int32_t x_max = -65535;
    int32_t y_min = 65535;
    int32_t y_max = -65535;
    int32_t z_min = 65535;
    int32_t z_max = -65535;
    Serial.println("Place the sensor on the plane and slowly rotate the sensor...");

    int32_t range = 1000;
    int32_t i = 0;
    int32_t x = 0, y = 0, z = 0;;
    float a = 0.5 ;
    float x_offset = 0;
    float y_offset = 0;
    float z_offset = 0;

    while (i < range) {
        i += 1;

        if (qmc.isDataReady()) {

            qmc.readData();

            x = a * qmc.getRawX() + (1 - a) * x;
            y = a * qmc.getRawY() + (1 - a) * y;
            z = a * qmc.getRawZ() + (1 - a) * z;
            if (x < x_min) {
                x_min = x;
                i = 0;
            }
            if (x > x_max) {
                x_max = x;
                i = 0;
            }
            if (y < y_min) {
                y_min = y;
                i = 0;
            }
            if (y > y_max) {
                y_max = y;
                i = 0;
            }
            if (z < z_min) {
                z_min = z;
                i = 0;
            }
            if (z > z_max) {
                z_max = z;
                i = 0;
            }
            int j = round(10 * i / range);

            Serial.print("[");
            for (int k = 0; k < j; ++k) {
                Serial.print("*");
            }
            Serial.println("]");
        }
        delay(5);
    }

    x_offset = (x_max + x_min) / 2;
    y_offset = (y_max + y_min) / 2;
    z_offset = (z_max + z_min) / 2;

    Serial.print("x_min:");
    Serial.print(x_min);

    Serial.print("x_max:");
    Serial.print(x_max);

    Serial.print("y_min:");
    Serial.print(y_min);

    Serial.print("y_max:");
    Serial.print(y_max);

    Serial.print("z_min:");
    Serial.print(z_min);

    Serial.print("z_max:");
    Serial.println(z_max);



    Serial.print("x_offset:");
    Serial.print(x_offset);

    Serial.print("y_offset:");
    Serial.print(y_offset);

    Serial.print("z_offset:");
    Serial.print(z_offset);


    // Serial.printf("x_min:%d x_max:%d y_min:%d y_max:%d z_min:%d z_max:%d\n", x_min, x_max, y_min, y_max, z_min, z_max);
    // Serial.printf("x_offset:%.2f y_offset:%.2f z_offset:%.2f \n", x_offset, y_offset, z_offset);

    // Set the calibration value and the user calculates the deviation
    qmc.setOffset(x_offset, y_offset, z_offset);
}


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
        SensorQMC6310::MODE_CONTINUOUS,
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
        SensorQMC6310::OSR_1,

        /*
        * Down sample Ratio1
        * DSR_8
        * DSR_4
        * DSR_2
        * DSR_1
        * * */
        SensorQMC6310::DSR_1);




    // Calibration algorithm reference from
    // https://github.com/CoreElectronics/CE-PiicoDev-QMC6310-MicroPython-Module
    calibrate();

    Serial.println("Calibration done .");
    delay(5000);

    Serial.println("Read data now...");
}

void loop()
{

    //Wait data ready
    if (qmc.isDataReady()) {

        qmc.readData();

        Serial.print("GYR: ");
        Serial.print("X:");
        Serial.print(qmc.getX());
        Serial.print(" Y:");
        Serial.print(qmc.getY());
        Serial.print(" Z:");
        Serial.print(qmc.getZ());
        Serial.println(" uT");
        Serial.print("RAW: ");
        Serial.print("X:");
        Serial.print(qmc.getRawX());
        Serial.print(" Y:");
        Serial.print(qmc.getRawY());
        Serial.print(" Z:");
        Serial.println(qmc.getRawZ());

        /*
        float x,  y,  z;
        qmc.getMag(x, y, z);
        Serial.print("X:");
        Serial.print(x);
        Serial.print(" Y:");
        Serial.print(y);
        Serial.print(" Z:");
        Serial.println(x);
        */
    }


    delay(100);
}



