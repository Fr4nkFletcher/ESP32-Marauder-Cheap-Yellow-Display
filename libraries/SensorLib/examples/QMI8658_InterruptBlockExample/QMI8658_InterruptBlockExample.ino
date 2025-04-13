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
 * @file      QMI8658_InterruptBlockExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#ifdef ARDUINO_ARCH_ESP32
#include "SensorQMI8658.hpp"
#include <MadgwickAHRS.h>       //MadgwickAHRS from https://github.com/arduino-libraries/MadgwickAHRS
#include "SH1106Wire.h"         //Oled display from https://github.com/ThingPulse/esp8266-oled-ssd1306

// #define USE_WIRE

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  -1
#endif


#define I2C1_SDA    22      //Display Wire SDA Pin
#define I2C1_SCL    21      //Display Wire SCL Pin

SH1106Wire display(0x3c, I2C1_SDA, I2C1_SCL);
SensorQMI8658 qmi;

IMUdata acc;
IMUdata gyr;

Madgwick filter;
unsigned long microsPerReading, microsPrevious;

float posX = 64;
float posY = 32;
float lastPosX = posX;
float lastPosY = posY;

void setup()
{
    Serial.begin(115200);
    while (!Serial);



    display.init();
    display.flipScreenVertically();

#ifdef USE_WIRE
    //Using WIRE !!
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
#else

#ifndef CONFIG_IDF_TARGET_ESP32
//Use LilyGo-T-Beam-S3 default spi pin
#define SPI_MOSI                    (35)
#define SPI_SCK                     (36)
#define SPI_MISO                    (37)
#define SPI_CS                      (47)
#define IMU_CS                      (34)
#define IMU_INT1                    (33)    //INTERRUPT PIN1 & PIN2 ,Use or logic to form a pin

    pinMode(SPI_CS, OUTPUT);    //sdcard pin set high
    digitalWrite(SPI_CS, HIGH);
    if (!qmi.begin(IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {

#else
//Use esp32dev module default spi pin
#define IMU_CS                      (5)
#define IMU_INT1                    (15)
#define IMU_INT2                    (22)
    if (!qmi.begin(IMU_CS)) {
#endif
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
#endif

    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);


    qmi.configAccelerometer(
        /*
         * ACC_RANGE_2G
         * ACC_RANGE_4G
         * ACC_RANGE_8G
         * ACC_RANGE_16G
         * */
        SensorQMI8658::ACC_RANGE_2G,
        /*
         * ACC_ODR_1000H
         * ACC_ODR_500Hz
         * ACC_ODR_250Hz
         * ACC_ODR_125Hz
         * ACC_ODR_62_5Hz
         * ACC_ODR_31_25Hz
         * ACC_ODR_LOWPOWER_128Hz
         * ACC_ODR_LOWPOWER_21Hz
         * ACC_ODR_LOWPOWER_11Hz
         * ACC_ODR_LOWPOWER_3H
        * */
        SensorQMI8658::ACC_ODR_1000Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        *  LPF_OFF        // OFF Low-Pass Fitter
        * */
        SensorQMI8658::LPF_MODE_0);


    qmi.configGyroscope(
        /*
        * GYR_RANGE_16DPS
        * GYR_RANGE_32DPS
        * GYR_RANGE_64DPS
        * GYR_RANGE_128DPS
        * GYR_RANGE_256DPS
        * GYR_RANGE_512DPS
        * GYR_RANGE_1024DPS
        * */
        SensorQMI8658::GYR_RANGE_64DPS,
        /*
         * GYR_ODR_7174_4Hz
         * GYR_ODR_3587_2Hz
         * GYR_ODR_1793_6Hz
         * GYR_ODR_896_8Hz
         * GYR_ODR_448_4Hz
         * GYR_ODR_224_2Hz
         * GYR_ODR_112_1Hz
         * GYR_ODR_56_05Hz
         * GYR_ODR_28_025H
         * */
        SensorQMI8658::GYR_ODR_896_8Hz,
        /*
        *  LPF_MODE_0     //2.66% of ODR
        *  LPF_MODE_1     //3.63% of ODR
        *  LPF_MODE_2     //5.39% of ODR
        *  LPF_MODE_3     //13.37% of ODR
        *  LPF_OFF        // OFF Low-Pass Fitter
        * */
        SensorQMI8658::LPF_MODE_3);


    /*
    * If both the accelerometer and gyroscope sensors are turned on at the same time,
    * the output frequency will be based on the gyroscope output frequency.
    * The example configuration is 896.8HZ output frequency,
    * so the acceleration output frequency is also limited to 896.8HZ
    * */
    qmi.enableGyroscope();
    qmi.enableAccelerometer();

    pinMode(IMU_INT1, INPUT);
#ifdef  IMU_INT2
    pinMode(IMU_INT2, INPUT);
#endif

    // qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_1); //no use
    // Enable data ready to interrupt pin2
    qmi.enableINT(SensorQMI8658::INTERRUPT_PIN_2);
    qmi.enableDataReadyINT();

    // Print register configuration information
    qmi.dumpCtrlRegister();

    // start  filter
    filter.begin(25);

    // initialize variables to pace updates to correct rate
    microsPerReading = 1000000 / 25;
    microsPrevious = micros();

    Serial.println("Read data now...");
}

const uint8_t rectWidth = 10;

void loop()
{
    float roll, pitch, heading;

    // check if it's time to read data and update the filter
    if (micros() - microsPrevious >= microsPerReading) {

        // read raw data from IMU
        if (digitalRead(IMU_INT1) == HIGH) {

            qmi.getAccelerometer(acc.x, acc.y, acc.z);
            qmi.getGyroscope(gyr.x, gyr.y, gyr.z);

            // update the filter, which computes orientation
            filter.updateIMU(gyr.x, gyr.y, gyr.z, acc.x, acc.y, acc.z);

            // print the heading, pitch and roll
            roll = filter.getRoll();
            pitch = filter.getPitch();
            heading = filter.getYaw();

            posX -= roll * 2;
            posY += pitch;

            Serial.printf("x:%.2f y:%.2f \n", posX, posY);
            posX = constrain(posX, 0, display.width() - rectWidth);
            posY = constrain(posY, 0, display.height() - rectWidth);

            display.setColor(BLACK);
            display.fillRect(lastPosX, lastPosY, 10, 10);
            display.setColor(WHITE);
            display.fillRect(posX, posY, 10, 10);
            display.display();

            lastPosX = posX;
            lastPosY = posY;
        }
        // increment previous time, so we keep proper pace
        microsPrevious = microsPrevious + microsPerReading;
    }
}
#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The graphics library may not support your current platform"); delay(1000);
}
#endif


