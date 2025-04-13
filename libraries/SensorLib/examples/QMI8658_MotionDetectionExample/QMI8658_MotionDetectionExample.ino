/**
 *
 * @license MIT License
 *
 * Copyright (c) 2024 lewis he
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
 * @file      QMI8658_MotionDetectionExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-26
 * @note      No motion detection does not work
 */
#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include "SensorQMI8658.hpp"


// #define USE_WIRE

#if defined(USE_WIRE)
#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  -1
#endif

#else

//USE SPI
#ifndef SPI_MOSI
#define SPI_MOSI                    (35)
#endif

#ifndef SPI_SCK
#define SPI_SCK                     (36)
#endif

#ifndef SPI_MISO
#define SPI_MISO                    (37)
#endif

#ifndef IMU_CS
#define IMU_CS                      (34)
#endif

#ifndef IMU_INT
#define IMU_INT                     (33)
#endif

#endif

SensorQMI8658 qmi;


bool interruptFlag = false;

void setFlag(void)
{
    interruptFlag = true;
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);

#if IMU_INT > 0
    qmi.setPins(IMU_INT);
#endif

#ifdef USE_WIRE
    //Using WIRE !!
    if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
#else
    if (!qmi.begin(IMU_CS, SPI_MOSI, SPI_MISO, SPI_SCK)) {
        Serial.println("Failed to find QMI8658 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }
#endif

    /* Get chip id*/
    Serial.print("Device ID:");
    Serial.println(qmi.getChipID(), HEX);

    //** The recommended output data rate for detection is higher than 500HZ
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_500Hz);

    // Enable the accelerometer
    qmi.enableAccelerometer();


    //* Configure the motion detection axis direction
    uint8_t modeCtrl = SensorQMI8658::ANY_MOTION_EN_X |
                       SensorQMI8658::ANY_MOTION_EN_Y |
                       SensorQMI8658::ANY_MOTION_EN_Z |
                       SensorQMI8658::NO_MOTION_EN_X |
                       SensorQMI8658::NO_MOTION_EN_Y |
                       SensorQMI8658::NO_MOTION_EN_Z ;

    //* Define the slope threshold of the x-axis for arbitrary motion detection
    float AnyMotionXThr = 100.0;    //  x-axis 100mg threshold
    //* Define the slope threshold of the y-axis for arbitrary motion detection
    float AnyMotionYThr = 100.0;    //  y-axis 100mg threshold
    //* Define the slope threshold of the z-axis for arbitrary motion detection
    float AnyMotionZThr = 1.0;      //  z-axis 1mg threshold
    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep higher than the threshold
    uint8_t AnyMotionWindow = 1;    //  1 samples


    //TODO: No motion detection does not work
    //* Defines the slope threshold of the x-axis for no motion detection
    float NoMotionXThr = 0.1;
    //* Defines the slope threshold of the y-axis for no motion detection
    float NoMotionYThr = 0.1;
    //* Defines the slope threshold of the z-axis for no motion detection
    float NoMotionZThr = 0.1;

    //* Defines the minimum number of consecutive samples (duration) that the absolute
    //* of the slope of the enabled axis/axes data should keep lower than the threshold
    uint8_t NoMotionWindow = 1; //  1 samples
    //* Defines the wait window (idle time) starts from the first Any-Motion event until
    //* starting to detecting another Any-Motion event form confirmation
    uint16_t SigMotionWaitWindow = 1;   //  1 samples
    //* Defines the maximum duration for detecting the other Any-Motion
    //* event to confirm Significant-Motion, starts from the first Any -Motion event
    uint16_t SigMotionConfirmWindow = 1;    //  1 samples

    qmi.configMotion(modeCtrl,
                     AnyMotionXThr, AnyMotionYThr, AnyMotionZThr, AnyMotionWindow,
                     NoMotionXThr, NoMotionYThr, NoMotionZThr, NoMotionWindow,
                     SigMotionWaitWindow, SigMotionConfirmWindow);

    // Enable the Motion Detection and enable the interrupt
    qmi.enableMotionDetect(SensorQMI8658::INTERRUPT_PIN_1);

    /*
     * When the QMI8658 is configured as Wom, the interrupt level is arbitrary,
     * not absolute high or low, and it is in the jump transition state
     */
    attachInterrupt(IMU_INT, setFlag, CHANGE);
}


void loop()
{
    if (interruptFlag) {
        interruptFlag = false;
        uint8_t status =  qmi.getStatusRegister();
        Serial.printf("status:0x%X BIN:", status);
        Serial.println(status, BIN);

        if (status & SensorQMI8658::EVENT_SIGNIFICANT_MOTION) {
            Serial.println("Significant motion");
        }
        if (status & SensorQMI8658::EVENT_NO_MOTION) {
            Serial.println("No Motion");
        }
        if (status & SensorQMI8658::EVENT_ANY_MOTION) {
            Serial.println("Any Motion");
        }
    }
    delay(300);
}