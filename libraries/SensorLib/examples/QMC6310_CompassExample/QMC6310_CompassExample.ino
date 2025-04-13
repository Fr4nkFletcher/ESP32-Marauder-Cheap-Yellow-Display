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
 * @file      QMC6310_GetDataExample.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-10-16
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>

#if defined(ARDUINO_ARCH_ESP32)

#include "SensorQMC6310.hpp"
#include "SH1106Wire.h"         //Oled display from https://github.com/ThingPulse/esp8266-oled-ssd1306

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
SensorQMC6310 qmc;

int last_dx, last_dy, dx, dy, angle;

const int centreX = 32;
const int centreY = 30;
const int radius  = 22;

//Compass application from  https://github.com/G6EJD/ESP8266_micro_compass_HMC5883_OLED
void arrow(int x2, int y2, int x1, int y1, int alength, int awidth, OLEDDISPLAY_COLOR  color)
{
    display.setColor(color);
    float distance;
    int dx, dy, x2o, y2o, x3, y3, x4, y4, k;
    distance = sqrt(pow((x1 - x2), 2) + pow((y1 - y2), 2));
    dx = x2 + (x1 - x2) * alength / distance;
    dy = y2 + (y1 - y2) * alength / distance;
    k = awidth / alength;
    x2o = x2 - dx;
    y2o = dy - y2;
    x3 = y2o * k + dx;
    y3 = x2o * k + dy;
    x4 = dx - y2o * k;
    y4 = dy - x2o * k;
    display.drawLine(x1, y1, x2, y2);
    display.drawLine(x1, y1, dx, dy);
    display.drawLine(x3, y3, x4, y4);
    display.drawLine(x3, y3, x2, y2);
    display.drawLine(x2, y2, x4, y4);
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

    display.init();

    last_dx = centreX;
    last_dy = centreY;

    /* Get Magnetometer chip id*/
    Serial.print("Device ID:");
    Serial.println(qmc.getChipID(), HEX);

    /* Config Magnetometer */
    int r = qmc.configMagnetometer(
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
                SensorQMC6310::RANGE_2G,
                /*
                * Output data rate
                * DATARATE_10HZ
                * DATARATE_50HZ
                * DATARATE_100HZ
                * DATARATE_200HZ
                * * */
                SensorQMC6310::DATARATE_100HZ,
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

    if (r != DEV_WIRE_NONE) {
        Serial.println("Device config failed!");
        while (1)delay(1000);
    }

    // Print register configuration information
    qmc.dumpCtrlRegister();

    Serial.println("Read data now...");
}

void loop()
{

    //Wiat data ready
    if (qmc.isDataReady()) {

        qmc.readData();


        display.drawString(29,  0, "N");
        display.drawString( 0, 28, "W");
        display.drawString(60, 28, "E");
        display.drawString(29, 53, "S");

        display.drawLine(1, 1, 7, 7);
        display.drawLine(62, 1, 56, 7);
        display.drawLine(1, 62, 7, 56);
        display.drawLine(56, 56, 62, 62);

        //Compass application from  https://github.com/G6EJD/ESP8266_micro_compass_HMC5883_OLED
        float heading = atan2(qmc.getY(), qmc.getX()); // Result is in radians
        // Now add the 'Declination Angle' for you location. Declination is the variation in magnetic field at your location.
        // Find your declination here: http://www.magnetic-declination.com/
        // At my location it is :  -2° 20' W, or -2.33 Degrees, which needs to be in radians so = -2.33 / 180 * PI = -0.041 West is + E is -
        // Make declination = 0 if you can't find your Declination value, the error is negible for nearly all locations
        float declination = -0.041;
        heading = heading + declination;

        if (heading < 0)    heading += 2 * PI; // Correct for when signs are reversed.
        if (heading > 2 * PI) heading -= 2 * PI; // Correct for when heading exceeds 360-degree, especially when declination is included
        angle = int(heading * 180 / M_PI); // Convert radians to degrees for more a more usual result
        // For the screen -X = up and +X = down and -Y = left and +Y = right, so does not follow coordinate conventions
        dx = (0.7 * radius * cos((angle - 90) * 3.14 / 180)) + centreX; // calculate X position for the screen coordinates - can be confusing!
        dy = (0.7 * radius * sin((angle - 90) * 3.14 / 180)) + centreY; // calculate Y position for the screen coordinates - can be confusing!
        arrow(last_dx, last_dy, centreX, centreY, 2, 2, BLACK);   // Erase last arrow
        arrow(dx, dy, centreX, centreY, 2, 2, WHITE);             // Draw arrow in new position

        display.setColor(BLACK);
        display.fillRect(80, 50, 25, 48);
        display.setColor(WHITE);
        display.drawString(80, 50, String(angle) + "°");
        display.display();

        last_dx = dx;
        last_dy = dy;

        // for debug.
        Serial.print("GYR: ");
        Serial.print("X:");
        Serial.print(qmc.getX());
        Serial.print(" Y:");
        Serial.print(qmc.getY());
        Serial.print(" Z:");
        Serial.print(qmc.getZ());
        Serial.println(" uT");
    }


    delay(100);
}
#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("The graphics library may not be supported on the current platform"); delay(1000);
}
#endif
