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
 * @file      PCF8563_TimeSynchronization.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2022-12-12
 *
 */
#include <Wire.h>
#include <SPI.h>
#include <Arduino.h>
#if defined(ARDUINO_ARCH_ESP32)
#include <time.h>
#include <WiFi.h>
#include <sntp.h>
#include "SensorPCF8563.hpp"

// lilygo t-beam-s3-core pin
#define SENSOR_SDA                     42
#define SENSOR_SCL                     41
#define SENSOR_IRQ                     14

const char *ssid       = "YOUR_SSID";
const char *password   = "YOUR_PASS";

const char *ntpServer1 = "pool.ntp.org";
const char *ntpServer2 = "time.nist.gov";
const long  gmtOffset_sec = 3600;
const int   daylightOffset_sec = 3600;

const char *time_zone = "CST-8";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

SensorPCF8563 rtc;
uint32_t lastMillis;


// Callback function (get's called when time adjusts via NTP)
void timeavailable(struct timeval *t)
{
    Serial.println("Got time adjustment from NTP, Write the hardware clock");

    // Write synchronization time to hardware
    rtc.hwClockWrite();
}

void setup()
{
    Serial.begin(115200);
    while (!Serial);



    pinMode(SENSOR_IRQ, INPUT_PULLUP);

    if (!rtc.begin(Wire, PCF8563_SLAVE_ADDRESS, SENSOR_SDA, SENSOR_SCL)) {
        Serial.println("Failed to find PCF8563 - check your wiring!");
        while (1) {
            delay(1000);
        }
    }

    // set notification call-back function
    sntp_set_time_sync_notification_cb( timeavailable );

    /**
     * NTP server address could be aquired via DHCP,
     *
     * NOTE: This call should be made BEFORE esp32 aquires IP address via DHCP,
     * otherwise SNTP option 42 would be rejected by default.
     * NOTE: configTime() function call if made AFTER DHCP-client run
     * will OVERRIDE aquired NTP server address
     */
    sntp_servermode_dhcp(1);    // (optional)

    /**
     * This will set configured ntp servers and constant TimeZone/daylightOffset
     * should be OK if your time zone does not need to adjust daylightOffset twice a year,
     * in such a case time adjustment won't be handled automagicaly.
     */
    // configTime(gmtOffset_sec, daylightOffset_sec, ntpServer1, ntpServer2);

    /**
     * A more convenient approach to handle TimeZones with daylightOffset
     * would be to specify a environmnet variable with TimeZone definition including daylight adjustmnet rules.
     * A list of rules for your zone could be obtained from https://github.com/esp8266/Arduino/blob/master/cores/esp8266/TZ.h
     */
    configTzTime(time_zone, ntpServer1, ntpServer2);

    //connect to WiFi
    Serial.printf("Connecting to %s ", ssid);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println(" CONNECTED");

}




void loop()
{
    if (millis() - lastMillis > 1000) {

        lastMillis = millis();

        // hardware clock
        struct tm hwTimeinfo;
        rtc.getDateTime(&hwTimeinfo);
        Serial.print("Hardware clock :");
        Serial.println(&hwTimeinfo, "%A, %B %d %Y %H:%M:%S");

        // system clock
        struct tm timeinfo;
        if (!getLocalTime(&timeinfo)) {
            Serial.println("No time available (yet)");
            return;
        }

        Serial.print("System   clock :");
        Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
        Serial.println();

    }
}
#else
void setup()
{
    Serial.begin(115200);
}

void loop()
{
    Serial.println("Examples only ESP32"); delay(1000);
}
#endif


