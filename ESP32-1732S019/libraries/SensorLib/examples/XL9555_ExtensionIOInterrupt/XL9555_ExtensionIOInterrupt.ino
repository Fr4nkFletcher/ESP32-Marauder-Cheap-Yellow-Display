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
 * @file      XL9555_ExtensionIOInterrupt.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @date      2024-09-14
 *
 */
#include <Arduino.h>
#include "ExtensionIOXL9555.hpp"

#ifndef SENSOR_SDA
#define SENSOR_SDA  17
#endif

#ifndef SENSOR_SCL
#define SENSOR_SCL  18
#endif

#ifndef SENSOR_IRQ
#define SENSOR_IRQ  -1
#endif

ExtensionIOXL9555 io;

void printBinary(uint8_t port, uint32_t num, int bits, bool new_line = false)
{

    Serial.print("Port");
    Serial.print(port);
    Serial.print(":");

    char binary[17];
    int i = bits - 1;
    binary[bits] = '\0';
    for (int j = 0; j < bits; j++) {
        binary[j] = '0';
    }
    for (; i >= 0; i--) {
        binary[i] = (num & 1) + '0';
        num >>= 1;
    }

    Serial.print(binary);
    if (new_line) {
        Serial.println();
    } else {
        Serial.print("\t");
    }
}


void setup()
{
    Serial.begin(115200);

    // Set the interrupt input to input pull-up
    if (SENSOR_IRQ > 0) {
        pinMode(SENSOR_IRQ, INPUT_PULLUP);
    }

    /*
    *
    *    If the device address is not known, the 0xFF parameter can be passed in.
    *
    *    XL9555_UNKOWN_ADDRESS  = 0xFF
    *
    *    If the device address is known, the device address is given
    *
    *    XL9555_SLAVE_ADDRESS0  = 0x20
    *    XL9555_SLAVE_ADDRESS1  = 0x21
    *    XL9555_SLAVE_ADDRESS2  = 0x22
    *    XL9555_SLAVE_ADDRESS3  = 0x23
    *    XL9555_SLAVE_ADDRESS4  = 0x24
    *    XL9555_SLAVE_ADDRESS5  = 0x25
    *    XL9555_SLAVE_ADDRESS6  = 0x26
    *    XL9555_SLAVE_ADDRESS7  = 0x27
    */
    const uint8_t chip_address = XL9555_UNKOWN_ADDRESS;

    if (!io.init(Wire, SENSOR_SDA, SENSOR_SCL, chip_address)) {
        while (1) {
            Serial.println("Failed to find XL9555 - check your wiring!");
            delay(1000);
        }
    }

    /*
    * Note:
    *   XL9555 has an internal pull-up resistor and maintains a high level when idle.
    *   XL9535 has a floating input internally and an uncertain state when idle, requiring an external pull-up/pull-down resistor
    * * */

    // Set PORT0 as input,mask = 0xFF = all pin input
    io.configPort(ExtensionIOXL9555::PORT0, 0xFF);

    // Set PORT1 as input,mask = 0xFF = all pin input
    io.configPort(ExtensionIOXL9555::PORT1, 0xFF);


}

void loop()
{
    // When the interrupt occurs, we read the mask value of PORT
    if (digitalRead(SENSOR_IRQ) == LOW) {

        // Read 8-bit port 0 input status
        int val = io.readPort(0);

        // Read 8-bit port 1 input status
        int val1 = io.readPort(1);

        printBinary(1, val1, 8);
        printBinary(0, val, 8);

        // Read 16-bit gpio input status
        uint16_t all_val = io.read();
        printBinary(3, all_val, 16, true);

        int i = 15;
        for (; i >= 0; i--) {
            if (!(all_val & 1)) {
                Serial.printf("GPIO %d is low\n", 15 - i);
            }
            all_val >>= 1;
        }
    }
}

