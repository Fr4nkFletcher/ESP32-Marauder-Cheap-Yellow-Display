/**
 ******************************************************************************
 * @file    X_NUCLEO_GNSS1A1_MicroNMEA_I2C.ino
 * @author  AST
 * @version V1.0.0
 * @date    January 2018
 * @brief   Arduino test application for the STMicrolectronics X-NUCLEO-GNSS1A1
 *          GNSS module expansion board based on TeseoLIV3F.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2019 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
 */

//NOTE: In order for this example to work, the jumper J4 on the device should
//      be moved to J8
//NOTE: This example is compatible with the Arduino Uno board

#include <MicroNMEA.h>
#include <Wire.h>

//I2C communication parameters
#define DEFAULT_DEVICE_ADDRESS 0x3A
#define DEFAULT_DEVICE_PORT 0xFF
#define I2C_DELAY 1

#define RESET_PIN 7

#ifdef ARDUINO_SAM_DUE
#define DEV_I2C Wire1
#endif

#ifdef ARDUINO_ARCH_STM32
#define DEV_I2C Wire
#endif

#ifdef ARDUINO_ARCH_AVR
#define DEV_I2C Wire
#endif

// Refer to Stream devices by use
HardwareSerial& console = Serial;
TwoWire& gps = DEV_I2C;

//I2C read data structures
char buff[32];
int idx = 0;

//MicroNMEA library structures
char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));


bool ledState = LOW;
volatile bool ppsTriggered = false;


void ppsHandler(void);


void ppsHandler(void)
{
  ppsTriggered = true;
}


void gpsHardwareReset()
{
   //reset the device
   digitalWrite(RESET_PIN, LOW);
   delay(50);
   digitalWrite(RESET_PIN, HIGH);

   //wait for reset to apply
   delay(2000);

}

//Read 32 bytes from I2C
void readI2C(char *inBuff)
{
   gps.beginTransmission(DEFAULT_DEVICE_ADDRESS);
   gps.write((uint8_t) DEFAULT_DEVICE_PORT);
   gps.endTransmission(false);
   gps.requestFrom((uint8_t)DEFAULT_DEVICE_ADDRESS, (uint8_t) 32);
   int i = 0;
   while (gps.available())
   {
      inBuff[i]= gps.read();
      i++;
   }
}

//Send a NMEA command via I2C
void sendCommand(char *cmd)
{
   gps.beginTransmission(DEFAULT_DEVICE_ADDRESS);
   gps.write((uint8_t) DEFAULT_DEVICE_PORT);
   MicroNMEA::sendSentence(gps, cmd);
   gps.endTransmission(true);
}

void setup(void)
{
   console.begin(115200); // console
   gps.begin(); // gps

   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, ledState);

   //Start the module
   pinMode(RESET_PIN, OUTPUT);
   digitalWrite(RESET_PIN, HIGH);
   console.println("Resetting GPS module ...");
   gpsHardwareReset();
   console.println("... done");

   // Change the echoing messages to the ones recognized by the MicroNMEA library
   sendCommand("$PSTMSETPAR,1231,0x00000042");
   sendCommand("$PSTMSAVEPAR");

   //Reset the device so that the changes could take plaace
   sendCommand("$PSTMSRR");

   delay(4000);

   //Reinitialize I2C after the reset
   gps.begin();

   //clear i2c buffer
   char c;
   idx = 0;
   memset(buff, 0, 32);
   do
   {
      if (idx == 0)
      {
         readI2C(buff);
         delay(I2C_DELAY);
      }
      c = buff[idx];
      idx++;
      idx %= 32;
   }
   while ((uint8_t) c != 0xFF);

   pinMode(2, INPUT);
   attachInterrupt(digitalPinToInterrupt(2), ppsHandler, RISING);
}

void loop(void)
{
   //If a message is recieved print all the informations
   if (ppsTriggered)
   {
      ppsTriggered = false;
      ledState = !ledState;
      digitalWrite(LED_BUILTIN, ledState);

      // Output GPS information from previous second
      console.print("Valid fix: ");
      console.println(nmea.isValid() ? "yes" : "no");

      console.print("Nav. system: ");
      if (nmea.getNavSystem())
         console.println(nmea.getNavSystem());
      else
         console.println("none");

      console.print("Num. satellites: ");
      console.println(nmea.getNumSatellites());

      console.print("HDOP: ");
      console.println(nmea.getHDOP()/10., 1);

      console.print("Date/time: ");
      console.print(nmea.getYear());
      console.print('-');
      console.print(int(nmea.getMonth()));
      console.print('-');
      console.print(int(nmea.getDay()));
      console.print('T');
      console.print(int(nmea.getHour()));
      console.print(':');
      console.print(int(nmea.getMinute()));
      console.print(':');
      console.println(int(nmea.getSecond()));

      long latitude_mdeg = nmea.getLatitude();
      long longitude_mdeg = nmea.getLongitude();
      console.print("Latitude (deg): ");
      console.println(latitude_mdeg / 1000000., 6);

      console.print("Longitude (deg): ");
      console.println(longitude_mdeg / 1000000., 6);

      long alt;
      console.print("Altitude (m): ");
      if (nmea.getAltitude(alt))
         console.println(alt / 1000., 3);
      else
         console.println("not available");

      console.print("Speed: ");
      console.println(nmea.getSpeed() / 1000., 3);
      console.print("Course: ");
      console.println(nmea.getCourse() / 1000., 3);
      console.println("-----------------------");
      nmea.clear();
   }

   //While the message isn't complete
   while (!ppsTriggered )
   {
      char c ;
      if (idx == 0)
      {
         readI2C(buff);
         delay(I2C_DELAY);
      }
      //Fetch the character one by one
      c = buff[idx];
      idx++;
      idx %= 32;
      //If we have a valid character pass it to the library
      if ((uint8_t) c != 0xFF)
      {
         console.print(c);
         nmea.process(c);
      }
   }

}
