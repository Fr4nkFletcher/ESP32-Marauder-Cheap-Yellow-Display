/**
 ******************************************************************************
 * @file    X_NUCLEO_GNSS1A1_MicroNMEA_UART.ino
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
 
//NOTE: for compatibility with the Arduino Due some additional cabling needs to be performed:
//      pin D8 should be connected to pin D18 and pin D2 should be connected to pin D19

#include <MicroNMEA.h>

//Define Serial1 for STM32 Nucleo boards
#ifdef ARDUINO_ARCH_STM32
HardwareSerial Serial1(PA10, PA9);
#endif

#define RESET_PIN 7

// Refer to serial devices by use
HardwareSerial& console = Serial;
HardwareSerial& gps = Serial1;

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
   // Empty input buffer
   while (gps.available())
      gps.read();

   //reset the device
   digitalWrite(RESET_PIN, LOW);
   delay(50);
   digitalWrite(RESET_PIN, HIGH);

   //wait for reset to apply
   delay(2000);

}

void setup(void)
{
   console.begin(115200); // console
   gps.begin(9600); // gps

   pinMode(LED_BUILTIN, OUTPUT);
   digitalWrite(LED_BUILTIN, ledState);

   //Start the module
   pinMode(RESET_PIN, OUTPUT);
   digitalWrite(RESET_PIN, HIGH);
   console.println("Resetting GPS module ...");
   gpsHardwareReset();
   console.println("... done");

   // Change the echoing messages to the ones recognized by the MicroNMEA library
   MicroNMEA::sendSentence(gps, "$PSTMSETPAR,1201,0x00000042");
   MicroNMEA::sendSentence(gps, "$PSTMSAVEPAR");

   //Reset the device so that the changes could take plaace
   MicroNMEA::sendSentence(gps, "$PSTMSRR");

   delay(4000);

   //clear serial buffer
   while (gps.available())
      gps.read();

   pinMode(6, INPUT);
   attachInterrupt(digitalPinToInterrupt(6), ppsHandler, RISING);
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
   while (!ppsTriggered && gps.available())
   {
      //Fetch the character one by one
      char c = gps.read();
      console.print(c);
      //Pass the character to the library
      nmea.process(c);
   }

}
