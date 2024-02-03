#include <MicroNMEA.h>

// To display free memory include the MemoryFree library, see
// https://github.com/maniacbug/MemoryFree and uncomment the line
// below
//#include <MemoryFree.h>

// Refer to serial devices by use
HardwareSerial& console = Serial;
HardwareSerial& gps = Serial1;

char nmeaBuffer[100];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));
bool ledState = LOW;
volatile bool ppsTriggered = false;

void ppsHandler(void);


void ppsHandler(void)
{
	ppsTriggered = true;
}

void printUnknownSentence(MicroNMEA& nmea)
{
	console.println();
	console.print("Unknown sentence: ");
	console.println(nmea.getSentence());
}

void gpsHardwareReset()
{
	// Empty input buffer
	while (gps.available())
		gps.read();

	digitalWrite(A0, LOW);
	delay(50);
	digitalWrite(A0, HIGH);

	// Reset is complete when the first valid message is received
	while (1) {
		while (gps.available()) {
			char c = gps.read();
			if (nmea.process(c))
				return;

		}
	}
}

void setup(void)
{
	console.begin(115200); // console
	gps.begin(115200); // gps

	pinMode(LED_BUILTIN, OUTPUT);
	digitalWrite(LED_BUILTIN, ledState);

	nmea.setUnknownSentenceHandler(printUnknownSentence);

	pinMode(A0, OUTPUT);
	digitalWrite(A0, HIGH);
	console.println("Resetting GPS module ...");
	gpsHardwareReset();
	console.println("... done");

	// Clear the list of messages which are sent.
	MicroNMEA::sendSentence(gps, "$PORZB");

	// Send only RMC and GGA messages.
	MicroNMEA::sendSentence(gps, "$PORZB,RMC,1,GGA,1");

	// Disable compatability mode (NV08C-CSM proprietary message) and
	// adjust precision of time and position fields
	MicroNMEA::sendSentence(gps, "$PNVGNME,2,9,1");
	// MicroNMEA::sendSentence(gps, "$PONME,2,4,1,0");

#ifdef ARDUINO_AVR_CALUNIUM
	pinMode(6, INPUT);
	attachInterrupt(2, ppsHandler, RISING);
#else
#error Please configure interrupt handler code for alternative board.
#endif

}

void loop(void)
{
	if (ppsTriggered) {
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

#ifdef MEMORY_FREE_H
		console.print("freeMemory()=");
		console.println(freeMemory());
#endif
		console.println("-----------------------");
		nmea.clear();
	}

	while (!ppsTriggered && gps.available()) {
		char c = gps.read();
		console.print(c);
		nmea.process(c);
	}

}
