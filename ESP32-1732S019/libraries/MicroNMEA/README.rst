MicroNMEA
=========

MicroNMEA is a compact Arduino library to parse a subset of NMEA
sentences, which can originate from either GPS or GNSS receivers. Only
two types of messages are parsed, ``$GPGGA`` (and its GNSS
versions ``$GNGGA``, ``$GLGGA``, and ``$GAGGA``) and ``$GPRMC`` (and
its GNSS versions ``$GNRMC``, ``$GLRMC``, and ``$GARMC``). From these
two NMEA sentences MicroNMEA can output date, time, latitude,
longitude, altitude, number of satellites used, horizontal dilution of
precision (HDOP), course and speed. When other NMEA sentences are
detected they can be passed to an optional callback function for
decoding or logging. Checksum failures can be indicated with another
optional callback function.

NMEA sentences can easily be sent to an output stream with the
``sendSentence()`` function which computes and appends the checksum,
as well as the correct ``<CR><LF>`` terminators.

License
-------

The MicroNMEA library is released under the GNU Lesser General Public License, version 2.1. 
http://www.gnu.org/licenses/lgpl-2.1.html

Initialization and basic usage
------------------------------

A ``MicroNMEA`` object must be defined with a pointer to a buffer and the buffer length::

    char buffer[85];
    MicroNMEA nmea(buffer, sizeof(buffer));
  
This approach enables the user to size the buffer appropriately according to use without requiring ``malloc()``; for instance the buffer size can be increased if the temporal and spatial resolution reported have been increased by some properietary NMEA command.

Output data from the GPS/GNSS device must be passed to the library for processing::

    while (gps.available()) {
        char c = gps.read();
        if (nmea.process(c)) {
            // Complete NMEA command read and processed, do something
            ...
        }
    }

In the code fragment above ``gps`` is the output stream of the GPS/GNSS device.

Retrieving information from MicroNMEA
-------------------------------------
Location, date, time and various status information can be requested using the appropriate member functions which are described below. To obtain all of the information listed below MicroNMEA must process both ``GxGGA`` and ``GxRMC`` sentences::

    char getNavSystem() const

Returns a single character indicating the navigation system in use:

+--------+-----------------------------------------------------------------+
| ``P``  | Navigation results based only on GPS satellites.                |
+--------+-----------------------------------------------------------------+
| ``L``  | Navigation results based only on GLONASS satellites.            |
+--------+-----------------------------------------------------------------+
| ``A``  | Navigation results based only on Galileo satellites.            |
+--------+-----------------------------------------------------------------+
| ``N``  | GNSS, navigation results from multiple satellite constellations.|
+--------+-----------------------------------------------------------------+
| ``\0`` | No valid fix                                                    |
+--------+-----------------------------------------------------------------+

::

    uint8_t getHDOP(void) const

Horizontal dilution of precision in tenths (i.e., divide by 10 to get true HDOP). ::
 
    bool isValid(void) const

Validity of latest fix. ::

    long getLatitude(void) const

Latitude in millionths of a degree, North is positive. ::

    long getLongitude(void) const

Longitude in millionths of a degree, East is positive. ::

    bool getAltitude(long &alt) const

Altitude in millimetres, returns true if the altitude was obtained from a valid fix. ::

    uint16_t getYear(void) const
    uint8_t getMonth(void) const
    uint8_t getDay(void) const
    uint8_t getHour(void) const
    uint8_t getMinute(void) const
    uint8_t getSecond(void) const
    uint8_t getHundredths(void) const
    
Date and time. ::

    long getCourse(void) const

Course over ground, in thousandths of a degree. ::

    long getSpeed(void) const

Speed over ground, in thousandths of a knot. ::

    void clear(void)

Clear all stored values. ``isValid()`` will return false. Year, month and day will all be zero. Hour, minute and second time will be set to 99. Speed, course and altitude will be set to ``LONG_MIN``; the altitude validity flag will be false. Latitude and longitude will be set to 999 degrees.

Callback and associated functions
---------------------------------

::

    void setBadChecksumHandler(void (*handler)(MicroNMEA& nmea))

``setBadChecksumHandler()`` enables MicroNMEA to call a function when a bad NMEA checksum is detected. The callback function should accept a single parameter (a ``MicroNMEA`` object passed by reference) and return ``void``. ::

    void setUnknownSentenceHandler(void (*handler)(MicroNMEA& nmea))

``setUnknownSentenceHandler()`` enables MicroNMEA to call a function when a valid but unknown NMEA command is 
received. The callback function should accept a single parameter (a ``MicroNMEA`` object passed by reference) and return ``void``. ::

    const char* getSentence(void) const

Return the current NMEA sentence. Useful when using callback functions. ::

    char getTalkerID(void) const
    
Return the talker ID from the last processed NMEA sentence. The meaning is the same as the return value from ``getNavSystem()``. If ``$GxGSV`` messages are received then talker ID could be from any of the GNSS constellations. ::

    const char* getMessageID(void) const
    
Return the message ID from the last processed NMEA sentence, e.g, ``RMC``, ``GGA``. Useful when using callback functions.


Contributors
------------

- Steve Marple
- Christopher Liebman
- per1234
- Noah-Jonathan Rosa
- Philipp Tölke

Documentation
-------------

For documentation please see Read The Docs, https://micronmea.readthedocs.io/en/latest/.
