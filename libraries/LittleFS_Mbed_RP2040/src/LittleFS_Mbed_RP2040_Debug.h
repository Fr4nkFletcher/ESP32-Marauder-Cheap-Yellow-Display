/****************************************************************************************************************************
  LittleFS_Mbed_RP2040_Debug.h - Filesystem wrapper for LittleFS on the Mbed RP2040
  
  For MBED RP2040-based boards such as Nano_RP2040_Connect, RASPBERRY_PI_PICO, ADAFRUIT_FEATHER_RP2040 and GENERIC_RP2040.
  Written by Khoi Hoang

  Built by Khoi Hoang https://github.com/khoih-prog/LittleFS_Mbed_RP2040
  Licensed under MIT license

  Version: 1.1.0

  Version Modified By   Date      Comments
  ------- -----------  ---------- -----------
  1.0.0   K Hoang      11/06/2021 Initial coding to support MBED RP2040-based boards such as RASPBERRY_PI_PICO. etc.
  1.0.1   K Hoang      16/08/2021 Fix FORCE_REFORMAT bug in example
  1.0.2   K Hoang      07/09/2021 Add mbed and ArduinoCore-mbed to architectures
  1.0.3   K Hoang      02/11/2021 Fix crashing issue for new flash
  1.1.0   K Hoang     30/12/2021 Fix `multiple-definitions` linker error
*****************************************************************************************************************************/  

#ifndef LittleFS_Mbed_RP2040_Debug_h
#define LittleFS_Mbed_RP2040_Debug_h

#ifdef LFS_DEBUG_OUTPUT
  #define DBG_PORT_LFS       LFS_DEBUG_OUTPUT
#else
  #define DBG_PORT_LFS       Serial
#endif

// Change _LFS_LOGLEVEL_ to set tracing and logging verbosity
// 0: DISABLED: no logging
// 1: ERROR: errors
// 2: WARN: errors and warnings
// 3: INFO: errors, warnings and informational (default)
// 4: DEBUG: errors, warnings, informational and debug

#ifndef _LFS_LOGLEVEL_
  #define _LFS_LOGLEVEL_       0
#endif

//////////////////////////////////////////

const char LFS_MARK[] = "[LFS] ";

#define LFS_PRINT_MARK    DBG_PORT_LFS.print(LFS_MARK)

#define LFS_PRINT         DBG_PORT_LFS.print
#define LFS_PRINTLN       DBG_PORT_LFS.println


//////////////////////////////////////////

#define LFS_LOGERROR0(x)     if(_LFS_LOGLEVEL_>0) { LFS_PRINT(x); }
#define LFS_LOGERROR(x)      if(_LFS_LOGLEVEL_>0) { LFS_PRINT_MARK; LFS_PRINTLN(x); }
#define LFS_LOGERROR1(x,y)   if(_LFS_LOGLEVEL_>0) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINTLN(y); }
#define LFS_LOGERROR2(x,y,z) if(_LFS_LOGLEVEL_>0) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINTLN(z); }
#define LFS_LOGERROR3(x,y,z,w) if(_LFS_LOGLEVEL_>0) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINTLN(w); }
#define LFS_LOGERROR5(x,y,z,w,xx,yy) if(_LFS_LOGLEVEL_>0) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINT(w); LFS_PRINT(xx); LFS_PRINTLN(yy); }

//////////////////////////////////////////

#define LFS_LOGWARN0(x)     if(_LFS_LOGLEVEL_>1) { LFS_PRINT(x); }
#define LFS_LOGWARN(x)      if(_LFS_LOGLEVEL_>1) { LFS_PRINT_MARK; LFS_PRINTLN(x); }
#define LFS_LOGWARN1(x,y)   if(_LFS_LOGLEVEL_>1) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINTLN(y); }
#define LFS_LOGWARN2(x,y,z) if(_LFS_LOGLEVEL_>1) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINTLN(z); }
#define LFS_LOGWARN3(x,y,z,w) if(_LFS_LOGLEVEL_>1) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINTLN(w); }
#define LFS_LOGWARN5(x,y,z,w,xx,yy) if(_LFS_LOGLEVEL_>1) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINT(w); LFS_PRINT(xx); LFS_PRINTLN(yy); }

//////////////////////////////////////////

#define LFS_LOGINFO0(x)     if(_LFS_LOGLEVEL_>2) { LFS_PRINT(x); }
#define LFS_LOGINFO(x)      if(_LFS_LOGLEVEL_>2) { LFS_PRINT_MARK; LFS_PRINTLN(x); }
#define LFS_LOGINFO1(x,y)   if(_LFS_LOGLEVEL_>2) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINTLN(y); }
#define LFS_LOGINFO2(x,y,z) if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINTLN(z); }
#define LFS_LOGINFO3(x,y,z,w) if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINTLN(w); }
#define LFS_LOGINFO5(x,y,z,w,xx,yy) if(_LFS_LOGLEVEL_>2) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINT(w); LFS_PRINT(xx); LFS_PRINTLN(yy); }

//////////////////////////////////////////

#define LFS_LOGDEBUG0(x)     if(_LFS_LOGLEVEL_>3) { LFS_PRINT(x); }
#define LFS_LOGDEBUG(x)      if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINTLN(x); }
#define LFS_LOGDEBUG1(x,y)   if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINTLN(y); }
#define LFS_LOGDEBUG2(x,y,z) if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINTLN(z); }
#define LFS_LOGDEBUG3(x,y,z,w) if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINTLN(w); }
#define LFS_LOGDEBUG5(x,y,z,w,xx,yy) if(_LFS_LOGLEVEL_>3) { LFS_PRINT_MARK; LFS_PRINT(x); LFS_PRINT(y); LFS_PRINT(z); LFS_PRINT(w); LFS_PRINT(xx); LFS_PRINTLN(yy); }

//////////////////////////////////////////

#endif    //LittleFS_Mbed_RP2040_Debug_h
