/* Copyright (c) 2002,2003,2005,2006, Marek Michalkiewicz, Joerg Wunsch
   All rights reserved.

   Redistribution and use in source and binary forms, with or without
   modification, are permitted provided that the following conditions are met:

   * Redistributions of source code must retain the above copyright
     notice, this list of conditions and the following disclaimer.

   * Redistributions in binary form must reproduce the above copyright
     notice, this list of conditions and the following disclaimer in
     the documentation and/or other materials provided with the
     distribution.

   * Neither the name of the copyright holders nor the names of
     contributors may be used to endorse or promote products derived
     from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
  ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
  LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
  CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
  SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
  INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
  POSSIBILITY OF SUCH DAMAGE. */

/* $Id: io.h,v 1.24.2.7 2006/04/19 16:53:25 aesok Exp $ */

/** \defgroup avr_io <avr/io.h>: AVR device-specific IO definitions
    \code #include <avr/io.h> \endcode

    This header file includes the apropriate IO definitions for the
    device that has been specified by the <tt>-mmcu=</tt> compiler
    command-line switch.  This is done by diverting to the appropriate
    file <tt>&lt;avr/io</tt><em>XXXX</em><tt>.h&gt;</tt> which should
    never be included directly.  Some register names common to all
    AVR devices are defined directly within <tt>&lt;avr/io.h&gt;</tt>,
    but most of the details come from the respective include file.

    Note that this file always includes
    \code #include <avr/sfr_defs.h> \endcode
    See \ref avr_sfr for the details.

    Included are definitions of the IO register set and their
    respective bit values as specified in the Atmel documentation.
    Note that Atmel is not very consistent in its naming conventions,
    so even identical functions sometimes get different names on
    different devices.

    Also included are the specific names useable for interrupt
    function definitions as documented
    \ref avr_signames "here".

    Finally, the following macros are defined:

    - \b RAMEND
    <br>
    A constant describing the last on-chip RAM location.
    <br>
    - \b XRAMEND
    <br>
    A constant describing the last possible location in RAM.
    This is equal to RAMEND for devices that do not allow for
    external RAM.
    <br>
    - \b E2END
    <br>
    A constant describing the address of the last EEPROM cell.
    <br>
    - \b FLASHEND
    <br>
    A constant describing the last byte address in flash ROM.
    <br>
    - \b SPM_PAGESIZE
    <br>
    For devices with bootloader support, the flash pagesize
    (in bytes) to be used for the \c SPM instruction. */

#ifndef _AVR_IO_H_
#define _AVR_IO_H_

#include <avr/sfr_defs.h>

#ifndef __AVR_HAVE_MOVW__
#  if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
#   define __AVR_HAVE_MOVW__ 1
#  endif
#endif

#ifndef __AVR_HAVE_LPMX__
# if  defined(__AVR_ENHANCED__) && __AVR_ENHANCED__
#  define __AVR_HAVE_LPMX__ 1
# endif
#endif

/*
 * Registers common to all AVR devices.
 */

#if __AVR_ARCH__ != 1
/*
 * AVR architecture 1 has no RAM, thus no stack pointer.
 *
 * All other archs do have a stack pointer.  Some devices have only
 * less than 256 bytes of possible RAM locations (128 Bytes of SRAM
 * and no option for external RAM), thus SPH is officially "reserved"
 * for them.  We catch this case below after including the
 * device-specific ioXXXX.h file, by examining XRAMEND, and
 * #undef-ining SP and SPH in that case.
 */
/* Stack Pointer */
#define SP        _SFR_IO16(0x3D)
#define SPL       _SFR_IO8(0x3D)
#define SPH       _SFR_IO8(0x3E)
#endif /* #if __AVR_ARCH__ != 1 */

/* Status REGister */
#define SREG      _SFR_IO8(0x3F)

/* Status Register - SREG */
#define    SREG_I       7
#define    SREG_T       6
#define    SREG_H       5
#define    SREG_S       4
#define    SREG_V       3
#define    SREG_N       2
#define    SREG_Z       1
#define    SREG_C       0

/* Pointer definition */
#if __AVR_ARCH__ != 1
/* avr1 has only the Z pointer */
#define    XL           r26
#define    XH           r27
#define    YL           r28
#define    YH           r29
#endif /* #if __AVR_ARCH__ != 1 */
#define    ZL           r30
#define    ZH           r31

/*
 * Only few devices come without EEPROM.  In order to assemble the
 * EEPROM library components without defining a specific device, we
 * keep the EEPROM-related definitions here.
 */
#if defined(__COMPILING_AVR_LIBC__)

/* EEPROM Control Register */
#define EECR	_SFR_IO8(0x1C)

/* EEPROM Data Register */
#define EEDR	_SFR_IO8(0x1D)

/* EEPROM Address Register */
#define EEAR	_SFR_IO16(0x1E)
#define EEARL	_SFR_IO8(0x1E)
#define EEARH	_SFR_IO8(0x1F)

/* EEPROM Control Register */
#define    EERIE        3
#define    EEMWE        2
#define    EEWE         1
#define    EERE         0

#endif /* __COMPILING_AVR_LIBC__ */

#if defined (__AVR_AT94K__)
#  include <avr/ioat94k.h>
#elif defined (__AVR_AT43USB320__)
#  include <avr/io43u32x.h>
#elif defined (__AVR_AT43USB355__)
#  include <avr/io43u35x.h>
#elif defined (__AVR_AT76C711__)
#  include <avr/io76c711.h>
#elif defined (__AVR_AT86RF401__)
#  include <avr/io86r401.h>
#elif defined (__AVR_AT90PWM2__)
#  include <avr/io90pwmx.h>
#elif defined (__AVR_AT90PWM3__)
#  include <avr/io90pwmx.h>
#elif defined (__AVR_ATmega128__)
#  include <avr/iom128.h>
#elif defined (__AVR_ATmega1280__)
#  include <avr/iom1280.h>
#elif defined (__AVR_ATmega1281__)
#  include <avr/iom1281.h>
#elif defined (__AVR_AT90CAN32__)
#  include <avr/iocan32.h>
#elif defined (__AVR_AT90CAN64__)
#  include <avr/iocan64.h>
#elif defined (__AVR_AT90CAN128__)
#  include <avr/iocan128.h>
#elif defined (__AVR_AT90USB646__)
#  include <avr/iousb646.h>
#elif defined (__AVR_AT90USB647__)
#  include <avr/iousb647.h>
#elif defined (__AVR_AT90USB1286__)
#  include <avr/iousb1286.h>
#elif defined (__AVR_AT90USB1287__)
#  include <avr/iousb1287.h>
#elif defined (__AVR_ATmega64__)
#  include <avr/iom64.h>
#elif defined (__AVR_ATmega640__)
#  include <avr/iom640.h>
#elif defined (__AVR_ATmega644__)
#  include <avr/iom644.h>
#elif defined (__AVR_ATmega644P__)
#  include <avr/iom644.h>
#elif defined (__AVR_ATmega645__)
#  include <avr/iom645.h>
#elif defined (__AVR_ATmega6450__)
#  include <avr/iom6450.h>
#elif defined (__AVR_ATmega649__)
#  include <avr/iom649.h>
#elif defined (__AVR_ATmega6490__)
#  include <avr/iom6490.h>
#elif defined (__AVR_ATmega103__)
#  include <avr/iom103.h>
#elif defined (__AVR_ATmega32__)
#  include <avr/iom32.h>
#elif defined (__AVR_ATmega323__)
#  include <avr/iom323.h>
#elif defined (__AVR_ATmega324P__)
#  include <avr/iom324.h>
#elif defined (__AVR_ATmega325__)
#  include <avr/iom325.h>
#elif defined (__AVR_ATmega3250__)
#  include <avr/iom3250.h>
#elif defined (__AVR_ATmega329__)
#  include <avr/iom329.h>
#elif defined (__AVR_ATmega3290__)
#  include <avr/iom3290.h>
#elif defined (__AVR_ATmega406__)
#  include <avr/iom406.h>
#elif defined (__AVR_ATmega16__)
#  include <avr/iom16.h>
#elif defined (__AVR_ATmega161__)
#  include <avr/iom161.h>
#elif defined (__AVR_ATmega162__)
#  include <avr/iom162.h>
#elif defined (__AVR_ATmega163__)
#  include <avr/iom163.h>
#elif defined (__AVR_ATmega164P__)
#  include <avr/iom164.h>
#elif defined (__AVR_ATmega165__)
#  include <avr/iom165.h>
#elif defined (__AVR_ATmega168__)
#  include <avr/iom168.h>
#elif defined (__AVR_ATmega169__)
#  include <avr/iom169.h>
#elif defined (__AVR_ATmega8__)
#  include <avr/iom8.h>
#elif defined (__AVR_ATmega48__)
#  include <avr/iom48.h>
#elif defined (__AVR_ATmega88__)
#  include <avr/iom88.h>
#elif defined (__AVR_ATmega8515__)
#  include <avr/iom8515.h>
#elif defined (__AVR_ATmega8535__)
#  include <avr/iom8535.h>
#elif defined (__AVR_AT90S8535__)
#  include <avr/io8535.h>
#elif defined (__AVR_AT90C8534__)
#  include <avr/io8534.h>
#elif defined (__AVR_AT90S8515__)
#  include <avr/io8515.h>
#elif defined (__AVR_AT90S4434__)
#  include <avr/io4434.h>
#elif defined (__AVR_AT90S4433__)
#  include <avr/io4433.h>
#elif defined (__AVR_AT90S4414__)
#  include <avr/io4414.h>
#elif defined (__AVR_ATtiny22__)
#  include <avr/iotn22.h>
#elif defined (__AVR_ATtiny26__)
#  include <avr/iotn26.h>
#elif defined (__AVR_AT90S2343__)
#  include <avr/io2343.h>
#elif defined (__AVR_AT90S2333__)
#  include <avr/io2333.h>
#elif defined (__AVR_AT90S2323__)
#  include <avr/io2323.h>
#elif defined (__AVR_AT90S2313__)
#  include <avr/io2313.h>
#elif defined (__AVR_ATtiny2313__)
#  include <avr/iotn2313.h>
#elif defined (__AVR_ATtiny13__)
#  include <avr/iotn13.h>
#elif defined (__AVR_ATtiny25__)
#  include <avr/iotn25.h>
#elif defined (__AVR_ATtiny45__)
#  include <avr/iotn45.h>
#elif defined (__AVR_ATtiny85__)
#  include <avr/iotn85.h>
#elif defined (__AVR_ATtiny24__)
#  include <avr/iotn24.h>
#elif defined (__AVR_ATtiny44__)
#  include <avr/iotn44.h>
#elif defined (__AVR_ATtiny84__)
#  include <avr/iotn84.h>
#elif defined (__AVR_ATtiny261__)
#  include <avr/iotn261.h>
#elif defined (__AVR_ATtiny461__)
#  include <avr/iotn461.h>
#elif defined (__AVR_ATtiny861__)
#  include <avr/iotn861.h>
/* avr1: the following only supported for assembler programs */
#elif defined (__AVR_ATtiny28__)
#  include <avr/iotn28.h>
#elif defined (__AVR_AT90S1200__)
#  include <avr/io1200.h>
#elif defined (__AVR_ATtiny15__)
#  include <avr/iotn15.h>
#elif defined (__AVR_ATtiny12__)
#  include <avr/iotn12.h>
#elif defined (__AVR_ATtiny11__)
#  include <avr/iotn11.h>
#else
#  if !defined(__COMPILING_AVR_LIBC__)
#    warning "device type not defined"
#  endif
#endif

#include <avr/portpins.h>

#if __AVR_ARCH__ != 1
#  if XRAMEND < 0x100 && !defined(__COMPILING_AVR_LIBC__)
#    undef SP
#    define SP     _SFR_IO8(0x3D)
#    undef SPH
#  endif
#endif

#include <avr/version.h>

#endif /* _AVR_IO_H_ */
