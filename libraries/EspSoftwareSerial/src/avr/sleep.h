/* Copyright (c) 2002, 2004 Theodore A. Roth
   Copyright (c) 2004 Eric B. Weddington
   Copyright (c) 2005 Joerg Wunsch
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

/* $Id: sleep.h,v 1.13.2.1 2005/12/28 19:11:41 joerg_wunsch Exp $ */

#ifndef _AVR_SLEEP_H_
#define _AVR_SLEEP_H_ 1

#include <avr/io.h>



/* Define internal sleep types for the various devices. */
/* Also define some internal masks for use in set_sleep_mode() */
#if defined(__AVR_ATmega161__)

#define _SLEEP_TYPE 5

#elif defined(__AVR_ATmega162__) || defined(__AVR_ATmega8515__)

#define _SLEEP_TYPE 4

#elif defined(SM) && !defined(SM0) && !defined(SM1) && !defined(SM2)

#define _SLEEP_TYPE 1
#define _SLEEP_MODE_MASK _BV(SM)

#elif !defined(SM) && defined(SM0) && defined(SM1) && !defined(SM2)

#define _SLEEP_TYPE 2
#define _SLEEP_MODE_MASK (_BV(SM0) | _BV(SM1))

#elif !defined(SM) && defined(SM0) && defined(SM1) && defined(SM2)

#define _SLEEP_TYPE 3
#define _SLEEP_MODE_MASK (_BV(SM0) | _BV(SM1) | _BV(SM2))

#else

#error "No SLEEP mode defined for this device."

#endif



/* Define the internal control register to use for sleep_mode(). */
#if defined(SMCR)

#define _SLEEP_CONTROL_REG SMCR

#elif defined(__AVR_AT94K__)

#define _SLEEP_CONTROL_REG MCUR

#else

#define _SLEEP_CONTROL_REG MCUCR

#endif



/** \defgroup avr_sleep <avr/sleep.h>: Power Management and Sleep Modes

    \code #include <avr/sleep.h>\endcode

    Use of the \c SLEEP instruction can allow an application to reduce its
    power comsumption considerably. AVR devices can be put into different
    sleep modes. Refer to the datasheet for the details relating to the device
    you are using.

    There are several macros provided in this header file to actually
    put the device into sleep mode.  The simplest way is to optionally
    set the desired sleep mode using \c set_sleep_mode() (it usually
    defaults to idle mode where the CPU is put on sleep but all
    peripheral clocks are still running), and then call
    \c sleep_mode().  Unless it is the purpose to lock the CPU hard
    (until a hardware reset), interrupts need to be enabled at this
    point.  This macro automatically takes care to enable the sleep mode
    in the CPU before going to sleep, and disable it again afterwards.

    As this combined macro might cause race conditions in some
    situations, the individual steps of manipulating the sleep enable
    (SE) bit, and actually issuing the \c SLEEP instruction are provided
    in the macros \c sleep_enable(), \c sleep_disable(), and
    \c sleep_cpu().  This also allows for test-and-sleep scenarios that
    take care of not missing the interrupt that will awake the device
    from sleep.

    Example:
    \code
    #include <avr/interrupt.h>
    #include <avr/sleep.h>

    ...
      cli();
      if (some_condition) {
        sleep_enable();
        sei();
	sleep_cpu();
	sleep_disable();
      }
      sei();
    \endcode

    This sequence ensures an atomic test of \c some_condition with
    interrupts being disabled.  If the condition is met, sleep mode
    will be prepared, and the \c SLEEP instruction will be scheduled
    immediately after an \c SEI instruction.  As the intruction right
    after the \c SEI is guaranteed to be executed before an interrupt
    could trigger, it is sure the device will really be put on sleep.
*/


/** \name Sleep Modes

    \note Some of these modes are not available on all devices. See the
    datasheet for target device for the available sleep modes. */

/* @{ */


/* Define the sleep modes according to the internal sleep types. */
#if _SLEEP_TYPE == 1
#define SLEEP_MODE_IDLE         0
#define SLEEP_MODE_PWR_DOWN     _BV(SM)
#endif


#if _SLEEP_TYPE == 2

/*
 * Type 2 devices are not completely identical, so we need a few
 * #ifdefs here.
 *
 * Note that it appears the datasheet of the tiny2313 has the bottom
 * two lines of table 13 with the wrong SM0/SM1 values.
 */
#define SLEEP_MODE_IDLE         0

#if !defined(__AVR_ATtiny2313__) && !defined(__AVR_AT94K__)
/* no ADC in ATtiny2313, SM0 is alternative powerdown mode */
/* no ADC in AT94K, setting SM0 only is reserved */
# define SLEEP_MODE_ADC          _BV(SM0)
#endif /* !defined(__AVR_ATtiny2313__) && !defined(__AVR_AT94K__) */

#define SLEEP_MODE_PWR_DOWN     _BV(SM1)

#if defined(__AVR_ATtiny2313__) || defined(__AVR_ATtiny26__)
/* tiny2313 and tiny26 have standby rather than powersave */
# define SLEEP_MODE_STANDBY      (_BV(SM0) | _BV(SM1))
#elif !defined(__AVR_ATtiny13__)
/* SM0|SM1 is reserved on the tiny13 */
# define SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
#endif

#endif


#if _SLEEP_TYPE == 3 || defined(__DOXYGEN__)
/** \ingroup avr_sleep
    \def SLEEP_MODE_IDLE
    Idle mode. */
#define SLEEP_MODE_IDLE         0
/** \ingroup avr_sleep
    \def SLEEP_MODE_ADC
    ADC Noise Reduction Mode. */
#define SLEEP_MODE_ADC          _BV(SM0)
/** \ingroup avr_sleep
    \def SLEEP_MODE_PWR_DOWN
    Power Down Mode. */
#define SLEEP_MODE_PWR_DOWN     _BV(SM1)
/** \ingroup avr_sleep
    \def SLEEP_MODE_PWR_SAVE
    Power Save Mode. */
#define SLEEP_MODE_PWR_SAVE     (_BV(SM0) | _BV(SM1))
/** \ingroup avr_sleep
    \def SLEEP_MODE_STANDBY
    Standby Mode. */
#define SLEEP_MODE_STANDBY      (_BV(SM1) | _BV(SM2))
/** \ingroup avr_sleep
    \def SLEEP_MODE_EXT_STANDBY
    Extended Standby Mode. */
#define SLEEP_MODE_EXT_STANDBY  (_BV(SM0) | _BV(SM1) | _BV(SM2))
#endif


#if _SLEEP_TYPE == 4
#define SLEEP_MODE_IDLE         0
#define SLEEP_MODE_PWR_DOWN     1
#define SLEEP_MODE_PWR_SAVE     2
#define SLEEP_MODE_ADC          3
#define SLEEP_MODE_STANDBY      4
#define SLEEP_MODE_EXT_STANDBY  5
#endif


#if _SLEEP_TYPE == 5
#define SLEEP_MODE_IDLE         0
#define SLEEP_MODE_PWR_DOWN     1
#define SLEEP_MODE_PWR_SAVE     2
#endif





/* @} */

/** \name Sleep Functions */

/* @{ */

/** \ingroup avr_sleep

    Select a sleep mode. */

#if defined(__DOXYGEN__)

extern void set_sleep_mode (uint8_t mode);

#elif _SLEEP_TYPE == 5

#define set_sleep_mode(mode) \
do { \
    MCUCR = ((MCUCR & ~_BV(SM1)) | ((mode) == SLEEP_MODE_PWR_DOWN || (mode) == SLEEP_MODE_PWR_SAVE ? _BV(SM1) : 0)); \
    EMCUCR = ((EMCUCR & ~_BV(SM0)) | ((mode) == SLEEP_MODE_PWR_SAVE ? _BV(SM0) : 0)); \
} while(0)

#elif _SLEEP_TYPE == 4

#define set_sleep_mode(mode) \
do { \
    MCUCR = ((MCUCR & ~_BV(SM1)) | ((mode) == SLEEP_MODE_IDLE ? 0 : _BV(SM1))); \
    MCUCSR = ((MCUCSR & ~_BV(SM2)) | ((mode) == SLEEP_MODE_STANDBY  || (mode) == SLEEP_MODE_EXT_STANDBY ? _BV(SM2) : 0)); \
    EMCUCR = ((EMCUCR & ~_BV(SM0)) | ((mode) == SLEEP_MODE_PWR_SAVE || (mode) == SLEEP_MODE_EXT_STANDBY ? _BV(SM0) : 0)); \
} while(0)

#elif _SLEEP_TYPE == 3 || _SLEEP_TYPE == 2 || _SLEEP_TYPE == 1

#define set_sleep_mode(mode) \
do { \
    _SLEEP_CONTROL_REG = ((_SLEEP_CONTROL_REG & ~_SLEEP_MODE_MASK) | (mode)); \
} while(0)

#endif




/** \ingroup avr_sleep

    Put the device in sleep mode. How the device is brought out of sleep mode
    depends on the specific mode selected with the set_sleep_mode() function.
    See the data sheet for your device for more details. */
#if defined(__DOXYGEN__)

extern void sleep_mode (void);

#else

#define sleep_mode()                           \
do {                                           \
    _SLEEP_CONTROL_REG |= _BV(SE);             \
    __asm__ __volatile__ ("sleep" "\n\t" :: ); \
    _SLEEP_CONTROL_REG &= ~_BV(SE);            \
} while (0)

#endif



#if defined(__DOXYGEN__)

/** \ingroup avr_sleep

    Set the SE (sleep enable) bit.
*/
extern void sleep_enable (void);

#else

#define sleep_enable()             \
do {                               \
  _SLEEP_CONTROL_REG |= _BV(SE);   \
} while(0)

#endif


#if defined(__DOXYGEN__)

/** \ingroup avr_sleep

    Clear the SE (sleep enable) bit.
*/
extern void sleep_disable (void);

#else

#define sleep_disable()            \
do {                               \
  _SLEEP_CONTROL_REG &= ~_BV(SE);  \
} while(0)

#endif


/** \ingroup avr_sleep

    Put the device into sleep mode.  The SE bit must be set
    beforehand, and it is recommended to clear it afterwards.
*/
#if defined(__DOXYGEN__)

extern void sleep_cpu (void);

#else

#define sleep_cpu()                              \
do {                                             \
  __asm__ __volatile__ ( "sleep" "\n\t" :: );    \
} while(0)

#endif


/*@}*/

#endif /* _AVR_SLEEP_H_ */
