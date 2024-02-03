/* Copyright (c) 2002,2005 Marek Michalkiewicz
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

/* $Id: interrupt.h,v 1.16.2.1 2006/04/19 20:51:30 joerg_wunsch Exp $ */

#ifndef _AVR_INTERRUPT_H_
#define _AVR_INTERRUPT_H_

#include <avr/io.h>

/** \name Global manipulation of the interrupt flag

    The global interrupt flag is maintained in the I bit of the status
    register (SREG). */

/*@{*/

#if defined(__DOXYGEN__)
/** \def sei()
    \ingroup avr_interrupts

    \code#include <avr/interrupt.h>\endcode

    Enables interrupts by setting the global interrupt mask. This function
    actually compiles into a single line of assembly, so there is no function
    call overhead. */
extern void sei(void);
#else  /* !DOXYGEN */
# define sei()  __asm__ __volatile__ ("sei" ::)
#endif /* DOXYGEN */

#if defined(__DOXYGEN__)
/** \def cli()
    \ingroup avr_interrupts

    \code#include <avr/interrupt.h>\endcode

    Disables all interrupts by clearing the global interrupt mask. This function
    actually compiles into a single line of assembly, so there is no function
    call overhead. */
extern void cli(void);
#else  /* !DOXYGEN */
# define cli()  __asm__ __volatile__ ("cli" ::)
#endif /* DOXYGEN */

/*@}*/

/** \name Macros for writing interrupt handler functions */

/*@{*/

/** \def ISR(vector)
    \ingroup avr_interrupts

    \code#include <avr/interrupt.h>\endcode

    Introduces an interrupt handler function (interrupt service
    routine) that runs with global interrupts initially disabled.

    \c vector must be one of the interrupt vector names that are
    valid for the particular MCU type.
*/

#ifdef __cplusplus
#define ISR(vector)					\
extern "C" void vector(void) __attribute__ ((signal));	\
void vector (void)
#else
#define ISR(vector)					\
void vector (void) __attribute__ ((signal));		\
void vector (void)
#endif

/** \def SIGNAL(signame)
    \ingroup avr_interrupts

    \code#include <avr/interrupt.h>\endcode

    Introduces an interrupt handler function that runs with global interrupts
    initially disabled.

    This is the same as the ISR macro.
    \note Do not use anymore in new code, it will be deprecated
    in a future release.
*/

#ifdef __cplusplus
#define SIGNAL(signame)					\
extern "C" void signame(void) __attribute__ ((signal));	\
void signame (void)
#else
#define SIGNAL(signame)					\
void signame (void) __attribute__ ((signal));		\
void signame (void)
#endif

/** \def EMPTY_INTERRUPT(signame)
    \ingroup avr_interrupts

    \code#include <avr/interrupt.h>\endcode

    Defines an empty interrupt handler function. This will not generate
    any prolog or epilog code and will only return from the ISR. Do not
    define a function body as this will define it for you.
    Example:
    \code EMPTY_INTERRUPT(ADC_vect);\endcode */

#ifdef __cplusplus
#define EMPTY_INTERRUPT(vector)                \
extern "C" void vector(void) __attribute__ ((naked));    \
void vector (void) {  __asm__ __volatile__ ("reti" ::); }
#else
#define EMPTY_INTERRUPT(vector)                \
void vector (void) __attribute__ ((naked));    \
void vector (void) { __asm__ __volatile__ ("reti" ::); }
#endif



/*@}*/

#ifdef __cplusplus
extern "C" {
#endif

#ifdef __cplusplus
}
#endif

#endif
