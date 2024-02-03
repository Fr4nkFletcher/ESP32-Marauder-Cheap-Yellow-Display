/* Copyright (c) 2002, 2003, 2004 Marek Michalkiewicz
   Copyright (c) 2005, 2006 Bjoern Haase
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

/* $Id: eeprom.h,v 1.17.2.1 2006/02/26 21:51:04 aesok Exp $ */

/*
   eeprom.h

   Contributors:
     Created by Marek Michalkiewicz <marekm@linux.org.pl>
     eeprom_write_word and eeprom_write_block added by Artur Lipowski 
     <LAL@pro.onet.pl>
     Complete rewrite using the original interface by Bjoern Haase 
     <bjoern.haase@de.bosch.com>. 
 */

#ifndef _EEPROM_H_
#define _EEPROM_H_ 1

#define __need_size_t
#include <stddef.h>
#include <inttypes.h>


#ifdef __AVR_MEGA__
#define XCALL "call"
#else
#define XCALL "rcall"
#endif

#include <avr/io.h>
#ifndef __EEPROM_REG_LOCATIONS__
/** \def __EEPROM_REG_LOCATIONS__
    \ingroup avr_eeprom
     In order to be able to work without a requiring a multilib 
     approach for dealing with controllers having the EEPROM registers
     at different positions in memory space, the eeprom functions evaluate
     __EEPROM_REG_LOCATIONS__: It is assumed to be defined by
     the device io header and contains 6 uppercase hex digits encoding the 
     addresses of EECR,EEDR and EEAR. 
     First two letters:  EECR address.
     Second two letters: EEDR address.
     Last two letters:   EEAR address.
     The default 1C1D1E corresponds to the
     register location that is valid for most controllers. The value
     of this define symbol is used for appending it to the base name of the
     assembler functions.  */
#define __EEPROM_REG_LOCATIONS__ 1C1D1E
#endif
#define _STR2(EXP) _STR1(EXP)
#define _STR1(EXP) #EXP
#define _REG_LOCATION_SUFFIX _STR2(__EEPROM_REG_LOCATIONS__)

#ifndef CR_TAB
#define CR_TAB "\n\t"
#endif


/** \defgroup avr_eeprom <avr/eeprom.h>: EEPROM handling
    \code #include <avr/eeprom.h> \endcode

    This header file declares the interface to some simple library
    routines suitable for handling the data EEPROM contained in the
    AVR microcontrollers.  The implementation uses a simple polled
    mode interface.  Applications that require interrupt-controlled
    EEPROM access to ensure that no time will be wasted in spinloops
    will have to deploy their own implementation.

    \note All of the read/write functions first make sure the EEPROM
     is ready to be accessed.  Since this may cause long delays if a
     write operation is still pending, time-critical applications
     should first poll the EEPROM e. g. using eeprom_is_ready() before
     attempting any actual I/O.

    \note This header file declares inline functions that call the
     assembler subroutines directly. This prevents that the compiler
     generates push/pops for the call-clobbered registers. This way
     also a specific calling convention could be used for the eeprom
     routines e.g. by passing values in __tmp_reg__, eeprom addresses in
     X and memory addresses in Z registers. Method is optimized for code 
     size.

    \note Presently supported are two locations of the EEPROM register
     set: 0x1F,0x20,0x21 and 0x1C,0x1D,0x1E 
     (see ::__EEPROM_REG_LOCATIONS__).

    \note As these functions modify IO registers, they are known to be
     non-reentrant.  If any of these functions are used from both,
     standard and interrupt context, the applications must ensure
     proper protection (e.g. by disabling interrupts before accessing
     them).

*/


/* forward declarations of the inline functions so that doxygen does
   not get confused by the attribute expression.  */

static inline uint8_t __attribute__ ((always_inline))
eeprom_read_byte (const uint8_t *addr);

static inline uint16_t __attribute__ ((always_inline)) 
eeprom_read_word (const uint16_t *addr);

static inline void __attribute__ ((always_inline))
eeprom_read_block (void *pointer_ram,
                   const void *pointer_eeprom,
                   size_t size);

static inline void __attribute__ ((always_inline))
eeprom_write_byte (uint8_t *addr,uint8_t value);

static inline void __attribute__ ((always_inline))
eeprom_write_word (uint16_t *addr,uint16_t value);

static inline void __attribute__ ((always_inline))
eeprom_write_block (const void *pointer_ram,
                    void *pointer_eeprom,
                    size_t size);

/** \name avr-libc declarations */

/*@{*/

/** \def EEMEM
    \ingroup avr_eeprom
    Attribute expression causing a variable to be allocated within the .eeprom
     section.  */
#define EEMEM __attribute__((section(".eeprom")))

/** \def eeprom_is_ready
    \ingroup avr_eeprom
    \returns 1 if EEPROM is ready for a new read/write operation, 0 if not. */

#if defined(__DOXYGEN__)
# define eeprom_is_ready()
#elif defined(EEWE)
# define eeprom_is_ready() bit_is_clear(EECR, EEWE)
#elif defined(EEPE)
# define eeprom_is_ready() bit_is_clear(EECR, EEPE)
#elif defined(DEECR) && defined(EEL)
# define eeprom_is_ready() bit_is_clear(DEECR, EEL)
#else
# error "No write enable bit known for this device's EEPROM."
#endif

/** \def eeprom_busy_wait
    \ingroup avr_eeprom

    Loops until the eeprom is no longer busy.

    \returns Nothing. */

#define eeprom_busy_wait() do {} while (!eeprom_is_ready())


/** \ingroup avr_eeprom
    Read one byte from EEPROM address \c addr. */

uint8_t 
eeprom_read_byte (const uint8_t *addr) 
{
  uint8_t result;
  asm volatile
      ( XCALL " __eeprom_read_byte_" _REG_LOCATION_SUFFIX CR_TAB
        "mov %1,__tmp_reg__"
       : "+x" (addr),
         "=r" (result)
       : );
  return result;
}

/** \ingroup avr_eeprom
    Read one 16-bit word (little endian) from EEPROM address \c addr. */
uint16_t
eeprom_read_word (const uint16_t *addr)
{
  uint16_t result;

  asm ( 
        XCALL " __eeprom_read_word_" _REG_LOCATION_SUFFIX CR_TAB
       : "+x" (addr),
         "=z" (result)
       : );
  return result;
}

/** \ingroup avr_eeprom
    Read a block of \c n bytes from EEPROM address \c pointer_eeprom to
    \c pointer_ram.  For constant n <= 256 bytes a library function is used.
    For block sizes unknown at compile time or block sizes > 256 an inline
    loop is expanded. */

void 
eeprom_read_block (void *pointer_ram,
                   const void *pointer_eeprom,
                   size_t n)
{
  if (!__builtin_constant_p (n)
      || n > 256)
    {
      /* make sure size is a 16 bit variable.  */
      uint16_t size = n; 

      asm volatile ( 
            ".%=_start:" CR_TAB
            "sbiw %2,1" CR_TAB
            "brlt .%=_finished" CR_TAB
             XCALL " __eeprom_read_byte_" _REG_LOCATION_SUFFIX CR_TAB
            "st z+,__tmp_reg__" CR_TAB
            "rjmp .%=_start" CR_TAB
            ".%=_finished:" 
          : "=x" (pointer_eeprom),
            "=z" (pointer_ram),
            "+w" (size)
           : "x" (pointer_eeprom), 
             "z" (pointer_ram)
           : "memory");
    }
  else
    {
      if (n != 0)
        {
          if (n == 256)
            {
              asm volatile (
                  XCALL " __eeprom_read_block_" _REG_LOCATION_SUFFIX 
                : "+x" (pointer_eeprom),
                  "=z" (pointer_ram)
                : "z"  (pointer_ram)
                : "memory");
            }
          else
            {
              /* Needed in order to truncate to 8 bit.  */
              uint8_t len;
              len = (uint8_t) n; 

              asm volatile (
                  "mov __zero_reg__,%2"      CR_TAB
                   XCALL " __eeprom_read_block_" _REG_LOCATION_SUFFIX 
                : "+x" (pointer_eeprom),
                  "=z" (pointer_ram)
                : "r"  (len),
                  "z"  (pointer_ram)
                : "memory");
            }
        }
    }
}

/** \ingroup avr_eeprom
    Write a byte \c value to EEPROM address \c addr. */

void 
eeprom_write_byte (uint8_t *addr,uint8_t value)
{
  asm volatile (
         "mov __tmp_reg__,%1"      CR_TAB
         XCALL " __eeprom_write_byte_" _REG_LOCATION_SUFFIX
       : "+x" (addr)
       : "r"  (value)
       : "memory"
      );
}

/** \ingroup avr_eeprom
    Write a word \c value to EEPROM address \c addr. */

void 
eeprom_write_word (uint16_t *addr,uint16_t value)
{
  asm volatile (
#if __AVR_HAVE_MOVW__
         "movw __tmp_reg__,%A1" CR_TAB
#else
         "mov __tmp_reg__,%A1"      CR_TAB
         "mov __zero_reg__,%B1"     CR_TAB
#endif
          XCALL " __eeprom_write_word_" _REG_LOCATION_SUFFIX CR_TAB
       : "+x" (addr)
       : "r"  (value)
       : "memory"
      );
}

/** \ingroup avr_eeprom
    Write a block of \c n bytes to EEPROM address \c pointer_eeprom from
    \c pointer_ram. */

void 
eeprom_write_block (const void *pointer_ram,
                    void *pointer_eeprom,
                    size_t n)
{
  if (!__builtin_constant_p (n)
      || n > 256)
    {
      /* make sure size is a 16 bit variable.  */
      uint16_t size = n; 

      asm volatile ( 
            ".%=_start:" CR_TAB
            "sbiw %2,1" CR_TAB
            "brlt .%=_finished" CR_TAB
            "ld __tmp_reg__,z+" CR_TAB
             XCALL " __eeprom_write_byte_" _REG_LOCATION_SUFFIX CR_TAB
            "rjmp .%=_start" CR_TAB
            ".%=_finished:" 
          : "=x" (pointer_eeprom),
            "=z" (pointer_ram),
            "+w" (size)
           : "x" (pointer_eeprom), 
             "z" (pointer_ram)
           : "memory");
    }
  else
    {
      /* Do nothing for compile time constant transfer size n == 0.  */
      if (n != 0)
        {
          if (n == 256)
            {
              asm volatile (
                 XCALL " __eeprom_write_block_" _REG_LOCATION_SUFFIX
               : "+x" (pointer_eeprom),
                 "=z" (pointer_ram)
               : "z"  (pointer_ram)
               : "memory" );
            }
          else
            {
              uint8_t len;
              len = (uint8_t) n;

              asm volatile (
                 "mov __zero_reg__,%2" CR_TAB
                 XCALL " __eeprom_write_block_" _REG_LOCATION_SUFFIX
               : "+x" (pointer_eeprom),
                 "=z" (pointer_ram)
               : "r"  (len),
                 "z"  (pointer_ram)
               : "memory" );
            }

        }
    }
}

/*@}*/

/** \name IAR C compatibility defines */

/*@{*/

/** \def _EEPUT
    \ingroup avr_eeprom
    Write a byte to EEPROM. Compatibility define for IAR C. */

#define _EEPUT(addr, val) eeprom_write_byte ((uint8_t *)(addr), (uint8_t)(val))

/** \def _EEGET
    \ingroup avr_eeprom
    Read a byte from EEPROM. Compatibility define for IAR C. */

#define _EEGET(var, addr) (var) = eeprom_read_byte ((uint8_t *)(addr))

/*@}*/

#endif /* _EEPROM_H_ */
