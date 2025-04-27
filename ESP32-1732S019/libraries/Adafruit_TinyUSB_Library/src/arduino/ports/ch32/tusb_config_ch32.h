/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2018, hathach for Adafruit
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#ifndef TUSB_CONFIG_CH32_H_
#define TUSB_CONFIG_CH32_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#if defined(CH32V10x)
#define CFG_TUSB_MCU OPT_MCU_CH32V103
#warnning "CH32v103 is not working yet"
#elif defined(CH32V20x)
#define CFG_TUSB_MCU OPT_MCU_CH32V20X
#elif defined(CH32V30x)
#define CFG_TUSB_MCU OPT_MCU_CH32V307
#endif

#define CFG_TUSB_OS OPT_OS_NONE

#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG 0
#endif

// #define SERIAL_TUSB_DEBUG Serial2

// For selectively disable device log (when > CFG_TUSB_DEBUG)
// #define CFG_TUD_LOG_LEVEL 3
// #define CFG_TUH_LOG_LEVEL 3

#define CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_ALIGN __attribute__((aligned(4)))

#define CFG_TUD_ENABLED 1

// #ifdef USE_TINYUSB
//// Enable device stack
// #define CFG_TUD_ENABLED 1
//
//// Enable host stack with MAX3421E (host shield)
// #define CFG_TUH_ENABLED 1
// #define CFG_TUH_MAX3421 1
//
// #else
// #define CFG_TUD_ENABLED 0
// #define CFG_TUH_ENABLED 0
// #endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

#define CFG_TUD_ENDPOINT0_SIZE 64

//------------- CLASS -------------//
#ifndef CFG_TUD_CDC
#define CFG_TUD_CDC 1
#endif

#ifndef CFG_TUD_MSC
#define CFG_TUD_MSC 1
#endif

#ifndef CFG_TUD_HID
#define CFG_TUD_HID 2
#endif

#ifndef CFG_TUD_MIDI
#define CFG_TUD_MIDI 1
#endif

#ifndef CFG_TUD_VENDOR
#define CFG_TUD_VENDOR 1
#endif

#ifndef CFG_TUD_VIDEO
#define CFG_TUD_VIDEO 0 // number of video control interfaces
#endif

#ifndef CFG_TUD_VIDEO_STREAMING
#define CFG_TUD_VIDEO_STREAMING 0 // number of video streaming interfaces
#endif

// video streaming endpoint buffer size
#define CFG_TUD_VIDEO_STREAMING_EP_BUFSIZE 256

// CDC FIFO size of TX and RX
#define CFG_TUD_CDC_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 256)
#define CFG_TUD_CDC_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 256)

// MSC Buffer size of Device Mass storage
#define CFG_TUD_MSC_EP_BUFSIZE 512

// HID buffer size Should be sufficient to hold ID (if any) + Data
#define CFG_TUD_HID_EP_BUFSIZE 64

// MIDI FIFO size of TX and RX
#define CFG_TUD_MIDI_RX_BUFSIZE 128
#define CFG_TUD_MIDI_TX_BUFSIZE 128

// Vendor FIFO size of TX and RX
#ifndef CFG_TUD_VENDOR_RX_BUFSIZE
#define CFG_TUD_VENDOR_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

#ifndef CFG_TUD_VENDOR_TX_BUFSIZE
#define CFG_TUD_VENDOR_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#endif

//--------------------------------------------------------------------
// Host Configuration
//--------------------------------------------------------------------
#if 0
// Size of buffer to hold descriptors and other data used for enumeration
#define CFG_TUH_ENUMERATION_BUFSIZE 256

// Number of hub devices
#define CFG_TUH_HUB 1

// max device support (excluding hub device): 1 hub typically has 4 ports
#define CFG_TUH_DEVICE_MAX (3 * CFG_TUH_HUB + 1)

// Enable tuh_edpt_xfer() API
// #define CFG_TUH_API_EDPT_XFER       1

// Number of mass storage
#define CFG_TUH_MSC 1

// Number of HIDs
// typical keyboard + mouse device can have 3,4 HID interfaces
#define CFG_TUH_HID (3 * CFG_TUH_DEVICE_MAX)

// Number of CDC interfaces
// FTDI and CP210x are not part of CDC class, only to re-use CDC driver API
#define CFG_TUH_CDC 1
#define CFG_TUH_CDC_FTDI 1
#define CFG_TUH_CDC_CP210X 1
#define CFG_TUH_CDC_CH34X 1

// RX & TX fifo size
#define CFG_TUH_CDC_RX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)
#define CFG_TUH_CDC_TX_BUFSIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

// Set Line Control state on enumeration/mounted:
// DTR ( bit 0), RTS (bit 1)
#define CFG_TUH_CDC_LINE_CONTROL_ON_ENUM 0x03

// Set Line Coding on enumeration/mounted, value for cdc_line_coding_t
// bit rate = 115200, 1 stop bit, no parity, 8 bit data width
// This need Pico-PIO-USB at least 0.5.1
#define CFG_TUH_CDC_LINE_CODING_ON_ENUM                                        \
  { 115200, CDC_LINE_CONDING_STOP_BITS_1, CDC_LINE_CODING_PARITY_NONE, 8 }
#endif

#ifdef __cplusplus
}
#endif

#endif
