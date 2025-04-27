/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2024 Ha Thach (tinyusb.org)
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
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUD_VIDEO && CFG_TUD_VIDEO_STREAMING

#include "Adafruit_USBD_Video.h"

#define BULK_PACKET_SIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

Adafruit_USBD_Video::Adafruit_USBD_Video(void) {
  _vc_id = 0;
  memset(&_camera_terminal, 0, sizeof(_camera_terminal));
}

bool Adafruit_USBD_Video::addTerminal(
    tusb_desc_video_control_camera_terminal_t const *camera_terminal) {
  _camera_terminal = *camera_terminal;

  // override constants
  _camera_terminal.bLength = sizeof(tusb_desc_video_control_camera_terminal_t);
  _camera_terminal.bDescriptorType = TUSB_DESC_CS_INTERFACE;
  _camera_terminal.bDescriptorSubType = VIDEO_CS_ITF_VC_INPUT_TERMINAL;

  _camera_terminal.bControlSize = 3;

  return true;
}

bool Adafruit_USBD_Video::addTerminal(
    tusb_desc_video_control_output_terminal_t const *output_terminal) {
  _output_terminal = *output_terminal;

  // override constants
  _output_terminal.bLength = sizeof(tusb_desc_video_control_output_terminal_t);
  _output_terminal.bDescriptorType = TUSB_DESC_CS_INTERFACE;
  _output_terminal.bDescriptorSubType = VIDEO_CS_ITF_VC_OUTPUT_TERMINAL;

  return true;
}

bool Adafruit_USBD_Video::addFormat(
    tusb_desc_video_format_uncompressed_t const *format) {
  _format.uncompressed = *format;

  // override constants
  _format.uncompressed.bLength = sizeof(tusb_desc_video_format_uncompressed_t);
  _format.uncompressed.bDescriptorType = TUSB_DESC_CS_INTERFACE;
  _format.uncompressed.bDescriptorSubType = VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED;

  return true;
}

bool Adafruit_USBD_Video::addFrame(
    tusb_desc_video_frame_uncompressed_continuous_t const *frame) {
  _frame.uncompressed_cont = *frame;

  // override constants
  _frame.uncompressed_cont.bLength =
      sizeof(tusb_desc_video_frame_uncompressed_continuous_t);
  _frame.uncompressed_cont.bDescriptorType = TUSB_DESC_CS_INTERFACE;
  _frame.uncompressed_cont.bDescriptorSubType =
      VIDEO_CS_ITF_VS_FRAME_UNCOMPRESSED;
  _frame.uncompressed_cont.bFrameIntervalType = 0; // continuous

  return true;
}

void Adafruit_USBD_Video::addColorMatching(
    tusb_desc_video_streaming_color_matching_t const *color) {
  _color_matching = *color;

  // override constants
  _color_matching.bLength = sizeof(tusb_desc_video_streaming_color_matching_t);
  _color_matching.bDescriptorType = TUSB_DESC_CS_INTERFACE;
  _color_matching.bDescriptorSubType = VIDEO_CS_ITF_VS_COLORFORMAT;
}

bool Adafruit_USBD_Video::begin() {
  if (!TinyUSBDevice.addInterface(*this)) {
    return false;
  }

  return true;
}

uint16_t Adafruit_USBD_Video::getInterfaceDescriptor(uint8_t itfnum_deprecated,
                                                     uint8_t *buf,
                                                     uint16_t bufsize) {
  (void)itfnum_deprecated;

  uint8_t itfnum = 0;
  uint8_t ep_in = 0;

  // check if necessary descriptors are added
  if (!(_camera_terminal.bLength && _output_terminal.bLength &&
        _format.uncompressed.bLength && _frame.uncompressed_cont.bLength &&
        _color_matching.bLength)) {
    return 0;
  }

  // Null buf is for length only
  if (buf) {
    itfnum = TinyUSBDevice.allocInterface(2);
    ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
  }

  typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t itf;
    tusb_desc_video_control_header_1itf_t header;
    tusb_desc_video_control_camera_terminal_t camera_terminal;
    tusb_desc_video_control_output_terminal_t output_terminal;
  } uvc_control_desc_t;

  // hard code for now
#define UVC_CLOCK_FREQUENCY 27000000

  /* Windows support YUY2 and NV12
   * https://docs.microsoft.com/en-us/windows-hardware/drivers/stream/usb-video-class-driver-overview
   */
  typedef struct TU_ATTR_PACKED {
    tusb_desc_interface_t itf;
    tusb_desc_video_streaming_input_header_1byte_t header;
    tusb_desc_video_format_uncompressed_t format;
    tusb_desc_video_frame_uncompressed_continuous_t frame;
    tusb_desc_video_streaming_color_matching_t color;

    // #if USE_ISO_STREAMING
    //     // For ISO streaming, USB spec requires to alternate interface
    //   tusb_desc_interface_t itf_alt;
    // #endif

    tusb_desc_endpoint_t ep;
  } uvc_streaming_desc_t;

  const tusb_desc_interface_assoc_t desc_iad = {
      .bLength = sizeof(tusb_desc_interface_assoc_t),
      .bDescriptorType = TUSB_DESC_INTERFACE_ASSOCIATION,

      .bFirstInterface = itfnum,
      .bInterfaceCount = 2,
      .bFunctionClass = TUSB_CLASS_VIDEO,
      .bFunctionSubClass = VIDEO_SUBCLASS_INTERFACE_COLLECTION,
      .bFunctionProtocol = VIDEO_ITF_PROTOCOL_UNDEFINED,
      .iFunction = 0};

  uvc_control_desc_t desc_video_control{
      .itf = {.bLength = sizeof(tusb_desc_interface_t),
              .bDescriptorType = TUSB_DESC_INTERFACE,

              .bInterfaceNumber = itfnum,
              .bAlternateSetting = 0,
              .bNumEndpoints = 0,
              .bInterfaceClass = TUSB_CLASS_VIDEO,
              .bInterfaceSubClass = VIDEO_SUBCLASS_CONTROL,
              .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
              .iInterface = _strid},
      .header = {.bLength = sizeof(tusb_desc_video_control_header_1itf_t),
                 .bDescriptorType = TUSB_DESC_CS_INTERFACE,
                 .bDescriptorSubType = VIDEO_CS_ITF_VC_HEADER,

                 .bcdUVC = VIDEO_BCD_1_50,
                 .wTotalLength =
                     sizeof(uvc_control_desc_t) -
                     sizeof(tusb_desc_interface_t), // CS VC descriptors only
                 .dwClockFrequency = UVC_CLOCK_FREQUENCY,
                 .bInCollection = 1,
                 .baInterfaceNr = {(uint8_t)(itfnum + 1)}},
      .camera_terminal = _camera_terminal,
      .output_terminal = _output_terminal};

  uvc_streaming_desc_t desc_video_streaming = {
      .itf =
          {
              .bLength = sizeof(tusb_desc_interface_t),
              .bDescriptorType = TUSB_DESC_INTERFACE,

              .bInterfaceNumber = (uint8_t)(itfnum + 1),
              .bAlternateSetting = 0,
              .bNumEndpoints = 1, // bulk 1, iso 0
              .bInterfaceClass = TUSB_CLASS_VIDEO,
              .bInterfaceSubClass = VIDEO_SUBCLASS_STREAMING,
              .bInterfaceProtocol = VIDEO_ITF_PROTOCOL_15,
              .iInterface = _strid,
          },
      .header = {.bLength =
                     sizeof(tusb_desc_video_streaming_input_header_1byte_t),
                 .bDescriptorType = TUSB_DESC_CS_INTERFACE,
                 .bDescriptorSubType = VIDEO_CS_ITF_VS_INPUT_HEADER,

                 .bNumFormats = 1,
                 .wTotalLength =
                     sizeof(uvc_streaming_desc_t) -
                     sizeof(tusb_desc_interface_t) -
                     sizeof(tusb_desc_endpoint_t), // CS VS descriptors only
                 .bEndpointAddress = ep_in,
                 .bmInfo = 0,
                 .bTerminalLink = _output_terminal.bTerminalID,
                 .bStillCaptureMethod = 0,
                 .bTriggerSupport = 0,
                 .bTriggerUsage = 0,
                 .bControlSize = 1,
                 .bmaControls = {0}},
      .format = _format.uncompressed,
      .frame = _frame.uncompressed_cont,
      .color = _color_matching,
      .ep = {.bLength = sizeof(tusb_desc_endpoint_t),
             .bDescriptorType = TUSB_DESC_ENDPOINT,

             .bEndpointAddress = ep_in,
             .bmAttributes = {.xfer = TUSB_XFER_BULK, .sync = 0, .usage = 0},
             .wMaxPacketSize = BULK_PACKET_SIZE,
             .bInterval = 1}};

  uint16_t const len_iad = sizeof(desc_iad);
  uint16_t const len_vc = sizeof(desc_video_control);
  uint16_t const len_vs = sizeof(desc_video_streaming);
  uint16_t const len_total = len_iad + len_vc + len_vs;

  if (buf) {
    if (bufsize < len_total) {
      return 0;
    }

    memcpy(buf, &desc_iad, len_iad);
    buf += len_iad;

    memcpy(buf, &desc_video_control, len_vc);
    buf += len_vc;

    memcpy(buf, &desc_video_streaming, len_vs);
  }

  return len_total;
}

//--------------------------------------------------------------------+
// API
//--------------------------------------------------------------------+

// bool Adafruit_USBD_Video::isStreaming(uint8_t stream_idx) {
//   return tud_video_n_streaming(_vc_id, stream_idx);
// }

#endif
