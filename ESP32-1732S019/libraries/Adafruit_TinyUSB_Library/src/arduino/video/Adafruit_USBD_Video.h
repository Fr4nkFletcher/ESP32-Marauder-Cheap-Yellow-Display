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

#ifndef ADAFRUIT_USBD_VIDEO_H
#define ADAFRUIT_USBD_VIDEO_H

#include "arduino/Adafruit_USBD_Device.h"

class Adafruit_USBD_Video : public Adafruit_USBD_Interface {
public:
  Adafruit_USBD_Video(void);

  bool begin();

  //------------- Video Control -------------//
  //  bool isStreaming(uint8_t stream_idx);
  bool
  addTerminal(tusb_desc_video_control_camera_terminal_t const *camera_terminal);
  bool
  addTerminal(tusb_desc_video_control_output_terminal_t const *output_terminal);

  //------------- Video Streaming -------------//
  //  bool setIsochronousStreaming(bool enabled);

  // Add format descriptor, return format index
  bool addFormat(tusb_desc_video_format_uncompressed_t const *format);
  bool addFrame(tusb_desc_video_frame_uncompressed_continuous_t const *frame);
  void
  addColorMatching(tusb_desc_video_streaming_color_matching_t const *color);

  // from Adafruit_USBD_Interface
  virtual uint16_t getInterfaceDescriptor(uint8_t itfnum_deprecated,
                                          uint8_t *buf, uint16_t bufsize);

private:
  uint8_t _vc_id;

  tusb_desc_video_control_camera_terminal_t _camera_terminal;
  tusb_desc_video_control_output_terminal_t _output_terminal;

  // currently only support 1 format
  union {
    tusb_desc_video_format_uncompressed_t uncompressed;
    tusb_desc_video_format_mjpeg_t mjpeg;
  } _format;

  union {
    tusb_desc_video_frame_uncompressed_continuous_t uncompressed_cont;
    tusb_desc_video_frame_mjpeg_continuous_t mjpeg;
  } _frame;

  tusb_desc_video_streaming_color_matching_t _color_matching;
};

#endif
