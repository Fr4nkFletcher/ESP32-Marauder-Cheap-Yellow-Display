/*********************************************************************
 Adafruit invests time and resources providing this open source code,
 please support Adafruit and open-source hardware by purchasing
 products from Adafruit!

 MIT license, check LICENSE for more information
 Copyright (c) 2019 Ha Thach for Adafruit Industries
 All text above, and the splash screen below must be included in
 any redistribution
*********************************************************************/

#include "Adafruit_TinyUSB.h"

//--------------------------------------------------------------------+
// Video descriptors
//--------------------------------------------------------------------+

/* Time stamp base clock. It is a deprecated parameter. */
#define FRAME_WIDTH   128
#define FRAME_HEIGHT  96
#define FRAME_RATE    30

/* video capture path
 *
 * | Camera Terminal (0x01) | ----> | Output Terminal (0x02 Streaming) |
 * */
#define TERMID_CAMERA 0x01
#define TERMID_OUTPUT 0x02

tusb_desc_video_control_camera_terminal_t const desc_camera_terminal = {
    .bLength = sizeof(tusb_desc_video_control_camera_terminal_t),
    .bDescriptorType = TUSB_DESC_CS_INTERFACE,
    .bDescriptorSubType = VIDEO_CS_ITF_VC_INPUT_TERMINAL,

    .bTerminalID = TERMID_CAMERA,
    .wTerminalType = VIDEO_ITT_CAMERA,
    .bAssocTerminal = 0,
    .iTerminal = 0,
    .wObjectiveFocalLengthMin = 0,
    .wObjectiveFocalLengthMax = 0,
    .wOcularFocalLength = 0,
    .bControlSize = 3,
    .bmControls = { 0, 0, 0 }
};

tusb_desc_video_control_output_terminal_t const desc_output_terminal = {
    .bLength = sizeof(tusb_desc_video_control_output_terminal_t),
    .bDescriptorType = TUSB_DESC_CS_INTERFACE,
    .bDescriptorSubType = VIDEO_CS_ITF_VC_OUTPUT_TERMINAL,

    .bTerminalID = TERMID_OUTPUT,
    .wTerminalType = VIDEO_TT_STREAMING,
    .bAssocTerminal = 0,
    .bSourceID = TERMID_CAMERA,
    .iTerminal = 0
};

tusb_desc_video_format_uncompressed_t const desc_format = {
    .bLength = sizeof(tusb_desc_video_format_uncompressed_t),
    .bDescriptorType = TUSB_DESC_CS_INTERFACE,
    .bDescriptorSubType = VIDEO_CS_ITF_VS_FORMAT_UNCOMPRESSED,
    .bFormatIndex = 1, // 1-based index
    .bNumFrameDescriptors = 1,
    .guidFormat = { TUD_VIDEO_GUID_YUY2 },
    .bBitsPerPixel = 16,
    .bDefaultFrameIndex = 1,
    .bAspectRatioX = 0,
    .bAspectRatioY = 0,
    .bmInterlaceFlags = 0,
    .bCopyProtect = 0
};

tusb_desc_video_frame_uncompressed_continuous_t desc_frame = {
    .bLength = sizeof(tusb_desc_video_frame_uncompressed_continuous_t),
    .bDescriptorType = TUSB_DESC_CS_INTERFACE,
    .bDescriptorSubType = VIDEO_CS_ITF_VS_FRAME_UNCOMPRESSED,
    .bFrameIndex = 1, // 1-based index
    .bmCapabilities = 0,
    .wWidth = FRAME_WIDTH,
    .wHeight = FRAME_HEIGHT,
    .dwMinBitRate = FRAME_WIDTH * FRAME_HEIGHT * 16 * 1,
    .dwMaxBitRate = FRAME_WIDTH * FRAME_HEIGHT * 16 * FRAME_RATE,
    .dwMaxVideoFrameBufferSize = FRAME_WIDTH * FRAME_HEIGHT * 16 / 8,
    .dwDefaultFrameInterval = 10000000 / FRAME_RATE,
    .bFrameIntervalType = 0, // continuous
    .dwFrameInterval = {
        10000000 / FRAME_RATE, // min
        10000000, // max
        10000000 / FRAME_RATE // step
    }
};

tusb_desc_video_streaming_color_matching_t desc_color = {
    .bLength = sizeof(tusb_desc_video_streaming_color_matching_t),
    .bDescriptorType = TUSB_DESC_CS_INTERFACE,
    .bDescriptorSubType = VIDEO_CS_ITF_VS_COLORFORMAT,

    .bColorPrimaries = VIDEO_COLOR_PRIMARIES_BT709,
    .bTransferCharacteristics = VIDEO_COLOR_XFER_CH_BT709,
    .bMatrixCoefficients = VIDEO_COLOR_COEF_SMPTE170M
};

//--------------------------------------------------------------------+
// Video and frame buffer
//--------------------------------------------------------------------+
Adafruit_USBD_Video usb_video;

// YUY2 frame buffer
static uint8_t frame_buffer[FRAME_WIDTH * FRAME_HEIGHT * 16 / 8];

static unsigned frame_num = 0;
static unsigned tx_busy = 0;
static unsigned interval_ms = 1000 / FRAME_RATE;
static unsigned start_ms = 0;
static unsigned already_sent = 0;

//--------------------------------------------------------------------+
//
//--------------------------------------------------------------------+
static void fill_color_bar(uint8_t* buffer, unsigned start_position);

void setup() {
  // Manual begin() is required on core without built-in support e.g. mbed rp2040
  if (!TinyUSBDevice.isInitialized()) {
    TinyUSBDevice.begin(0);
  }

  Serial.begin(115200);

  usb_video.addTerminal(&desc_camera_terminal);
  usb_video.addTerminal(&desc_output_terminal);
  usb_video.addFormat(&desc_format);
  usb_video.addFrame(&desc_frame);
  usb_video.addColorMatching(&desc_color);

  usb_video.begin();

  // If already enumerated, additional class driverr begin() e.g msc, hid, midi won't take effect until re-enumeration
  if (TinyUSBDevice.mounted()) {
    TinyUSBDevice.detach();
    delay(10);
    TinyUSBDevice.attach();
  }
}

void loop() {
  #ifdef TINYUSB_NEED_POLLING_TASK
  // Manual call tud_task since it isn't called by Core's background
  TinyUSBDevice.task();
  #endif

  if (!tud_video_n_streaming(0, 0)) {
    already_sent = 0;
    frame_num = 0;
    return;
  }

  if (!already_sent) {
    already_sent = 1;
    tx_busy = 1;
    start_ms = millis();
    fill_color_bar(frame_buffer, frame_num);
    tud_video_n_frame_xfer(0, 0, (void*) frame_buffer, FRAME_WIDTH * FRAME_HEIGHT * 16 / 8);
  }

  unsigned cur = millis();
  if (cur - start_ms < interval_ms) return; // not enough time
  if (tx_busy) return;
  start_ms += interval_ms;
  tx_busy = 1;

  fill_color_bar(frame_buffer, frame_num);
  tud_video_n_frame_xfer(0, 0, (void*) frame_buffer, FRAME_WIDTH * FRAME_HEIGHT * 16 / 8);
}

//--------------------------------------------------------------------+
// TinyUSB Video Callbacks
//--------------------------------------------------------------------+
extern "C" {

void tud_video_frame_xfer_complete_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx) {
  (void) ctl_idx;
  (void) stm_idx;
  tx_busy = 0;
  /* flip buffer */
  ++frame_num;
}

int tud_video_commit_cb(uint_fast8_t ctl_idx, uint_fast8_t stm_idx,
                        video_probe_and_commit_control_t const* parameters) {
  (void) ctl_idx;
  (void) stm_idx;
  /* convert unit to ms from 100 ns */
  interval_ms = parameters->dwFrameInterval / 10000;
  return VIDEO_ERROR_NONE;
}

} // extern C

//------------- Helper -------------//
static void fill_color_bar(uint8_t* buffer, unsigned start_position) {
  /* EBU color bars
   * See also https://stackoverflow.com/questions/6939422 */
  static uint8_t const bar_color[8][4] = {
      /*  Y,   U,   Y,   V */
      { 235, 128, 235, 128}, /* 100% White */
      { 219,  16, 219, 138}, /* Yellow */
      { 188, 154, 188,  16}, /* Cyan */
      { 173,  42, 173,  26}, /* Green */
      {  78, 214,  78, 230}, /* Magenta */
      {  63, 102,  63, 240}, /* Red */
      {  32, 240,  32, 118}, /* Blue */
      {  16, 128,  16, 128}, /* Black */
  };
  uint8_t* p;

  /* Generate the 1st line */
  uint8_t* end = &buffer[FRAME_WIDTH * 2];
  unsigned idx = (FRAME_WIDTH / 2 - 1) - (start_position % (FRAME_WIDTH / 2));
  p = &buffer[idx * 4];
  for (unsigned i = 0; i < 8; ++i) {
    for (int j = 0; j < FRAME_WIDTH / (2 * 8); ++j) {
      memcpy(p, &bar_color[i], 4);
      p += 4;
      if (end <= p) {
        p = buffer;
      }
    }
  }

  /* Duplicate the 1st line to the others */
  p = &buffer[FRAME_WIDTH * 2];
  for (unsigned i = 1; i < FRAME_HEIGHT; ++i) {
    memcpy(p, buffer, FRAME_WIDTH * 2);
    p += FRAME_WIDTH * 2;
  }
}
