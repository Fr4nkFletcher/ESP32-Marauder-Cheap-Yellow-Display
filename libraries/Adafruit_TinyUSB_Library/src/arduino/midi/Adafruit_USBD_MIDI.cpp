/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 hathach for Adafruit Industries
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

#include "tusb_option.h"

#if CFG_TUD_ENABLED && CFG_TUD_MIDI

#include "Adafruit_USBD_MIDI.h"

//--------------------------------------------------------------------+
// MACRO TYPEDEF CONSTANT ENUM DECLARATION
//--------------------------------------------------------------------+
#define BULK_PACKET_SIZE (TUD_OPT_HIGH_SPEED ? 512 : 64)

// TODO multiple instances
static Adafruit_USBD_MIDI *_midi_dev = NULL;

Adafruit_USBD_MIDI::Adafruit_USBD_MIDI(uint8_t n_cables) {
  _n_cables = n_cables;
  memset(_cable_name_strid, 0, sizeof(_cable_name_strid));
}

void Adafruit_USBD_MIDI::setCables(uint8_t n_cables) { _n_cables = n_cables; }

bool Adafruit_USBD_MIDI::setCableName(uint8_t cable_id, const char *str) {
  if (cable_id == 0 || cable_id > sizeof(_cable_name_strid)) {
    return false;
  }

  uint8_t strid = TinyUSBDevice.addStringDescriptor(str);
  _cable_name_strid[cable_id - 1] = strid;

  return strid > 0;
}

bool Adafruit_USBD_MIDI::begin(void) {
  if (!TinyUSBDevice.addInterface(*this)) {
    return false;
  }

  _midi_dev = this;
  return true;
}

uint16_t Adafruit_USBD_MIDI::getInterfaceDescriptor(uint8_t itfnum_deprecated,
                                                    uint8_t *buf,
                                                    uint16_t bufsize) {
  (void)itfnum_deprecated;

  uint16_t const desc_len = TUD_MIDI_DESC_HEAD_LEN +
                            TUD_MIDI_DESC_JACK_LEN * _n_cables +
                            2 * TUD_MIDI_DESC_EP_LEN(_n_cables);

  // null buffer is used to get the length of descriptor only
  if (!buf) {
    return desc_len;
  }

  if (bufsize < desc_len) {
    return 0;
  }

  uint8_t itfnum = TinyUSBDevice.allocInterface(2);
  uint8_t ep_in = TinyUSBDevice.allocEndpoint(TUSB_DIR_IN);
  uint8_t ep_out = TinyUSBDevice.allocEndpoint(TUSB_DIR_OUT);

  uint16_t len = 0;

  // Header
  {
    uint8_t desc[] = {TUD_MIDI_DESC_HEAD(itfnum, _strid, _n_cables)};
    memcpy(buf + len, desc, sizeof(desc));
    len += sizeof(desc);
  }

  // Jack
  for (uint8_t i = 1; i <= _n_cables; i++) {
    uint8_t jack[] = {TUD_MIDI_DESC_JACK_DESC(i, _cable_name_strid[i - 1])};
    memcpy(buf + len, jack, sizeof(jack));
    len += sizeof(jack);
  }

  // Endpoint OUT + jack mapping
  {
    uint8_t desc[] = {TUD_MIDI_DESC_EP(ep_out, BULK_PACKET_SIZE, _n_cables)};
    memcpy(buf + len, desc, sizeof(desc));
    len += sizeof(desc);
  }

  for (uint8_t i = 1; i <= _n_cables; i++) {
    uint8_t jack[] = {TUD_MIDI_JACKID_IN_EMB(i)};
    memcpy(buf + len, jack, sizeof(jack));
    len += sizeof(jack);
  }

  // Endpoint IN + jack mapping
  {
    uint8_t desc[] = {TUD_MIDI_DESC_EP(ep_in, BULK_PACKET_SIZE, _n_cables)};
    memcpy(buf + len, desc, sizeof(desc));
    len += sizeof(desc);
  }

  for (uint8_t i = 1; i <= _n_cables; i++) {
    uint8_t jack[] = {TUD_MIDI_JACKID_OUT_EMB(i)};
    memcpy(buf + len, jack, sizeof(jack));
    len += sizeof(jack);
  }

  if (len != desc_len) {
    // TODO should throw an error message
    return 0;
  }

  return desc_len;
}

int Adafruit_USBD_MIDI::read(void) {
  uint8_t ch;
  return tud_midi_stream_read(&ch, 1) ? (int)ch : (-1);
}

size_t Adafruit_USBD_MIDI::write(uint8_t b) {
  return tud_midi_stream_write(0, &b, 1);
}

int Adafruit_USBD_MIDI::available(void) { return tud_midi_available(); }

int Adafruit_USBD_MIDI::peek(void) {
  // MIDI Library does not use peek
  return -1;
}

void Adafruit_USBD_MIDI::flush(void) {
  // MIDI Library does not use flush
}

bool Adafruit_USBD_MIDI::writePacket(const uint8_t packet[4]) {
  return tud_midi_packet_write(packet);
}

bool Adafruit_USBD_MIDI::readPacket(uint8_t packet[4]) {
  return tud_midi_packet_read(packet);
}

#endif // CFG_TUD_ENABLED
