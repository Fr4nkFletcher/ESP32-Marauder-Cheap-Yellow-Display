//
// Parallel LCD support for bb_spi_lcd
// written by Larry Bank
//
// Copyright 2017 BitBank Software, Inc. All Rights Reserved.
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//    http://www.apache.org/licenses/LICENSE-2.0
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
//===========================================================================
//
#ifndef __LINUX__
#include <Arduino.h>
#include <SPI.h>
#endif // __LINUX__

#include <bb_spi_lcd.h>

static uint8_t u8BW, u8WR, u8RD, u8DC, u8CS, u8CMD;
static uint8_t *_data_pins;
#ifdef __LINUX__
#include <sys/mman.h>
volatile uint32_t *gpio_port, *set_reg, *clr_reg;
// ---- GPIO specific defines
#define GPIO_REGISTER_BASE 0x200000
#define GPIO_SET_OFFSET 0x1C
#define GPIO_CLR_OFFSET 0x28
#define PHYSICAL_GPIO_BUS (0x7E000000 + GPIO_REGISTER_BASE)
#define BCM2708_PI1_PERI_BASE  0x20000000
#define BCM2709_PI2_PERI_BASE  0x3F000000
#define BCM2711_PI4_PERI_BASE  0xFE000000
#define PERI_BASE BCM2709_PI2_PERI_BASE
#define PAGE_SIZE 4096
#endif // __LINUX__
//#define USE_ESP32_GPIO
#if defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ESP32C3_DEV) && !defined(ARDUINO_ESP32C6_DEV)
#if __has_include (<esp_lcd_panel_io.h>)
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#if __has_include (<esp_lcd_panel_rgb.h>)
#include <esp_lcd_panel_rgb.h>
#endif
#include <esp_lcd_panel_vendor.h>
#include <driver/gpio.h>
#include <esp_private/gdma.h>
#include <hal/dma_types.h>
#include <driver/dedic_gpio.h>
#include <esp32-hal-gpio.h>
#include <hal/gpio_ll.h>
#include <hal/lcd_hal.h>
//#include <soc/lcd_cam_reg.h>
//#include <soc/lcd_cam_struct.h>
#include <hal/lcd_types.h>
//extern DMA_ATTR uint8_t *ucTXBuf;
extern int bSetPosition;
extern volatile bool transfer_is_done;
#ifdef CONFIG_IDF_TARGET_ESP32
uint32_t u32IOMask, u32IOMask2, u32IOLookup[256], u32IOLookup2[256]; // for old ESP32
#endif // CONFIG_IDF_TARGET_ESP32
void spilcdParallelData(uint8_t *pData, int iLen);
static bool s3_notify_dma_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx)
{
//    lv_disp_drv_t *disp_driver = (lv_disp_drv_t *)user_ctx;
//    lv_disp_flush_ready(disp_driver);
    transfer_is_done = true;
    return false;
}
// from esp-idf/components/esp_lcd/src/esp_lcd_panel_io_i80.c
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_i80_bus_handle_t i80_bus = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
struct esp_lcd_i80_bus_t {
    int bus_id;            // Bus ID, index from 0
    portMUX_TYPE spinlock; // spinlock used to protect i80 bus members(hal, device_list, cur_trans)
    lcd_hal_context_t hal; // Hal object
    size_t bus_width;      // Number of data lines
    intr_handle_t intr;    // LCD peripheral interrupt handle
    void* pm_lock; // Power management lock
    size_t num_dma_nodes;  // Number of DMA descriptors
    uint8_t *format_buffer;  // The driver allocates an internal buffer for DMA to do data format transformer
    size_t resolution_hz;    // LCD_CLK resolution, determined by selected clock source
    gdma_channel_handle_t dma_chan; // DMA channel handle
};
#define MAX_TX_SIZE 4096
esp_lcd_i80_bus_config_t s3_bus_config = {
    .dc_gpio_num = 0,
    .wr_gpio_num = 0,
    .clk_src = LCD_CLK_SRC_PLL160M,
    .data_gpio_nums = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    .bus_width = 0,
    .max_transfer_bytes = MAX_TX_SIZE, // debug
//    .psram_trans_align = 0, // 0 = use default values
//    .sram_trans_align = 0,
};

esp_lcd_panel_io_i80_config_t s3_io_config = {
        .cs_gpio_num = 0,
        .pclk_hz = 12000000, // >12Mhz doesn't work on my setup
        .trans_queue_depth = 4,
        .on_color_trans_done = s3_notify_dma_ready,
        .user_ctx = nullptr, // debug
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .dc_levels = {
            .dc_idle_level = 0,
            .dc_cmd_level = 0,
            .dc_dummy_level = 0,
            .dc_data_level = 1,
        },
        .flags = {
            .cs_active_high = 0,
            .reverse_color_bits = 0,
            .swap_color_bytes = 0, // Swap can be done in software (default) or DMA
            .pclk_active_neg = 0,
            .pclk_idle_low = 0,
        },
    };
//static void _gpio_pin_init(int pin)
//{
//  if (pin >= 0)
//  {
//    gpio_pad_select_gpio(pin);
   // gpio_hi(pin);
//    gpio_set_direction((gpio_num_t)pin, GPIO_MODE_OUTPUT);
//  }
//}

#endif // has lcd panel include

#ifdef USE_ESP32_GPIO
int bundleA_gpios[8];
gpio_config_t io_conf = {
    .mode = GPIO_MODE_OUTPUT,
};
dedic_gpio_bundle_handle_t bundleA = NULL;
dedic_gpio_bundle_config_t bundleA_config = {
    .gpio_array = bundleA_gpios,
    .array_size = 8, // debug
    .flags = {
        .out_en = 1,
    },
};
#endif // USE_ESP32_GPIO
volatile lcd_cam_dev_t* _dev;
esp_lcd_i80_bus_handle_t _i80_bus = nullptr;
#ifdef USE_ESP32_GPIO
static void esp32_gpio_clear(int8_t pin)
{
    if (pin < 32) {
        GPIO.out_w1tc = ((uint32_t)1 << pin);
    } else {
        GPIO.out1_w1tc.val = ((uint32_t)1 << (pin-32));
    }
}
static void esp32_gpio_set(int8_t pin)
{
    if (pin < 32) {
        GPIO.out_w1ts = ((uint32_t)1 << pin);
    } else {
        GPIO.out1_w1ts.val = ((uint32_t)1 << (pin-32));
    }
}
#endif // USE_ESP32_GPIO
#endif // ARDUINO_ARCH_ESP32

#ifdef ARDUINO_ARCH_RP2040
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/pio.h"
// This PIO code is Copyright (c) 2021 Pimoroni Ltd
// --------------- //
// st7789_parallel //
// --------------- //

#define st7789_parallel_wrap_target 0
#define st7789_parallel_wrap 1

static const uint16_t st7789_parallel_program_instructions[] = {
            //     .wrap_target
    0x6008, //  0: out    pins, 8         side 0
    0xb042, //  1: nop                    side 1
            //     .wrap
};
static const struct pio_program st7789_parallel_program = {
    .instructions = st7789_parallel_program_instructions,
    .length = 2,
    .origin = -1,
};

static inline pio_sm_config st7789_parallel_program_get_default_config(uint offset) {
    pio_sm_config c = pio_get_default_sm_config();
    sm_config_set_wrap(&c, offset + st7789_parallel_wrap_target, offset + st7789_parallel_wrap);
    sm_config_set_sideset(&c, 1, false, false);
    return c;
}
uint32_t parallel_sm;
PIO parallel_pio;
uint32_t parallel_offset;
uint32_t parallel_dma;
#endif // ARDUINO_ARCH_RP2040

void ParallelReset(void) {
    
} /* ParallelReset() */

void ParallelDataWrite(uint8_t *pData, int len, int iMode)
{
#ifdef __LINUX__
    uint32_t c;
    *clr_reg = (1 << u8CS); // activate CS
    if (iMode == MODE_DATA) {
	    *set_reg = (1 << u8DC);
    } else {
	    *clr_reg = (1 << u8DC);
    }
    for (int i=0; i<len; i++) {
	*clr_reg = (1 << u8WR); // WR low
	c = *pData++;
	*set_reg = (c << 14); // set 1 bits
	c ^= 0xff; // invert for zero bits
	*clr_reg = (c << 14); // set 0 bits
	for (int j=0; j<1; j++) { // add some delay
           iMode |= (c << j);
	}
	transfer_is_done = iMode; // force compiler to actually use the code
        *set_reg = (1 << u8WR); // WR high to latch new data
    } // for i
    *set_reg = (1 << u8CS); // deactivate CS
    return;
#endif // __LINUX__

#ifdef ARDUINO_TEENSY41
    uint32_t c, old = pData[0] -1;
    uint32_t u32 = GPIO6_DR & 0xff00ffff; // clear bits we will change
//    if (iMode == MODE_COMMAND) {
//        Serial.printf("cmd, len=%d\n", len);
//    } else {
//        Serial.printf("data, len=%d\n", len);
//    }
        digitalWrite(u8CS, LOW); // activate CS
        digitalWrite(u8DC, iMode == MODE_DATA); // DC
        for (int i=0; i<len; i++) {
            c = pData[i];
            digitalWrite(u8WR, LOW); // WR low
            if (c != old) {
                GPIO6_DR = (u32 | (c << 16));
                old = c;
             }
            digitalWrite(u8WR, HIGH); // toggle WR high to latch data
        } // for i
        digitalWrite(u8CS, HIGH); // deactivate CS
        return;
#endif // ARDUINO_TEENSY41

#ifdef CONFIG_IDF_TARGET_ESP32
    uint32_t c;
    uint32_t u32Data, u32WR, u32 = REG_READ(GPIO_OUT_REG) & ~u32IOMask;
    uint32_t u32Data2, u32_2 = REG_READ(GPIO_OUT1_REG) & ~u32IOMask2;
//    if (iMode == MODE_COMMAND) {
//        Serial.printf("cmd, len=%d\n", len);
//    } else {
//        Serial.printf("data, len=%d\n", len);
//    }
//        digitalWrite(u8CS, LOW); // activate CS
//        digitalWrite(u8DC, iMode == MODE_DATA); // DC
        u32WR = 1 << u8WR;
        u32 &= ~u32WR; // Write low for first half of operation
        if (u8CS < 32) {
           u32 &= ~(1 << u8CS);
        }
        if (iMode == MODE_DATA)
           u32 |= (1 << u8DC);
        else
           u32 &= ~(1 << u8DC);
        for (int i=0; i<len; i++) {
            c = pData[i];
            digitalWrite(u8WR, LOW); // WR low
#ifdef BRUTE_FORCE
            for (int j=0; j<8; j++) {
                digitalWrite(_data_pins[j], (c & (1<<j));
            }
            digitalWrite(u8WR, 1);
#else
            u32Data = u32 | u32IOLookup[c];
            u32Data2 = u32_2 | u32IOLookup2[c];
            REG_WRITE(GPIO_OUT_REG, u32Data);
            REG_WRITE(GPIO_OUT1_REG, u32Data2);
            REG_WRITE(GPIO_OUT_REG, u32Data | u32WR); // toggle WR high to latch data
#endif // BRUTE_FORCE
        } // for i
        digitalWrite(u8CS, HIGH); // deactivate CS
        return;
#endif // CONFIG_IDF_TARGET_ESP32
 
#ifdef ARDUINO_ARCH_RP2040
// Do everything with DMA since it's the simplest way to push the data
    while (dma_channel_is_busy(parallel_dma))
      ;
    gpio_put(u8DC, (iMode == MODE_DATA)); // DC pin (change after last DMA action completes)
    dma_channel_set_trans_count(parallel_dma, len, false);
    dma_channel_set_read_addr(parallel_dma, pData, true);

// If we didn't use the PIO state machine, this is how we would do it
// (I used this first before enabling the state machine code)
//  for (int i=0; i<len; i++) {
//     uint32_t c = pData[i];
//     if (c != old) {
//        gpio_clr_mask(u32Mask); // clear bits 14-21
//        gpio_set_mask(c << 14);
//        old = c;
//     }
//     digitalWrite(12, LOW); // toggle WR low to high to latch the data
//     digitalWrite(12, HIGH);
//  } // for i
//  gpio_put(10, 1); // deactivate CS
#endif // ARDUINO_ARCH_RP2040
#if defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ESP32C3_DEV) && !defined(ARDUINO_ESP32C6_DEV)
#ifdef FUTURE
    uint8_t c, old = pData[0] -1;
        
        esp32_gpio_clear(u8CS); // activate CS (IO33)
        if (iMode == MODE_COMMAND)
            esp32_gpio_clear(u8DC); // clear DC
        else
            esp32_gpio_set(u8DC); // set DC for data mode
        for (int i=0; i<len; i++) {
            c = pData[i];
            esp32_gpio_clear(u8WR); // WR low
            if (c != old) {
//            GPIO.out_w1tc = u32BitMask; // clear our 8+1 bits
//            GPIO.out_w1ts = u32TransBits[c]; // set the current byte
                dedic_gpio_bundle_write(bundleA, 0xff, c);
                old = c;
             }
            esp32_gpio_set(u8WR); // toggle WR high to latch data
        } // for i
        esp32_gpio_set(u8CS); // (IO33) deactivate CS
#endif // FUTURE
#endif // ARDUINO_ARCH_ESP32
} /* ParallelDataWrite() */
#ifdef __LINUX__
// Return a pointer to a periphery subsystem register.
static void *mmap_bcm_register(off_t register_offset) {
  const off_t base = PERI_BASE;

  int mem_fd;
  if ((mem_fd = open("/dev/mem", O_RDWR|O_SYNC) ) < 0) {
    perror("can't open /dev/mem: ");
    fprintf(stderr, "You need to run this as root!\n");
    return NULL;
  }

  uint32_t *result =
    (uint32_t*) mmap(NULL,                  // Any adddress in our space will do
                     PAGE_SIZE,
                     PROT_READ|PROT_WRITE,  // Enable r/w on GPIO registers.
                     MAP_SHARED,
                     mem_fd,                // File to map
                     base + register_offset // Offset to bcm register
                     );
  close(mem_fd);

  if (result == MAP_FAILED) {
    fprintf(stderr, "mmap error %p\n", result);
    return NULL;
  }
  return result;
} /* mmap_bcm_register() */
#endif // __LINUX__
//
// Initialize a RGB parallel panel (needs continuous pixels)
//
uint16_t * RGBInit(BB_RGB *pRGB)
{
#if defined ARDUINO_ESP32S3_DEV
esp_lcd_rgb_panel_config_t panel_config;

   memset(&panel_config, 0, sizeof(panel_config));
   panel_config.num_fbs = 1; // single framebuffer
   panel_config.psram_trans_align = 64;
   panel_config.sram_trans_align = 8;
   panel_config.data_width = 16;
   panel_config.bits_per_pixel = 16;
   panel_config.clk_src = LCD_CLK_SRC_PLL160M;
   panel_config.disp_gpio_num = -1; // reset?
   panel_config.pclk_gpio_num = pRGB->pclk; // pixel clock
   panel_config.vsync_gpio_num = pRGB->vsync;
   panel_config.hsync_gpio_num = pRGB->hsync;  
   panel_config.de_gpio_num = pRGB->de;
   panel_config.data_gpio_nums[0] = pRGB->g3;
   panel_config.data_gpio_nums[1] = pRGB->g4;
   panel_config.data_gpio_nums[2] = pRGB->g5;
   panel_config.data_gpio_nums[3] = pRGB->r0;
   panel_config.data_gpio_nums[4] = pRGB->r1;
   panel_config.data_gpio_nums[5] = pRGB->r2;
   panel_config.data_gpio_nums[6] = pRGB->r3;
   panel_config.data_gpio_nums[7] = pRGB->r4;
   panel_config.data_gpio_nums[8] = pRGB->b0;
   panel_config.data_gpio_nums[9] = pRGB->b1;
   panel_config.data_gpio_nums[10] = pRGB->b2;
   panel_config.data_gpio_nums[11] = pRGB->b3;
   panel_config.data_gpio_nums[12] = pRGB->b4;
   panel_config.data_gpio_nums[13] = pRGB->g0;
   panel_config.data_gpio_nums[14] = pRGB->g1;
   panel_config.data_gpio_nums[15] = pRGB->g2;
   panel_config.flags.fb_in_psram = 1; // use PSRAM
   panel_config.timings.pclk_hz = pRGB->speed;
   panel_config.timings.h_res = pRGB->width;
   panel_config.timings.v_res = pRGB->height;
   panel_config.timings.flags.hsync_idle_low = (pRGB->hsync_polarity == 0) ? 1: 0;
   panel_config.timings.flags.vsync_idle_low = (pRGB->vsync_polarity == 0) ? 1: 0;
   panel_config.timings.hsync_back_porch = pRGB->hsync_back_porch;
   panel_config.timings.hsync_front_porch = pRGB->hsync_front_porch;
   panel_config.timings.hsync_pulse_width = pRGB->hsync_pulse_width;
   panel_config.timings.vsync_back_porch = pRGB->vsync_back_porch;
   panel_config.timings.vsync_front_porch = pRGB->vsync_front_porch;
   panel_config.timings.vsync_pulse_width = pRGB->vsync_pulse_width;
   ESP_ERROR_CHECK(esp_lcd_new_rgb_panel(&panel_config, &panel_handle));
   ESP_ERROR_CHECK(esp_lcd_panel_reset(panel_handle));
   ESP_ERROR_CHECK(esp_lcd_panel_init(panel_handle));
   uint16_t *p;
   esp_lcd_rgb_panel_get_frame_buffer(panel_handle, 1, (void **)&p);
   return p;
#else // not S3
   return NULL;
#endif // ARDUINO_ESP32S3_DEV
} /* RGBInit() */

//
// Initialize the parallel bus info
//
void ParallelDataInit(uint8_t RD_PIN, uint8_t WR_PIN, uint8_t CS_PIN, uint8_t DC_PIN, int iBusWidth, uint8_t *data_pins, int iFlags, uint32_t u32Freq)
{
    u8WR = WR_PIN;
    u8RD = RD_PIN;
    u8DC = DC_PIN;
    u8CS = CS_PIN;
    u8BW = (uint8_t)iBusWidth;
    u8CMD = (iFlags & FLAGS_MEM_RESTART) ? 0x2c : 0x3c;
    if (RD_PIN != 0xff) {
        pinMode(RD_PIN, OUTPUT); // RD
        digitalWrite(RD_PIN, HIGH); // RD deactivated
    }
// Linux and Teensy 4.x use parallel GPIO for now
#if defined ( __LINUX__ ) // || defined( ARDUINO_TEENSY41 )
    pinMode(u8WR, OUTPUT);
    pinMode(u8CS, OUTPUT);
    pinMode(u8DC, OUTPUT);
    for (int i=0; i<8; i++) {
        pinMode(data_pins[i], OUTPUT);
    }
    _data_pins = data_pins;
  // Prepare GPIO
  gpio_port = mmap_bcm_register(GPIO_REGISTER_BASE);
  set_reg = gpio_port + (GPIO_SET_OFFSET / sizeof(uint32_t));
  clr_reg = gpio_port + (GPIO_CLR_OFFSET / sizeof(uint32_t));
#ifdef CONFIG_IDF_TARGET_ESP32
// Create a bit mask and lookup table to allow fast 8-bit writes
// to the 32-bit GPIO register
   u32IOMask = u32IOMask2 = 0;
   for (int i=0; i<8; i++) {
       if (data_pins[i] < 32)
          u32IOMask |= 1 << data_pins[i];
       else
          u32IOMask2 |= 1 << (data_pins[i]-32);
   }
   REG_WRITE(GPIO_ENABLE_W1TS_REG, u32IOMask); // enable all as outputs
   REG_WRITE(GPIO_ENABLE1_W1TS_REG, u32IOMask2);
// Fast lookup table to translate 8 scattered bits to 32-bit values
   for (int i=0; i<256; i++) {
       uint32_t u32 = 0, u32_2 = 0;
       for (int j=0; j<8; j++) {
           if (i & (1<<j)) { // set bit
               if (data_pins[j] < 32)
                  u32 |= (1 << data_pins[j]);
               else
                  u32_2 |= (1 << (data_pins[j]-32));
           }
       } // for j
       u32IOLookup[i] = u32;
       u32IOLookup2[i] = u32_2;
   } // for i
#endif // CONFIG_IDF_TARGET_ESP32
   return;
#endif // __LINUX__ || ARDUINO_TEENSY41
#ifdef ARDUINO_ARCH_RP2040

// Set up GPIO for output mode
//  for (int i=10; i<=21; i++) { // I/O lines
//     pinMode(i, OUTPUT);
//  }
//  digitalWrite(12, HIGH); // WR deactivated
//  pinMode(10, OUTPUT); // CS
//  digitalWrite(10, HIGH); // CS deactivated
    gpio_set_function(DC_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(DC_PIN, GPIO_OUT);
    gpio_set_function(WR_PIN, GPIO_FUNC_SIO);
    gpio_set_dir(WR_PIN, GPIO_OUT);

    if (CS_PIN >= 0 && CS_PIN < 40) {
        gpio_set_function(CS_PIN, GPIO_FUNC_SIO);
        gpio_set_dir(CS_PIN, GPIO_OUT);
        gpio_put(CS_PIN, 0); // CS always active
    }

      parallel_pio = pio1;
      parallel_sm = pio_claim_unused_sm(parallel_pio, true);
      parallel_offset = pio_add_program(parallel_pio, &st7789_parallel_program);
      pio_gpio_init(parallel_pio, WR_PIN);
      for(int i = 0; i < 8; i++) {
        pio_gpio_init(parallel_pio, data_pins[0] + i); // NB: must be sequential GPIO numbers starting from D0
      }
      pio_sm_set_consecutive_pindirs(parallel_pio, parallel_sm, data_pins[0], 8, true);
      pio_sm_set_consecutive_pindirs(parallel_pio, parallel_sm, WR_PIN, 1, true);

      pio_sm_config c = st7789_parallel_program_get_default_config(parallel_offset);

      sm_config_set_out_pins(&c, data_pins[0], 8);
      sm_config_set_sideset_pins(&c, WR_PIN);
      sm_config_set_fifo_join(&c, PIO_FIFO_JOIN_TX);
      sm_config_set_out_shift(&c, false, true, 8);
      sm_config_set_clkdiv(&c, 4);
      
      pio_sm_init(parallel_pio, parallel_sm, parallel_offset, &c);
      pio_sm_set_enabled(parallel_pio, parallel_sm, true);

      parallel_dma = dma_claim_unused_channel(true);
      dma_channel_config config = dma_channel_get_default_config(parallel_dma);
      channel_config_set_transfer_data_size(&config, DMA_SIZE_8);
      channel_config_set_bswap(&config, false);
      channel_config_set_dreq(&config, pio_get_dreq(parallel_pio, parallel_sm, true));
      dma_channel_configure(parallel_dma, &config, &parallel_pio->txf[parallel_sm], NULL, 0, false);
#endif // ARDUINO_ARCH_RP2040
#if defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ESP32C3_DEV) && !defined(ARDUINO_ESP32C6_DEV)
    if (iFlags & FLAGS_SWAP_COLOR) {
        s3_io_config.flags.swap_color_bytes = 1;
    }
#ifdef USE_ESP32_GPIO
    pinMode(u8WR, OUTPUT);
    if (u8CS < 99) {
        pinMode(u8CS, OUTPUT);
    }
    pinMode(u8DC, OUTPUT);
    // Create N-bit dedicated GPIO bundle to toggle all pins at once
    for (int i = 0; i < iBusWidth; i++) {
        bundleA_gpios[i] = data_pins[i]; // convert uint8_t to int
        io_conf.pin_bit_mask = 1ULL << bundleA_gpios[i];
        gpio_config(&io_conf);
    }
    // Create bundleA, output only
    ESP_ERROR_CHECK(dedic_gpio_new_bundle(&bundleA_config, &bundleA));
    transfer_is_done = true;
#else // USE_ESP32_GPIO
    s3_bus_config.dc_gpio_num = u8DC;
    s3_bus_config.wr_gpio_num = u8WR;
    s3_bus_config.bus_width = iBusWidth;
    for (int i=0; i<iBusWidth; i++) {
        s3_bus_config.data_gpio_nums[i] = data_pins[i];
    }
    ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&s3_bus_config, &i80_bus));
//    pinMode(u8CS, OUTPUT);
//    digitalWrite(u8CS, LOW); // permanently active
    if (u8CS < 99)
        s3_io_config.cs_gpio_num = u8CS;
    else
        s3_io_config.cs_gpio_num = -1;
    s3_io_config.pclk_hz = u32Freq;
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &s3_io_config, &io_handle));
    transfer_is_done = true;
#endif // USE_ESP32_GPIO
#endif // ARDUINO_ARCH_ESP32
} /* ParallelDataInit() */

void spilcdParallelCMDParams(uint8_t ucCMD, uint8_t *pParams, int iLen)
{
#if defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ESP32C3_DEV) && !defined(ARDUINO_ESP32C6_DEV)
#ifdef USE_ESP32_GPIO
    esp32_gpio_clear(u8DC); // clear DC
    spilcdParallelData(&ucCMD, 1);
    esp32_gpio_set(u8DC);
    if (iLen) {
        spilcdParallelData(pParams, iLen);
    }
#else
    while (!transfer_is_done) {
//        delayMicroseconds(1);
    }
    esp_lcd_panel_io_tx_param(io_handle, ucCMD, pParams, iLen);
    u8CMD = 0x2c; // memory restart
#endif // USE_ESP32_GPIO
#endif // ARDUINO_ARCH_ESP32
} /* spilcdParallelCMDParams() */

void spilcdParallelData(uint8_t *pData, int iLen)
{
#if defined(ARDUINO_ARCH_ESP32) && !defined(ARDUINO_ESP32C3_DEV) && !defined(ARDUINO_ESP32C6_DEV)
#ifdef USE_ESP32_GPIO
    uint8_t c, old = pData[0] -1;
        
        esp32_gpio_clear(u8CS); // activate CS (IO33)
//        if (iMode == MODE_COMMAND)
//            esp32_gpio_clear(u8DC); // clear DC
//        else
//            esp32_gpio_set(u8DC); // set DC for data mode
        for (int i=0; i<iLen; i++) {
            c = pData[i];
            esp32_gpio_clear(u8WR); // WR low
            if (c != old) {
//            GPIO.out_w1tc = u32BitMask; // clear our 8+1 bits
//            GPIO.out_w1ts = u32TransBits[c]; // set the current byte
                dedic_gpio_bundle_write(bundleA, 0xff, c);
                old = c;
             }
            esp32_gpio_set(u8WR); // toggle WR high to latch data
        } // for i
        esp32_gpio_set(u8CS); // (IO33) deactivate CS
#else
    int iSize;
    while (iLen) {
        while (!transfer_is_done) {
           // delayMicroseconds(1);
        }
        transfer_is_done = false; // since we're not using a ping-pong buffer scheme
        iSize = iLen;
        if (iSize > MAX_TX_SIZE) iSize = MAX_TX_SIZE;
        esp_lcd_panel_io_tx_color(io_handle, u8CMD, pData, iSize);
        u8CMD = 0x3c; // memory continue;
        iLen -= iSize;
        pData += iSize;
    }
#endif // USE_ESP32_GPIO
#endif // ARDUINO_ARCH_ESP32
} /* spilcdParallelData() */
