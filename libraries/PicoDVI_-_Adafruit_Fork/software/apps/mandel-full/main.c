#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"

#include "tmds_encode.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"

#include "mandelbrot.h"

// TMDS bit clock 252 MHz
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 480
#define VREG_VSEL VREG_VOLTAGE_1_10
#define DVI_TIMING dvi_timing_640x480p_60hz

#define N_IMAGES 3
#define FRAMES_PER_IMAGE 300

uint8_t mandel[FRAME_WIDTH * (FRAME_HEIGHT / 2)];

#define PALETTE_BITS 8
#define PALETTE_SIZE (1 << PALETTE_BITS)
uint16_t palette[PALETTE_SIZE];

uint32_t tmds_palette[PALETTE_SIZE * 6];

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

FractalBuffer fractal;

static uint8_t palette_offset = 0;

void init_palette() {
  palette[0] = 0;
  for (int i = 1; i < PALETTE_SIZE; ++i) {
    uint8_t c = i + palette_offset;
    if (c < 0x20) palette[i] = c;
    else if (c < 0x40) palette[i] = (c - 0x20) << 6;
    else if (c < 0x60) palette[i] = (c - 0x40) << 11;
    else if (c < 0x80) palette[i] = ((c - 0x60) & 0x1f) * 0x0840;
    else if (c < 0xa0) palette[i] = ((c - 0x80) & 0x1f) * 0x0041;
    else if (c < 0xc0) palette[i] = ((c - 0xa0) & 0x1f) * 0x0801;
    else if (c < 0xe0) palette[i] = ((c - 0xc0) & 0x1f) * 0x0841;
    else palette[i] = 0;
  }
  ++palette_offset;

  tmds_setup_palette_symbols(palette, tmds_palette, PALETTE_SIZE);
}

void init_mandel() {
  for (int y = 0; y < (FRAME_HEIGHT / 2); ++y) {
    uint8_t* buf = &mandel[y * FRAME_WIDTH];
    for (int i = 0; i < FRAME_WIDTH; ++i) {
      buf[i] = ((i + y) & 0x3f);
    }
  }

  fractal.buff = mandel;
  fractal.rows = FRAME_HEIGHT / 2;
  fractal.cols = FRAME_WIDTH;
  fractal.max_iter = PALETTE_SIZE;
  fractal.iter_offset = 0;
  fractal.minx = -2.25f;
  fractal.maxx = 0.75f;
  fractal.miny = -1.6f;
  fractal.maxy = 0.f - (1.6f / FRAME_HEIGHT); // Half a row
  fractal.use_cycle_check = true;
  init_fractal(&fractal);
}

#define NUM_ZOOMS 64
static uint32_t zoom_count = 0;

void zoom_mandel() {
  if (++zoom_count == NUM_ZOOMS)
  {
    init_mandel();
    zoom_count = 0;
    return;
  }

  printf("Zoom: %ld\n", zoom_count);

  float zoomx = -.75f - .7f * ((float)zoom_count / (float)NUM_ZOOMS);
  float sizex = fractal.maxx - fractal.minx;
  float sizey = fractal.miny * -2.f;
  float zoomr = 0.96f * 0.5f;
  fractal.minx = zoomx - zoomr * sizex;
  fractal.maxx = zoomx + zoomr * sizex;
  fractal.miny = -zoomr * sizey;
  fractal.maxy = 0.f + fractal.miny / FRAME_HEIGHT;
  init_fractal(&fractal);
}

// Core 1 handles DMA IRQs and runs TMDS encode on scanline buffers it
// receives through the mailbox FIFO
void __not_in_flash("core1_main") core1_main() {
  dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
  sem_acquire_blocking(&dvi_start_sem);
  dvi_start(&dvi0);

  while (1) {
    const uint32_t *colourbuf = (const uint32_t*)multicore_fifo_pop_blocking();
    uint32_t *tmdsbuf = (uint32_t*)multicore_fifo_pop_blocking();
    tmds_encode_palette_data((const uint32_t*)colourbuf, tmds_palette, tmdsbuf, FRAME_WIDTH, PALETTE_BITS);
    multicore_fifo_push_blocking(0);
    while (!fractal.done && queue_get_level(&dvi0.q_tmds_valid) >= 5) generate_steal_one(&fractal);
  }
  __builtin_unreachable();
}

int __not_in_flash("main") main() {
  vreg_set_voltage(VREG_VSEL);
  sleep_ms(10);
  set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

  setup_default_uart();

  gpio_init(PICO_DEFAULT_LED_PIN);
  gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
  
  init_palette();
  init_mandel();

  printf("Configuring DVI\n");

  dvi0.timing = &DVI_TIMING;
  dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
  dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

  printf("Core 1 start\n");
  sem_init(&dvi_start_sem, 0, 1);
  hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
  multicore_launch_core1(core1_main);

  uint heartbeat = 0;
  uint32_t encode_time = 0;

  sem_release(&dvi_start_sem);
  while (1) {
    if (++heartbeat >= 30) {
      heartbeat = 0;
      gpio_xor_mask(1u << PICO_DEFAULT_LED_PIN);

      printf("Encode total time: %ldus\n", encode_time);
      encode_time = 0;
    }
    if (fractal.done) zoom_mandel();
    //if (heartbeat & 1) init_palette();
    for (int y = 0; y < FRAME_HEIGHT / 2; y += 2) {
      uint32_t *our_tmds_buf, *their_tmds_buf;
      queue_remove_blocking_u32(&dvi0.q_tmds_free, &their_tmds_buf);
      multicore_fifo_push_blocking((uint32_t)(&mandel[y*FRAME_WIDTH]));
      multicore_fifo_push_blocking((uint32_t)their_tmds_buf);
  
      queue_remove_blocking_u32(&dvi0.q_tmds_free, &our_tmds_buf);
      absolute_time_t start_time = get_absolute_time();
      tmds_encode_palette_data((const uint32_t*)(&mandel[(y+1)*FRAME_WIDTH]), tmds_palette, our_tmds_buf, FRAME_WIDTH, PALETTE_BITS);
      encode_time += absolute_time_diff_us(start_time, get_absolute_time());
      
      multicore_fifo_pop_blocking();

      while (!fractal.done && queue_get_level(&dvi0.q_tmds_valid) >= 5) generate_one_forward(&fractal);

      queue_add_blocking_u32(&dvi0.q_tmds_valid, &their_tmds_buf);
      queue_add_blocking_u32(&dvi0.q_tmds_valid, &our_tmds_buf);
    }
    for (int y = FRAME_HEIGHT / 2 - 2; y >= 0; y -= 2) {
      uint32_t *our_tmds_buf, *their_tmds_buf;
      queue_remove_blocking_u32(&dvi0.q_tmds_free, &their_tmds_buf);
      multicore_fifo_push_blocking((uint32_t)(&mandel[(y+1)*FRAME_WIDTH]));
      multicore_fifo_push_blocking((uint32_t)their_tmds_buf);
  
      queue_remove_blocking_u32(&dvi0.q_tmds_free, &our_tmds_buf);
      absolute_time_t start_time = get_absolute_time();
      tmds_encode_palette_data((const uint32_t*)(&mandel[y*FRAME_WIDTH]), tmds_palette, our_tmds_buf, FRAME_WIDTH, PALETTE_BITS);
      encode_time += absolute_time_diff_us(start_time, get_absolute_time());
      
      multicore_fifo_pop_blocking();

      while (!fractal.done && queue_get_level(&dvi0.q_tmds_valid) >= 5) generate_one_forward(&fractal);

      queue_add_blocking_u32(&dvi0.q_tmds_valid, &their_tmds_buf);
      queue_add_blocking_u32(&dvi0.q_tmds_valid, &our_tmds_buf);
    }
  }
  __builtin_unreachable();
}
  
