#include <stdio.h>
#include <stdlib.h>
#include "hardware/clocks.h"
#include "hardware/dma.h"
#include "hardware/gpio.h"
#include "hardware/irq.h"
#include "hardware/pll.h"
#include "hardware/sync.h"
#include "hardware/structs/bus_ctrl.h"
#include "hardware/structs/ssi.h"
#include "hardware/vreg.h"
#include "pico/multicore.h"
#include "pico/sem.h"
#include "pico/stdlib.h"

#include "tmds_encode.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "common_dvi_pin_configs.h"

// TMDS bit clock 372 MHz
// DVDD 1.25V (1.2V seems ok too)
#define FRAME_WIDTH 1280
#define FRAME_HEIGHT 720
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_1280x720p_30hz

#define LED_PIN 25

#define IMAGE_BASE 0x1003c000

#define FRAMES_PER_IMAGE 300

struct dvi_inst dvi0;
struct semaphore dvi_start_sem;

#define PALETTE_BITS 8
#define MAX_PALETTE_SIZE (1 << PALETTE_BITS)
uint32_t palette[MAX_PALETTE_SIZE];
uint32_t tmds_palette[MAX_PALETTE_SIZE * 6];

void __no_inline_not_in_flash_func(flash_bulk_dma_start)(uint32_t *rxbuf, uint32_t *flash_ptr, size_t len, uint dma_chan)
{
	ssi_hw->ssienr = 0;
	ssi_hw->ctrlr1 = len - 1; // NDF, number of data frames
	ssi_hw->dmacr = SSI_DMACR_TDMAE_BITS | SSI_DMACR_RDMAE_BITS;
	ssi_hw->ssienr = 1;
	// Other than NDF, the SSI configuration used for XIP is suitable for a bulk read too.

	dma_hw->ch[dma_chan].read_addr = (uint32_t)&ssi_hw->dr0;
	dma_hw->ch[dma_chan].write_addr = (uint32_t)rxbuf;
	dma_hw->ch[dma_chan].transfer_count = len;
	dma_hw->ch[dma_chan].ctrl_trig =
		DMA_CH0_CTRL_TRIG_BSWAP_BITS |
		DREQ_XIP_SSIRX << DMA_CH0_CTRL_TRIG_TREQ_SEL_LSB |
		dma_chan << DMA_CH0_CTRL_TRIG_CHAIN_TO_LSB |
		DMA_CH0_CTRL_TRIG_INCR_WRITE_BITS |
		DMA_CH0_CTRL_TRIG_DATA_SIZE_VALUE_SIZE_WORD << DMA_CH0_CTRL_TRIG_DATA_SIZE_LSB |
		DMA_CH0_CTRL_TRIG_EN_BITS;

	// Now DMA is waiting, kick off the SSI transfer (mode continuation bits in LSBs)
	ssi_hw->dr0 = ((uintptr_t)flash_ptr << 8) | 0xa0;
}

// Core 1 handles DMA IRQs and runs TMDS encode on scanline buffers it
// receives through the mailbox FIFO
void __not_in_flash("main") core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	sem_acquire_blocking(&dvi_start_sem);
	dvi_start(&dvi0);

	while (1) {
		const uint32_t *colourbuf = (const uint32_t*)multicore_fifo_pop_blocking();
		uint32_t *tmdsbuf = (uint32_t*)multicore_fifo_pop_blocking();
		tmds_encode_palette_data(colourbuf, tmds_palette, tmdsbuf, FRAME_WIDTH, PALETTE_BITS);
		multicore_fifo_push_blocking(0);
	}
	__builtin_unreachable();
}

static uint img_dma_chan;
static uint8_t img_buf[2][2 * FRAME_WIDTH];
static uint32_t* img_base = (uint32_t*)IMAGE_BASE;
static uint32_t palette_size;
static uint32_t img_data_size;
static uint8_t* img_data_ptr;

static void setup_image() {
	printf("Setup image from %p\n", img_base);
	palette_size = *img_base;
	img_data_size = *(img_base + 1);
	img_data_ptr = (uint8_t*)(img_base + 2 + palette_size);

	assert(palette_size <= MAX_PALETTE_SIZE);
	assert(2 * FRAME_WIDTH > sizeof(uint32_t) * palette_size);
	flash_bulk_dma_start((uint32_t*)img_buf[0], img_base + 2, palette_size, img_dma_chan);
	dma_channel_wait_for_finish_blocking(img_dma_chan);
	tmds_setup_palette24_symbols((uint32_t*)img_buf[0], tmds_palette, MAX_PALETTE_SIZE);
}

int __not_in_flash("main") main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);
	// gpio_put(LED_PIN, 1);

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	printf("DMA palette and setup\n");
	img_dma_chan = dma_claim_unused_channel(true);
	setup_image();

	printf("DMA first image line\n");
	flash_bulk_dma_start((uint32_t*)img_buf[0], (uint32_t*)img_data_ptr, FRAME_WIDTH * 2 / sizeof(uint32_t), img_dma_chan);
	dma_channel_wait_for_finish_blocking(img_dma_chan);
	int img_buf_front = 0;
	int img_buf_back = 1;

	printf("Core 1 start\n");
	sem_init(&dvi_start_sem, 0, 1);
	hw_set_bits(&bus_ctrl_hw->priority, BUSCTRL_BUS_PRIORITY_PROC1_BITS);
	multicore_launch_core1(core1_main);

	uint heartbeat = 0;
	uint slideshow_ctr = 0;

	sem_release(&dvi_start_sem);
	while (1) {
		if (++heartbeat >= 30) {
			heartbeat = 0;
			gpio_xor_mask(1u << LED_PIN);
		}
		if (++slideshow_ctr >= FRAMES_PER_IMAGE) {
			slideshow_ctr = 0;
			img_base += 2 + palette_size + (img_data_size / sizeof(uint32_t));
			if (*img_base == 0) img_base = (uint32_t*)IMAGE_BASE;
			
			setup_image();
		}
		for (int y = 0; y < FRAME_HEIGHT; y += 2) {
			// Start DMA to back buffer before starting to encode the front buffer (each buffer is two scanlines)
			flash_bulk_dma_start(
				(uint32_t*)img_buf[img_buf_back],
				(uint32_t*)(img_data_ptr + ((y + 2) % FRAME_HEIGHT) * FRAME_WIDTH),
				FRAME_WIDTH * 2 / sizeof(uint32_t),
				img_dma_chan
			);
			const uint8_t *img = (const uint8_t*)img_buf[img_buf_front];			
			uint32_t *our_tmds_buf, *their_tmds_buf;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &their_tmds_buf);
			multicore_fifo_push_blocking((uint32_t)(img));
			multicore_fifo_push_blocking((uint32_t)their_tmds_buf);
	
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &our_tmds_buf);
			tmds_encode_palette_data((const uint32_t*)(img + FRAME_WIDTH), tmds_palette, our_tmds_buf, FRAME_WIDTH, PALETTE_BITS);
			
			multicore_fifo_pop_blocking();
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &their_tmds_buf);
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &our_tmds_buf);
			// Swap the buffers after each scanline pair completion
			img_buf_front = !img_buf_front;
			img_buf_back = !img_buf_back;
		}
	}
	__builtin_unreachable();
}
	
