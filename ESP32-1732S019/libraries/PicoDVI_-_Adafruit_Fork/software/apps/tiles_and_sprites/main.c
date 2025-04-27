#include <stdio.h>
#include <stdlib.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "hardware/clocks.h"
#include "hardware/irq.h"
#include "hardware/sync.h"
#include "hardware/gpio.h"
#include "hardware/vreg.h"
#include "pico/sem.h"

#include "dvi.h"
#include "dvi_serialiser.h"
#include "tmds_encode.h"
#include "common_dvi_pin_configs.h"
#include "sprite.h"
#include "tile.h"

#include "tilemap.h"
#include "zelda_mini_plus_walk_rgab5515.h"

// Pick one:
#define MODE_640x480_60Hz
// #define MODE_800x480_60Hz
// #define MODE_800x600_60Hz
// #define MODE_960x540p_60Hz
// #define MODE_1280x720_30Hz

#if defined(MODE_640x480_60Hz)
// DVDD 1.2V (1.1V seems ok too)
#define FRAME_WIDTH 320
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_640x480p_60hz

#elif defined(MODE_800x480_60Hz)
#define FRAME_WIDTH 400
#define FRAME_HEIGHT 240
#define VREG_VSEL VREG_VOLTAGE_1_20
#define DVI_TIMING dvi_timing_800x480p_60hz

#elif defined(MODE_800x600_60Hz)
// DVDD 1.3V, going downhill with a tailwind
#define FRAME_WIDTH 400
#define FRAME_HEIGHT 300
#define VREG_VSEL VREG_VOLTAGE_1_30
#define DVI_TIMING dvi_timing_800x600p_60hz

#elif defined(MODE_960x540p_60Hz)
// DVDD 1.25V (slower silicon may need the full 1.3, or just not work)
// Frame resolution is almost the same as a PSP :)
#define FRAME_WIDTH 480
#define FRAME_HEIGHT 270
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_960x540p_60hz

#elif defined(MODE_1280x720_30Hz)
// 1280x720p 30 Hz (nonstandard)
// DVDD 1.25V (slower silicon may need the full 1.3, or just not work)
#define FRAME_WIDTH 640
#define FRAME_HEIGHT 360
#define VREG_VSEL VREG_VOLTAGE_1_25
#define DVI_TIMING dvi_timing_1280x720p_30hz

#else
#error "Select a video mode!"
#endif

#define LED_PIN 21

// ----------------------------------------------------------------------------
// Rendering/animation stuff

#define MAP_WIDTH  512
#define MAP_HEIGHT 256

#define N_CHARACTERS 100

typedef struct {
	int16_t pos_x;
	int16_t pos_y;
	uint8_t tile;
	uint8_t tilestride;
	uint8_t ntiles;

	int16_t xmin, ymin;
	int16_t xmax, ymax;
	uint8_t dir;
	uint8_t anim_frame;
} character_t;

typedef struct {
	int16_t cam_x;
	int16_t cam_y;
	uint32_t frame_ctr;
	character_t chars[N_CHARACTERS];
} game_state_t;

static inline int clip(int x, int min, int max) {
	return x < min ? min : x > max ? max : x;
}

void game_init(game_state_t *state) {
	state->cam_x = 0;
	state->cam_y = 0;
	state->frame_ctr = 0;
	for (int i = 0; i < N_CHARACTERS; ++i) {
		state->chars[i].dir = (rand() >> 16) & 0x3;
		state->chars[i].anim_frame = 0;
		state->chars[i].xmin = 8;
		state->chars[i].ymin = -6;
		state->chars[i].xmax = MAP_WIDTH - 24;
		state->chars[i].ymax = 128+87;
		state->chars[i].pos_x = rand() & 0xff;
		state->chars[i].pos_x = rand() & 0xff;
		state->chars[i].tile = 102;
		state->chars[i].tilestride = 17;
		state->chars[i].ntiles = 2;
	}
}

void update(game_state_t *state) {
	static bool cointoss = false;
	if ((cointoss = !cointoss))
		return;

	state->frame_ctr++;

	const int CAMERA_SPEED = 3;
	if (state->frame_ctr % 200 < 50)
		state->cam_x += CAMERA_SPEED;
	else if (state->frame_ctr % 200 < 100)
		state->cam_y += CAMERA_SPEED;
	else if (state->frame_ctr % 200 < 150)
		state->cam_x -= CAMERA_SPEED;
	else
		state->cam_y -= CAMERA_SPEED;
	state->cam_x = clip(state->cam_x, 0, MAP_WIDTH - FRAME_WIDTH);
	state->cam_y = clip(state->cam_y, 0, MAP_HEIGHT - FRAME_HEIGHT);

	const int CHAR_SPEED = 2;
	for (int i = 0; i < N_CHARACTERS; ++i) {
		character_t *ch = &state->chars[i];
		if ((state->frame_ctr & 0x3u) == 0)
			ch->anim_frame = (ch->anim_frame + 1) & 0x3;
		if (!(rand() & 0xf00)) {
			ch->anim_frame = 0;
			ch->dir = (rand() >> 16) & 0x3;
		}
		ch->pos_x += ch->dir == 1 ? CHAR_SPEED : ch->dir == 3 ? -CHAR_SPEED : 0;
		ch->pos_y += ch->dir == 0 ? CHAR_SPEED : ch->dir == 2 ? -CHAR_SPEED : 0;
		ch->pos_x = clip(ch->pos_x, ch->xmin, ch->xmax);
		ch->pos_y = clip(ch->pos_y, ch->ymin, ch->ymax);
	}

	static uint heartbeat = 0;
	if (++heartbeat >= 30) {
		heartbeat = 0;
		gpio_xor_mask(1u << LED_PIN);
	}
}


void render_scanline(uint16_t *pixbuf, uint y, const game_state_t *gstate) {
	tilebg_t bg = {
		.xscroll = gstate->cam_x,
		.yscroll = gstate->cam_y,
		.tileset = zelda_mini_plus_walk,
		.tilemap = tilemap,
		.log_size_x = 9,
		.log_size_y = 8,
		.tilesize = TILESIZE_16,
		.fill_loop = (tile_loop_t)tile16_16px_loop
	};

	sprite_t sp = {
		.log_size = 4,
		.has_opacity_metadata = false,
	};

	tile16(pixbuf, &bg, y, FRAME_WIDTH);

	for (int i = 0; i < N_CHARACTERS; ++i) {
		const character_t *ch = &gstate->chars[i];
		sp.x = ch->pos_x - gstate->cam_x;
		const uint16_t *basetile = (const uint16_t*)zelda_mini_plus_walk +
			16 * 16 * (102 + (ch->dir << 2) + ch->anim_frame);
		for (int tile = 0; tile < ch->ntiles; ++tile) {
			sp.y = ch->pos_y - gstate->cam_y + tile * 16;
			sp.img = basetile + tile * ch->tilestride * 16 * 16;
			sprite_sprite16(pixbuf, &sp, y, FRAME_WIDTH);
		}
	}
}

// ----------------------------------------------------------------------------
// DVI setup & launch

struct dvi_inst dvi0;
game_state_t state;

uint16_t __scratch_y("render") __attribute__((aligned(4))) core0_scanbuf[FRAME_WIDTH];
uint16_t __scratch_x("render") __attribute__((aligned(4))) core1_scanbuf[FRAME_WIDTH];

// - Core 0 pops two TMDS buffers
// - Passes one to core 1
// - Renders own buffer and pushes to DVI queue  <- core 1 waits here before starting DVI
// - Retrieves core 1's TMDS buffer and pushes that to DVI queue as well

void encode_scanline(uint16_t *pixbuf, uint32_t *tmdsbuf) {
	uint pixwidth = dvi0.timing->h_active_pixels;
	uint words_per_channel = pixwidth / DVI_SYMBOLS_PER_WORD;
	tmds_encode_data_channel_16bpp((uint32_t*)pixbuf, tmdsbuf + 0 * words_per_channel, pixwidth / 2, DVI_16BPP_BLUE_MSB,  DVI_16BPP_BLUE_LSB );
	tmds_encode_data_channel_16bpp((uint32_t*)pixbuf, tmdsbuf + 1 * words_per_channel, pixwidth / 2, DVI_16BPP_GREEN_MSB, DVI_16BPP_GREEN_LSB);
	tmds_encode_data_channel_16bpp((uint32_t*)pixbuf, tmdsbuf + 2 * words_per_channel, pixwidth / 2, DVI_16BPP_RED_MSB,   DVI_16BPP_RED_LSB  );
}

void core1_main() {
	dvi_register_irqs_this_core(&dvi0, DMA_IRQ_0);
	while (queue_is_empty(&dvi0.q_tmds_valid))
		__wfe();
	dvi_start(&dvi0);
	while (1) {
		for (uint y = 1; y < FRAME_HEIGHT; y += 2) {
			render_scanline(core1_scanbuf, y, &state);
			uint32_t *tmdsbuf = (uint32_t*)multicore_fifo_pop_blocking();
			encode_scanline(core1_scanbuf, tmdsbuf);
			multicore_fifo_push_blocking((uintptr_t)tmdsbuf);
		}
	}
}

int main() {
	vreg_set_voltage(VREG_VSEL);
	sleep_ms(10);
	set_sys_clock_khz(DVI_TIMING.bit_clk_khz, true);

	setup_default_uart();

	gpio_init(LED_PIN);
	gpio_set_dir(LED_PIN, GPIO_OUT);

	printf("Configuring DVI\n");

	dvi0.timing = &DVI_TIMING;
	dvi0.ser_cfg = DVI_DEFAULT_SERIAL_CONFIG;
	dvi_init(&dvi0, next_striped_spin_lock_num(), next_striped_spin_lock_num());

	printf("Core 1 start\n");
	multicore_launch_core1(core1_main);

	printf("Start rendering\n");
	game_init(&state);
	while (1) {
		for (uint y = 0; y < FRAME_HEIGHT; y += 2) {
			uint32_t *tmds0, *tmds1;
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmds0);
			queue_remove_blocking_u32(&dvi0.q_tmds_free, &tmds1);
			multicore_fifo_push_blocking((uintptr_t)tmds1);
			render_scanline(core0_scanbuf, y, &state);
			encode_scanline(core0_scanbuf, tmds0);
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmds0);
			tmds1 = (uint32_t*)multicore_fifo_pop_blocking();
			queue_add_blocking_u32(&dvi0.q_tmds_valid, &tmds1);
		}
		update(&state);
	}

	__builtin_unreachable();
}
	
