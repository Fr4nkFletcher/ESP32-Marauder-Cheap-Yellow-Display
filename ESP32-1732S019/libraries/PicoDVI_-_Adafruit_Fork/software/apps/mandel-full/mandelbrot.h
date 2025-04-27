// Init pico resources used for generation
void mandel_init();

// Fixed point with 6 bits to the left of the point.
// Range [-32,32) with precision 2^-26
typedef int32_t fixed_pt_t;

typedef struct {
  // Configuration
  uint8_t* buff;
  int16_t rows;
  int16_t cols;

  uint16_t max_iter;
  uint16_t iter_offset;
  float minx, miny, maxx, maxy;
  bool use_cycle_check;

  // State
  volatile bool done;
  volatile uint16_t min_iter;
  fixed_pt_t iminx, iminy, imaxx, imaxy;
  fixed_pt_t incx, incy;
  volatile uint32_t count_inside;

  int16_t ipos, jpos;
  // Tracks work stealing on core 0
  volatile int16_t iend, jend;
} FractalBuffer;

// Make a fixed_pt_t from an int or float.
fixed_pt_t make_fixed(int32_t x);
fixed_pt_t make_fixedf(float x);

// Generate a section of the fractal into buff
// Result written to buff is 0 for inside Mandelbrot set
// Otherwise iteration of escape minus min_iter (clamped to 1)
void init_fractal(FractalBuffer* fractal);
void generate_fractal(FractalBuffer* fractal);
void generate_one_forward(FractalBuffer* f);
void generate_steal_one(FractalBuffer* f);
