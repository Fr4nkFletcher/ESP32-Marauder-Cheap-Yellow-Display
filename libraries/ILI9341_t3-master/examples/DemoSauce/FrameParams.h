#ifndef FRAME_PARAMS_H__
#define FRAME_PARAMS_H__

struct FrameParams {
  uint_fast16_t audioPeak;  // range: 0..512
  float audioMean;  // range: 0..1
  float timeMult; // animations are expected to run at 60fps I guess? 1.0==exactly 60fps. 4.0==achieving 15fps, so move 4x faster.
};

struct Point {
  uint_fast16_t x;
  uint_fast16_t y;
};

#endif
