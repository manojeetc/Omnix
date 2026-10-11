// Control-loop timing counters. These are updated by the main loop, not by worker tasks.

#pragma once
#include <stdint.h>

struct LoopTiming {
  enum Stage { INPUT_SERVICE, CENTER, MPU, ASSIST, IO, FEET, SCREEN, HOUSEKEEPING, COUNT };

  uint32_t peakUs[COUNT] = {}, workPeakUs = 0, workOver20 = 0;

  // Unsigned subtraction also handles the micros() counter wrapping.
  void record(Stage stage, uint32_t start, uint32_t end) {
    uint32_t elapsed = end - start;
    if (elapsed > peakUs[stage])
      peakUs[stage] = elapsed;
  }

  void work(uint32_t start, uint32_t end) {
    uint32_t elapsed = end - start;
    if (elapsed > workPeakUs)
      workPeakUs = elapsed;
    if (elapsed > 20000)
      workOver20++;
  }

  void reset() {
    *this = LoopTiming();
  }
};
