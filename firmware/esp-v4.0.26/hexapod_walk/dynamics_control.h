// Experimental demand shaping from estimated joint torques. SI units throughout.
// The default monitor mode records estimates without changing the gait clock.

#pragma once
#include "equations.h"
#include <stdint.h>

namespace heq {

// At fixed pose, base at rest and fixed contact forces: tau(s) = tau0 + s*s*(tau1 - tau0).
// Changing the time scale adds acceleration terms that this local estimate does not include.
inline float demandScale(float stationary, float moving, float clock, float cap) {
  if (!isfinite(stationary) || !isfinite(moving) || !isfinite(clock) || !isfinite(cap) ||
      clock < .02f || clock > 1 || cap <= 0)
    return 1;

  // Slowing cannot fix static overload; do not rely on dynamic torque cancellation to carry it.
  if (fabsf(stationary) >= cap)
    return 1;
  float delta = moving - stationary;
  if (fabsf(delta) < 1e-8f)
    return 1;
  float room = cap - (delta > 0 ? stationary : -stationary);
  return fminf(1, clock * sqrtf(fmaxf(0, room / fabsf(delta))));
}

// Differentiate commanded angles in radians. Reset history after gaps or large command jumps.
struct MotionHistory {
  float previous[18] = {}, rate[18] = {}, accel[18] = {};
  bool ready = false;
  uint8_t warm = 0;

  void reset() {
    ready = false;
    warm = 0;
    for (int j = 0; j < 18; j++)
      rate[j] = accel[j] = 0;
  }

  bool push(const float q[18], float dt) {
    if (!finiteArray(q, 18) || !isfinite(dt) || dt < .005f || dt > .060f) {
      reset();
      return false;
    }
    bool good = ready;
    if (ready)
      for (int j = 0; j < 18; j++)
        if (fabsf(q[j] - previous[j]) > .174533f)
          good = false;
    if (!good) {
      reset();
      for (int j = 0; j < 18; j++)
        previous[j] = q[j];
      ready = true;
      return false;
    }
    float ar = 1 - expf(-dt / .04f), aa = 1 - expf(-dt / .12f);
    for (int j = 0; j < 18; j++) {
      float raw = (q[j] - previous[j]) / dt, prior = rate[j];
      rate[j] += ar * (raw - rate[j]);
      accel[j] += aa * ((rate[j] - prior) / dt - accel[j]);
      previous[j] = q[j];
    }
    if (warm < 8)
      warm++;
    return warm >= 8 && finiteArray(rate, 18) && finiteArray(accel, 18);
  }
};

// Monitor is passive; ON may slow the clock when all joints have valid estimates.
struct DynamicDemand {
  enum Mode { OFF, MONITOR, ON };

  Mode mode = MONITOR;
  float limit = .70f, ceiling = 1, requested = 1, minCeiling = 1;
  float latestStationary[18] = {}, latestMoving[18] = {};
  float peakStationary[18] = {}, peakMoving[18] = {}, peakIncrement[18] = {};
  float peakFraction = 0, peakStaticFraction = 0, peakIncrementFraction = 0;
  float lastMassG = 0, lastStallKgcm = 0, lastLimit = 0;
  uint32_t frames = 0, walkFrames = 0, skipped = 0, shaped = 0, staticHigh = 0, floorLimited = 0,
           worstUs = 0;
  int peakJoint = -1;
  MotionHistory history;
  // Keep this shaper above 60%; the existing speed governor can still impose a lower clock.
  static constexpr float FLOOR = .60f;

  void resetWindow() {
    frames = walkFrames = skipped = shaped = staticHigh = floorLimited = worstUs = 0;
    peakJoint = -1;
    peakFraction = peakStaticFraction = peakIncrementFraction = 0;
    minCeiling = 1;
    for (int j = 0; j < 18; j++)
      peakStationary[j] = peakMoving[j] = peakIncrement[j] = 0;
  }

  void reset() {
    history.reset();
    ceiling = requested = 1;
    resetWindow();
  }

  void ease(float want, float dt) {
    if (!isfinite(dt) || dt <= 0)
      dt = .02f;
    dt = fminf(dt, .060f);
    want = fmaxf(FLOOR, fminf(1, want));
    float maxChange = (want < ceiling ? 1.0f : .25f) * dt;
    ceiling += fmaxf(-maxChange, fminf(maxChange, want - ceiling));
  }

  void invalidate(float dt) {
    history.reset();
    skipped++;
    requested = 1;
    ease(1, dt);
  }

  // Validate all 18 torques before changing statistics or the clock ceiling.
  bool observe(const float stationary[18], const float moving[18], float stallNm, float clock,
               float dt, bool walking, bool eligible) {
    if (!finiteArray(stationary, 18) || !finiteArray(moving, 18) || !isfinite(stallNm) ||
        stallNm <= 0 || !isfinite(clock) || clock < .02f || clock > 1 || !isfinite(dt) ||
        dt < .005f || dt > .060f || !isfinite(limit) || limit < .20f || limit > .90f) {
      invalidate(dt);
      return false;
    }
    float want = 1;
    bool staticOver = false;
    for (int j = 0; j < 18; j++) {
      latestStationary[j] = stationary[j];
      latestMoving[j] = moving[j];
      if (fabsf(stationary[j]) >= limit * stallNm)
        staticOver = true;
      want = fminf(want, demandScale(stationary[j], moving[j], clock, limit * stallNm));
      if (walking) {
        float a = fabsf(stationary[j]), b = fabsf(moving[j]), d = fabsf(moving[j] - stationary[j]);
        peakStationary[j] = fmaxf(peakStationary[j], a);
        peakMoving[j] = fmaxf(peakMoving[j], b);
        peakIncrement[j] = fmaxf(peakIncrement[j], d);
        if (b / stallNm > peakFraction) {
          peakFraction = b / stallNm;
          peakJoint = j;
        }
        peakStaticFraction = fmaxf(peakStaticFraction, a / stallNm);
        peakIncrementFraction = fmaxf(peakIncrementFraction, d / stallNm);
      }
    }
    frames++;
    if (walking) {
      walkFrames++;
      if (staticOver)
        staticHigh++;
      if (want < FLOOR)
        floorLimited++;
    }
    requested = want;
    ease(mode == ON && eligible ? fmaxf(FLOOR, want) : 1, dt);
    if (mode == ON && eligible && ceiling < .999f) {
      shaped++;
      minCeiling = fminf(minCeiling, ceiling);
    }
    return true;
  }

  float clockCeiling(bool eligible) const {
    return mode == ON && eligible ? ceiling : 1;
  }
};
}
