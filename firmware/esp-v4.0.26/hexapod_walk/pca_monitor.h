// Estimate PCA9685 pulse timing from signal edges. Monitoring does not change servo commands.

#pragma once
#include <stdint.h>
#include <string.h>
#include <math.h>

static const int MARKER_ON = 4030, MARKER_TICKS = 40;
static const float MARKER_HI_MIN = 150.0f, MARKER_HI_MAX = 250.0f;
static const float PM_T_MIN = 18000.0f, PM_T_MAX = 22000.0f;

struct PcaMonResult {
  bool valid, tracking, stale;
  double periodUs;
  float medianUs, spreadUs;
  float hiUs;
  uint32_t frames, gaps, unexpected, badWidth, malformed, lostCount;
  uint32_t sinceFrameMs;
};

struct PcaMon {
  static const int N = 51;

  int lastLevel = -1;
  bool haveRise = false;
  uint32_t tRise = 0;

  float ring[N];
  int nRing = 0, head = 0;
  bool tracking = false, havePulse = false;
  uint32_t lastPulse = 0;
  double T = 0;
  uint32_t anchor = 0;
  double runTime = 0;
  uint32_t runCount = 0;
  double hiSum = 0;
  uint32_t hiN = 0;
  uint32_t frames = 0, gaps = 0, unexpected = 0, badWidth = 0, malformed = 0, lostCount = 0;
  bool everFrame = false;
  uint32_t lastFrameUs = 0;

  void reset() {
    *this = PcaMon();
  }

  static float median(const float *v, int n) {
    float s[N];
    memcpy(s, v, n * sizeof(float));
    for (int a = 1; a < n; a++) {
      float x = s[a];
      int b = a - 1;
      while (b >= 0 && s[b] > x) {
        s[b + 1] = s[b];
        b--;
      }
      s[b + 1] = x;
    }
    return n ? s[n / 2] : 0;
  }

  void pushRing(float iv) {
    ring[head] = iv;
    head = (head + 1) % N;
    if (nRing < N)
      nRing++;
  }

  void startAcquire() {
    tracking = false;
    nRing = 0;
    head = 0;
    havePulse = false;
    runTime = 0;
    runCount = 0;
  }

  void pulse(uint32_t t) {
    if (!tracking) {
      if (havePulse) {
        float iv = (float)(uint32_t)(t - lastPulse);
        if (iv >= PM_T_MIN && iv <= PM_T_MAX)
          pushRing(iv);
        else if (iv < PM_T_MIN) {
          unexpected++;
          return;
        }
      }
      lastPulse = t;
      havePulse = true;
      if (nRing == N) {
        float m = median(ring, N);
        int near = 0;
        for (int k = 0; k < N; k++)
          if (fabsf(ring[k] - m) <= 0.003f * m)
            near++;
        if (near >= (N * 9) / 10) {
          tracking = true;
          T = m;
          anchor = t;
          frames++;
          everFrame = true;
          lastFrameUs = t;
        }
      }
      return;
    }
    double dt = (double)(uint32_t)(t - anchor);
    int k = (int)floor(dt / T + 0.5);
    double dev = dt - k * T;
    if (k >= 1 && k <= 3 && fabs(dev) <= 150.0) {
      if (k == 1)
        pushRing((float)dt);
      else
        gaps += k - 1;
      runTime += dt;
      runCount += k;
      if (runCount >= 50)
        T = runTime / runCount;
      anchor = t;
      frames++;
      lastFrameUs = t;
    } else
      unexpected++;
  }

  void edge(uint32_t t, int level) {
    if (tracking && (double)(uint32_t)(t - anchor) > 3.5 * T) {
      lostCount++;
      startAcquire();
    }
    if (level == lastLevel) {
      malformed++;
      if (level == 1) {
        haveRise = true;
        tRise = t;
      } else
        haveRise = false;
      return;
    }
    lastLevel = level;
    if (level == 1) {
      haveRise = true;
      tRise = t;
      return;
    }
    if (!haveRise)
      return;
    haveRise = false;
    float hi = (float)(uint32_t)(t - tRise);
    if (hi < MARKER_HI_MIN || hi > MARKER_HI_MAX) {
      badWidth++;
      return;
    }
    hiSum += hi;
    hiN++;
    pulse(tRise);
  }

  void tick(uint32_t now) {
    if (tracking && (double)(uint32_t)(now - anchor) > 3.5 * T) {
      lostCount++;
      startAcquire();
    }
  }

  PcaMonResult result(uint32_t now) const {
    PcaMonResult r;
    memset(&r, 0, sizeof r);
    r.tracking = tracking;
    r.sinceFrameMs = everFrame ? (uint32_t)(now - lastFrameUs) / 1000 : 0xFFFFFFFF;
    r.periodUs = runCount ? runTime / runCount : T;
    r.medianUs = median(ring, nRing);
    float mn = 1e9f, mx = 0;
    for (int k = 0; k < nRing; k++) {
      mn = fminf(mn, ring[k]);
      mx = fmaxf(mx, ring[k]);
    }
    r.spreadUs = nRing ? mx - mn : 0;
    r.hiUs = hiN ? (float)(hiSum / hiN) : 0;
    r.frames = frames;
    r.gaps = gaps;
    r.unexpected = unexpected;
    r.badWidth = badWidth;
    r.malformed = malformed;
    r.lostCount = lostCount;
    r.valid = tracking && runCount >= 50 && r.sinceFrameMs < 100;
    r.stale = everFrame && !r.valid;
    return r;
  }
};
