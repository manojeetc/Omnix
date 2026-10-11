// Physical trick poses in mm and radians. Leg order is FL, ML, RL, FR, MR, RR.

#pragma once
#include <math.h>
#include <string.h>

enum Trick {
  TR_NONE,
  TR_WAVE,
  TR_BOW,
  TR_TWIST,
  TR_DANCE,
  TR_PUSHUP,
  TR_LOOK,
  TR_STOMP,
  TR_SHAKE,
  TR_NOD,
  TR_NO,
  TR_STRETCH,
  TR_BEG,
  TR_CIRCLE,
  TR_HEADBANG,
  TR_RIPPLE,
  TR_POINT,
  TR_TAP,
  TR_SLEEPY,
  TR_SHIVER,
  TR_POUNCE,
  TR_SWAY,
  TR_FLEX,
  TR_CURIOUS,
  TR_SCARED,
  TR_COUNT
};

static const char *const TRICK_NAME[TR_COUNT] = {
    "",       "wave",   "bow",     "twist", "dance",  "pushup",   "look",   "stomp", "shake",
    "nod",    "no",     "stretch", "beg",   "circle", "headbang", "ripple", "point", "tap",
    "sleepy", "shiver", "pounce",  "sway",  "flex",   "curious",  "scared"};
static const float TRICK_DUR[TR_COUNT] = {0,    2.0f, 3.2f, 3.6f, 7.2f, 4.8f, 4.6f, 3.4f, 1.8f,
                                          2.4f, 2.4f, 5.6f, 5.6f, 4.8f, 4.2f, 4.2f, 3.6f, 2.6f,
                                          5.4f, 2.0f, 4.4f, 4.8f, 3.8f, 3.8f, 3.6f};

static const int PARTY_SET[] = {TR_DANCE, TR_HEADBANG, TR_SWAY,   TR_CIRCLE,
                                TR_TWIST, TR_SHAKE,    TR_RIPPLE, TR_TAP};
static const int FIDGET_SET[] = {TR_LOOK, TR_SHAKE, TR_TWIST, TR_NOD, TR_CURIOUS, TR_SWAY};

struct TrickPose {
  float tx, ty, tz, roll, pitch, yaw;
  float foot[6][3];
};

static const float TP_PI = 3.14159265f;

static inline float tpClamp(float v, float a, float b) {
  return v < a ? a : v > b ? b : v;
}

static inline float tpEase(float s) {
  s = tpClamp(s, 0, 1);
  return tpClamp(s * s * s * (10 + s * (-15 + 6 * s)), 0, 1);
}

static inline float tpRamp(float t, float t0, float t1) {
  return tpEase((t - t0) / (t1 - t0));
}

static inline float tpEnv(float t, float D, float r) {
  return tpEase(t / r) * tpEase((D - t) / r);
}

static inline float tpBump(float t, float t0, float t1) {
  if (t <= t0 || t >= t1)
    return 0;
  float u = (t - t0) / (t1 - t0);
  return u < 0.5f ? tpEase(u * 2) : tpEase((1 - u) * 2);
}

static inline float tpStep(float s) {
  if (s <= 0 || s >= 1)
    return 0;
  return s < 0.5f ? tpEase(s / 0.5f) : tpEase((1 - s) / 0.5f);
}

static void trickPose(int id, float t, float a, TrickPose &p) {
  memset(&p, 0, sizeof p);
  if (!isfinite(t) || !isfinite(a))
    return;
  float amplitude = tpClamp(a, 0, 1);
  a = 1;
  if (id <= TR_NONE || id >= TR_COUNT || t <= 0 || t >= TRICK_DUR[id])
    return;
  const float D = TRICK_DUR[id];
  switch (id) {
  case TR_WAVE: {
    float shift = tpRamp(t, 0, 0.3f) - tpRamp(t, 1.7f, 2.0f);
    float up = tpRamp(t, 0.3f, 0.7f) - tpRamp(t, 1.25f, 1.7f);
    float wave = tpEnv(t - 0.65f, 0.65f, 0.15f);
    p.tx = -6 * shift;
    p.ty = -8 * shift;
    p.foot[0][0] = 24 * up;
    p.foot[0][2] = 55 * up;
    p.foot[0][1] = 18 * up * wave * sinf(2 * TP_PI * 1.5f * (t - 0.65f));
    break;
  }
  case TR_BOW: {
    float e = tpEnv(t, D, 0.8f);
    p.pitch = -15 * a * e;
    p.tz = -8 * a * e;
    p.tx = -6 * a * e;
    break;
  }
  case TR_TWIST: {
    float e = tpEnv(t, D, 0.5f);
    p.yaw = 15 * a * sinf(2 * TP_PI * t / 1.2f) * e;
    break;
  }
  case TR_DANCE: {
    float e = tpEnv(t, D, 0.6f), beat = 0.6f, b = 0.5f - 0.5f * cosf(2 * TP_PI * t / beat);
    p.tz = (-6 + 7 * b) * a * e;
    p.tx = 11 * a * e * cosf(2 * TP_PI * t / 2.4f);
    p.ty = 11 * a * e * sinf(2 * TP_PI * t / 2.4f);
    p.roll = 6 * a * e * sinf(2 * TP_PI * t / 1.2f);
    p.yaw = 8 * a * e * sinf(2 * TP_PI * t / 2.4f + 1);
    int k = (int)(t / beat);
    float s = fmodf(t, beat) / beat;
    float tap = (s < 0.5f ? tpStep(s / 0.5f) : 0) * 8 * a * e;
    const int A[3] = {0, 2, 4}, B[3] = {3, 5, 1};
    for (int j = 0; j < 3; j++)
      p.foot[(k % 2) ? B[j] : A[j]][2] = tap;
    break;
  }
  case TR_PUSHUP: {
    float e = tpEnv(t, D, 0.4f);
    p.tz = -18 * a * e * (0.5f - 0.5f * cosf(2 * TP_PI * t / 1.6f));
    break;
  }
  case TR_LOOK: {
    float y;
    if (t < 0.6f)
      y = 18 * tpEase(t / 0.6f);
    else if (t < 1.8f)
      y = 18;
    else if (t < 2.8f)
      y = 18 - 36 * tpEase((t - 1.8f) / 1.0f);
    else if (t < 3.8f)
      y = -18;
    else
      y = -18 + 18 * tpEase((t - 3.8f) / 0.8f);
    p.yaw = y * a;
    p.pitch = 4 * a * (y * y) / (18 * 18);
    break;
  }
  case TR_STOMP: {
    float e = tpEnv(t, D, 0.4f);
    p.pitch = 5 * a * e;
    p.tx = -10 * a * e;
    for (int k = 0; k < 4; k++) {
      float s = (t - 0.4f - 0.7f * k) / 0.7f;
      if (s < 0 || s >= 1)
        continue;
      float l = s < 0.52f ? tpEase(s / 0.52f) : tpEase((1 - s) / 0.48f);
      p.foot[(k % 2) ? 3 : 0][2] = 24 * a * l;
    }
    break;
  }
  case TR_SHAKE: {
    float d = tpEnv(t, D, 0.3f) * (1 - tpRamp(t, 0.4f, D));
    p.roll = 4.5f * a * sinf(2 * TP_PI * 2.2f * t) * d;
    p.yaw = 2 * a * sinf(2 * TP_PI * 2.2f * t + 1.2f) * d;
    break;
  }
  case TR_NOD: {
    float e = tpEnv(t, D, 0.3f);
    p.pitch = -10 * a * e * (0.5f - 0.5f * cosf(2 * TP_PI * t / 0.8f));
    p.tz = -3 * a * e;
    break;
  }
  case TR_NO: {
    float e = tpEnv(t, D, 0.3f);
    p.yaw = 13 * a * e * sinf(2 * TP_PI * t / 0.6f);
    p.pitch = 2 * a * e;
    break;
  }
  case TR_STRETCH: {
    float f = tpRamp(t, 0.2f, 1.2f) - tpRamp(t, 2.2f, 2.9f);
    float r = tpRamp(t, 2.6f, 3.5f) - tpRamp(t, 4.6f, 5.4f);
    float wig = 2.5f * sinf(2 * TP_PI * t * 1.6f) * (f + r);
    p.pitch = a * (-13 * f + 11 * r);
    p.tx = a * (9 * f - 9 * r);
    p.tz = a * (-4 * f - 3 * r);
    p.roll = a * wig * 0.6f;
    break;
  }
  case TR_BEG: {
    float sh = tpRamp(t, 0.0f, 0.7f) - tpRamp(t, 4.9f, 5.6f);
    float up = tpRamp(t, 0.7f, 1.3f) - tpRamp(t, 4.2f, 4.8f);
    float tap = tpEnv(t - 1.4f, 2.6f, 0.25f) * sinf(2 * TP_PI * (t - 1.4f) / 0.85f);
    p.tx = -24 * sh;
    p.tz = 3 * sh;
    p.pitch = 7 * sh;
    for (int k = 0; k < 2; k++) {
      int L = k ? 3 : 0;
      float s = k ? 1 : -1;
      p.foot[L][2] = 58 * up;
      p.foot[L][0] = 10 * up;
      p.foot[L][1] = s * (18 + 8 * tap) * up;
    }
    break;
  }
  case TR_CIRCLE: {
    float e = tpEnv(t, D, 0.6f), w = 2 * TP_PI * t / 2.4f;
    p.tx = 17 * a * e * cosf(w);
    p.ty = 17 * a * e * sinf(w);
    p.pitch = -5 * a * e * cosf(w);
    p.roll = -5 * a * e * sinf(w);
    break;
  }
  case TR_HEADBANG: {
    float e = tpEnv(t, D, 0.4f), s = sinf(TP_PI * t / 0.6f);
    p.pitch = -9 * a * e * s * s;
    p.tz = -5 * a * e * s * s;
    p.tx = 3 * a * e * s * s;
    break;
  }
  case TR_RIPPLE: {
    const int ord[6] = {0, 1, 2, 5, 4, 3};
    float e = tpEnv(t, D, 0.4f);
    for (int k = 0; k < 12; k++) {
      float s = (t - 0.3f - 0.3f * k) / 0.28f;
      if (s > 0 && s < 1)
        p.foot[ord[k % 6]][2] = 14 * a * tpStep(s);
    }
    p.roll = 0;
    p.tz = -2 * e;
    break;
  }
  case TR_POINT: {
    float e = tpRamp(t, 0, 0.45f) - tpRamp(t, 3.0f, 3.6f);
    float up = tpRamp(t, 0.35f, 0.85f) - tpRamp(t, 2.4f, 3.0f);
    float wob = 1.5f * sinf(2 * TP_PI * t * 0.8f) * up;
    p.tx = -10 * e;
    p.ty = -7 * e;
    p.pitch = 3 * e;
    p.foot[0][0] = 36 * up;
    p.foot[0][1] = 6 * up + wob;
    p.foot[0][2] = 45 * up;
    break;
  }
  case TR_TAP: {
    float e = tpEnv(t, D, 0.3f);
    p.tx = -4 * e;
    for (int k = 0; k < 7; k++) {
      float s = (t - 0.25f - 0.31f * k) / 0.22f;
      if (s > 0 && s < 1)
        p.foot[(k % 2) ? 3 : 0][2] = 10 * a * tpStep(s);
    }
    break;
  }
  case TR_SLEEPY: {
    float sink = tpRamp(t, 0.2f, 3.6f), wake = tpRamp(t, 4.05f, 4.35f);
    float down = sink * (1 - wake);
    float sway = 3.5f * sinf(2 * TP_PI * t / 2.2f) * down;
    float jolt = tpBump(t, 4.05f, 4.9f);
    p.tz = -14 * a * down + 3 * jolt;
    p.pitch = -6 * a * down + 3 * jolt;
    p.roll = a * sway;
    break;
  }
  case TR_SHIVER: {
    float e = tpEnv(t, D, 0.3f);
    p.tz = 0.8f * a * e * sinf(2 * TP_PI * 4.2f * t);
    p.roll = 1.1f * a * e * sinf(2 * TP_PI * 3.7f * t + 0.7f);
    p.yaw = 0.65f * a * e * sinf(2 * TP_PI * 3.1f * t + 2.1f);
    p.tz -= 3 * e;
    break;
  }
  case TR_POUNCE: {
    float c = tpRamp(t, 0.0f, 0.8f) * (1 - tpRamp(t, 2.6f, 3.15f));
    float l = tpRamp(t, 2.6f, 3.15f) * (1 - tpRamp(t, 3.5f, 4.4f));
    float wig =
        (t > 0.9f && t < 2.5f) ? sinf(2 * TP_PI * 2.5f * t) * tpEnv(t - 0.9f, 1.6f, 0.3f) : 0;
    p.tx = a * (-12 * c + 14 * l);
    p.tz = a * (-12 * c + 3 * l);
    p.pitch = a * (4 * c - 7 * l);
    p.yaw = 2.5f * a * wig;
    break;
  }
  case TR_SWAY: {
    float e = tpEnv(t, D, 0.6f), w = 2 * TP_PI * t / 1.6f;
    p.ty = 15 * a * e * sinf(w);
    p.roll = -6 * a * e * sinf(w);
    p.tz = -3 * a * e * (0.5f + 0.5f * cosf(2 * w));
    break;
  }
  case TR_FLEX: {
    float up = tpRamp(t, 0.1f, 0.8f) - tpRamp(t, 3.0f, 3.8f);
    float b = tpBump(t, 1.2f, 1.7f) + tpBump(t, 1.9f, 2.4f);
    p.tz = a * (10 * up - 5 * b);
    p.pitch = a * 5 * up;
    p.roll = 0;
    break;
  }
  case TR_CURIOUS: {
    float l = tpRamp(t, 0.2f, 0.6f) - tpRamp(t, 1.5f, 1.9f);
    float r = tpRamp(t, 1.9f, 2.3f) - tpRamp(t, 3.2f, 3.6f);
    p.roll = a * (11 * l - 11 * r);
    p.pitch = a * 4 * (l + r);
    p.ty = a * (3 * l - 3 * r);
    break;
  }
  case TR_SCARED: {
    float c = tpRamp(t, 0.0f, 0.45f) * (1 - tpRamp(t, 2.3f, 3.6f));
    float tr = tpEnv(t - 0.35f, 2.05f, 0.3f);
    p.tx = -13 * a * c;
    p.tz = -17 * a * c;
    p.pitch = 4 * a * c;
    p.roll = tr * sinf(2 * TP_PI * 3.5f * t);
    p.tz += tr * 0.7f * sinf(2 * TP_PI * 3 * t + 1);
    break;
  }
  default:
    break;
  }
  p.tx *= amplitude;
  p.ty *= amplitude;
  p.tz *= amplitude;
  p.roll *= amplitude;
  p.pitch *= amplitude;
  p.yaw *= amplitude;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      p.foot[i][j] *= amplitude;
}

struct TrickPlayer {
  int id = TR_NONE, pending = TR_NONE;
  float amp = 1, pendingAmp = 1, time = 0;
  bool returning = false;
  float returnTime = 0, footDuration = 0, bodyDuration = 0;
  TrickPose pose = {}, from = {};

  void clear() {
    *this = TrickPlayer();
  }

  void start(int next, float strength) {
    clear();
    if (next <= TR_NONE || next >= TR_COUNT || !isfinite(strength))
      return;
    id = next;
    amp = tpClamp(strength, 0, 1);
  }

  void beginReturn() {
    if (id == TR_NONE || returning)
      return;
    returning = true;
    from = pose;
    returnTime = 0;
    float distance = 0;
    for (int i = 0; i < 6; i++) {
      float d = sqrtf(from.foot[i][0] * from.foot[i][0] + from.foot[i][1] * from.foot[i][1] +
                      from.foot[i][2] * from.foot[i][2]);
      distance = fmaxf(distance, d);
    }

    footDuration = distance > 0.001f ? fmaxf(0.45f, 1.875f * distance / 70) : 0;
    float translation = sqrtf(from.tx * from.tx + from.ty * from.ty + from.tz * from.tz);
    float rotation = fmaxf(fabsf(from.roll), fmaxf(fabsf(from.pitch), fabsf(from.yaw)));
    bodyDuration = fmaxf(0.5f, fmaxf(1.875f * translation / 35, 1.875f * rotation / 25));
  }

  void cancel() {
    pending = TR_NONE;
    beginReturn();
  }

  void request(int next, float strength) {
    if (next <= TR_NONE || next >= TR_COUNT || !isfinite(strength))
      return;
    if (id == TR_NONE)
      start(next, strength);
    else {
      pending = next;
      pendingAmp = strength;
      beginReturn();
    }
  }

  bool tick(float dt) {
    if (id == TR_NONE || !isfinite(dt) || dt <= 0)
      return false;
    dt = tpClamp(dt, 0, 0.1f);
    if (!returning) {
      time = fminf(time + dt, TRICK_DUR[id]);
      trickPose(id, time, amp, pose);
      if (time < TRICK_DUR[id])
        return false;
      int next = pending;
      float strength = pendingAmp;
      clear();
      if (next != TR_NONE)
        start(next, strength);
      return true;
    }
    returnTime += dt;
    pose = from;
    float feetK = footDuration > 0 ? 1 - tpEase(returnTime / footDuration) : 0;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        pose.foot[i][j] *= feetK;
    float k = 1 - tpEase((returnTime - footDuration) / bodyDuration);
    pose.tx *= k;
    pose.ty *= k;
    pose.tz *= k;
    pose.roll *= k;
    pose.pitch *= k;
    pose.yaw *= k;
    if (returnTime < footDuration + bodyDuration)
      return false;
    int next = pending;
    float strength = pendingAmp;
    clear();
    if (next != TR_NONE)
      start(next, strength);
    return true;
  }
};
