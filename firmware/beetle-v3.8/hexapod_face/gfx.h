// RGB565 drawing primitives for a 320 x 240 frame. Each core clips writes to its own row band.
// The caller defines GFX_CORE() and assigns a target before drawing.

#pragma once
#include <stdint.h>
#include <math.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include "fonts.h"

namespace gfx {

static const int W = 320, H = 240;

struct Target {
  uint16_t *buf;
  int y0, y1;
  float ox;
  uint32_t ga = 32;
};

static Target T[2];

static inline Target &tg() {
  return T[GFX_CORE()];
}

static constexpr uint16_t RGB(uint8_t r, uint8_t g, uint8_t b) {
  return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3);
}

static inline uint8_t R8(uint16_t c) {
  return ((c >> 11) & 31) * 255 / 31;
}

static inline uint8_t G8(uint16_t c) {
  return ((c >> 5) & 63) * 255 / 63;
}

static inline uint8_t B8(uint16_t c) {
  return (c & 31) * 255 / 31;
}

static inline uint16_t blend(uint16_t bg, uint16_t fg, uint32_t a) {
  uint32_t b = ((uint32_t)bg | ((uint32_t)bg << 16)) & 0x07E0F81Fu;
  uint32_t f = ((uint32_t)fg | ((uint32_t)fg << 16)) & 0x07E0F81Fu;
  uint32_t r = ((f * a + b * (32 - a)) >> 5) & 0x07E0F81Fu;
  return (uint16_t)(r | (r >> 16));
}

static inline uint16_t mix(uint16_t a, uint16_t b, float t) {
  t = t < 0 ? 0 : t > 1 ? 1 : t;
  return blend(a, b, (uint32_t)(t * 32 + 0.5f));
}

static inline uint16_t scale(uint16_t c, float k) {
  return mix(0, c, k);
}

static uint16_t hsv(float h, float s, float v) {
  h = fmodf(h, 360);
  if (h < 0)
    h += 360;
  float c = v * s, x = c * (1 - fabsf(fmodf(h / 60, 2) - 1)), m = v - c, r, g, b;
  if (h < 60) {
    r = c;
    g = x;
    b = 0;
  } else if (h < 120) {
    r = x;
    g = c;
    b = 0;
  } else if (h < 180) {
    r = 0;
    g = c;
    b = x;
  } else if (h < 240) {
    r = 0;
    g = x;
    b = c;
  } else if (h < 300) {
    r = x;
    g = 0;
    b = c;
  } else {
    r = c;
    g = 0;
    b = x;
  }
  return RGB((uint8_t)((r + m) * 255), (uint8_t)((g + m) * 255), (uint8_t)((b + m) * 255));
}

static inline float clampf(float v, float a, float b) {
  return v < a ? a : v > b ? b : v;
}

static inline float mn(float a, float b) {
  return a < b ? a : b;
}

static inline float mx(float a, float b) {
  return a > b ? a : b;
}

static inline int clampi(int v, int a, int b) {
  return v < a ? a : v > b ? b : v;
}

// The caller clips x/y first. Alpha and the target's global alpha use 0..32, not 0..255.
static inline void put(int x, int y, uint16_t c, uint32_t a) {
  Target &t = tg();
  if (t.ga < 32) {
    a = (a * t.ga) >> 5;
    if (!a)
      return;
  }
  uint16_t *p = t.buf + y * W + x;
  *p = a >= 32 ? c : blend(*p, c, a);
}

static inline void span(int y, int xa, int xb, uint16_t c, uint32_t a) {
  if (xa < 0)
    xa = 0;
  if (xb > W - 1)
    xb = W - 1;
  if (xa > xb)
    return;
  Target &t = tg();
  if (t.ga < 32) {
    a = (a * t.ga) >> 5;
    if (!a)
      return;
  }
  uint16_t *p = t.buf + y * W + xa, *e = p + (xb - xa) + 1;
  if (a >= 32) {
    if (((uintptr_t)p & 2) && p < e)
      *p++ = c;
    uint32_t c2 = c | ((uint32_t)c << 16);
    uint32_t *q = (uint32_t *)p, *qe = (uint32_t *)(p + ((e - p) & ~1));
    while (q + 4 <= qe) {
      q[0] = c2;
      q[1] = c2;
      q[2] = c2;
      q[3] = c2;
      q += 4;
    }
    while (q < qe)
      *q++ = c2;
    p = (uint16_t *)q;
    if (p < e)
      *p = c;
  } else {
    uint32_t f = ((uint32_t)c | ((uint32_t)c << 16)) & 0x07E0F81Fu, fa = f * a, ia = 32 - a;
    while (p < e) {
      uint32_t b = ((uint32_t)*p | ((uint32_t)*p << 16)) & 0x07E0F81Fu;
      uint32_t r = ((fa + b * ia) >> 5) & 0x07E0F81Fu;
      *p++ = (uint16_t)(r | (r >> 16));
    }
  }
}

static void clear(uint16_t c) {
  Target &t = tg();
  for (int y = t.y0; y < t.y1; y++)
    span(y, 0, W - 1, c, 32);
}

static void rect(float x, float y, float w, float h, uint16_t c, float alpha = 1) {
  Target &t = tg();
  x += t.ox;
  int x0 = (int)floorf(x + 0.5f), x1 = (int)floorf(x + w + 0.5f) - 1;
  int y0 = (int)floorf(y + 0.5f), y1 = (int)floorf(y + h + 0.5f) - 1;
  if (y0 < t.y0)
    y0 = t.y0;
  if (y1 > t.y1 - 1)
    y1 = t.y1 - 1;
  uint32_t a = (uint32_t)(clampf(alpha, 0, 1) * 32 + 0.5f);
  if (!a)
    return;
  for (int yy = y0; yy <= y1; yy++)
    span(yy, x0, x1, c, a);
}

static void rectV(float x, float y, float w, float h, uint16_t top, uint16_t bot) {
  Target &t = tg();
  x += t.ox;
  int x0 = (int)floorf(x + 0.5f), x1 = (int)floorf(x + w + 0.5f) - 1;
  int y0 = (int)floorf(y + 0.5f), y1 = (int)floorf(y + h + 0.5f) - 1;
  for (int yy = (y0 < t.y0 ? t.y0 : y0); yy <= y1 && yy < t.y1; yy++)
    span(yy, x0, x1, mix(top, bot, (yy - y0) / (h > 1 ? h - 1 : 1)), 32);
}

static inline uint32_t cov(float d, float alpha) {
  float c = 0.5f - d;
  c = c < 0 ? 0 : c > 1 ? 1 : c;
  return (uint32_t)(c * alpha * 32 + 0.5f);
}

static void rrect(float x, float y, float w, float h, float r, uint16_t c, float alpha = 1) {
  if (w <= 0 || h <= 0)
    return;
  Target &t = tg();
  x += t.ox;
  float hw = w / 2, hh = h / 2, cx = x + hw, cy = y + hh;
  r = clampf(r, 0, mn(hw, hh));
  alpha = clampf(alpha, 0, 1);
  uint32_t A = (uint32_t)(alpha * 32 + 0.5f);
  if (!A)
    return;
  int ya = (int)floorf(y - 0.5f), yb = (int)ceilf(y + h + 0.5f);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  auto halfw = [&](float dy, float g) -> float {
    float HH = hh + g, HW = hw + g, RR = r + g;
    if (RR < 0)
      RR = 0;
    if (dy > HH)
      return -1;
    if (dy <= HH - RR)
      return HW;
    float q = dy - (HH - RR);
    return HW - RR + sqrtf(mx(RR * RR - q * q, 0));
  };
  auto sdf = [&](float px, float py) -> float {
    float qx = fabsf(px - cx) - hw + r, qy = fabsf(py - cy) - hh + r;
    float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0;
    return sqrtf(ox * ox + oy * oy) + mn(mx(qx, qy), 0.0f) - r;
  };
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, dy = fabsf(py - cy);
    float outer = halfw(dy, 0.5f);
    if (outer < 0)
      continue;
    float inner = halfw(dy, -0.5f);
    int xa = (int)floorf(cx - outer - 0.5f), xb = (int)ceilf(cx + outer - 0.5f);
    int sa, sb;
    if (inner > 0) {
      sa = (int)ceilf(cx - inner - 0.5f);
      sb = (int)floorf(cx + inner - 0.5f);
    } else {
      sa = xb + 1;
      sb = xb;
    }
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    for (int xx = xa; xx < sa && xx <= xb; xx++) {
      uint32_t a = cov(sdf(xx + 0.5f, py), alpha);
      if (a)
        put(xx, yy, c, a);
    }
    if (sa <= sb)
      span(yy, sa, sb, c, A);
    for (int xx = (sb + 1 > xa ? sb + 1 : xa); xx <= xb; xx++) {
      if (xx < sa)
        continue;
      uint32_t a = cov(sdf(xx + 0.5f, py), alpha);
      if (a)
        put(xx, yy, c, a);
    }
  }
}

static inline void disc(float cx, float cy, float r, uint16_t c, float alpha = 1) {
  rrect(cx - r, cy - r, 2 * r, 2 * r, r, c, alpha);
}

static void rrectLine(float x, float y, float w, float h, float r, float th, uint16_t c,
                      float alpha = 1) {
  Target &t = tg();
  x += t.ox;
  float hw = w / 2, hh = h / 2, cx = x + hw, cy = y + hh, ht = th / 2;
  r = clampf(r, 0, mn(hw, hh));
  int ya = (int)floorf(y - ht - 1), yb = (int)ceilf(y + h + ht + 1);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  auto sdfPx = [&](int xx, int yy) {
    float qx = fabsf(xx + 0.5f - cx) - hw + r, qy = fabsf(yy + 0.5f - cy) - hh + r;
    float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0;
    float d = fabsf(sqrtf(ox * ox + oy * oy) + mn(mx(qx, qy), 0.0f) - r) - ht;
    uint32_t a = cov(d, alpha);
    if (a)
      put(xx, yy, c, a);
  };
  int L0 = (int)floorf(x - ht - 1), L1 = (int)ceilf(x + r + 1), R0 = (int)floorf(x + w - r - 1),
      R1 = (int)ceilf(x + w + ht + 1);
  int S0 = (int)floorf(x - ht - 1), S1 = (int)ceilf(x + ht + 1), S2 = (int)floorf(x + w - ht - 1),
      S3 = (int)ceilf(x + w + ht + 1);
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, dy = fabsf(py - cy);
    if (dy < hh - r) {
      for (int xx = (S0 < 0 ? 0 : S0); xx <= S1 && xx < W; xx++) {
        float d = fabsf(fabsf(xx + 0.5f - cx) - hw) - ht;
        uint32_t a = cov(d, alpha);
        if (a)
          put(xx, yy, c, a);
      }
      for (int xx = (S2 < 0 ? 0 : S2); xx <= S3 && xx < W; xx++) {
        float d = fabsf(fabsf(xx + 0.5f - cx) - hw) - ht;
        uint32_t a = cov(d, alpha);
        if (a)
          put(xx, yy, c, a);
      }
    } else {
      for (int xx = (L0 < 0 ? 0 : L0); xx <= L1 && xx < W; xx++)
        sdfPx(xx, yy);
      uint32_t a = cov(fabsf(dy - hh) - ht, alpha);
      if (a && L1 + 1 <= R0 - 1)
        span(yy, L1 + 1, R0 - 1, c, a);
      for (int xx = (R0 < 0 ? 0 : R0); xx <= R1 && xx < W; xx++) {
        if (xx <= L1)
          continue;
        sdfPx(xx, yy);
      }
    }
  }
}

static void ring(float cx, float cy, float r, float th, uint16_t c, float alpha = 1) {
  Target &t = tg();
  cx += t.ox;
  float ht = th / 2, ro = r + ht + 0.6f, ri = r - ht - 0.6f, so = r + ht - 0.6f, si = r - ht + 0.6f;
  int ya = (int)floorf(cy - ro), yb = (int)ceilf(cy + ro);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  uint32_t A = (uint32_t)(clampf(alpha, 0, 1) * 32 + 0.5f);
  for (int yy = ya; yy <= yb; yy++) {
    float dy = yy + 0.5f - cy, dy2 = dy * dy;
    if (dy2 >= ro * ro)
      continue;
    float wo = sqrtf(ro * ro - dy2);
    float wi = (ri > 0 && dy2 < ri * ri) ? sqrtf(ri * ri - dy2) : 0;
    float wso = (so > 0 && dy2 < so * so) ? sqrtf(so * so - dy2) : -1;
    float wsi = (si > 0 && dy2 < si * si) ? sqrtf(si * si - dy2) : 0;
    int xa = (int)floorf(cx - wo), xb = (int)ceilf(cx + wo);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    int ha = wi > 0 ? (int)ceilf(cx - wi) : 1 << 20,
        hb = wi > 0 ? (int)floorf(cx + wi) - 1 : -(1 << 20);
    int s1a = 1, s1b = 0, s2a = 1, s2b = 0;
    if (wso > 0) {
      if (wsi > 0) {
        s1a = (int)ceilf(cx - wso);
        s1b = (int)floorf(cx - wsi) - 1;
        s2a = (int)ceilf(cx + wsi);
        s2b = (int)floorf(cx + wso) - 1;
      } else {
        s1a = (int)ceilf(cx - wso);
        s1b = (int)floorf(cx + wso) - 1;
      }
    }
    for (int xx = xa; xx <= xb; xx++) {
      if (xx == ha && ha <= hb) {
        xx = hb;
        continue;
      }
      if (xx == s1a && s1a <= s1b) {
        span(yy, s1a, s1b, c, A);
        xx = s1b;
        continue;
      }
      if (xx == s2a && s2a <= s2b) {
        span(yy, s2a, s2b, c, A);
        xx = s2b;
        continue;
      }
      float dx = xx + 0.5f - cx;
      uint32_t a = cov(fabsf(sqrtf(dx * dx + dy2) - r) - ht, alpha);
      if (a)
        put(xx, yy, c, a);
    }
  }
}

static void line(float x0, float y0, float x1, float y1, float th, uint16_t c, float alpha = 1) {
  Target &t = tg();
  x0 += t.ox;
  x1 += t.ox;
  float R = th / 2, dx = x1 - x0, dy = y1 - y0, L2 = dx * dx + dy * dy, L = sqrtf(L2);
  float ux = L > 1e-4f ? dx / L : 1, uy = L > 1e-4f ? dy / L : 0, nx = -uy, ny = ux;
  int ya = (int)floorf(mn(y0, y1) - R - 1), yb = (int)ceilf(mx(y0, y1) + R + 1);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  uint32_t A = (uint32_t)(clampf(alpha, 0, 1) * 32 + 0.5f);

  auto interval = [&](float py, float r, float &lo, float &hi) -> bool {
    if (r <= 0)
      return false;
    bool any = false;
    lo = 1e9f;
    hi = -1e9f;
    auto add = [&](float a, float b) {
      if (a <= b) {
        lo = mn(lo, a);
        hi = mx(hi, b);
        any = true;
      }
    };
    float d0 = py - y0, d1 = py - y1;
    if (fabsf(d0) < r) {
      float h = sqrtf(r * r - d0 * d0);
      add(x0 - h, x0 + h);
    }
    if (fabsf(d1) < r) {
      float h = sqrtf(r * r - d1 * d1);
      add(x1 - h, x1 + h);
    }
    if (L > 1e-4f) {
      float bl = -1e9f, bh = 1e9f;
      bool ok = true;
      auto lim = [&](float Ac, float B, float fmin, float fmax) {
        if (fabsf(Ac) < 1e-6f) {
          if (B < fmin || B > fmax)
            ok = false;
          return;
        }
        float xa = (fmin - B) / Ac, xb = (fmax - B) / Ac;
        if (xa > xb) {
          float q = xa;
          xa = xb;
          xb = q;
        }
        bl = mx(bl, xa);
        bh = mn(bh, xb);
      };
      lim(nx, ny * d0 - nx * x0, -r, r);
      lim(ux, uy * d0 - ux * x0, 0, L);
      if (ok)
        add(bl, bh);
    }
    return any;
  };
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, lo, hi;
    if (!interval(py, R + 0.6f, lo, hi))
      continue;
    int xa = (int)floorf(lo - 0.5f), xb = (int)ceilf(hi - 0.5f);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    int sa = 1 << 20, sb = -(1 << 20);
    float ilo, ihi;
    if (interval(py, R - 0.6f, ilo, ihi)) {
      sa = (int)ceilf(ilo - 0.5f);
      sb = (int)floorf(ihi - 0.5f);
    }
    for (int xx = xa; xx <= xb; xx++) {
      if (xx == sa && sa <= sb) {
        span(yy, sa, sb, c, A);
        xx = sb;
        continue;
      }
      float px = xx + 0.5f, wx = px - x0, wy = py - y0;
      float h = L2 > 0 ? clampf((wx * dx + wy * dy) / L2, 0, 1) : 0;
      float ex = wx - dx * h, ey = wy - dy * h;
      uint32_t a = cov(sqrtf(ex * ex + ey * ey) - R, alpha);
      if (a)
        put(xx, yy, c, a);
    }
  }
}

static void lineThin(float x0, float y0, float x1, float y1, uint16_t c, float alpha = 1) {
  Target &t = tg();
  x0 += t.ox;
  x1 += t.ox;
  bool steep = fabsf(y1 - y0) > fabsf(x1 - x0);
  if (steep) {
    float q = x0;
    x0 = y0;
    y0 = q;
    q = x1;
    x1 = y1;
    y1 = q;
  }
  if (x0 > x1) {
    float q = x0;
    x0 = x1;
    x1 = q;
    q = y0;
    y0 = y1;
    y1 = q;
  }
  float dx = x1 - x0, g = dx > 0.001f ? (y1 - y0) / dx : 0;
  int xa = (int)floorf(x0 + 0.5f), xb = (int)floorf(x1 + 0.5f);
  float y = y0 + g * (xa - x0);
  float A = clampf(alpha, 0, 1) * 32;
  for (int x = xa; x <= xb; x++, y += g) {
    int yi = (int)floorf(y - 0.5f);
    float fr = y - 0.5f - yi;
    uint32_t a1 = (uint32_t)((1 - fr) * A + 0.5f), a2 = (uint32_t)(fr * A + 0.5f);
    if (steep) {
      if (x >= t.y0 && x < t.y1) {
        if (yi >= 0 && yi < W && a1)
          put(yi, x, c, a1);
        if (yi + 1 >= 0 && yi + 1 < W && a2)
          put(yi + 1, x, c, a2);
      }
    } else if (x >= 0 && x < W) {
      if (yi >= t.y0 && yi < t.y1 && a1)
        put(x, yi, c, a1);
      if (yi + 1 >= t.y0 && yi + 1 < t.y1 && a2)
        put(x, yi + 1, c, a2);
    }
  }
}

static void arc(float cx, float cy, float r, float th, float a0, float a1, uint16_t c,
                float alpha = 1) {
  if (a1 - a0 >= 359.9f) {
    ring(cx, cy, r, th, c, alpha);
    return;
  }
  if (a1 <= a0)
    return;
  Target &t = tg();
  cx += t.ox;
  float ht = th / 2, ro = r + ht + 1, ri = r - ht - 1;
  float m = (a0 + a1) / 2 * 0.0174533f, hspan = (a1 - a0) / 2 * 0.0174533f;
  float mx = sinf(m), my = -cosf(m), ch = cosf(hspan);
  float e0x = cx + r * sinf(a0 * 0.0174533f), e0y = cy - r * cosf(a0 * 0.0174533f);
  float e1x = cx + r * sinf(a1 * 0.0174533f), e1y = cy - r * cosf(a1 * 0.0174533f);
  int ya = (int)floorf(cy - ro), yb = (int)ceilf(cy + ro);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, dy = py - cy;
    if (fabsf(dy) > ro)
      continue;
    float wo = sqrtf(ro * ro - dy * dy), wi = fabsf(dy) < ri ? sqrtf(ri * ri - dy * dy) : -1;
    for (int side = 0; side < 2; side++) {
      int xa, xb;
      if (wi < 0) {
        if (side)
          break;
        xa = (int)floorf(cx - wo);
        xb = (int)ceilf(cx + wo);
      } else if (!side) {
        xa = (int)floorf(cx - wo);
        xb = (int)ceilf(cx - wi);
      } else {
        xa = (int)floorf(cx + wi);
        xb = (int)ceilf(cx + wo);
      }
      if (xa < 0)
        xa = 0;
      if (xb > W - 1)
        xb = W - 1;
      for (int xx = xa; xx <= xb; xx++) {
        float px = xx + 0.5f - cx, py2 = dy, len = sqrtf(px * px + py2 * py2);
        float d;
        if (len > 1e-3f && (px * mx + py2 * my) / len >= ch)
          d = fabsf(len - r) - ht;
        else {
          float d0 = sqrtf((xx + 0.5f - e0x) * (xx + 0.5f - e0x) + (py - e0y) * (py - e0y));
          float d1 = sqrtf((xx + 0.5f - e1x) * (xx + 0.5f - e1x) + (py - e1y) * (py - e1y));
          d = mn(d0, d1) - ht;
        }
        uint32_t a = cov(d, alpha);
        if (a)
          put(xx, yy, c, a);
      }
    }
  }
}

static uint8_t GLOW_LUT[257];

static void glowInit() {
  for (int i = 0; i <= 256; i++) {
    float d = sqrtf(i / 256.0f), v = 1 - d;
    GLOW_LUT[i] = (uint8_t)(v * v * 32 + 0.5f);
  }
}

static void glow(float cx, float cy, float r, uint16_t c, float strength) {
  if (strength <= 0.01f || r < 1)
    return;
  Target &t = tg();
  cx += t.ox;
  int ya = (int)floorf(cy - r), yb = (int)ceilf(cy + r);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  float k = 256.0f / (r * r);
  uint32_t S = (uint32_t)(clampf(strength, 0, 1) * 8 * t.ga);
  uint32_t f = ((uint32_t)c | ((uint32_t)c << 16)) & 0x07E0F81Fu;
  uint8_t lut[257];
  for (int i2 = 0; i2 <= 256; i2++)
    lut[i2] = (uint8_t)((GLOW_LUT[i2] * S) >> 8);
  for (int yy = ya; yy <= yb; yy++) {
    float dy = yy + 0.5f - cy;
    float w2 = r * r - dy * dy;
    if (w2 <= 0)
      continue;
    float w = sqrtf(w2);
    int xa = (int)floorf(cx - w), xb = (int)ceilf(cx + w);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    uint16_t *p = t.buf + yy * W + xa;
    float dx = xa + 0.5f - cx, q = (dx * dx + dy * dy) * k, dq = (2 * dx + 1) * k, ddq = 2 * k;
    for (int xx = xa; xx <= xb; xx++, p++, q += dq, dq += ddq) {
      int i2 = (int)q;
      if (i2 > 256)
        continue;
      uint32_t a = lut[i2];
      if (!a)
        continue;
      uint32_t b = ((uint32_t)*p | ((uint32_t)*p << 16)) & 0x07E0F81Fu;
      uint32_t rr = ((f * a + b * (32 - a)) >> 5) & 0x07E0F81Fu;
      *p = (uint16_t)(rr | (rr >> 16));
    }
  }
}

static void glowBox(float x, float y, float w, float h, float r, float spread, uint16_t c,
                    float strength) {
  if (strength <= 0.01f || w < 1 || h < 1)
    return;
  Target &t = tg();
  x += t.ox;
  float hw = w / 2, hh = h / 2, cx = x + hw, cy = y + hh;
  r = clampf(r, 0, mn(hw, hh));
  int ya = (int)floorf(y - spread), yb = (int)ceilf(y + h + spread);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  float k = 256.0f / (spread * spread);
  uint32_t S = (uint32_t)(clampf(strength, 0, 1) * 256);
  auto chord = [&](float dy, float g) -> float {
    float HH = hh + g, RR = r + g;
    if (dy > HH)
      return -1;
    if (dy <= HH - RR)
      return hw + g;
    float q = dy - (HH - RR);
    return hw - r + sqrtf(mx(RR * RR - q * q, 0));
  };
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, dy = fabsf(py - cy);
    float wo = chord(dy, spread);
    if (wo < 0)
      continue;
    float wi = chord(dy, 0);
    int xa = (int)floorf(cx - wo), xb = (int)ceilf(cx + wo);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    int ia = wi > 0 ? (int)ceilf(cx - wi) : xb + 1, ib = wi > 0 ? (int)ceilf(cx + wi) - 1 : xb;
    float qy = dy - hh + r;
    for (int xx = xa; xx <= xb; xx++) {
      if (xx == ia) {
        xx = ib;
        continue;
      }
      float qx = fabsf(xx + 0.5f - cx) - hw + r;
      float ox = qx > 0 ? qx : 0, oy = qy > 0 ? qy : 0, d2 = ox * ox + oy * oy;
      float d = (qx > 0 && qy > 0) ? sqrtf(d2) - r : mx(qx, qy) - r;
      if (d <= 0)
        continue;
      int i = (int)(d * d * k);
      if (i > 256)
        continue;
      uint32_t a = (GLOW_LUT[i] * S) >> 8;
      if (a)
        put(xx, yy, c, a);
    }
  }
}

static void tri(float x0, float y0, float x1, float y1, float x2, float y2, uint16_t c,
                float alpha = 1) {
  Target &t = tg();
  x0 += t.ox;
  x1 += t.ox;
  x2 += t.ox;
  float area = (x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0);
  if (fabsf(area) < 0.01f)
    return;
  if (area < 0) {
    float tx = x1, ty = y1;
    x1 = x2;
    y1 = y2;
    x2 = tx;
    y2 = ty;
  }
  float vx[3] = {x0, x1, x2}, vy[3] = {y0, y1, y2}, na[3], nb[3], nc[3];
  for (int k = 0; k < 3; k++) {
    float ex = vx[(k + 1) % 3] - vx[k], ey = vy[(k + 1) % 3] - vy[k],
          il = 1.0f / sqrtf(ex * ex + ey * ey);
    na[k] = -ey * il * -1;
    nb[k] = ex * il * -1;
    nc[k] = -(na[k] * vx[k] + nb[k] * vy[k]);
  }
  int ya = (int)floorf(mn(y0, mn(y1, y2)) - 1), yb = (int)ceilf(mx(y0, mx(y1, y2)) + 1);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  uint32_t A = (uint32_t)(clampf(alpha, 0, 1) * 32 + 0.5f);
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f, lo = -1e9f, hi = 1e9f, slo = -1e9f, shi = 1e9f;
    bool empty = false;
    for (int k = 0; k < 3; k++) {
      float base = nb[k] * py + nc[k];
      if (fabsf(na[k]) < 1e-6f) {
        if (base > 0.6f)
          empty = true;
        if (base > -0.6f) {
          slo = 1e9f;
        }
        continue;
      }
      float xo = (0.6f - base) / na[k], xi = (-0.6f - base) / na[k];
      if (na[k] > 0) {
        hi = mn(hi, xo);
        shi = mn(shi, xi);
      } else {
        lo = mx(lo, xo);
        slo = mx(slo, xi);
      }
    }
    if (empty || lo > hi)
      continue;
    int xa = (int)floorf(lo - 0.5f), xb = (int)ceilf(hi - 0.5f);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;
    int sa = (int)ceilf(slo - 0.5f), sb = (int)floorf(shi - 0.5f);
    for (int xx = xa; xx <= xb; xx++) {
      if (xx == sa && sa <= sb) {
        span(yy, sa, sb, c, A);
        xx = sb;
        continue;
      }
      float px = xx + 0.5f, d = -1e9f;
      for (int k = 0; k < 3; k++)
        d = mx(d, na[k] * px + nb[k] * py + nc[k]);
      uint32_t a = cov(d, alpha);
      if (a)
        put(xx, yy, c, a);
    }
  }
}

enum { AL_L = 0, AL_C = 1, AL_R = 2 };

static const uint8_t N2A[16] = {0, 2, 4, 6, 9, 11, 13, 15, 17, 19, 21, 23, 26, 28, 30, 32};

static int textW(const AFont &f, const char *s) {
  int w = 0;
  for (; *s; s++) {
    uint8_t ch = *s;
    if (ch < f.first || ch > f.last)
      ch = '?' < f.first ? f.first : '?';
    w += f.g[ch - f.first].adv;
  }
  return w;
}

static int text(const AFont &f, const char *s, float x, float y, uint16_t c, int align = AL_L,
                float alpha = 1) {
  Target &t = tg();
  int w = textW(f, s);
  int px = (int)floorf(x + t.ox + 0.5f) - (align == AL_C ? w / 2 : align == AL_R ? w : 0);
  int base = (int)floorf(y + 0.5f), endx = px + w;
  uint32_t A = (uint32_t)(clampf(alpha, 0, 1) * t.ga + 0.5f);
  if (!A)
    return endx - (int)t.ox;
  if (base - f.lineH > t.y1 || base + f.lineH < t.y0)
    return endx - (int)t.ox;
  for (; *s; s++) {
    uint8_t ch = *s;
    if (ch < f.first || ch > f.last)
      ch = '?' < f.first ? f.first : '?';
    const AGlyph &g = f.g[ch - f.first];
    int gx = px + g.xo, gy = base + g.yo;
    if (g.w && gx + g.w > 0 && gx < W && gy + g.h > t.y0 && gy < t.y1) {
      const uint8_t *bm = f.bmp + g.off;
      for (int r = 0; r < g.h; r++) {
        int yy = gy + r;
        if (yy < t.y0 || yy >= t.y1)
          continue;
        uint16_t *row = t.buf + yy * W;
        uint32_t i = r * g.w;
        for (int cc = 0; cc < g.w; cc++, i++) {
          uint8_t v = (bm[i >> 1] >> ((i & 1) ? 0 : 4)) & 15;
          if (!v)
            continue;
          int xx = gx + cc;
          if (xx < 0 || xx >= W)
            continue;
          uint32_t a = (N2A[v] * A) >> 5;
          row[xx] = a >= 32 ? c : blend(row[xx], c, a);
        }
      }
    }
    px += g.adv;
  }
  return endx - (int)t.ox;
}

static inline int textMid(const AFont &f, const char *s, float x, float y, uint16_t c,
                          int align = AL_L, float alpha = 1) {
  return text(f, s, x, y + f.capH / 2.0f, c, align, alpha);
}

static int vfmt(char *out, int cap, const char *f, va_list ap) {
  int n = 0;
  auto put = [&](char ch) {
    if (n < cap - 1)
      out[n] = ch;
    n++;
  };
  auto putu = [&](unsigned long v) {
    char t[12];
    int k = 0;
    do {
      t[k++] = '0' + v % 10;
      v /= 10;
    } while (v);
    while (k)
      put(t[--k]);
  };
  for (; *f; f++) {
    if (*f != '%') {
      put(*f);
      continue;
    }
    f++;
    bool plus = false, lng = false;
    int prec = -1;
    if (*f == '+') {
      plus = true;
      f++;
    }
    if (*f == '.') {
      f++;
      prec = 0;
      while (*f >= '0' && *f <= '9')
        prec = prec * 10 + (*f++ - '0');
    }
    if (*f == 'l') {
      lng = true;
      f++;
    }
    switch (*f) {
    case 'd':
    case 'i': {
      long v = lng ? va_arg(ap, long) : va_arg(ap, int);
      if (v < 0) {
        put('-');
        v = -v;
      } else if (plus)
        put('+');
      putu((unsigned long)v);
      break;
    }
    case 'u': {
      unsigned long v = lng ? va_arg(ap, unsigned long) : va_arg(ap, unsigned);
      putu(v);
      break;
    }
    case 's': {
      const char *s = va_arg(ap, const char *);
      while (s && *s)
        put(*s++);
      break;
    }
    case 'c':
      put((char)va_arg(ap, int));
      break;
    case 'f': {
      double v = va_arg(ap, double);
      if (prec < 0)
        prec = 2;
      if (prec > 4)
        prec = 4;
      if (v < 0) {
        put('-');
        v = -v;
      } else if (plus)
        put('+');
      unsigned long m = 1;
      for (int k = 0; k < prec; k++)
        m *= 10;
      unsigned long whole = (unsigned long)(v * m + 0.5);
      putu(whole / m);
      if (prec) {
        put('.');
        unsigned long fr = whole % m;
        char t[6];
        for (int k = prec - 1; k >= 0; k--) {
          t[k] = '0' + fr % 10;
          fr /= 10;
        }
        for (int k = 0; k < prec; k++)
          put(t[k]);
      }
      break;
    }
    case '%':
      put('%');
      break;
    default:
      put('%');
      put(*f);
      break;
    }
  }
  if (cap > 0)
    out[n < cap ? n : cap - 1] = 0;
  return n;
}

static int fmt(char *out, int cap, const char *f, ...) __attribute__((format(printf, 3, 4)));

static int fmt(char *out, int cap, const char *f, ...) {
  va_list ap;
  va_start(ap, f);
  int n = vfmt(out, cap, f, ap);
  va_end(ap);
  return n;
}

static int textf(const AFont &f, float x, float y, uint16_t c, int align, const char *fm, ...)
    __attribute__((format(printf, 6, 7)));

static int textf(const AFont &f, float x, float y, uint16_t c, int align, const char *fm, ...) {
  char b[64];
  va_list ap;
  va_start(ap, fm);
  vfmt(b, sizeof b, fm, ap);
  va_end(ap);
  return text(f, b, x, y, c, align);
}

}
