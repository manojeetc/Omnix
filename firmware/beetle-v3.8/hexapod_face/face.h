// Face poses, eye drawing and particles. Animation updates run once per frame on core 0.

#pragma once
#include "gfx.h"
#include "state.h"
using namespace gfx;

static const uint16_t C_BG0 = RGB(9, 12, 17), C_BG1 = RGB(16, 21, 30), C_CARD = RGB(22, 28, 38),
                      C_CARD2 = RGB(30, 37, 50), C_LINE = RGB(44, 53, 68),
                      C_TXT = RGB(236, 240, 245), C_DIM = RGB(130, 142, 160),
                      C_MUTE = RGB(78, 89, 106), C_EYE = RGB(92, 225, 255),
                      C_ACC = RGB(255, 138, 31), C_OK = RGB(64, 222, 140),
                      C_WARN = RGB(255, 184, 48), C_BAD = RGB(255, 82, 82),
                      C_PINK = RGB(255, 92, 170), C_GOLD = RGB(255, 206, 64),
                      C_ICE = RGB(150, 210, 255), C_RED_BG = RGB(44, 8, 12);

enum Shape { SH_NORMAL, SH_HEART, SH_SPIRAL, SH_CHEVRON, SH_STAR, SH_COUNT };

struct EyeP {
  float dx = 0, dy = 0, w = 70, h = 92, r = 26;
  float lidT = 0, slant = 0, lidB = 0, happy = 0;
  float sparkle = 0, glow = 0.35f;
  uint16_t col = C_EYE;
};

struct FaceP {
  EyeP e[2];
  float gx = 0, gy = 0, rot = 0;
  int shape = SH_NORMAL;
};

enum PType : uint8_t {
  PT_NONE,
  PT_NOTE,
  PT_HEART,
  PT_ZZZ,
  PT_SPARK,
  PT_SWEAT,
  PT_CONF,
  PT_TEXT,
  PT_RING
};

struct Part {
  uint8_t type = PT_NONE;
  char ch = 0;
  uint16_t col = 0;
  float x, y, vx, vy, age, life, size, rot, vr;
};

static const int MAXP = 48;

struct UI {
  uint32_t ms = 0;
  float t = 0, dt = 0.016f;
  bool linkOK = false;
  uint32_t lastRx = 0;

  int page = PG_FACE;
  int view = -1, prevView = -1;
  float viewT0 = -10, viewDir = 0;
  float touchT[6] = {-10, -10, -10, -10, -10, -10};
  uint8_t lastSwing = 0;
  int wantPage = PG_FACE;
  float pageDotsUntil = 0;
  bool trickFace = false;

  FaceP cur, tgt;
  float blinkT0 = -10, nextBlink = 3, nextSacc = 1.5f;
  float sgx = 0, sgy = 0;
  bool dblBlink = false;
  int shownShape = SH_NORMAL;
  float shapeWeight[SH_COUNT] = {1, 0, 0, 0, 0}, shapeFrom[SH_COUNT] = {1, 0, 0, 0, 0};
  float shapeT0 = -10;
  float shakeX = 0, shakeY = 0, spin = 0;
  Part p[MAXP];
  float spawnAcc = 0;

  char cap[32] = "";
  float capT0 = -10, capDur = 2;
  uint16_t capCol = C_TXT;
  int prevTrick = 0;
  float trickT0 = -10;
  int bannerTrick = 0;
  float bannerT0 = -10;
  uint32_t lastEvt = 0;
  int react = 0;
  float reactT0 = -10;
  int prevState = -1;
  bool prevBT = false;

  uint8_t swingHist[128] = {};
  int histHead = 0;
  float histAcc = 0;
  float pitchHist[100] = {}, rollHist[100] = {};
  int attHead = 0;
  float attAcc = 0;

  float fps = 0, c0ms = 0, c1ms = 0, frameMs = 0;
  int split = 120;
  float spiMHz = 0, linkHz = 0, tempC = 0;
  uint32_t linkErr = 0, linkLines = 0;

  float tipT0 = -10, stumT0 = -10, yawnT0 = -10, nextYawn = 0;
  uint32_t prevF2 = 0;
  int prevIdle = -1, batTier = 0;
};

static UI U;

static inline float frand(float a, float b) {
  return a + (b - a) * (rand() / (float)RAND_MAX);
}

static inline float easeIO(float s) {
  s = clampf(s, 0, 1);
  return s * s * (3 - 2 * s);
}

static inline float approach(float cur, float tgt, float dt, float tau) {
  return cur + (tgt - cur) * (1 - expf(-dt / tau));
}

#include "nav.h"

static void spawn(PType ty, float x, float y, float vx, float vy, float life, float size,
                  uint16_t col, char ch = 0) {
  for (int i = 0; i < MAXP; i++)
    if (U.p[i].type == PT_NONE) {
      Part &q = U.p[i];
      q.type = ty;
      q.x = x;
      q.y = y;
      q.vx = vx;
      q.vy = vy;
      q.age = 0;
      q.life = life;
      q.size = size;
      q.col = col;
      q.ch = ch;
      q.rot = frand(0, 6.28f);
      q.vr = frand(-4, 4);
      return;
    }
}

static void particlesUpdate(float dt) {
  for (int i = 0; i < MAXP; i++) {
    Part &q = U.p[i];
    if (q.type == PT_NONE)
      continue;
    q.age += dt;
    if (q.age >= q.life) {
      q.type = PT_NONE;
      continue;
    }
    if (q.type == PT_CONF || q.type == PT_SWEAT)
      q.vy += 220 * dt;
    if (q.type == PT_NOTE || q.type == PT_HEART || q.type == PT_ZZZ)
      q.vx += sinf(q.age * 3 + q.rot) * 30 * dt;
    q.x += q.vx * dt;
    q.y += q.vy * dt;
    q.rot += q.vr * dt;
  }
}

static void drawNote(float x, float y, float s, uint16_t c, float a) {
  disc(x, y, 5.5f * s, c, a);
  line(x + 4.5f * s, y, x + 4.5f * s, y - 22 * s, 2.6f * s, c, a);
  line(x + 4.5f * s, y - 22 * s, x + 13 * s, y - 16 * s, 3 * s, c, a);
}

static void drawHeart(float x, float y, float s, uint16_t c, float a) {
  disc(x - 5.2f * s, y - 3 * s, 6 * s, c, a);
  disc(x + 5.2f * s, y - 3 * s, 6 * s, c, a);
  tri(x - 10.9f * s, y - 0.8f * s, x + 10.9f * s, y - 0.8f * s, x, y + 11 * s, c, a);
}

static void drawSpark(float x, float y, float s, uint16_t c, float a, float rot) {
  glow(x, y, 9 * s, c, 0.5f * a);
  float cs = cosf(rot) * 9 * s, sn = sinf(rot) * 9 * s;
  line(x - cs, y - sn, x + cs, y + sn, 2.2f * s, c, a);
  line(x + sn, y - cs, x - sn, y + cs, 2.2f * s, c, a);
  disc(x, y, 2.2f * s, RGB(255, 255, 255), a);
}

static void drawSweat(float x, float y, float s, uint16_t c, float a) {
  disc(x, y + 3 * s, 5 * s, c, a);
  tri(x - 4.6f * s, y + 1.5f * s, x + 4.6f * s, y + 1.5f * s, x, y - 9 * s, c, a);
  disc(x - 1.5f * s, y + 2 * s, 1.5f * s, RGB(255, 255, 255), a * 0.8f);
}

static void particlesDraw() {
  for (int i = 0; i < MAXP; i++) {
    const Part &q = U.p[i];
    if (q.type == PT_NONE)
      continue;
    float f = q.age / q.life, a = mn(1, mn(q.age / 0.15f, (1 - f) / 0.3f));
    switch (q.type) {
    case PT_NOTE:
      drawNote(q.x, q.y, q.size, q.col, a);
      break;
    case PT_HEART:
      drawHeart(q.x, q.y, q.size * (1 + 0.1f * sinf(q.age * 10)), q.col, a);
      break;
    case PT_ZZZ: {
      char b[2] = {q.ch ? q.ch : 'z', 0};
      text(q.size > 1.2f ? F_BIG : F_MED, b, q.x, q.y, q.col, AL_C, a);
      break;
    }
    case PT_SPARK:
      drawSpark(q.x, q.y, q.size * (0.6f + 0.4f * sinf(f * 3.14f)), q.col, a, q.rot);
      break;
    case PT_SWEAT:
      drawSweat(q.x, q.y, q.size, q.col, a);
      break;
    case PT_CONF: {
      float cs = cosf(q.rot);
      rrect(q.x - 3 * q.size * fabsf(cs) - 0.6f, q.y - 2 * q.size, 6 * q.size * fabsf(cs) + 1.2f,
            4 * q.size, 1, q.col, a);
      break;
    }
    case PT_TEXT: {
      char b[2] = {q.ch, 0};
      text(F_HUGE, b, q.x, q.y, q.col, AL_C, a);
      break;
    }
    case PT_RING:
      ring(q.x, q.y, q.size * (6 + f * 27), 1.5f, q.col, a * (1 - f) * 0.6f);
      break;
    default:
      break;
    }
  }
}

static void caption(const char *s, uint16_t col = C_TXT, float dur = 2.2f) {
  strncpy(U.cap, s, sizeof U.cap - 1);
  U.cap[sizeof U.cap - 1] = 0;
  U.capT0 = U.t;
  U.capDur = dur;
  U.capCol = col;
}

static const float EYE_X[2] = {104, 216}, EYE_Y = 124;

static void eyeNormal(float cx, float cy, const EyeP &e, int side, float rot) {
  float w = mx(e.w, 2), h = mx(e.h, 2), r = clampf(e.r, 0, mn(w, h) / 2);
  float hw = w / 2, hh = h / 2;
  float lt = -hh + e.lidT * h;
  float ms = tanf(e.slant * 0.0174533f) * (side == 0 ? 1 : -1);
  float lb = hh - e.lidB * h;
  float kT = 1 / sqrtf(1 + ms * ms);
  float Rc = w * 0.72f, ccy = hh + Rc * (1.1f - e.happy * 1.45f);
  bool useHappy = e.happy > 0.02f, useT = e.lidT > 0.005f || fabsf(e.slant) > 0.5f,
       useB = e.lidB > 0.005f;
  float cr = cosf(rot), sr = sinf(rot);
  Target &t = tg();
  float ox = t.ox;
  float ext = (fabsf(rot) > 0.01f) ? sqrtf(hw * hw + hh * hh) + 2 : 0;
  float bx0 = cx - (ext ? ext : hw + 2), bx1 = cx + (ext ? ext : hw + 2),
        by0 = cy - (ext ? ext : hh + 2), by1 = cy + (ext ? ext : hh + 2);
  int ya = (int)floorf(by0), yb = (int)ceilf(by1);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  int xa = (int)floorf(bx0 + ox), xb = (int)ceilf(bx1 + ox);
  if (xa < 0)
    xa = 0;
  if (xb > W - 1)
    xb = W - 1;
  uint16_t c = e.col;
  float Rin2 = (Rc - 0.6f) * (Rc - 0.6f), Rout2 = (Rc + 0.6f) * (Rc + 0.6f);
  bool straight = fabsf(rot) <= 0.01f;
  auto chord = [&](float dy, float g) -> float {
    float HH = hh + g, RR = mx(r + g, 0);
    if (dy > HH)
      return -1;
    if (dy <= HH - RR)
      return hw + g;
    float q = dy - (HH - RR);
    return hw - r + sqrtf(mx(RR * RR - q * q, 0));
  };
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f - cy;
    int x0 = xa, x1 = xb;
    float wi = -1;
    if (straight) {
      float wo = chord(fabsf(py), 0.6f);
      if (wo < 0)
        continue;
      wi = chord(fabsf(py), -0.6f);
      x0 = (int)floorf(cx + ox - wo);
      x1 = (int)ceilf(cx + ox + wo);
      if (x0 < 0)
        x0 = 0;
      if (x1 > W - 1)
        x1 = W - 1;
      if (wi > 0 && !useT && !useB && !useHappy) {
        int sa = (int)ceilf(cx + ox - wi), sb = (int)floorf(cx + ox + wi) - 1;
        span(yy, sa, sb, c, 32);
        for (int xx = x0; xx <= x1; xx++) {
          if (xx == sa && sa <= sb) {
            xx = sb;
            continue;
          }
          float lx = xx + 0.5f - ox - cx, qx = fabsf(lx) - hw + r, qy = fabsf(py) - hh + r;
          float oxx = qx > 0 ? qx : 0, oyy = qy > 0 ? qy : 0;
          uint32_t a = cov(sqrtf(oxx * oxx + oyy * oyy) + mn(mx(qx, qy), 0.0f) - r, 1);
          if (a)
            put(xx, yy, c, a);
        }
        continue;
      }
    }
    for (int xx = x0; xx <= x1; xx++) {
      float px = xx + 0.5f - ox - cx;
      float lx = straight ? px : px * cr + py * sr, ly = straight ? py : -px * sr + py * cr;
      float d;
      if (straight && fabsf(lx) < wi)
        d = -1;
      else {
        float qx = fabsf(lx) - hw + r, qy = fabsf(ly) - hh + r;
        float oxx = qx > 0 ? qx : 0, oyy = qy > 0 ? qy : 0;
        d = sqrtf(oxx * oxx + oyy * oyy) + mn(mx(qx, qy), 0.0f) - r;
        if (d > 0.6f)
          continue;
      }
      if (useT)
        d = mx(d, (lt + lx * ms - ly) * kT);
      if (useB)
        d = mx(d, ly - lb);
      if (useHappy && d < 0.6f) {
        float ddy = ly - ccy, q2 = lx * lx + ddy * ddy;
        if (q2 <= Rin2)
          continue;
        if (q2 < Rout2)
          d = mx(d, Rc - sqrtf(q2));
      }
      uint32_t a = cov(d, 1);
      if (a)
        put(xx, yy, c, a);
    }
  }
}

static void eyeHeart(float cx, float cy, float s, uint16_t c) {
  glow(cx, cy, 52 * s, c, 0.35f);
  drawHeart(cx, cy - 4 * s, 3.4f * s, c, 1);
  disc(cx - 17 * s, cy - 20 * s, 5 * s, RGB(255, 220, 235), 0.9f);
}

static void eyeSpiral(float cx, float cy, float s, float spin, uint16_t c, int side) {
  float px = cx, py = cy;
  for (int j = 1; j < 46; j++) {
    float a = j * 0.42f + spin * (side ? -1 : 1), r = j * 0.78f * s;
    float nx = cx + r * cosf(a), ny = cy + r * sinf(a);
    line(px, py, nx, ny, 5 * s, c);
    px = nx;
    py = ny;
  }
}

static void eyeChevron(float cx, float cy, float s, uint16_t c, int side) {
  float d = side == 0 ? 1 : -1;
  line(cx - 18 * s * d, cy - 26 * s, cx + 16 * s * d, cy, 13 * s, c);
  line(cx + 16 * s * d, cy, cx - 18 * s * d, cy + 26 * s, 13 * s, c);
}

static void eyeStar(float cx, float cy, float s, float rot, uint16_t c) {
  glow(cx, cy, 60 * s, c, 0.4f);
  float ro = 40 * s, ri = 17 * s, X[10], Y[10];
  for (int k = 0; k < 10; k++) {
    float a = rot + k * 0.6283f - 1.5708f, rr = (k & 1) ? ri : ro;
    X[k] = cx + rr * cosf(a);
    Y[k] = cy + rr * sinf(a);
  }
  for (int k = 0; k < 10; k += 2)
    tri(cx, cy, X[k], Y[k], X[(k + 1) % 10], Y[(k + 1) % 10], c),
        tri(cx, cy, X[k], Y[k], X[(k + 9) % 10], Y[(k + 9) % 10], c);
}

static float blinkCurve(float t) {
  if (t < 0 || t >= 0.25f)
    return 0;
  if (t < 0.065f)
    return easeIO(t / 0.065f);
  if (t < 0.105f)
    return 1;
  return 1 - easeIO((t - 0.105f) / 0.145f);
}

static float blinkAmount() {
  float t = U.t - U.blinkT0;
  return mx(blinkCurve(t), U.dblBlink ? blinkCurve(t - 0.32f) : 0);
}

static void shapesUpdate(int desired) {
  desired = clampi(desired, 0, SH_COUNT - 1);
  if (desired != U.shownShape) {
    for (int i = 0; i < SH_COUNT; i++)
      U.shapeFrom[i] = U.shapeWeight[i];
    U.shownShape = desired;
    U.shapeT0 = U.t;
  }
  float q = easeIO((U.t - U.shapeT0) / 0.22f);
  for (int i = 0; i < SH_COUNT; i++)
    U.shapeWeight[i] = U.shapeFrom[i] * (1 - q) + (i == desired ? q : 0);
}

static float eyeWink(int side);

static void drawEyes() {
  const FaceP &F = U.cur;
  float blink = blinkAmount();
  float cxm = 160, cym = EYE_Y;
  float cr = cosf(F.rot), sr = sinf(F.rot);
  for (int k = 0; k < 2; k++) {
    float bl = mx(blink, eyeWink(k));
    EyeP e = F.e[k];

    float persp = 1 + 0.07f * F.gx * (k == 0 ? -1 : 1);
    e.w *= persp;
    e.h *= persp;
    float hx = EYE_X[k] - cxm + F.gx * 24 + e.dx, hy = EYE_Y - cym + F.gy * 15 + e.dy;
    float cx = cxm + hx * cr - hy * sr + U.shakeX, cy = cym + hx * sr + hy * cr + U.shakeY;
    if (bl > 0.985f) {
      line(cx - e.w * 0.40f, cy, cx + e.w * 0.40f, cy, 3, e.col, 0.75f);
      continue;
    }
    uint32_t savedAlpha = tg().ga;
    if (U.shapeWeight[SH_NORMAL] > 0) {
      tg().ga = (uint32_t)(savedAlpha * U.shapeWeight[SH_NORMAL] + 0.5f);
      e.h = mx(3, e.h * (1 - bl * 0.965f));
      e.lidT *= 1 - bl;
      e.lidB *= 1 - bl;
      e.happy *= 1 - bl;
      {
        float top = cy - e.h / 2 + e.lidT * e.h, bot = cy + e.h / 2 - e.lidB * e.h;
        if (e.happy > 0.02f) {
          float Rc = e.w * 0.72f;
          bot = mn(bot, cy + e.h / 2 + Rc * (1.1f - e.happy * 1.45f) - Rc * 0.75f);
        }
        if (bot - top > 2)
          glowBox(cx - e.w / 2, top, e.w, bot - top, mn(e.r, (bot - top) / 2), 20, e.col,
                  e.glow * (1 - bl * 0.6f));
      }
      eyeNormal(cx, cy, e, k, F.rot);
      if (e.sparkle > 0.02f && bl < 0.5f) {
        float sx = cx - e.w * 0.2f, sy = cy - e.h * 0.24f;
        disc(sx, sy, e.w * 0.12f * e.sparkle, RGB(255, 255, 255), 0.95f);
        disc(sx + e.w * 0.3f, sy + e.h * 0.32f, e.w * 0.06f * e.sparkle, RGB(255, 255, 255), 0.8f);
      }
    }
    for (int shape = SH_HEART; shape < SH_COUNT; shape++) {
      if (U.shapeWeight[shape] <= 0)
        continue;
      tg().ga = (uint32_t)(savedAlpha * U.shapeWeight[shape] + 0.5f);
      float s = (F.e[k].w * persp / 70) * (1 - bl * 0.98f);
      switch (shape) {
      case SH_HEART:
        eyeHeart(cx, cy, s, e.col);
        break;
      case SH_SPIRAL:
        eyeSpiral(cx, cy, s, U.spin, e.col, k);
        break;
      case SH_CHEVRON:
        eyeChevron(cx, cy, s, e.col, k);
        break;
      case SH_STAR:
        eyeStar(cx, cy, s, U.spin * 0.3f, e.col);
        break;
      }
    }
    tg().ga = savedAlpha;
  }
}

static void exNeutral(FaceP &f) {
  f = FaceP();
}

static void exHappy(FaceP &f, float k = 1) {
  f.e[0].happy = f.e[1].happy = k;
  f.e[0].sparkle = f.e[1].sparkle = 0;
}

static void exAngry(FaceP &f, float lid, float slant, uint16_t col) {
  for (auto &e : f.e) {
    e.lidT = lid;
    e.slant = slant;
    e.col = col;
    e.glow = 0.45f;
  }
}

static void exSleepy(FaceP &f, float lid) {
  for (auto &e : f.e) {
    e.lidT = lid;
    e.lidB = 0.08f;
    e.glow = 0.2f;
  }
}

#include "personality.h"

static float eyeWink(int side) {
  return A.wink[side];
}

static void faceTarget(const Robot &R, FaceP &f) {
  exNeutral(f);
  float sw = R.swapped() ? -1 : 1;
  float vxN = R.vx * sw, vyN = R.vy * sw;
  float tt = U.t - U.trickT0;
  int tr = R.trick;
  float breath = sinf(U.t * 1.7f);
  for (auto &e : f.e)
    e.dy = 0.9f * breath;

  bool glance = tr == TR_NONE && !R.walking() && A.act == FA_NONE && A.idle < 0 && !faceUrgent(R);
  f.gx = glance ? U.sgx : 0;
  f.gy = glance ? U.sgy : 0;

  if (tr != TR_NONE) {
    float D = TRICK_DUR[tr];
    float yawG = clampf(R.yaw / 16, -1, 1);
    switch (tr) {
    case TR_WAVE:
      exHappy(f);
      f.gx = 0.45f;
      for (auto &e : f.e)
        e.dy = -3 + 2.5f * sinf(tt * 7);
      break;
    case TR_BOW: {
      float c = mn(easeIO(tt / 0.8f), easeIO((D - tt) / 0.6f));
      exHappy(f, 0.5f * c);
      for (auto &e : f.e) {
        e.lidT = 0.75f * c;
        e.dy = 10 * c;
      }
      f.gy = 0.6f * c;
      break;
    }
    case TR_TWIST:
    case TR_LOOK:
      f.gx = yawG * 1.1f;
      f.gy = -0.15f * fabsf(yawG);
      for (auto &e : f.e)
        e.sparkle = 0.6f;
      break;
    case TR_DANCE: {
      float beat = 0.5f - 0.5f * cosf(2 * 3.14159f * tt / 0.6f);
      exHappy(f, 0.75f);
      f.e[0].col = hsv(U.t * 140, 0.75f, 1);
      f.e[1].col = hsv(U.t * 140 + 120, 0.75f, 1);
      for (auto &e : f.e) {
        e.dy = -7 * beat;
        e.h = 92 * (1 - 0.12f * (1 - beat));
        e.glow = 0.6f;
      }
      f.gx = 0.5f * sinf(tt * 2.6f);
      break;
    }
    case TR_PUSHUP: {
      float dip = 0.5f - 0.5f * cosf(2 * 3.14159f * tt / 1.6f);
      exAngry(f, 0.36f, 14, C_EYE);
      for (auto &e : f.e)
        e.dy = 10 * dip;
      f.gy = 0.3f * dip;
      break;
    }
    case TR_STOMP:
      exAngry(f, 0.42f, 26, C_BAD);
      break;
    case TR_SHAKE:
      f.shape = SH_SPIRAL;
      break;
    case TR_NOD:
      exHappy(f, 0.55f);
      f.gy = clampf(-R.pPitch * (R.swapped() ? -1 : 1) / 10, -0.2f, 1);
      break;
    case TR_NO:
      for (auto &e : f.e) {
        e.lidT = 0.34f;
      }
      f.gx = yawG * 1.2f;
      break;
    case TR_STRETCH: {
      float fr = (tt > 0.2f && tt < 2.9f) ? 1 : 0, rr = (tt > 2.6f && tt < 5.4f) ? 1 : 0;
      if (fr) {
        exHappy(f, 0.85f);
        f.gy = 0.4f;
      } else if (rr) {
        for (auto &e : f.e) {
          e.lidT = 0.45f;
          e.lidB = 0.1f;
        }
        f.gy = -0.5f;
      }
      break;
    }
    case TR_BEG:
      for (auto &e : f.e) {
        e.w = 80;
        e.h = 104;
        e.r = 34;
        e.sparkle = 1;
        e.slant = -10;
      }
      f.gy = -0.55f;
      break;
    case TR_CIRCLE: {
      float w = 2 * 3.14159f * tt / 2.4f;
      f.gx = -0.7f * sinf(w);
      f.gy = 0.5f * cosf(w);
      break;
    }
    case TR_HEADBANG: {
      float s = sinf(3.14159f * tt / 0.35f);
      exAngry(f, 0.4f + 0.3f * s * s, 18, hsv(330 + 30 * s, 0.7f, 1));
      f.gy = 0.6f * s * s;
      break;
    }
    case TR_RIPPLE: {
      int k = (int)((tt - 0.3f) / 0.3f);
      const float ang[6] = {0.6f, 1.0f, 0.6f, -0.6f, -1.0f, -0.6f};
      const int ord[6] = {0, 1, 2, 5, 4, 3};
      if (k >= 0 && k < 12) {
        int L = ord[k % 6];
        f.gx = ang[L];
        f.gy = (L == 2 || L == 5) ? 0.5f : (L == 1 || L == 4) ? 0.2f : -0.2f;
      }
      for (auto &e : f.e)
        e.sparkle = 0.5f;
      break;
    }
    case TR_POINT:
      f.gx = 0.75f;
      f.gy = -0.35f;
      for (auto &e : f.e) {
        e.w = 76;
        e.h = 98;
        e.sparkle = 0.7f;
      }
      break;
    case TR_TAP:
      for (auto &e : f.e) {
        e.lidT = 0.48f;
      }
      f.gx = -0.6f;
      f.gy = 0.15f;
      break;
    case TR_SLEEPY: {
      float sink = clampf((tt - 0.2f) / 3.4f, 0, 1);
      bool woke = tt > 4.05f;
      if (!woke)
        exSleepy(f, 0.15f + 0.75f * sink);
      else
        for (auto &e : f.e) {
          e.w = 80;
          e.h = 104;
          e.r = 32;
        }
      break;
    }
    case TR_SHIVER:
      for (int k = 0; k < 2; k++) {
        EyeP &e = f.e[k];
        e.w = 58;
        e.h = 70;
        e.col = C_ICE;
        e.dx = sinf(tt * 25 + k * 0.35f);
        e.dy = 0.8f * cosf(tt * 25 + k * 0.35f);
        e.slant = -14;
        e.lidT = 0.1f;
      }
      break;
    case TR_POUNCE: {
      bool lunge = tt > 2.6f && tt < 3.5f;
      if (lunge)
        for (auto &e : f.e) {
          e.w = 82;
          e.h = 100;
          e.sparkle = 0.4f;
        }
      else {
        exAngry(f, 0.5f, 12, C_EYE);
        f.gx = 0.25f * sinf(2 * 3.14159f * 5 * tt);
      }
      break;
    }
    case TR_SWAY: {
      exHappy(f, 0.7f);
      f.gx = 0.6f * sinf(2 * 3.14159f * tt / 1.6f);
      break;
    }
    case TR_FLEX:
      f.shape = SH_STAR;
      for (auto &e : f.e)
        e.col = C_GOLD;
      break;
    case TR_CURIOUS: {
      float l = tt < 1.9f ? 1 : -1;
      f.rot = -0.2f * l * mn(1, mn(tt / 0.4f, (D - tt) / 0.4f));
      f.e[0].w = l > 0 ? 80 : 64;
      f.e[0].h = l > 0 ? 100 : 82;
      f.e[1].w = l > 0 ? 64 : 80;
      f.e[1].h = l > 0 ? 82 : 100;
      for (auto &e : f.e)
        e.sparkle = 0.8f;
      break;
    }
    case TR_SCARED:
      f.shape = tt < 2.4f ? SH_CHEVRON : SH_NORMAL;
      if (tt >= 2.4f)
        for (auto &e : f.e) {
          e.slant = -18;
          e.lidT = 0.12f;
        }
      break;
    }
    if (R.party > 0 && tr == TR_DANCE) {
    }
    return;
  }

  if (U.t - U.tipT0 < 2.6f) {
    f.shape = SH_SPIRAL;
    for (auto &e : f.e)
      e.col = mix(C_EYE, C_WARN, 0.4f);
    return;
  }
  if (U.t - U.stumT0 < 1.0f) {
    for (auto &e : f.e) {
      e.w = 84;
      e.h = 108;
      e.r = 36;
      e.sparkle = 0.35f;
    }
    f.gy = -0.25f;
    return;
  }

  if (R.park == P_PARKED || R.park == P_LOWERING) {
    exSleepy(f, 0.86f);
    f.gy = 0.3f;
    return;
  }
  if (!R.servosOn()) {
    exSleepy(f, 0.55f);
    return;
  }
  if (R.sitting()) {
    exHappy(f, 0.25f);
    for (auto &e : f.e) {
      e.lidT = 0.22f;
      e.dy += 6;
    }
    return;
  }

  float tiltDeg = mx(fabsf(R.pitch), fabsf(R.roll));
  if (R.imuOK() && tiltDeg > 16) {
    for (auto &e : f.e) {
      e.w = 64;
      e.h = 86;
      e.slant = -16;
      e.lidT = 0.08f;
    }
    f.gx = clampf(R.roll / 25, -1, 1);
    return;
  }

  int bat = R.onBatt() ? R.pct() : 100;
  float tired = bat <= 10 ? 0.5f : bat <= 20 ? 0.38f : bat <= 35 ? 0.18f : 0;
  int hot = -1;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 2; j++)
      hot = R.heat[i][j] > hot ? R.heat[i][j] : hot;
  auto overlays = [&]() {
    for (auto &e : f.e) {
      if (tired > 0) {
        e.lidT = mx(e.lidT, tired);
        if (bat <= 20) {
          e.slant = mn(e.slant, -8.0f);
          e.col = mix(e.col, C_WARN, bat <= 10 ? 0.65f : 0.45f);
        }
      }
      if (hot > 55) {
        float k = clampf((hot - 55) / 25.0f, 0, 1);
        e.slant = mx(e.slant, 6 + 8 * k);
        e.lidT = mx(e.lidT, 0.22f + 0.12f * k);
        e.col = mix(e.col, RGB(255, 150, 70), 0.35f * k);
      }
    }
  };
  if (R.boardDown()) {
    for (auto &e : f.e) {
      e.slant = -16;
      e.lidT = 0.1f;
      e.w = 64;
      e.h = 84;
      e.col = mix(C_EYE, C_BAD, 0.4f);
    }
    f.gx = 0.5f * sinf(U.t * 2.2f);
    return;
  }

  if (R.walking()) {
    f.gx = clampf(vyN / 50 + R.wz * 1.6f, -1, 1);
    f.gy = vxN < -1 ? -0.35f : 0.12f;
    float real = sqrtf(R.vx * R.vx + R.vy * R.vy) * R.gov;
    for (auto &e : f.e) {
      e.lidT = vxN > 1 ? 0.16f : 0.0f;
      if (vxN < -1) {
        e.w = 76;
        e.h = 98;
      }
    }
    if (real > 60)
      for (auto &e : f.e) {
        e.lidT = mx(e.lidT, 0.24f);
        e.slant = 8;
        e.glow = 0.5f;
      }
    overlays();
    return;
  }
  if (R.party > 0) {
    exHappy(f, 0.8f);
    float b = 0.5f - 0.5f * cosf(U.t * 6.5f);
    f.e[0].col = hsv(U.t * 60, 0.55f, 1);
    f.e[1].col = hsv(U.t * 60 + 60, 0.55f, 1);
    for (auto &e : f.e) {
      e.dy = -5 * b;
      e.glow = 0.55f;
    }
    f.gx = 0.35f * sinf(U.t * 2.1f);
    return;
  }
  if (R.idleLeft >= 0 && R.idleLeft <= 60) {
    float d = 1 - R.idleLeft / 60.0f, yt = U.t - U.yawnT0;
    exSleepy(f, 0.12f + 0.6f * d);
    for (auto &e : f.e) {
      e.lidB = 0.06f * d;
      e.glow = 0.35f - 0.15f * d;
    }
    f.gy = 0.25f * d;
    if (yt < 1.8f) {
      float y = sinf(3.14159f * yt / 1.8f);
      for (auto &e : f.e) {
        e.lidT = mx(e.lidT, 0.55f + 0.4f * y);
        e.happy = 0.35f * y;
        e.h = 92 * (1 + 0.08f * y);
      }
      f.gy = -0.3f * y;
    }
  }
  overlays();
}

static int robotStateId(const Robot &R) {
  if (R.batLocked())
    return 6;
  if (R.park == P_PARKED)
    return 5;
  if (R.park == P_LOWERING)
    return 4;
  if (!R.servosOn())
    return 3;
  if (R.walking())
    return 2;
  if (R.sitting())
    return 1;
  return 0;
}

static void faceUpdate(const Robot &R, float dt) {

  if (R.trick != U.prevTrick) {
    if (R.trick != TR_NONE) {
      U.trickT0 = U.t - R.trickMs / 1000.0f;
      U.bannerTrick = R.trick;
      U.bannerT0 = U.t;
      U.spawnAcc = 0;
    }
    U.prevTrick = R.trick;
  }

  if ((R.evt >> 4) != (U.lastEvt >> 4)) {
    int ty = R.evt & 15;
    U.lastEvt = R.evt;
    if (ty != EV_CMD || U.react == EV_NONE || U.t - U.reactT0 > 0.3f) {
      U.react = ty;
      U.reactT0 = U.t;
    }
    if (ty == EV_BT_ON)
      caption("Bluetooth connected", C_EYE);
    if (ty == EV_BT_OFF)
      caption("Bluetooth disconnected", C_DIM);
    if (ty == EV_DOUBLE)
      caption(R.sitting() ? "standing up" : "sitting", C_DIM);
  }

  {
    uint32_t up = R.flags2 & ~U.prevF2;
    U.prevF2 = R.flags2;
    if (up & 1) {
      U.tipT0 = U.t;
      caption("whoa!", C_WARN, 2.0f);
      for (int k = 0; k < 3; k++)
        spawn(PT_SWEAT, frand(40, 280), 50, frand(-20, 20), 10, 1.4f, 1.1f, C_ICE);
    }
    if ((up & 2) && U.t - U.tipT0 > 2.6f) {
      U.stumT0 = U.t;
      spawn(PT_TEXT, 286, 84, 0, -20, 1.0f, 1, C_ACC, '!');
    }
    if (up & 4)
      caption("servo board not answering", C_BAD, 3.5f);
    if (up & 32)
      caption("balance hold - set me upright", C_WARN, 3.5f);
    int il = R.idleLeft;
    if (il >= 0 && il <= 30 && (U.prevIdle > 30 || U.prevIdle < 0) && R.trick == TR_NONE)
      caption("getting sleepy...", C_DIM, 2.4f);
    if (il >= 0 && il <= 60 && R.trick == TR_NONE && !R.walking() && U.t >= U.nextYawn) {
      if (U.prevIdle >= 0 && U.prevIdle <= 60)
        U.yawnT0 = U.t;
      U.nextYawn = U.t + frand(7, 12);
    }
    U.prevIdle = il;
    int bat = R.onBatt() ? R.pct() : 100, tier = bat <= 10 ? 2 : bat <= 20 ? 1 : 0;
    if (tier > U.batTier) {
      char b[32], bl[24];
      batLeftText(bl, sizeof bl);
      if (bl[0])
        fmt(b, sizeof b, "battery %d%%  %s", bat, bl);
      else
        fmt(b, sizeof b, "battery %d%%", bat);
      caption(b, tier == 2 ? C_BAD : C_WARN, 3.0f);
    }
    U.batTier = tier;
  }

  int st = robotStateId(R);
  if (U.prevState >= 0 && st != U.prevState && R.trick == TR_NONE) {
    static const char *const ST[] = {"standing",   "sitting", "walking",      "relaxed",
                                     "lying down", "parked",  "battery empty"};
    char b[32];
    if (st == 2)
      fmt(b, sizeof b, "%s  %d mm/s", GAIT_NAME[R.gait], (int)R.speed);
    else if (st == 0)
      fmt(b, sizeof b, "standing  %d mm", (int)R.targetH);
    else
      fmt(b, sizeof b, "%s", ST[st]);
    caption(b, st == 2 ? C_OK : C_DIM, 1.8f);
  }
  U.prevState = st;

  personalityUpdate(R, dt);
  bool quietEyes =
      R.trick == TR_NONE && A.act == FA_NONE && A.idle < 0 && !R.party && !faceUrgent(R);
  if (quietEyes && U.t >= U.nextBlink) {
    U.blinkT0 = U.t;
    U.dblBlink = frand(0, 1) < 0.10f;
    U.nextBlink = U.t + (A.style == 0 ? frand(4, 8) : frand(2.8f, 6.5f));
  } else if (!quietEyes)
    U.nextBlink = mx(U.nextBlink, U.t + 0.8f);
  if (quietEyes && !R.walking() && U.t >= U.nextSacc) {
    if (frand(0, 1) < 0.45f) {
      U.sgx = 0;
      U.sgy = 0;
    } else {
      U.sgx = frand(-0.4f, 0.4f);
      U.sgy = frand(-0.22f, 0.22f);
    }
    U.nextSacc = U.t + frand(1.6f, 4.2f);
  } else if (!quietEyes || R.walking()) {
    U.sgx = U.sgy = 0;
    U.nextSacc = U.t + 1.2f;
  }

  faceTarget(R, U.tgt);
  personalityTarget(R, U.tgt);
  personalityFinish(dt);
  shapesUpdate(U.tgt.shape);
  FaceP &c = U.cur;
  const FaceP &g = U.tgt;
  float fast =
      (R.trick == TR_HEADBANG || R.trick == TR_SHIVER || R.trick == TR_DANCE) ? 0.055f : 0.095f;
  float damping = A.style == 2 ? 0.90f : 1.0f;
  float omega = fast < 0.07f ? 32 : 25;
  c.gx = springStep(c.gx, g.gx, A.vx, dt, 30, damping, -1.25f, 1.25f);
  c.gy = springStep(c.gy, g.gy, A.vy, dt, 30, damping, -1.2f, 1.2f);
  c.rot = springStep(c.rot, g.rot, A.vr, dt, 23, damping, -0.30f, 0.30f);
  c.shape = U.shownShape;
  for (int k = 0; k < 2; k++) {
    EyeP &e = c.e[k];
    const EyeP &t = g.e[k];
    e.dx = springStep(e.dx, t.dx, A.eyeV[k].dx, dt, 32, damping, -12, 12);
    e.dy = springStep(e.dy, t.dy, A.eyeV[k].dy, dt, omega, damping, -22, 25);
    e.w = springStep(e.w, t.w, A.eyeV[k].w, dt, omega, damping, 40, 92);
    e.h = springStep(e.h, t.h, A.eyeV[k].h, dt, omega, damping, 8, 116);
    e.r = approach(e.r, t.r, dt, fast);
    e.lidT = approach(e.lidT, t.lidT, dt, fast);
    e.slant = approach(e.slant, t.slant, dt, fast);
    e.lidB = approach(e.lidB, t.lidB, dt, fast);
    e.happy = approach(e.happy, t.happy, dt, fast);
    e.sparkle = approach(e.sparkle, t.sparkle, dt, 0.1f);
    e.glow = approach(e.glow, t.glow, dt, 0.15f);
    e.col = eyeColor(A.eyeV[k], e.col, t.col, dt,
                     R.trick == TR_DANCE || R.trick == TR_HEADBANG ? 0.045f : 0.14f);
  }
  U.spin += dt * 9;

  U.shakeX = U.shakeY = 0;
  float tt = U.t - U.trickT0;
  if (R.trick == TR_STOMP && !faceUrgent(R))
    for (int k = 0; k < 4; k++) {
      float t = tt - (0.862f + 0.7f * k);
      if (t >= 0 && t < 0.4f) {
        float env = easeIO(t / 0.04f) * (1 - easeIO(t / 0.4f)) * expf(-7 * t) * animGain();
        U.shakeX += 2.2f * sinf(t * 22) * env;
        U.shakeY += 1.1f * sinf(t * 26) * env;
      }
    }

  U.spawnAcc += dt;
  auto every = [&](float s) {
    if (U.spawnAcc >= s) {
      U.spawnAcc -= s;
      return true;
    }
    return false;
  };
  if (U.linkOK && !faceUrgent(R))
    switch (R.trick) {
    case TR_DANCE:
    case TR_SWAY:
    case TR_HEADBANG:
      if (every((R.trick == TR_SWAY ? 0.8f : 0.45f) / animGain())) {
        spawn(PT_NOTE, frand(20, 300), 250, frand(-10, 10), frand(-90, -60), 3, frand(0.8f, 1.2f),
              hsv(frand(0, 360), 0.6f, 1));
        if (R.trick == TR_DANCE)
          for (int k = 0; k < 2; k++)
            spawn(PT_CONF, frand(0, 320), -8, frand(-30, 30), frand(20, 60), 2.4f,
                  frand(0.7f, 1.1f), hsv(frand(0, 360), 0.85f, 1));
      }
      break;
    case TR_BEG:
    case TR_BOW:
      if (every(0.65f / animGain()))
        spawn(PT_HEART, frand(40, 280), 240, frand(-10, 10), frand(-60, -40), 3, frand(0.8f, 1.2f),
              C_PINK);
      break;
    case TR_PUSHUP:
    case TR_SHIVER:
    case TR_SCARED:
      if (every(0.55f))
        spawn(PT_SWEAT, frand(0, 1) < 0.5f ? 56 : 266, 58, frand(-8, 8), 10, 1.4f, 1.1f, C_ICE);
      break;
    case TR_FLEX:
    case TR_WAVE:
    case TR_POINT:
      if (every(0.5f / animGain()))
        spawn(PT_SPARK, frand(20, 300), frand(30, 220), 0, frand(-12, -4), 1.0f, frand(0.6f, 1.1f),
              R.trick == TR_FLEX ? C_GOLD : C_EYE);
      break;
    case TR_CURIOUS:
      if (every(1.6f))
        spawn(PT_TEXT, 280, 80, 0, -12, 1.5f, 1, C_ACC, '?');
      break;
    case TR_SLEEPY:
      if (tt < 4.0f && every(0.8f))
        spawn(PT_ZZZ, 250, 70, 14, -26, 2.5f, frand(0.9f, 1.4f), C_DIM, 'z');
      if (trickCue(0, 4.05f, tt))
        spawn(PT_TEXT, 285, 82, 0, -20, 1.0f, 1, C_ACC, '!');
      break;
    case TR_POUNCE:
      if (trickCue(0, 2.6f, tt)) {
        spawn(PT_TEXT, 286, 84, 0, -20, 1.0f, 1, C_ACC, '!');
        spawn(PT_RING, 160, 185, 0, 0, 0.6f, 1, C_EYE);
      }
      break;
    default:
      if (R.trick == TR_NONE && R.park == P_PARKED && every(1.3f))
        spawn(PT_ZZZ, 248, 64, 12, -22, 3, frand(0.9f, 1.4f), C_MUTE, 'z');
      if (R.trick == TR_NONE && R.party > 0 && every(0.3f))
        spawn(PT_CONF, frand(0, 320), -8, frand(-30, 30), frand(20, 60), 2.4f, 1,
              hsv(frand(0, 360), 0.85f, 1));
      if (R.trick == TR_NONE && !R.park && R.idleLeft >= 0 && R.idleLeft <= 20 && !R.walking() &&
          every(1.5f))
        spawn(PT_ZZZ, 250, 70, 14, -26, 2.5f, frand(0.8f, 1.2f), C_DIM, 'z');
      {
        int hot = -1;
        for (int i = 0; i < 6; i++)
          for (int j = 0; j < 2; j++)
            hot = R.heat[i][j] > hot ? R.heat[i][j] : hot;
        if (R.trick == TR_NONE && R.walking() && hot > 55 && every(1.3f))
          spawn(PT_SWEAT, frand(0, 1) < 0.5f ? 56 : 266, 58, frand(-8, 8), 10, 1.4f, 1.0f, C_ICE);
      }
      break;
    }
  if (R.trick == TR_STOMP && U.linkOK && !faceUrgent(R))
    for (unsigned k = 0; k < 4; k++) {
      if (trickCue(k, 0.862f + 0.7f * k, tt)) {
        spawn(PT_RING, 160, 185, 0, 0, 0.55f, 1, C_BAD);
        for (int j = 0; j < 2; j++)
          spawn(PT_SPARK, j ? 255 : 65, 175, j ? 25 : -25, -40, 0.7f, 0.7f, C_GOLD);
      }
    }
  if (R.trick == TR_WAVE && U.linkOK && !faceUrgent(R) && trickCue(0, 0.65f, tt))
    faceBurst(PT_SPARK, C_EYE, 3);
  if (R.trick == TR_FLEX && U.linkOK && !faceUrgent(R) && trickCue(0, 0.35f, tt))
    faceBurst(PT_SPARK, C_GOLD, 5);
  A.prevTrickTime = tt;
  U.spawnAcc = mn(U.spawnAcc, 1.5f);
  particlesUpdate(dt);
}
