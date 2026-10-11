// Dashboard rendering and frame dispatch. Both cores read the same frame state.

#pragma once
#include "face.h"
#include "drive_state.h"

static Robot RS;
static uint32_t bootMs = 0;
static float beetleUpS = 0;
static const char *const PAGE_TITLE[PG_COUNT] = {"FACE",       "STATUS",      "LEGS",  "LEVEL",
                                                 "SYSTEM",     "OFF",         "TRAIL", "STABILITY",
                                                 "SERVO LOAD", "WALK CONTROL"};

static void bgPage() {
  rectV(0, 0, W, H, C_BG1, C_BG0);
}

static void card(float x, float y, float w, float h, const char *label = nullptr) {
  rrect(x, y, w, h, 10, C_CARD);
  rrectLine(x + 0.5f, y + 0.5f, w - 1, h - 1, 10, 1, C_CARD2);
  if (label)
    text(F_TINY, label, x + 10, y + 15, C_DIM);
}

static void btIcon(float x, float y, uint16_t c) {
  line(x - 4, y - 3.5f, x + 4, y + 3.5f, 1.7f, c);
  line(x + 4, y + 3.5f, x, y + 7, 1.7f, c);
  line(x, y + 7, x, y - 7, 1.7f, c);
  line(x, y - 7, x + 4, y - 3.5f, 1.7f, c);
  line(x + 4, y - 3.5f, x - 4, y + 3.5f, 1.7f, c);
}

static uint16_t batColour(int p) {
  return p > 50 ? C_OK : p > 20 ? C_WARN : C_BAD;
}

static void batteryPill(float x, float y, const Robot &R) {
  float w = 28, h = 13, bx = x - w - 3;
  rrectLine(bx, y, w, h, 3.5f, 1.4f, C_DIM);
  rrect(bx + w + 0.5f, y + 4, 2.5f, 5, 1, C_DIM);
  if (R.onBatt()) {
    int p = R.pct();
    rrect(bx + 2.5f, y + 2.5f, fmaxf(2, (w - 5) * p / 100.0f), h - 5, 2, batColour(p));
    textf(F_TINY, bx - 5, y + 10.5f, C_TXT, AL_R, "%d%%", p);
  } else
    text(F_TINY, "USB", bx - 5, y + 10.5f, C_DIM, AL_R);
}

static void pageDots(int cur) {
  int c = cyclePos(cur);
  for (int k = 0; k < 9; k++)
    disc(160 - 48 + k * 12, 10, k == c ? 3.2f : 2.2f, k == c ? C_ACC : C_MUTE);
}

static void header(const char *title, const Robot &R) {
  rrect(10, 7, 3.5f, 14, 1.5f, C_ACC);
  text(F_LABEL, title, 20, 19.5f, C_TXT);
  batteryPill(W - 8, 7, R);
  btIcon(W - 82, 13.5f, R.btClient() ? C_EYE : C_MUTE);
}

static void chip(float x, float y, float w, const char *label, const char *val, uint16_t dot) {
  rrect(x, y, w, 26, 8, C_CARD);
  disc(x + 12, y + 13, 4, dot);
  glow(x + 12, y + 13, 9, dot, 0.35f);
  text(F_TINY, label, x + 22, y + 17, C_DIM);
  text(F_TINY, val, x + w - 9, y + 17, C_TXT, AL_R);
}

static void bar(float x, float y, float w, float h, float frac, uint16_t c, uint16_t bg = C_CARD2) {
  rrect(x, y, w, h, h / 2, bg);
  if (frac > 0.01f)
    rrect(x, y, fmaxf(h, w * clampf(frac, 0, 1)), h, h / 2, c);
}

static const char *stateName(const Robot &R) {
  if (R.batLocked())
    return "LOCKED";
  if (R.balanceHold())
    return "BALANCE HOLD";
  if (R.park == P_LOWERING)
    return "LYING DOWN";
  if (R.park == P_PARKED)
    return "PARKED";
  if (!R.servosOn())
    return "RELAXED";
  if (R.trick)
    return "TRICK";
  if (R.walking())
    return "WALKING";
  if (R.sitting())
    return "SITTING";
  return "STANDING";
}

static uint16_t FACE_BG[240];

static void pageFace(const Robot &R) {
  rectV(0, 0, W, H, C_BG0, RGB(6, 8, 12));

  if (R.party > 0 || R.trick == TR_DANCE) {
    for (int k = 0; k < 3; k++) {
      float a = U.t * (0.9f + 0.3f * k) + k * 2.1f;
      glow(160 + 120 * cosf(a), 120 + 70 * sinf(a * 1.3f), 76, hsv(U.t * 60 + k * 120, 0.8f, 1),
           0.32f);
    }
  }
  drawEyes();
  faceDetailsDraw();
  particlesDraw();

  float sa = 0.85f;
  char b[32];
  if (A.demo)
    fmt(b, sizeof b, "DEMO  %s",
        R.trick       ? TRICK_TITLE[R.trick]
        : R.walking() ? GAIT_NAME[R.gait]
                      : "IDLE");
  else if (R.walking())
    fmt(b, sizeof b, "%s  %d", GAIT_NAME[R.gait], (int)R.speed);
  else if (R.trick)
    fmt(b, sizeof b, "TRICK  %s", TRICK_TITLE[R.trick]);
  else
    fmt(b, sizeof b, "%s", stateName(R));
  text(F_TINY, b, 10, 17, R.walking() ? C_OK : R.trick ? C_ACC : C_MUTE, AL_L, sa);
  batteryPill(W - 8, 7, R);
  btIcon(W - 82, 13.5f, R.btClient() ? C_EYE : C_MUTE);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_FACE);
  if (R.party > 0) {
    fmt(b, sizeof b, "PARTY x%d", R.party);
    int tw = textW(F_LABEL, b);
    float px = 160 - tw / 2 - 12;
    rrect(px, 26, tw + 24, 22, 11, hsv(U.t * 90, 0.7f, 0.9f));
    text(F_LABEL, b, 160, 42, C_BG0, AL_C);
  }

  float bt = U.t - U.bannerT0;
  if (U.bannerTrick && bt < 2.0f) {
    float in = easeIO(bt / 0.25f), out = easeIO((2.0f - bt) / 0.3f), k = fminf(in, out);
    const char *s = TRICK_TITLE[U.bannerTrick];
    int tw = textW(F_MED, s);
    float y = 240 - 40 * k, x = 160 - tw / 2 - 16;
    rrect(x, y, tw + 32, 30, 15, C_CARD, 0.92f);
    rrectLine(x + 0.5f, y + 0.5f, tw + 31, 29, 15, 1.5f, C_ACC);
    text(F_MED, s, 160, y + 22, C_ACC, AL_C);
  } else {
    float ct = U.t - U.capT0;
    if (ct < U.capDur) {
      float a = fminf(easeIO(ct / 0.2f), easeIO((U.capDur - ct) / 0.4f));
      text(F_LABEL, U.cap, 160, 228 + 6 * (1 - a), U.capCol, AL_C, a);
    }
  }
}

static void pageDash(const Robot &R) {
  bgPage();
  header("STATUS", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_DASH);

  card(8, 30, 132, 134);
  float cx = 74, cy = 99;
  arc(cx, cy, 46, 9, -135, 135, C_CARD2);
  if (R.onBatt()) {
    int p = R.pct();
    float a1 = -135 + 270 * p / 100.0f;
    uint16_t c = batColour(p);
    glow(cx + 46 * sinf(a1 * 0.01745f), cy - 46 * cosf(a1 * 0.01745f), 14, c, 0.5f);
    arc(cx, cy, 46, 9, -135, a1, c);
    char b[8];
    fmt(b, sizeof b, "%d", p);
    int e = text(F_BIG, b, cx - 4, cy + 9, C_TXT, AL_C);
    text(F_LABEL, "%", e + 1 - 4, cy + 9, C_DIM);
    textf(F_SMALL, cx, cy + 28, C_DIM, AL_C, "%.2f V", R.batV);
    char bl[24];
    batLeftText(bl, sizeof bl);
    if (bl[0])
      text(F_TINY, bl, cx, 156,
           NV.minutesLeft < 10   ? C_BAD
           : NV.minutesLeft < 25 ? C_WARN
                                 : C_DIM,
           AL_C);
    else
      text(F_TINY, R.walking() || R.servosOn() ? "BATTERY  (estimating)" : "BATTERY", cx, 156,
           C_MUTE, AL_C);
  } else {
    text(F_MED, "USB", cx, cy + 8, C_DIM, AL_C);
    text(F_TINY, "no battery", cx, cy + 26, C_MUTE, AL_C);
  }

  card(146, 30, 166, 64, "MOTION");
  const char *st = stateName(R);
  uint16_t sc = R.walking() ? C_OK : R.trick ? C_ACC : (R.park || !R.servosOn()) ? C_DIM : C_TXT;
  text(F_MED, st, 156, 70, sc);
  if (R.walking()) {
    float sw = R.swapped() ? -1 : 1, vx = R.vx * sw, vy = R.vy * sw;
    float ang = (fabsf(vx) + fabsf(vy) > 1) ? atan2f(-vy, vx) : 0, ax = 290, ay = 60;
    if (fabsf(R.wz) > 0.01f && fabsf(vx) + fabsf(vy) < 1) {
      arc(ax, ay, 11, 3, R.wz > 0 ? -150 : -30, R.wz > 0 ? 30 : 150, C_OK);
    } else {
      float fx = -sinf(ang), fy = -cosf(ang);
      line(ax - fx * 11, ay - fy * 11, ax + fx * 7, ay + fy * 7, 3.2f, C_OK);
      tri(ax + fx * 15, ay + fy * 15, ax + fx * 5 - fy * 7, ay + fy * 5 + fx * 7,
          ax + fx * 5 + fy * 7, ay + fy * 5 - fx * 7, C_OK);
    }
  }
  float maxV = 50.0f / ((R.gait == 0   ? 0.5f
                         : R.gait == 1 ? 0.667f
                                       : 0.833f) *
                        (R.period > 0.1f ? R.period : 1));
  bar(156, 80, 146, 5, R.speed / maxV, R.walking() ? C_OK : C_MUTE);

  card(146, 100, 166, 64);
  const char *lab[3] = {"GAIT", "SPEED", "HEIGHT"};
  for (int k = 0; k < 3; k++) {
    float x = 156 + k * 52;
    text(F_TINY, lab[k], x, 117, C_DIM);
    if (k)
      rect(x - 7, 110, 1, 44, C_CARD2);
  }
  text(F_LABEL, GAIT_NAME[R.gait], 156, 145, C_TXT);
  textf(F_MED, 208, 145, C_TXT, AL_L, "%d", (int)R.speed);
  text(F_TINY, "mm/s", 208, 157, C_MUTE);
  textf(F_MED, 260, 145, C_TXT, AL_L, "%d", (int)(R.sitting() ? 40 : R.targetH));
  text(F_TINY, "mm", 260, 157, C_MUTE);

  char b[16];
  chip(8, 172, 98, "SERVOS", R.servosOn() ? "ON" : "OFF", R.servosOn() ? C_OK : C_MUTE);
  chip(112, 172, 98, "IMU", R.imuOK() ? (R.hasFront() ? "OK" : "SET UP") : "NONE",
       R.imuOK() ? (R.hasFront() ? C_OK : C_WARN) : C_MUTE);
  chip(216, 172, 96, "LEVEL", R.levelOn() ? "ON" : "OFF", R.levelOn() ? C_EYE : C_MUTE);
  chip(8, 204, 98, "ENDS", R.swapped() ? "SWAP" : "NORM", R.swapped() ? C_ACC : C_EYE);
  chip(112, 204, 98, "BT", R.btClient() ? "LINKED" : "READY", R.btClient() ? C_EYE : C_MUTE);
  fmt(b, sizeof b, "%lum", (unsigned long)(R.upS / 60));
  chip(216, 204, 96, "UP", b, C_DIM);
}

struct V3 {
  float x, y, z;
};

struct Cam {
  float cy, sy, ce, se, k, cx, cyy;
};

static inline void proj(const Cam &c, V3 p, float &sx, float &sy, float &depth) {
  float u = p.x * c.cy + p.y * c.sy, v = -p.x * c.sy + p.y * c.cy;
  depth = v * c.ce + p.z * c.se;
  float s = c.k / fmaxf(1 + depth * 0.0011f, 0.2f);
  sx = c.cx + u * s;
  sy = c.cyy - (p.z * c.ce - v * c.se) * s;
}

static uint16_t heatColour(int pct, uint16_t cool) {
  if (pct < 0 || pct <= 35)
    return cool;
  if (pct <= 55)
    return mix(cool, RGB(255, 184, 48), (pct - 35) / 20.0f);
  return mix(RGB(255, 184, 48), RGB(255, 70, 60), clampf((pct - 55) / 25.0f, 0, 1));
}

static void pageLegs(const Robot &R) {
  bgPage();
  header("LEGS", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_LEGS);
  text(F_TINY, R.walking() ? GAIT_NAME[R.gait] : stateName(R), W - 100, 19,
       R.walking() ? C_OK : C_DIM, AL_R);
  float phi = 0.62f + 0.42f * sinf(U.t * 0.33f), el = 0.5f;
  Cam cam{cosf(phi), sinf(phi), cosf(el), sinf(el), 0.54f, 160, 128};
  bool gm = R.gaitMode();
  float H = gm ? R.bodyH : PARK_H;
  float sw = R.swapped() ? -1 : 1;
  const float LX = -0.42f, LY = 0.38f, LZ = 0.82f;

  for (int g = -150; g <= 150; g += 50) {
    float a0, b0, a1, b1, d;
    proj(cam, {(float)g, -150, 0}, a0, b0, d);
    proj(cam, {(float)g, 150, 0}, a1, b1, d);
    lineThin(a0, b0, a1, b1, C_LINE, 0.7f);
    proj(cam, {-150, (float)g, 0}, a0, b0, d);
    proj(cam, {150, (float)g, 0}, a1, b1, d);
    lineThin(a0, b0, a1, b1, C_LINE, 0.7f);
  }

  float tx = sw * R.pTx, ty = sw * R.pTy, tz = R.pTz, ro = sw * R.pRoll * 0.01745f,
        pi = sw * R.pPitch * 0.01745f, ya = R.yaw * 0.01745f;
  float cr = cosf(ro), sr = sinf(ro), cp = cosf(pi), sp = sinf(pi), cyw = cosf(ya), syw = sinf(ya);
  auto rot = [&](V3 b) -> V3 {
    float y1 = b.y * cr - b.z * sr, z1 = b.y * sr + b.z * cr;
    float x2 = b.x * cp - z1 * sp, z2 = b.x * sp + z1 * cp;
    return {x2 * cyw - y1 * syw, x2 * syw + y1 * cyw, z2};
  };
  auto toWorld = [&](V3 b) -> V3 {
    V3 r = rot(b);
    return {r.x + tx, r.y + ty, r.z + tz + H};
  };
  auto toBody = [&](V3 w) -> V3 {
    float x = w.x - tx, y = w.y - ty, z = w.z - tz - H;
    float x1 = x * cyw + y * syw, y1 = -x * syw + y * cyw;
    float x2 = x1 * cp + z * sp, z2 = -x1 * sp + z * cp;
    return {x2, y1 * cr + z2 * sr, -y1 * sr + z2 * cr};
  };
  auto shadowPt = [&](V3 p, float &sx, float &sy) {
    float d;
    proj(cam, {p.x - LX * p.z / LZ, p.y - LY * p.z / LZ, 0}, sx, sy, d);
  };

  V3 P[6][4];
  bool air[6];
  for (int i = 0; i < 6; i++) {
    int pl = R.swapped() ? 5 - i : i;
    V3 foot;
    if (gm)
      foot = {sw * R.feet[pl][0], sw * R.feet[pl][1], R.feet[pl][2] + H};
    else {
      float nx, ny;
      neutralFoot(pl, R.width, nx, ny);
      foot = {sw * nx, sw * ny, 0};
    }
    V3 hipB = {sw * LEG[pl].x, sw * LEG[pl].y, COXA_Z};
    V3 fb = toBody(foot);
    float dx = fb.x - hipB.x, dy = fb.y - hipB.y, L = sqrtf(dx * dx + dy * dy);
    if (L < 1)
      L = 1;
    float ux = dx / L, uy = dy / L;
    V3 piv = {hipB.x + ux * HIP_OUT, hipB.y + uy * HIP_OUT, hipB.z + HIP_UP};
    float rr = (fb.x - piv.x) * ux + (fb.y - piv.y) * uy, zz = fb.z - piv.z,
          d = sqrtf(rr * rr + zz * zz);
    float ca = clampf((FEMUR * FEMUR + d * d - TIBIA * TIBIA) / (2 * FEMUR * mx(d, 1)), -1, 1);
    float a = atan2f(zz, rr) + acosf(ca);
    V3 knee = {piv.x + ux * FEMUR * cosf(a), piv.y + uy * FEMUR * cosf(a), piv.z + FEMUR * sinf(a)};
    P[i][0] = toWorld(hipB);
    P[i][1] = toWorld(piv);
    P[i][2] = toWorld(knee);
    P[i][3] = foot;
    air[i] = gm && ((R.swing >> pl) & 1 || foot.z > 4);
  }

  const int ord[6] = {0, 3, 4, 5, 2, 1};
  V3 top[6], bot[6];
  float hx[6], hy[6], gx[6], gy[6], d;
  for (int n = 0; n < 6; n++) {
    int pl = R.swapped() ? 5 - ord[n] : ord[n];
    top[n] = toWorld({sw * LEG[pl].x * 1.2f, sw * LEG[pl].y * 1.2f, COXA_Z + 9});
    bot[n] = toWorld({sw * LEG[pl].x * 1.2f, sw * LEG[pl].y * 1.2f, 2});
    proj(cam, top[n], hx[n], hy[n], d);
    proj(cam, bot[n], gx[n], gy[n], d);
  }

  {
    float sx[6], sy[6], cx0 = 0, cy0 = 0;
    for (int n = 0; n < 6; n++) {
      shadowPt(bot[n], sx[n], sy[n]);
      cx0 += sx[n] / 6;
      cy0 += sy[n] / 6;
    }
    for (int n = 0; n < 6; n++)
      tri(cx0, cy0, sx[n], sy[n], sx[(n + 1) % 6], sy[(n + 1) % 6], RGB(0, 0, 0), 0.38f);
  }
  for (int i = 0; i < 6; i++) {
    float ax, ay, bx, by, cx2, cy2;
    shadowPt(P[i][1], ax, ay);
    shadowPt(P[i][2], bx, by);
    shadowPt(P[i][3], cx2, cy2);
    line(ax, ay, bx, by, 5, RGB(0, 0, 0), 0.28f);
    line(bx, by, cx2, cy2, 4, RGB(0, 0, 0), 0.28f);
  }

  for (int i = 0; i < 6; i++) {
    float sx, sy;
    proj(cam, {P[i][3].x, P[i][3].y, 0}, sx, sy, d);
    float tt = U.t - U.touchT[i];
    if (tt < 0.5f) {
      float k = tt / 0.5f;
      rrectLine(sx - 8 - 12 * k, sy - 3.5f - 5 * k, 16 + 24 * k, 7 + 10 * k, 4 + 6 * k, 1.5f, C_EYE,
                1 - k);
    }
    if (!air[i])
      rrect(sx - 7, sy - 3, 14, 6, 3, C_EYE, 0.45f);
  }

  struct Item {
    float depth;
    int8_t kind, leg, seg;
  };

  Item items[6 * 5 + 1];
  int ni = 0;
  {
    float dd;
    proj(cam, toWorld({0, 0, COXA_Z}), d, d, dd);
    items[ni++] = {dd, 0, 0, 0};
  }
  for (int i = 0; i < 6; i++) {
    for (int sgm = 0; sgm < 3; sgm++) {
      V3 a = P[i][sgm], b = P[i][sgm + 1];
      float dd;
      proj(cam, {(a.x + b.x) / 2, (a.y + b.y) / 2, (a.z + b.z) / 2}, d, d, dd);
      items[ni++] = {dd + (sgm == 0 ? 6.0f : 0.0f), 1, (int8_t)i, (int8_t)sgm};
    }
    float d1, d2;
    proj(cam, P[i][1], d, d, d1);
    proj(cam, P[i][2], d, d, d2);
    items[ni++] = {d1 - 0.5f, 2, (int8_t)i, 1};
    items[ni++] = {d2 - 0.5f, 2, (int8_t)i, 2};
  }
  for (int a = 1; a < ni; a++) {
    Item it = items[a];
    int b = a - 1;
    while (b >= 0 && items[b].depth < it.depth) {
      items[b + 1] = items[b];
      b--;
    }
    items[b + 1] = it;
  }

  auto limb = [&](V3 a, V3 b, float wmm, uint16_t base) {
    float x0, y0, x1, y1, d0, d1;
    proj(cam, a, x0, y0, d0);
    proj(cam, b, x1, y1, d1);
    float dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z,
          L = sqrtf(dx * dx + dy * dy + dz * dz) + 1e-3f;
    float dl = (dx * LX + dy * LY + dz * LZ) / L;
    float lit = 0.45f + 0.55f * sqrtf(mx(0, 1 - dl * dl));
    float fog = clampf(1.05f - (d0 + d1) * 0.5f * 0.0035f, 0.55f, 1.1f);
    float s = cam.k / (1 + (d0 + d1) * 0.5f * 0.0011f), w = wmm * s;
    uint16_t c = scale(base, clampf(lit * fog, 0, 1));
    line(x0, y0, x1, y1, w + 2.2f, scale(base, 0.22f * fog));
    line(x0, y0, x1, y1, w, c);
    float ex = x1 - x0, ey = y1 - y0, el = sqrtf(ex * ex + ey * ey) + 1e-3f, nx = -ey / el,
          ny = ex / el;
    if (nx * -0.7f + ny * -0.7f < 0) {
      nx = -nx;
      ny = -ny;
    }
    float o = w * 0.22f;
    line(x0 + nx * o, y0 + ny * o, x1 + nx * o, y1 + ny * o, mx(1.2f, w * 0.28f),
         mix(c, RGB(255, 255, 255), 0.45f), 0.8f);
  };
  auto sphere = [&](V3 p, float rmm, uint16_t base) {
    float x, y, dd;
    proj(cam, p, x, y, dd);
    float s = cam.k / (1 + dd * 0.0011f), r = rmm * s;
    float fog = clampf(1.05f - dd * 0.0035f, 0.55f, 1.1f);
    disc(x, y, r + 1, scale(base, 0.25f * fog));
    disc(x, y, r, scale(base, 0.7f * fog));
    disc(x - r * 0.25f, y - r * 0.25f, r * 0.62f, scale(base, clampf(fog, 0, 1)));
    disc(x - r * 0.38f, y - r * 0.4f, r * 0.24f, RGB(255, 255, 255), 0.75f);
  };
  auto body = [&]() {
    V3 nUp = rot({0, 0, 1});
    for (int n = 0; n < 6; n++) {
      int m = (n + 1) % 6;
      float area = (gx[m] - gx[n]) * (hy[n] - gy[n]) - (hx[n] - gx[n]) * (gy[m] - gy[n]);
      if (area >= 0)
        continue;
      float ex = top[m].x - top[n].x, ey = top[m].y - top[n].y,
            el = sqrtf(ex * ex + ey * ey) + 1e-3f;
      float nx = ey / el, ny = -ex / el;
      float lit = clampf(0.35f + 0.65f * mx(0, nx * LX + ny * LY + 0.25f), 0.3f, 1);
      uint16_t c = scale(RGB(30, 110, 150), lit);
      tri(gx[n], gy[n], gx[m], gy[m], hx[m], hy[m], c);
      tri(gx[n], gy[n], hx[m], hy[m], hx[n], hy[n], c);
    }
    float lt = clampf(0.55f + 0.45f * (nUp.x * LX + nUp.y * LY + nUp.z * LZ), 0.4f, 1);
    float cxs = 0, cys = 0;
    for (int n = 0; n < 6; n++) {
      cxs += hx[n] / 6;
      cys += hy[n] / 6;
    }
    for (int n = 0; n < 6; n++) {
      int m = (n + 1) % 6;
      tri(cxs, cys, hx[n], hy[n], hx[m], hy[m], scale(RGB(52, 160, 205), lt));
    }
    for (int n = 0; n < 6; n++) {
      int m = (n + 1) % 6;
      tri(cxs, cys, cxs + (hx[n] - cxs) * 0.62f, cys + (hy[n] - cys) * 0.62f,
          cxs + (hx[m] - cxs) * 0.62f, cys + (hy[m] - cys) * 0.62f, scale(RGB(70, 185, 230), lt));
      line(hx[n], hy[n], hx[m], hy[m], 1.4f, mix(C_EYE, RGB(255, 255, 255), 0.25f), 0.9f);
    }
    float fx, fy, f2x, f2y;
    proj(cam, toWorld({60, 0, COXA_Z + 9.5f}), fx, fy, d);
    proj(cam, toWorld({90, 0, COXA_Z + 9.5f}), f2x, f2y, d);
    line(fx, fy, f2x, f2y, 4, C_ACC);
    disc(f2x, f2y, 4, C_ACC);
  };
  for (int k = 0; k < ni; k++) {
    const Item &it = items[k];
    if (it.kind == 0)
      body();
    else if (it.kind == 1) {
      int pl = R.swapped() ? 5 - it.leg : it.leg;
      uint16_t base =
          it.seg == 0 ? RGB(110, 120, 138) : heatColour(R.heat[pl][it.seg - 1], RGB(150, 165, 185));
      if (air[it.leg] && it.seg > 0)
        base = mix(base, RGB(90, 230, 150), 0.6f);
      const float wmm[3] = {13, 12, 9};
      limb(P[it.leg][it.seg], P[it.leg][it.seg + 1], wmm[it.seg], base);
      if (it.seg == 2)
        sphere(P[it.leg][3], 6, air[it.leg] ? C_OK : C_EYE);
    } else
      sphere(P[it.leg][it.seg], 8, RGB(200, 210, 225));
  }

  {
    int best = -1, bi = 0, bj = 0;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 2; j++)
        if (R.heat[i][j] > best) {
          best = R.heat[i][j];
          bi = i;
          bj = j;
        }
    if (best >= 0) {
      int named = R.swapped() ? 5 - bi : bi;
      textf(F_TINY, 20, 38, heatColour(best, C_DIM), AL_L, "LOAD %s %s %d%%", LEG[named].name,
            bj ? "tibia" : "femur", best);
    }
  }

  const char *nm[6] = {"FL", "ML", "RL", "FR", "MR", "RR"};
  float gy0 = 174;
  rrect(6, gy0 - 5, 308, 68, 9, C_CARD);
  for (int i = 0; i < 6; i++) {
    float y = gy0 + i * 10;
    int pl = R.swapped() ? 5 - i : i;
    text(F_TINY, nm[i], 14, y + 8, C_MUTE);
    rect(36, y + 1, 270, 5, C_CARD2);
    int runStart = -1;
    for (int k = 0; k <= 128; k++) {
      bool s = false;
      if (k < 128) {
        int idx = (U.histHead + k) & 127;
        s = (U.swingHist[idx] >> pl) & 1;
      }
      if (s && runStart < 0)
        runStart = k;
      if (!s && runStart >= 0) {
        rrect(36 + runStart * 2.11f, y + 0.5f, (k - runStart) * 2.11f, 6, 3, C_OK);
        runStart = -1;
      }
    }
  }
}

static void attitudeBall(float cx, float cy, float r, float pitchDeg, float rollDeg) {
  Target &t = tg();
  float ox = t.ox;
  float ro = rollDeg * 0.01745f, cr = cosf(ro), sr = sinf(ro), off = pitchDeg * 2.6f;
  const uint16_t SKY = RGB(46, 132, 210), GND = RGB(112, 72, 38);
  int ya = (int)floorf(cy - r - 1), yb = (int)ceilf(cy + r + 1);
  if (ya < t.y0)
    ya = t.y0;
  if (yb > t.y1 - 1)
    yb = t.y1 - 1;
  for (int yy = ya; yy <= yb; yy++) {
    float py = yy + 0.5f - cy;
    float w2 = (r + 0.6f) * (r + 0.6f) - py * py;
    if (w2 <= 0)
      continue;
    float wo = sqrtf(w2), wi2 = (r - 0.6f) * (r - 0.6f) - py * py, wi = wi2 > 0 ? sqrtf(wi2) : 0;
    int xa = (int)floorf(cx + ox - wo), xb = (int)ceilf(cx + ox + wo);
    if (xa < 0)
      xa = 0;
    if (xb > W - 1)
      xb = W - 1;

    float base = py * cr - off;
    for (int xx = xa; xx <= xb; xx++) {
      float px = xx + 0.5f - ox - cx;
      uint32_t ea = 32;
      if (fabsf(px) >= wi) {
        ea = cov(sqrtf(px * px + py * py) - r, 1);
        if (!ea)
          continue;
      }
      float ly = px * sr + base;
      uint16_t col = ly < -0.5f ? SKY : ly > 0.5f ? GND : mix(GND, SKY, 0.5f - ly);
      put(xx, yy, col, ea);
    }
  }

  for (int p = -30; p <= 30; p += 10) {
    if (!p)
      continue;
    float y = off - p * 2.6f, hw = (p % 20 == 0) ? 26 : 16;
    float x0 = -hw, x1 = hw;

    float ax = x0 * cr + y * sr, ay = -x0 * sr + y * cr, bx = x1 * cr + y * sr,
          by = -x1 * sr + y * cr;
    if (fabsf(y) > r * 0.8f)
      continue;
    lineThin(cx + ax, cy + ay, cx + bx, cy + by, RGB(255, 255, 255), 0.9f);
    char b[4];
    fmt(b, sizeof b, "%d", p > 0 ? p : -p);
    float lx = (x1 + 10) * cr + y * sr, lyy = -(x1 + 10) * sr + y * cr;
    text(F_TINY, b, cx + lx, cy + lyy + 4, RGB(255, 255, 255), AL_C, 0.8f);
  }
  ring(cx, cy, r + 2, 4, C_CARD2);

  const int ticks[] = {-60, -45, -30, -20, -10, 0, 10, 20, 30, 45, 60};
  for (int a : ticks) {
    float aa = a * 0.01745f, r0 = r + 6, r1 = r + ((a % 30 == 0) ? 15 : 11);
    line(cx + r0 * sinf(aa), cy - r0 * cosf(aa), cx + r1 * sinf(aa), cy - r1 * cosf(aa), a ? 2 : 3,
         a ? C_DIM : C_TXT);
  }
  {
    float aa = -ro, r0 = r - 6;
    float px = cx + r0 * sinf(aa), py = cy - r0 * cosf(aa), nx = sinf(aa), ny = -cosf(aa);
    tri(px, py, px - nx * 12 - ny * 7, py - ny * 12 + nx * 7, px - nx * 12 + ny * 7,
        py - ny * 12 - nx * 7, C_ACC);
  }

  line(cx - 46, cy, cx - 16, cy, 4, C_ACC);
  line(cx - 16, cy, cx - 9, cy + 7, 4, C_ACC);
  line(cx + 46, cy, cx + 16, cy, 4, C_ACC);
  line(cx + 16, cy, cx + 9, cy + 7, 4, C_ACC);
  disc(cx, cy, 3.5f, C_ACC);
}

static void sparkline(float x, float y, float w, float h, const float *v, int head, float range,
                      uint16_t c) {
  float px = 0, py = 0;
  for (int k = 0; k < 100; k++) {
    float val = v[(head + k) % 100], sx = x + k * w / 99,
          sy = y + h / 2 - clampf(val / range, -1, 1) * h / 2;
    if (k)
      lineThin(px, py, sx, sy, c);
    px = sx;
    py = sy;
  }
}

static void pageLevel(const Robot &R) {
  bgPage();
  header("LEVEL", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_LEVEL);
  float vp = R.swapped() ? -R.pitch : R.pitch, vr = R.swapped() ? -R.roll : R.roll;
  if (!R.imuOK() || !R.hasFront()) {
    attitudeBall(104, 124, 74, 0, 0);
    rrect(20, 106, 168, 36, 10, C_CARD, 0.92f);
    text(F_LABEL, R.imuOK() ? "RUN levelcal + mpufront" : "NO MPU6050", 104, 129, C_WARN, AL_C);
  } else
    attitudeBall(104, 124, 74, vp, vr);
  card(204, 34, 108, 56, "PITCH");
  textf(F_BIG, 302, 80, C_TXT, AL_R, "%+.1f", vp);
  card(204, 96, 108, 56, "ROLL");
  textf(F_BIG, 302, 142, C_TXT, AL_R, "%+.1f", vr);
  card(204, 158, 108, 74, "HISTORY");
  rect(212, 194, 92, 1, C_CARD2);
  sparkline(212, 174, 92, 40, U.pitchHist, U.attHead, 20, C_EYE);
  sparkline(212, 174, 92, 40, U.rollHist, U.attHead, 20, C_ACC);
  text(F_TINY, "pitch", 212, 228, C_EYE);
  text(F_TINY, "roll", 304, 228, C_ACC, AL_R);
  rrect(20, 222, 168, 14, 7, R.levelOn() ? C_EYE : C_CARD2, R.levelOn() ? 0.25f : 1);
  text(F_TINY, R.levelOn() ? "AUTO LEVEL ON" : "AUTO LEVEL OFF", 104, 232.5f,
       R.levelOn() ? C_EYE : C_DIM, AL_C);
}

static void pageSys(const Robot &R) {
  bgPage();
  header("SYSTEM", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_SYS);
  card(8, 30, 150, 100, "SCREEN");
  textf(F_BIG, 18, 74, C_TXT, AL_L, "%d", (int)(U.fps + 0.5f));
  text(F_LABEL, "FPS", 66, 74, C_DIM);
  textf(F_TINY, 148, 74, C_MUTE, AL_R, "%.1f ms", U.frameMs);
  text(F_TINY, "CORE 0", 18, 98, C_DIM);
  bar(62, 92, 52, 6, U.c0ms / 16.7f, C_EYE);
  textf(F_TINY, 148, 98, C_TXT, AL_R, "%.1f", U.c0ms);
  text(F_TINY, "CORE 1", 18, 118, C_DIM);
  bar(62, 112, 52, 6, U.c1ms / 16.7f, C_ACC);
  textf(F_TINY, 148, 118, C_TXT, AL_R, "%.1f", U.c1ms);
  card(164, 30, 148, 100, "ESP32 LINK");
  textf(F_BIG, 174, 74, U.linkOK ? C_TXT : C_BAD, AL_L, "%d", (int)(U.linkHz + 0.5f));
  text(F_LABEL, "/s", 174 + 4 + textW(F_BIG, "30"), 74, C_DIM);
  textf(F_TINY, 174, 96, C_DIM, AL_L, "status v%d  230400 baud", R.ver);
  textf(F_TINY, 174, 114, U.linkErr ? C_WARN : C_DIM, AL_L, "bad lines %lu",
        (unsigned long)U.linkErr);
  card(8, 136, 304, 96, "INFO");

  struct {
    const char *k;
    char v[24];
  } row[6];

  row[0].k = "Robot up";
  fmt(row[0].v, 24, "%lu min", (unsigned long)(R.upS / 60));
  row[1].k = "Screen up";
  fmt(row[1].v, 24, "%lu min", (unsigned long)(beetleUpS / 60));
  row[2].k = "Chip temp";
  fmt(row[2].v, 24, "%.1f C", U.tempC);
  row[3].k = "SPI clock";
  fmt(row[3].v, 24, "%.1f MHz", U.spiMHz);
  row[4].k = "Core split";
  fmt(row[4].v, 24, "row %d", U.split);
  row[5].k = "Version";
  fmt(row[5].v, 24, "face 3.6");
  for (int k = 0; k < 6; k++) {
    float x = 18 + (k % 2) * 150, y = 172 + (k / 2) * 20;
    text(F_SMALL, row[k].k, x, y, C_DIM);
    text(F_SMALL, row[k].v, x + 136, y, C_TXT, AL_R);
  }
}

static void drawBoot(float t) {
  clear(RGB(4, 6, 9));
  float o = easeIO((t - 0.25f) / 0.45f);
  if (t > 1.25f && t < 1.4f)
    o = 0.08f;
  for (int k = 0; k < 2; k++) {
    EyeP e;
    e.h = 4 + 88 * o;
    e.glow = 0.5f * o;
    float bounce = t > 0.55f ? sinf((t - 0.55f) * 15) * expf(-(t - 0.55f) * 5) * 7 : 0;
    float gaze = 8 * pulse(t, 0.8f, 1.2f) - 6 * pulse(t, 1.45f, 1.9f);
    float cx = EYE_X[k] + gaze, cy = EYE_Y - 8 - bounce;
    e.sparkle = pulse(t, 1.5f, 2.4f);
    e.happy = pulse(t, 1.65f, 2.4f) * 0.45f;
    glowBox(cx - 35, cy - e.h / 2, 70, e.h, 26, 22, C_EYE, e.glow);
    eyeNormal(cx, cy, e, k, 0);
  }
  if (t > 1.0f) {
    float f = easeIO((t - 1.0f) / 0.6f);
    text(F_BIG, "HEXAPOD", 160, 214, mix(RGB(4, 6, 9), C_TXT, f), AL_C);
    text(F_TINY, "dual-core face v3.8", 160, 230, mix(RGB(4, 6, 9), C_MUTE, f), AL_C);
  }
  if (t < 1.0f) {
    float y = 240 * t;
    rect(0, y, W, 2, C_EYE, 0.25f * (1 - t));
  }
}

static void drawWaiting(const Robot &R) {
  (void)R;
  rectV(0, 0, W, H, C_BG0, RGB(6, 8, 12));
  float scan = sinf(U.t * 0.8f) * 10;
  float peek = pulse(fmodf(U.t, 7), 4.6f, 6.4f);
  for (int k = 0; k < 2; k++) {
    EyeP e;
    e.h = 8 + (k ? 25 : 14) * peek;
    e.w = 65;
    e.col = mix(C_MUTE, C_EYE, peek * 0.4f);
    eyeNormal(EYE_X[k] + scan, EYE_Y - peek * 2, e, k, 0);
  }
  for (int k = 0; k < 8; k++) {
    float a = k * 0.785f + U.t * 3.5f;
    disc(160 + 16 * cosf(a), 186 + 16 * sinf(a), 2.4f + (k == 0 ? 1 : 0), k == 0 ? C_ACC : C_LINE);
  }
  text(F_SMALL, "waiting for the robot...", 160, 226, C_DIM, AL_C);
}

static void drawLowBat(const Robot &R, bool asleep) {
  float pulse = 0.5f + 0.5f * sinf(U.t * 3);
  rectV(0, 0, W, H, mix(C_RED_BG, RGB(70, 12, 18), asleep ? 0 : pulse * 0.6f), RGB(18, 4, 6));
  rrectLine(4, 4, W - 8, H - 8, 14, 2, C_BAD, asleep ? 0.4f : 0.5f + 0.5f * pulse);
  if (asleep) {
    disc(160, 70, 26, RGB(255, 200, 200));
    disc(172, 62, 24, mix(C_RED_BG, RGB(18, 4, 6), 0.3f));
  } else {
    rrectLine(118, 46, 76, 40, 7, 3, C_BAD);
    rrect(196, 58, 6, 16, 2, C_BAD);
    if (fmodf(U.t, 1.0f) < 0.6f)
      rrect(124, 52, 12, 28, 3, C_BAD);
  }
  text(F_BIG, asleep ? "ASLEEP" : "BATTERY LOW", 160, 128, C_TXT, AL_C);
  textf(F_MED, 160, 156, RGB(255, 170, 170), AL_C, "%.2f V", R.batV);
  text(F_LABEL, "UNPLUG THE BATTERY", 160, 196, C_TXT, AL_C);
  text(F_TINY, asleep ? "deep sleep: press EN to wake" : "legs parked, servos off", 160, 216,
       RGB(255, 170, 170), AL_C);
}

static void perfBox() {
  rrect(W - 132, 26, 124, 34, 8, RGB(0, 0, 0), 0.7f);
  textf(F_TINY, W - 124, 40, C_GOLD, AL_L, "%d fps  %.1f ms", (int)(U.fps + 0.5f), U.frameMs);
  textf(F_TINY, W - 124, 54, C_GOLD, AL_L, "c0 %.1f  c1 %.1f  %.0fMHz", U.c0ms, U.c1ms, U.spiMHz);
}

#include "navpages.h"
#include "drive_page.h"

static void drawPage(int pg, const Robot &R, float ox) {
  tg().ox = ox;
  switch (pg) {
  case PG_FACE:
    pageFace(R);
    break;
  case PG_DASH:
    pageDash(R);
    break;
  case PG_LEGS:
    pageLegs(R);
    break;
  case PG_LEVEL:
    pageLevel(R);
    break;
  case PG_SYS:
    pageSys(R);
    break;
  case PG_TRAIL:
    pageTrail(R);
    break;
  case PG_STAB:
    pageStab(R);
    break;
  case PG_HEAT:
    pageHeat(R);
    break;
  case PG_DRIVE:
    pageDrive(R);
    break;
  default:
    rect(0, 0, W, H, 0);
    break;
  }
  tg().ox = 0;
}

enum Takeover { TK_NONE, TK_BOOT, TK_WAIT, TK_LOWBAT, TK_ASLEEP };

static int takeover = TK_BOOT;

static void drawView(int v, const Robot &R, float ox, float alpha) {
  Target &t = tg();
  t.ga = (uint32_t)(clampf(alpha, 0, 1) * 32 + 0.5f);
  if (t.ga) {
    if (v >= 100) {
      t.ox = ox;
      switch (v - 100) {
      case TK_BOOT:
        drawBoot(U.t);
        break;
      case TK_WAIT:
        drawWaiting(R);
        break;
      case TK_LOWBAT:
        drawLowBat(R, false);
        break;
      case TK_ASLEEP:
        drawLowBat(R, true);
        break;
      }
      t.ox = 0;
    } else
      drawPage(v, R, ox);
  }
  t.ga = 32;
}

static const float VIEW_FADE = 0.38f;

// Render only; uiUpdate has already prepared the shared state for both cores.
static void renderFrame() {
  const Robot &R = RS;
  float s = easeIO((U.t - U.viewT0) / VIEW_FADE);
  if (s < 1 && U.prevView >= 0) {
    clear(C_BG0);
    drawView(U.prevView, R, -U.viewDir * 40 * s, 1);
    drawView(U.view, R, U.viewDir * 40 * (1 - s), s);
  } else
    drawView(U.view, R, 0, 1);
  if (R.perfBox() && takeover == TK_NONE)
    perfBox();
}

static void uiUpdate(float dt) {
  const Robot &R = RS;
  float bt = (U.ms - bootMs) / 1000.0f;
  if (bt < 2.4f)
    takeover = TK_BOOT;
  else if (R.sleeping())
    takeover = TK_ASLEEP;
  else if (!U.linkOK)
    takeover = TK_WAIT;
  else if (R.batLocked())
    takeover = TK_LOWBAT;
  else
    takeover = TK_NONE;

  static float faceHoldUntil = 0;
  int want = clampi(R.page, 0, PG_COUNT - 1);
  if (R.trick || R.party > 0)
    faceHoldUntil = U.t + 2.3f;
  if (U.t < faceHoldUntil && want != PG_OFF)
    want = PG_FACE;
  if (want != U.page) {
    U.page = want;
    U.pageDotsUntil = U.t + 2.2f;
  }
  int v = takeover != TK_NONE ? 100 + takeover : U.page;
  if (v != U.view) {
    U.viewDir = (v < 100 && U.view >= 0 && U.view < 100)
                    ? (cyclePos(v) > cyclePos(U.view) ? 1.0f : -1.0f)
                    : 0.0f;
    U.prevView = U.view;
    U.view = v;
    U.viewT0 = U.t;
  }

  {
    uint8_t sws = R.gaitMode() ? R.swing : 0;
    for (int i = 0; i < 6; i++) {
      int pl = R.swapped() ? 5 - i : i;
      if (((U.lastSwing >> pl) & 1) && !((sws >> pl) & 1))
        U.touchT[i] = U.t;
    }
    U.lastSwing = sws;
  }

  U.histAcc += dt;
  while (U.histAcc >= 1 / 30.0f) {
    U.histAcc -= 1 / 30.0f;
    U.swingHist[U.histHead] = R.gaitMode() ? R.swing : 0;
    U.histHead = (U.histHead + 1) & 127;
  }
  U.attAcc += dt;
  while (U.attAcc >= 0.1f) {
    U.attAcc -= 0.1f;
    U.pitchHist[U.attHead] = R.swapped() ? -R.pitch : R.pitch;
    U.rollHist[U.attHead] = R.swapped() ? -R.roll : R.roll;
    U.attHead = (U.attHead + 1) % 100;
  }
  navUpdate(R, dt);
  faceUpdate(R, dt);
}

static void uiInit() {
  glowInit();
  (void)FACE_BG;
  (void)PAGE_TITLE;
}
