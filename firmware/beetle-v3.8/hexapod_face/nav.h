// Page transitions and shared UI state. Core 0 updates these before either core renders.

#pragma once

struct P2 {
  float x, y;
};

static const int TRAIL_N = 600, PRINT_N = 240, MARG_N = 200, BAT_N = 180;

struct Nav {

  float px = 0, py = 0, th = 0, dist = 0, walkS = 0;
  P2 path[TRAIL_N];
  int pathN = 0;
  float pathStep = 6;

  struct Print {
    float x, y;
    uint8_t leg;
    float t;
  } prints[PRINT_N];

  int printHead = 0, printN = 0;
  uint8_t prevSwing = 0;
  uint32_t resetSeen = 0xFFFFFFFF;
  float viewCx = 0, viewCy = 0, viewK = 0.6f;
  bool viewInit = false;

  float fvx = 0, fvy = 0, ax = 0, ay = 0;
  P2 hull[6];
  int hullN = 0;
  P2 com = {0, 0}, zmp = {0, 0};
  float margin = 0;
  bool marginOK = false;
  int down = 0;
  float margHist[MARG_N] = {};
  int margHead = 0;
  float margAcc = 0;

  float peak[6][2] = {};

  float batT[BAT_N], batP[BAT_N];
  int batN = 0;
  float batAcc = 0, lastPct = -1, minutesLeft = -1;
};

static Nav NV;

static inline float wrapPi(float a) {
  while (a > 3.14159265f)
    a -= 6.2831853f;
  while (a < -3.14159265f)
    a += 6.2831853f;
  return a;
}

static inline void namedFoot(const Robot &R, int i, float &x, float &y, float &z) {
  int pl = R.swapped() ? 5 - i : i;
  float sw = R.swapped() ? -1 : 1;
  x = sw * R.feet[pl][0];
  y = sw * R.feet[pl][1];
  z = R.feet[pl][2] + R.bodyH;
}

static inline bool footDown(const Robot &R, int i) {
  int pl = R.swapped() ? 5 - i : i;
  float x, y, z;
  namedFoot(R, i, x, y, z);
  if (!R.gaitMode())
    return true;
  return !((R.swing >> pl) & 1) && z < 4;
}

static int convexHull(const P2 *p, int n, P2 *out) {
  if (n < 3) {
    for (int k = 0; k < n; k++)
      out[k] = p[k];
    return n;
  }
  int l = 0;
  for (int k = 1; k < n; k++)
    if (p[k].x < p[l].x || (p[k].x == p[l].x && p[k].y < p[l].y))
      l = k;
  int m = 0, a = l;
  do {
    out[m++] = p[a];
    int b = (a + 1) % n;
    for (int k = 0; k < n; k++) {
      float cr = (p[b].x - p[a].x) * (p[k].y - p[a].y) - (p[b].y - p[a].y) * (p[k].x - p[a].x);
      if (cr < 0)
        b = k;
    }
    a = b;
  } while (a != l && m < 6);
  return m;
}

static float segDist(P2 q, P2 a, P2 b) {
  float dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy,
        t = L2 > 1e-6f ? ((q.x - a.x) * dx + (q.y - a.y) * dy) / L2 : 0;
  t = clampf(t, 0, 1);
  float ex = a.x + t * dx - q.x, ey = a.y + t * dy - q.y;
  return sqrtf(ex * ex + ey * ey);
}

static float polyMargin(P2 q, const P2 *h, int n) {
  if (n == 0)
    return -999;
  if (n == 1)
    return -sqrtf((q.x - h[0].x) * (q.x - h[0].x) + (q.y - h[0].y) * (q.y - h[0].y));
  float d = 1e9f;
  bool inside = n >= 3;
  for (int k = 0; k < n; k++) {
    P2 a = h[k], b = h[(k + 1) % n];
    d = fminf(d, segDist(q, a, b));
    if (n >= 3 && (b.x - a.x) * (q.y - a.y) - (b.y - a.y) * (q.x - a.x) < 0)
      inside = false;
  }
  return inside ? d : -d;
}

static void trailClear() {
  NV.px = NV.py = NV.th = NV.dist = NV.walkS = 0;
  NV.pathN = 0;
  NV.pathStep = 6;
  NV.printN = 0;
  NV.printHead = 0;
  NV.viewInit = false;
}

static void navUpdate(const Robot &R, float dt) {
  if (!U.linkOK)
    return;
  const float sw = R.swapped() ? -1 : 1;

  if (R.trailReset != NV.resetSeen) {
    if (NV.resetSeen != 0xFFFFFFFF)
      trailClear();
    NV.resetSeen = R.trailReset;
  }
  float vx = sw * R.vx * R.gov, vy = sw * R.vy * R.gov, wz = R.wz * R.gov;
  if (R.gaitMode()) {
    float hm = NV.th + 0.5f * wz * dt, c = cosf(hm), s = sinf(hm);
    NV.px += (vx * c - vy * s) * dt;
    NV.py += (vx * s + vy * c) * dt;
    NV.th = wrapPi(NV.th + wz * dt);
    float sp = sqrtf(vx * vx + vy * vy);
    NV.dist += sp * dt;
    if (R.stepping())
      NV.walkS += dt;
  }
  if (NV.pathN == 0 ||
      hypotf(NV.px - NV.path[NV.pathN - 1].x, NV.py - NV.path[NV.pathN - 1].y) >= NV.pathStep) {
    if (NV.pathN == TRAIL_N) {
      for (int k = 0; k < TRAIL_N / 2; k++)
        NV.path[k] = NV.path[2 * k + 1];
      NV.pathN = TRAIL_N / 2;
      NV.pathStep *= 2;
    }
    NV.path[NV.pathN++] = {NV.px, NV.py};
  }
  uint8_t sws = R.gaitMode() ? R.swing : 0;
  for (int pl = 0; pl < 6; pl++)
    if (((NV.prevSwing >> pl) & 1) && !((sws >> pl) & 1)) {
      int i = R.swapped() ? 5 - pl : pl;
      float fx, fy, fz;
      namedFoot(R, i, fx, fy, fz);
      float c = cosf(NV.th), s = sinf(NV.th);
      NV.prints[NV.printHead] = {NV.px + fx * c - fy * s, NV.py + fx * s + fy * c, (uint8_t)i, U.t};
      NV.printHead = (NV.printHead + 1) % PRINT_N;
      if (NV.printN < PRINT_N)
        NV.printN++;
    }
  NV.prevSwing = sws;
  {
    float x0 = NV.px - 220, x1 = NV.px + 220, y0 = NV.py - 220, y1 = NV.py + 220;
    for (int k = 0; k < NV.pathN; k++) {
      x0 = fminf(x0, NV.path[k].x - 60);
      x1 = fmaxf(x1, NV.path[k].x + 60);
      y0 = fminf(y0, NV.path[k].y - 60);
      y1 = fmaxf(y1, NV.path[k].y + 60);
    }
    float cx = (x0 + x1) / 2, cy = (y0 + y1) / 2,
          k = clampf(fminf(196.0f / (y1 - y0), 192.0f / (x1 - x0)), 0.01f, 0.6f);
    if (!NV.viewInit) {
      NV.viewCx = cx;
      NV.viewCy = cy;
      NV.viewK = k;
      NV.viewInit = true;
    }
    NV.viewCx = approach(NV.viewCx, cx, dt, 0.5f);
    NV.viewCy = approach(NV.viewCy, cy, dt, 0.5f);
    NV.viewK = approach(NV.viewK, k, dt, 0.6f);
  }

  {
    float a = 1 - expf(-dt / 0.1f);
    float nvx = NV.fvx + (vx - NV.fvx) * a, nvy = NV.fvy + (vy - NV.fvy) * a;
    float iax = dt > 1e-4f ? (nvx - NV.fvx) / dt : 0, iay = dt > 1e-4f ? (nvy - NV.fvy) / dt : 0;
    NV.ax += (iax - wz * nvy - NV.ax) * a;
    NV.ay += (iay + wz * nvx - NV.ay) * a;
    NV.fvx = nvx;
    NV.fvy = nvy;
  }
  P2 pts[6];
  int n = 0;
  for (int i = 0; i < 6; i++)
    if (footDown(R, i)) {
      float x, y, z;
      namedFoot(R, i, x, y, z);
      pts[n++] = {x, y};
    }
  NV.down = n;
  NV.hullN = convexHull(pts, n, NV.hull);
  NV.com = {sw * (R.pTx + R.comX), sw * (R.pTy + R.comY)};
  float hc = (R.gaitMode() ? R.bodyH : PARK_H) + 20;
  NV.zmp = {NV.com.x - NV.ax * hc / 9810.0f, NV.com.y - NV.ay * hc / 9810.0f};
  NV.margin = fminf(polyMargin(NV.com, NV.hull, NV.hullN), polyMargin(NV.zmp, NV.hull, NV.hullN));
  NV.marginOK = R.gaitMode() && R.servosOn() && !R.park;
  NV.margAcc += dt;
  while (NV.margAcc >= 0.05f) {
    NV.margAcc -= 0.05f;
    NV.margHist[NV.margHead] = NV.marginOK ? NV.margin : NAN;
    NV.margHead = (NV.margHead + 1) % MARG_N;
  }

  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 2; j++) {
      float h = R.heat[i][j];
      NV.peak[i][j] = h < 0 ? -1 : fmaxf(h, NV.peak[i][j] < 0 ? h : NV.peak[i][j] - dt);
    }

  NV.batAcc += dt;
  if (NV.batAcc >= 2) {
    NV.batAcc = 0;
    float p = batPercentFromVf(R.batV);
    if (!R.onBatt() || R.batLocked() || (NV.lastPct >= 0 && p > NV.lastPct + 4)) {
      NV.batN = 0;
      NV.minutesLeft = -1;
    }
    if (R.onBatt() && !R.batLocked()) {
      if (NV.batN == BAT_N) {
        memmove(NV.batT, NV.batT + 1, (BAT_N - 1) * sizeof(float));
        memmove(NV.batP, NV.batP + 1, (BAT_N - 1) * sizeof(float));
        NV.batN--;
      }
      NV.batT[NV.batN] = U.t;
      NV.batP[NV.batN] = p;
      NV.batN++;
      if (NV.batN >= 30 && NV.batT[NV.batN - 1] - NV.batT[0] >= 60) {
        double st = 0, sp = 0, stt = 0, stp = 0;
        int m = NV.batN;
        for (int k = 0; k < m; k++) {
          double t = NV.batT[k] - NV.batT[0];
          st += t;
          sp += NV.batP[k];
          stt += t * t;
          stp += t * NV.batP[k];
        }
        double den = m * stt - st * st, slope = den > 0 ? (m * stp - st * sp) / den : 0;
        const float EMPTY = 7.5f;
        float mins = slope < -1e-4 ? (float)((p - EMPTY) / -slope / 60) : 999;
        mins = clampf(mins, 0, 999);
        NV.minutesLeft =
            NV.minutesLeft < 0 ? mins : NV.minutesLeft + (mins - NV.minutesLeft) * 0.25f;
      }
    }
    NV.lastPct = p;
  }
}

static void batLeftText(char *b, int cap) {
  float m = NV.minutesLeft;
  if (m < 0) {
    b[0] = 0;
    return;
  }
  if (m >= 180) {
    fmt(b, cap, "3 h+ left");
    return;
  }
  if (m >= 60) {
    fmt(b, cap, "~%d h %d min left", (int)(m / 60), (int)m % 60);
    return;
  }
  fmt(b, cap, "~%d min left", (int)(m + 0.5f));
}
