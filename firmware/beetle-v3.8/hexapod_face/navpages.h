// Trail, support and servo-load pages. Load colours are estimates, not measured temperatures.

#pragma once

static void pageTrail(const Robot &R) {
  bgPage();
  header("TRAIL", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_TRAIL);
  const float X0 = 6, Y0 = 28, MW = 212, MH = 208, cx = X0 + MW / 2, cy = Y0 + MH / 2, k = NV.viewK;
  rrect(X0, Y0, MW, MH, 10, C_CARD);
  auto S = [&](float wx, float wy, float &sx, float &sy) {
    sx = cx - (wy - NV.viewCy) * k;
    sy = cy - (wx - NV.viewCx) * k;
  };
  auto inMap = [&](float sx, float sy, float m) {
    return sx > X0 + m && sx < X0 + MW - m && sy > Y0 + m && sy < Y0 + MH - m;
  };

  const float steps[] = {20, 50, 100, 200, 500, 1000, 2000, 5000, 10000};
  float g = steps[8];
  for (float s : steps)
    if (s * k >= 26) {
      g = s;
      break;
    }
  {
    float wx0 = NV.viewCx - (MH / 2) / k, wx1 = NV.viewCx + (MH / 2) / k,
          wy0 = NV.viewCy - (MW / 2) / k, wy1 = NV.viewCy + (MW / 2) / k;
    for (float v = ceilf(wx0 / g) * g; v <= wx1; v += g) {
      float a, b;
      S(v, 0, a, b);
      if (b > Y0 + 4 && b < Y0 + MH - 4)
        lineThin(X0 + 4, b, X0 + MW - 4, b, C_LINE, 0.55f);
    }
    for (float v = ceilf(wy0 / g) * g; v <= wy1; v += g) {
      float a, b;
      S(0, v, a, b);
      if (a > X0 + 4 && a < X0 + MW - 4)
        lineThin(a, Y0 + 4, a, Y0 + MH - 4, C_LINE, 0.55f);
    }
  }

  for (int n = 0; n < NV.printN; n++) {
    const Nav::Print &p = NV.prints[(NV.printHead - 1 - n + PRINT_N) % PRINT_N];
    float sx, sy;
    S(p.x, p.y, sx, sy);
    if (!inMap(sx, sy, 3))
      continue;
    float a = clampf(1 - (U.t - p.t) / 40, 0.18f, 0.9f), r = clampf(8 * k, 1.4f, 4.5f);
    disc(sx, sy, r, p.leg < 3 ? C_EYE : C_ACC, a);
  }

  {
    float ax = 0, ay = 0;
    bool have = false;
    for (int n = 0; n < NV.pathN; n++) {
      float bx, by;
      S(NV.path[n].x, NV.path[n].y, bx, by);
      if (!inMap(bx, by, 2)) {
        have = false;
        continue;
      }
      if (!have) {
        ax = bx;
        ay = by;
        have = true;
        continue;
      }
      if (n < NV.pathN - 1 && fabsf(bx - ax) + fabsf(by - ay) < 2.5f)
        continue;
      line(ax, ay, bx, by, 2.2f, C_OK, 0.3f + 0.7f * n / NV.pathN);
      ax = bx;
      ay = by;
    }
  }
  if (NV.pathN) {
    float sx, sy;
    S(NV.path[0].x, NV.path[0].y, sx, sy);
    if (inMap(sx, sy, 6)) {
      ring(sx, sy, 5, 1.6f, C_DIM);
      text(F_TINY, "start", sx + 8, sy + 4, C_MUTE);
    }
  }

  {
    float rx, ry;
    S(NV.px, NV.py, rx, ry);
    float c = cosf(NV.th), s = sinf(NV.th);
    auto W2S = [&](float bx, float by, float &sx, float &sy) {
      S(NV.px + bx * c - by * s, NV.py + bx * s + by * c, sx, sy);
    };
    if (150 * k >= 22) {
      const int ord[6] = {0, 3, 4, 5, 2, 1};
      float hx[6], hy[6];
      for (int n = 0; n < 6; n++)
        W2S(LEG[ord[n]].x * 1.2f, LEG[ord[n]].y * 1.2f, hx[n], hy[n]);
      for (int i = 0; i < 6; i++) {
        float fx, fy, fz, sx, sy, hx2, hy2;
        namedFoot(R, i, fx, fy, fz);
        W2S(fx, fy, sx, sy);
        W2S(LEG[i].x, LEG[i].y, hx2, hy2);
        lineThin(hx2, hy2, sx, sy, C_DIM, 0.7f);
        if (footDown(R, i))
          disc(sx, sy, 3, i < 3 ? C_EYE : C_ACC);
        else
          ring(sx, sy, 3, 1.2f, C_TXT);
      }
      float mx0 = 0, my0 = 0;
      for (int n = 0; n < 6; n++) {
        mx0 += hx[n] / 6;
        my0 += hy[n] / 6;
      }
      for (int n = 0; n < 6; n++)
        tri(mx0, my0, hx[n], hy[n], hx[(n + 1) % 6], hy[(n + 1) % 6], RGB(52, 160, 205), 0.9f);
      float nx, ny;
      W2S(85, 0, nx, ny);
      line(mx0, my0, nx, ny, 3, C_ACC);
      disc(nx, ny, 3, C_ACC);
    } else {
      float ux = -s, uy = -c;
      tri(rx + ux * 10, ry + uy * 10, rx - ux * 6 + uy * 6, ry - uy * 6 - ux * 6,
          rx - ux * 6 - uy * 6, ry - uy * 6 + ux * 6, C_ACC);
      glow(rx, ry, 14, C_ACC, 0.35f);
    }
  }

  char b[32];
  card(224, 28, 90, 44, "DISTANCE");
  if (NV.dist < 1000)
    fmt(b, sizeof b, "%d cm", (int)(NV.dist / 10 + 0.5f));
  else
    fmt(b, sizeof b, "%.2f m", NV.dist / 1000);
  text(F_LABEL, b, 304, 63, C_TXT, AL_R);
  card(224, 76, 90, 44, "SPEED");
  {
    float v = sqrtf(R.vx * R.vx + R.vy * R.vy) * R.gov;
    fmt(b, sizeof b, "%d", (int)(v + 0.5f));
    int e = text(F_LABEL, b, 268, 111, R.stepping() ? C_OK : C_TXT, AL_R);
    (void)e;
    text(F_TINY, "mm/s", 272, 111, C_MUTE);
  }
  card(224, 124, 90, 44, "HEADING");
  fmt(b, sizeof b, "%+d", (int)lroundf(NV.th * 57.2958f));
  text(F_LABEL, b, 296, 159, C_TXT, AL_R);
  ring(301, 147, 2.2f, 1, C_DIM);
  card(224, 172, 90, 44, "WALKED");
  {
    int t = (int)NV.walkS;
    fmt(b, sizeof b, "%d:%d%d", t / 60, (t % 60) / 10, t % 10);
    text(F_LABEL, b, 304, 207, C_TXT, AL_R);
  }

  if (g >= 1000)
    fmt(b, sizeof b, "grid %d m", (int)(g / 1000));
  else
    fmt(b, sizeof b, "grid %d cm", (int)(g / 10));
  text(F_TINY, b, 269, 228, C_DIM, AL_C);
  text(F_TINY, "estimated", 269, 239, C_MUTE, AL_C);
}

static uint16_t marginColour(float m) {
  return m > 15 ? C_OK : m > 6 ? C_WARN : C_BAD;
}

static void pageStab(const Robot &R) {
  bgPage();
  header("STABILITY", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_STAB);
  const float X0 = 6, Y0 = 28, MW = 200, MH = 208, cx = X0 + MW / 2, cy = Y0 + MH / 2 + 4,
              k = 0.47f;
  rrect(X0, Y0, MW, MH, 10, C_CARD);
  auto S = [&](float x, float y, float &sx, float &sy) {
    sx = cx - y * k;
    sy = cy - x * k;
  };
  const float sw = R.swapped() ? -1 : 1;

  for (int r = 50; r <= 200; r += 50)
    ring(cx, cy, r * k, 1, C_LINE, 0.45f);

  {
    const int ord[6] = {0, 3, 4, 5, 2, 1};
    float hx[6], hy[6];
    for (int n = 0; n < 6; n++) {
      int i = ord[n], pl = R.swapped() ? 5 - i : i;
      S(sw * (LEG[pl].x * 1.2f + R.pTx), sw * (LEG[pl].y * 1.2f + R.pTy), hx[n], hy[n]);
    }
    float mx0 = 0, my0 = 0;
    for (int n = 0; n < 6; n++) {
      mx0 += hx[n] / 6;
      my0 += hy[n] / 6;
    }
    for (int n = 0; n < 6; n++)
      tri(mx0, my0, hx[n], hy[n], hx[(n + 1) % 6], hy[(n + 1) % 6], C_CARD2, 0.9f);
    for (int n = 0; n < 6; n++)
      lineThin(hx[n], hy[n], hx[(n + 1) % 6], hy[(n + 1) % 6], C_MUTE);
    float nx, ny, n2x, n2y;
    S(sw * R.pTx + 62, sw * R.pTy, nx, ny);
    S(sw * R.pTx + 80, sw * R.pTy, n2x, n2y);
    line(nx, ny, n2x, n2y, 2, C_MUTE);
    tri(n2x, n2y - 5, n2x - 4, n2y + 1, n2x + 4, n2y + 1, C_MUTE);
  }

  uint16_t mc = marginColour(NV.margin);
  if (NV.hullN >= 3) {
    float sx[6], sy[6], ax = 0, ay = 0;
    for (int n = 0; n < NV.hullN; n++) {
      S(NV.hull[n].x, NV.hull[n].y, sx[n], sy[n]);
      ax += sx[n] / NV.hullN;
      ay += sy[n] / NV.hullN;
    }
    for (int n = 0; n < NV.hullN; n++)
      tri(ax, ay, sx[n], sy[n], sx[(n + 1) % NV.hullN], sy[(n + 1) % NV.hullN], mc, 0.18f);
    for (int n = 0; n < NV.hullN; n++)
      line(sx[n], sy[n], sx[(n + 1) % NV.hullN], sy[(n + 1) % NV.hullN], 1.8f, mc, 0.9f);
  } else if (NV.hullN == 2) {
    float a, b, c2, d;
    S(NV.hull[0].x, NV.hull[0].y, a, b);
    S(NV.hull[1].x, NV.hull[1].y, c2, d);
    line(a, b, c2, d, 1.8f, C_BAD);
  }

  for (int i = 0; i < 6; i++) {
    int pl = R.swapped() ? 5 - i : i;
    float fx, fy, fz, sx, sy, hx, hy;
    namedFoot(R, i, fx, fy, fz);
    S(fx, fy, sx, sy);
    S(sw * (LEG[pl].x + R.pTx), sw * (LEG[pl].y + R.pTy), hx, hy);
    lineThin(hx, hy, sx, sy, C_DIM, 0.8f);
    if (footDown(R, i)) {
      disc(sx, sy, 5, C_EYE);
    } else {
      ring(sx, sy, 5, 1.4f, C_MUTE);
    }
    text(F_TINY, LEG[i].name, sx + (sx < cx ? -9 : 9), sy + 4, C_MUTE, sx < cx ? AL_R : AL_L);
  }

  {
    float qx, qy, zx, zy;
    S(NV.com.x, NV.com.y, qx, qy);
    S(NV.zmp.x, NV.zmp.y, zx, zy);
    if (NV.hullN >= 2 && NV.marginOK) {
      P2 q = polyMargin(NV.zmp, NV.hull, NV.hullN) < polyMargin(NV.com, NV.hull, NV.hullN) ? NV.zmp
                                                                                           : NV.com;
      float best = 1e9f;
      P2 bp = q;
      for (int n = 0; n < NV.hullN; n++) {
        P2 a = NV.hull[n], b = NV.hull[(n + 1) % NV.hullN];
        float dx = b.x - a.x, dy = b.y - a.y, L2 = dx * dx + dy * dy,
              t = L2 > 1e-6f ? clampf(((q.x - a.x) * dx + (q.y - a.y) * dy) / L2, 0, 1) : 0;
        P2 c2 = {a.x + t * dx, a.y + t * dy};
        float d = hypotf(c2.x - q.x, c2.y - q.y);
        if (d < best) {
          best = d;
          bp = c2;
        }
      }
      float a, b, c2, d;
      S(q.x, q.y, a, b);
      S(bp.x, bp.y, c2, d);
      line(a, b, c2, d, 1.4f, mc, 0.8f);
    }
    if (fabsf(zx - qx) + fabsf(zy - qy) > 0.8f) {
      lineThin(qx, qy, zx, zy, C_ACC);
    }
    disc(zx, zy, 3, C_ACC);
    line(qx - 8, qy, qx + 8, qy, 1.4f, C_TXT);
    line(qx, qy - 8, qx, qy + 8, 1.4f, C_TXT);
    disc(qx, qy, 3.2f, C_TXT);
  }

  char b[24];
  card(212, 28, 102, 74, "MARGIN");
  if (NV.marginOK) {
    fmt(b, sizeof b, "%d", (int)lroundf(NV.margin));
    int e = text(F_BIG, b, 286, 78, mc, AL_R);
    (void)e;
    text(F_TINY, "mm", 290, 78, C_MUTE);
    fmt(b, sizeof b, "%d feet down", NV.down);
    text(F_TINY, b, 222, 95, C_DIM);
  } else {
    text(F_LABEL, "--", 222, 74, C_MUTE);
    text(F_TINY, R.park ? "parked" : "not standing", 222, 95, C_MUTE);
  }

  card(212, 106, 102, 96, "LAST 10 s");
  {
    float x = 220, y = 124, w = 86, h = 52, lo = 1e9f, hi = 0;
    for (int n = 0; n < MARG_N; n++) {
      float v = NV.margHist[n];
      if (v == v)
        hi = fmaxf(hi, v);
    }
    float rng = clampf(ceilf(hi * 1.15f / 20) * 20, 40, 200);
    rect(x, y + h - h * 6 / rng, w, h * 6 / rng, C_BAD, 0.18f);
    textf(F_TINY, 306, 118, C_MUTE, AL_R, "%d", (int)rng);
    float px = 0, py = 0;
    bool have = false;
    for (int n = 0; n < MARG_N; n++) {
      float v = NV.margHist[(NV.margHead + n) % MARG_N];
      if (!(v == v)) {
        have = false;
        continue;
      }
      if (n >= MARG_N / 2)
        lo = fminf(lo, v);
      float sx = x + n * w / (MARG_N - 1), sy = y + h - clampf(v / rng, 0, 1) * h;
      if (have)
        lineThin(px, py, sx, sy, marginColour(v));
      px = sx;
      py = sy;
      have = true;
    }
    if (lo < 1e8f) {
      fmt(b, sizeof b, "worst 5 s: %d mm", (int)lroundf(lo));
      text(F_TINY, b, 220, 194, marginColour(lo));
    }
  }

  card(212, 206, 102, 30);
  line(220, 221, 230, 221, 1.4f, C_TXT);
  line(225, 216, 225, 226, 1.4f, C_TXT);
  text(F_TINY, "CoM", 234, 225, C_DIM);
  disc(268, 221, 3, C_ACC);
  text(F_TINY, "accel", 275, 225, C_DIM);
}

static void pageHeat(const Robot &R) {
  bgPage();
  header("SERVO LOAD", R);
  if (U.t < U.pageDotsUntil)
    pageDots(PG_HEAT);
  bool any = false;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 2; j++)
      if (R.heat[i][j] >= 0)
        any = true;

  const float X0 = 6, Y0 = 28, MW = 150, MH = 208, cx = X0 + MW / 2, cy = Y0 + MH / 2 - 6,
              k = 0.36f;
  rrect(X0, Y0, MW, MH, 10, C_CARD);
  auto S = [&](float x, float y, float &sx, float &sy) {
    sx = cx - y * k;
    sy = cy - x * k;
  };
  const float sw = R.swapped() ? -1 : 1;
  {
    const int ord[6] = {0, 3, 4, 5, 2, 1};
    float hx[6], hy[6];
    for (int n = 0; n < 6; n++) {
      int pl = R.swapped() ? 5 - ord[n] : ord[n];
      S(sw * LEG[pl].x * 1.2f, sw * LEG[pl].y * 1.2f, hx[n], hy[n]);
    }
    for (int n = 0; n < 6; n++)
      tri(cx, cy, hx[n], hy[n], hx[(n + 1) % 6], hy[(n + 1) % 6], RGB(40, 120, 160));
    float nx, ny;
    S(sw * 0 + 75, 0, nx, ny);
    disc(nx, ny, 3, C_ACC);
  }
  for (int i = 0; i < 6; i++) {
    int pl = R.swapped() ? 5 - i : i;
    float fx, fy, fz;
    namedFoot(R, i, fx, fy, fz);
    float hx = sw * LEG[pl].x, hy = sw * LEG[pl].y, dx = fx - hx, dy = fy - hy,
          L = fmaxf(hypotf(dx, dy), 1);
    dx /= L;
    dy /= L;
    float kx = hx + dx * (HIP_OUT + FEMUR * 0.9f), ky = hy + dy * (HIP_OUT + FEMUR * 0.9f),
          px = hx + dx * HIP_OUT, py = hy + dy * HIP_OUT;
    float a, b, c2, d, e, f, g2, h;
    S(hx, hy, a, b);
    S(px, py, c2, d);
    S(kx, ky, e, f);
    S(fx, fy, g2, h);
    line(a, b, c2, d, 6, RGB(110, 120, 138));
    line(c2, d, e, f, 8, heatColour(R.heat[pl][0], RGB(150, 165, 185)));
    line(e, f, g2, h, 6, heatColour(R.heat[pl][1], RGB(150, 165, 185)));
    disc(c2, d, 3.5f, RGB(200, 210, 225));
    disc(e, f, 3, RGB(200, 210, 225));
    text(F_TINY, LEG[i].name, g2 + (g2 < cx ? -6 : 6), h + 4, C_DIM, g2 < cx ? AL_R : AL_L);
  }

  card(162, 28, 152, 160);
  text(F_TINY, "FEMUR", 192, 44, C_DIM);
  text(F_TINY, "TIBIA", 254, 44, C_DIM);
  int best = -1, bi = 0, bj = 0;
  for (int i = 0; i < 6; i++) {
    int pl = R.swapped() ? 5 - i : i;
    float y = 52 + i * 22;
    text(F_TINY, LEG[i].name, 170, y + 11, C_TXT);
    for (int j = 0; j < 2; j++) {
      float x = 192 + j * 62, w = 34;
      int v = R.heat[pl][j];
      if (v > best) {
        best = v;
        bi = i;
        bj = j;
      }
      bar(x, y + 4, w, 7, v < 0 ? 0 : v / 80.0f, heatColour(v, C_EYE));
      float pk = NV.peak[pl][j];
      if (pk >= 0)
        rect(x + w * clampf(pk / 80, 0, 1) - 0.5f, y + 2, 1.5f, 11, C_TXT, 0.8f);
      char b[8];
      if (v < 0)
        fmt(b, sizeof b, "--");
      else
        fmt(b, sizeof b, "%d", v);
      text(F_TINY, b, x + w + 20, y + 11, v < 0 ? C_MUTE : heatColour(v, C_TXT), AL_R);
    }
  }

  card(162, 192, 152, 44);
  char b[40];
  if (best >= 0) {
    fmt(b, sizeof b, "hottest %s %s %d%%", LEG[bi].name, bj ? "tibia" : "femur", best);
    text(F_TINY, b, 172, 208, heatColour(best, C_TXT));
  } else
    text(F_TINY, any ? "" : "no load estimate (set mass)", 172, 208, C_MUTE);
  for (int x = 0; x < 80; x++)
    rect(172 + x * 1.0f, 216, 1.2f, 6, heatColour(x, C_EYE));
  text(F_TINY, "0", 172, 233, C_MUTE);
  text(F_TINY, "35", 207, 233, C_MUTE, AL_C);
  text(F_TINY, "55", 227, 233, C_MUTE, AL_C);
  text(F_TINY, "80%", 252, 233, C_MUTE, AL_C);
  text(F_TINY, "of stall", 306, 233, C_MUTE, AL_R);
}
