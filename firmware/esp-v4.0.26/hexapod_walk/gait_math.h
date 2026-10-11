// Shared geometry, gait paths and load estimates; this header also runs in the host tests.
// Body axes: x forward, y left, z up. Distances are mm; angles are radians unless named Deg.

#pragma once
#include <math.h>
#include <string.h>

static const float HIP_OUT = 38.0f, HIP_UP = 25.9f, FEMUR = 65.9f, TIBIA = 108.64f;
static const float SIDE = 16.7f + 17.6f;
static const float COXA_Z = 19.0f;

// CAD hip anchors and outward leg angles. Calibration limits stay in physical leg order.
struct LegDef {
  const char *name;
  float x, y, out;
};

static const LegDef LEG[6] = {{"FL", 51.677f, 22.323f, 45},   {"ML", 5.2f, 33.0f, 90},
                              {"RL", -51.677f, 22.323f, 135}, {"FR", 51.677f, -22.323f, -45},
                              {"MR", 5.2f, -33.0f, -90},      {"RR", -51.677f, -22.323f, -135}};
static const float JOINT_MIN[3] = {-40, -90, -65}, JOINT_MAX[3] = {40, 90, 90};
static const float GM_D2R = 0.01745329f, GM_R2D = 57.2957795f;

static const float KNEE_MAX_DEG = 65;

static const float RHO_MIN = 25;

static inline float gmClamp(float v, float a, float b) {
  return v < a ? a : v > b ? b : v;
}

// Reference-to-body rotation is Rx(-roll) Ry(pitch) Rz(-yaw). Directions exclude translation.
struct PoseFrame {
  float cy, sy, cp, sp, cr, sr;

  PoseFrame(float rollDeg, float pitchDeg, float yawDeg) {
    cy = cosf(yawDeg * GM_D2R);
    sy = sinf(yawDeg * GM_D2R);
    cp = cosf(pitchDeg * GM_D2R);
    sp = sinf(pitchDeg * GM_D2R);
    cr = cosf(rollDeg * GM_D2R);
    sr = sinf(rollDeg * GM_D2R);
  }

  void toBody(float x, float y, float z, float &bx, float &by, float &bz) const {
    float x1 = cy * x + sy * y, y1 = -sy * x + cy * y;
    float x2 = cp * x1 + sp * z, z2 = -sp * x1 + cp * z;
    bx = x2;
    by = cr * y1 + sr * z2;
    bz = -sr * y1 + cr * z2;
  }

  void toReference(float x, float y, float z, float &rx, float &ry, float &rz) const {
    float y1 = cr * y - sr * z, z1 = sr * y + cr * z;
    float x1 = cp * x - sp * z1;
    rz = sp * x + cp * z1;
    rx = cy * x1 - sy * y1;
    ry = sy * x1 + cy * y1;
  }
};

// Put the CoM and gravity in the contact frame before evaluating support.
struct SupportFrame {
  float cx, cy, cz, up[3];

  SupportFrame(const PoseFrame &R, float tx, float ty, float tz, float comX, float comY, float comZ,
               const float bodyUp[3]) {
    R.toReference(comX, comY, comZ, cx, cy, cz);
    cx += tx;
    cy += ty;
    cz += tz;
    R.toReference(bodyUp[0], bodyUp[1], bodyUp[2], up[0], up[1], up[2]);
  }

  bool projection(float groundZ, float &px, float &py) const {
    if (!isfinite(cx) || !isfinite(cy) || !isfinite(cz) || !isfinite(groundZ) || !isfinite(up[0]) ||
        !isfinite(up[1]) || !isfinite(up[2]) || up[2] <= .5f)
      return false;
    float h = (cz - groundZ) / up[2];
    px = cx - h * up[0];
    py = cy - h * up[1];
    return isfinite(px) && isfinite(py);
  }
};

// Return degrees: coxa positive counterclockwise from above, femur up, tibia outward.
// Unreachable points return false; this function does not clamp to calibration limits.
static bool ikRaw(int i, float fx, float fy, float fz, float q[3], float *reach = nullptr) {
  if (i < 0 || i >= 6 || !isfinite(fx) || !isfinite(fy) || !isfinite(fz))
    return false;
  float a = LEG[i].out * GM_D2R, ca = cosf(a), sa = sinf(a);
  float dx = fx - LEG[i].x, dy = fy - LEG[i].y;
  float U = dx * ca + dy * sa, W = dx * sa - dy * ca;
  float h2 = U * U + W * W - SIDE * SIDE;
  if (h2 < 1)
    return false;
  float u = sqrtf(h2);
  float r = u - HIP_OUT, z = fz - COXA_Z - HIP_UP;
  float D2 = r * r + z * z, D = sqrtf(D2);
  if (D > FEMUR + TIBIA - 0.5f || D < fabsf(TIBIA - FEMUR) + 0.5f)
    return false;
  float alpha = acosf(gmClamp((FEMUR * FEMUR + D2 - TIBIA * TIBIA) / (2 * FEMUR * D), -1, 1));
  float gamma = acosf(gmClamp((FEMUR * FEMUR + TIBIA * TIBIA - D2) / (2 * FEMUR * TIBIA), -1, 1));
  q[0] = (atan2f(SIDE, u) - atan2f(W, U)) * GM_R2D;
  q[1] = (atan2f(z, r) + alpha) * GM_R2D;
  q[2] = gamma * GM_R2D - 90.0f;
  if (reach)
    *reach = r;
  return true;
}

static float ikMargin(int i, float fx, float fy, float fz) {
  float q[3];
  if (!ikRaw(i, fx, fy, fz, q))
    return -999;
  float m = 1e9f;
  for (int j = 0; j < 3; j++)
    m = fminf(m, fminf(q[j] - JOINT_MIN[j], JOINT_MAX[j] - q[j]));
  return m;
}

static void neutralFoot(int i, float width, float comX, float comY, float &nx, float &ny) {
  float a = LEG[i].out * GM_D2R;
  nx = LEG[i].x + (HIP_OUT + FEMUR + width) * cosf(a) + SIDE * sinf(a) + comX;
  ny = LEG[i].y + (HIP_OUT + FEMUR + width) * sinf(a) - SIDE * cosf(a) + comY;
}

// Vertical-load torque magnitudes in N mm; the full 3D signed calculation lives in equations.h.
static inline void torqueFromIK(float Fn, float r, float q1Deg, float &tf, float &tt) {
  tf = Fn * fabsf(r);
  tt = Fn * fabsf(r - FEMUR * cosf(q1Deg * GM_D2R));
}

static inline void torqueTilted(float Fn, float r, float z, float q1Deg, float ur, float uz,
                                float &tf, float &tt) {
  float c = cosf(q1Deg * GM_D2R), sn = sinf(q1Deg * GM_D2R), lt = r - FEMUR * c,
        zt = z - FEMUR * sn;
  tf = Fn * fabsf(r * uz - z * ur);
  tt = Fn * fabsf(lt * uz - zt * ur);
}

// Workspace constraints combine calibrated joint limits, lift clearance and estimated loading.
struct LegLimits {
  float zOff[6];
  float zAir[6] = {0, 0, 0, 0, 0, 0};
  float Fn[6];
  float tauLim[6];
  bool hasQ = false;
  float tanP = 0, tanR = 0;
  float lift = 0;
  float qLo[6][3], qHi[6][3];
};

static const float APEX_EXTRA = 0.0f;

// Check both loaded stance and unloaded swing apex; applying sag at the apex hides folded-joint
// limits.
static bool footOK(int i, float px, float py, float h, float dz, float marginDeg,
                   const LegLimits *L, bool torque) {
  float lv = L ? gmClamp(px * L->tanP + py * L->tanR, -35.0f, 35.0f) : 0;
  float z0 = -h + (L ? L->zOff[i] : 0) + lv, zA = -h + (L ? L->zAir[i] : 0) + lv;
  int nz = (L && L->lift > 0) ? 3 : 2;
  for (int zi = 0; zi < nz; zi++) {
    bool apex = zi == 2;
    float zt = apex ? zA + L->lift + APEX_EXTRA : z0 + (zi == 0 ? -dz : dz);
    float q[3], r;
    if (!ikRaw(i, px, py, zt, q, &r))
      return false;
    for (int j = 0; j < 3; j++) {
      float lo = JOINT_MIN[j], hi = JOINT_MAX[j];
      if (L && L->hasQ) {
        lo = fmaxf(lo, L->qLo[i][j]);
        hi = fminf(hi, L->qHi[i][j]);
      }
      if (q[j] - lo < marginDeg || hi - q[j] < marginDeg)
        return false;
    }
    if (q[2] > KNEE_MAX_DEG)
      return false;
    if (r + HIP_OUT < RHO_MIN)
      return false;
    if (torque && !apex && L && L->Fn[i] > 0) {
      float tf, tt;
      torqueFromIK(L->Fn[i], r, q[1], tf, tt);
      if (tf > L->tauLim[i] || tt > L->tauLim[i])
        return false;
    }
  }
  return true;
}

static float strideBudget(float h, float width, float comX, float comY, float marginDeg = 7,
                          float dz = 6, const LegLimits *L = nullptr) {
  for (int i = 0; i < 6; i++) {
    float nx, ny;
    neutralFoot(i, width, comX, comY, nx, ny);
    if (!footOK(i, nx, ny, h, dz, marginDeg, L, false))
      return 0;
  }
  float lo = 0, hi = 110;
  for (int it = 0; it < 8; it++) {
    float S = (lo + hi) / 2, R = S / 2;
    bool ok = true;
    for (int i = 0; i < 6 && ok; i++) {
      float nx, ny;
      neutralFoot(i, width, comX, comY, nx, ny);
      for (int k = 0; k < 12 && ok; k++) {
        float an = k * 0.5236f, c = cosf(an), sn = sinf(an);
        if (!footOK(i, nx + R * c, ny + R * sn, h, dz, marginDeg, L, false) ||
            !footOK(i, nx + 0.5f * R * c, ny + 0.5f * R * sn, h, dz, marginDeg, L, false))
          ok = false;
      }
    }
    if (ok)
      lo = S;
    else
      hi = S;
  }
  return lo;
}

static const int NDIR = 24;

static void strideDirs(float h, float width, float comX, float comY, const LegLimits *L, float Rmax,
                       float R[6][NDIR], float marginDeg = 7, float dz = 6) {
  for (int i = 0; i < 6; i++) {
    float nx, ny;
    neutralFoot(i, width, comX, comY, nx, ny);
    if (!footOK(i, nx, ny, h, dz, marginDeg, L, true)) {
      for (int k = 0; k < NDIR; k++)
        R[i][k] = 0;
      continue;
    }
    for (int k = 0; k < NDIR; k++) {
      float c = cosf(k * 6.2831853f / NDIR), sn = sinf(k * 6.2831853f / NDIR);
      if (footOK(i, nx + Rmax * c, ny + Rmax * sn, h, dz, marginDeg, L, true) &&
          footOK(i, nx + 0.5f * Rmax * c, ny + 0.5f * Rmax * sn, h, dz, marginDeg, L, true)) {
        R[i][k] = Rmax;
        continue;
      }
      float lo = 0, hi = Rmax;
      for (int it = 0; it < 6; it++) {
        float m = (lo + hi) / 2;
        if (footOK(i, nx + m * c, ny + m * sn, h, dz, marginDeg, L, true))
          lo = m;
        else
          hi = m;
      }
      R[i][k] = lo;
    }

    const float step = 6.2831853f / NDIR;
    auto sectorOK = [&](int a, int b, float scale) {
      for (int t = 0; t <= 4; t++) {
        float f = t * .25f, ra = R[i][a], rb = R[i][b];
        float rad = t == 0 ? ra
                    : t == 4
                        ? rb
                        : (ra > 0 && rb > 0 ? ra * rb * sinf(step) /
                                                  (ra * sinf(f * step) + rb * sinf((1 - f) * step))
                                            : 0);
        float ang = (a + f) * step, c = cosf(ang), s = sinf(ang);
        for (int k = 1; k <= 4; k++) {
          float r = rad * scale * k * .25f;
          if (!footOK(i, nx + r * c, ny + r * s, h, dz, marginDeg, L, true))
            return false;
        }
      }
      return true;
    };
    for (int pass = 0; pass < 2; pass++)
      for (int a = 0; a < NDIR; a++) {
        int b = (a + 1) % NDIR;
        if (sectorOK(a, b, 1))
          continue;
        float lo = 0, hi = 1;
        for (int k = 0; k < 6; k++) {
          float m = .5f * (lo + hi);
          if (sectorOK(a, b, m))
            lo = m;
          else
            hi = m;
        }
        R[i][a] *= lo;
        R[i][b] *= lo;
      }
  }
}

// Interpolate along verified bin chords; the arc between endpoints has not been checked.
static inline float dirLookup(const float Ri[NDIR], float ang, float = 1e9f) {
  const float step = 6.2831853f / NDIR;
  float f = ang / step;
  f -= floorf(f / NDIR) * NDIR;
  int a = (int)f % NDIR, b = (a + 1) % NDIR;
  float ra = Ri[a], rb = Ri[b], fr = f - floorf(f);
  if (fr < 1e-4f)
    return ra;
  if (fr > 1 - 1e-4f)
    return rb;
  if (ra <= 0 || rb <= 0)
    return 0;
  float ta = fr * step, tb = (1 - fr) * step;
  return ra * rb * sinf(step) / (ra * sinf(ta) + rb * sinf(tb));
}

static float footSpeedScale(float vx, float vy, float w, float Tst, float strideMax, float width,
                            float comX, float comY, const float R[6][NDIR]) {
  float worst = 0;
  for (int i = 0; i < 6; i++) {
    float nx, ny;
    neutralFoot(i, width, comX, comY, nx, ny);
    float ux = vx - w * ny, uy = vy + w * nx, us = sqrtf(ux * ux + uy * uy);
    if (us < 1e-3f)
      continue;
    float a = atan2f(uy, ux);
    float allow = fminf(strideMax, 2 * fminf(dirLookup(R[i], a, strideMax / 2),
                                             dirLookup(R[i], a + 3.14159265f, strideMax / 2)));
    if (allow < 0.5f)
      return 0.0f;
    worst = fmaxf(worst, us * Tst / allow);
  }
  return worst > 1 ? 1.0f / worst : 1.0f;
}

static inline float rampAvg(float v0, float vT, float A, float t1, float t2) {
  float dv = vT - v0, sg = dv >= 0 ? 1.0f : -1.0f, tc = fabsf(dv) / fmaxf(A, 1e-6f);
  auto I = [&](float t) {
    return t <= tc ? v0 * t + 0.5f * sg * A * t * t
                   : v0 * tc + 0.5f * sg * A * tc * tc + vT * (t - tc);
  };
  return t2 - t1 > 1e-6f ? (I(t2) - I(t1)) / (t2 - t1) : v0 + gmClamp(dv, -A * t1, A * t1);
}

static inline void arcChord(float phi, float &A, float &B) {
  if (fabsf(phi) < 1e-3f) {
    A = 1 - phi * phi / 6;
    B = phi / 2;
  } else {
    A = sinf(phi) / phi;
    B = (1 - cosf(phi)) / phi;
  }
}

// Choose a landing point that reaches neutral at mid-stance, including the body's turning arc.
static void touchdownTarget(float nx, float ny, float vx, float vy, float w, float Tst, float &tx,
                            float &ty) {

  float th = w * Tst / 2, A, B, c = cosf(th), s = sinf(th);
  arcChord(th, A, B);
  float h = Tst / 2;
  tx = c * nx - s * ny + (A * vx - B * vy) * h;
  ty = s * nx + c * ny + (B * vx + A * vy) * h;
}

// Integrate the exact planar turning arc so a planted foot stays fixed in world coordinates.
static void stanceStep(float &px, float &py, float vx, float vy, float w, float dt) {
  float phi = w * dt, A, B;
  arcChord(phi, A, B);
  float tx = (A * vx - B * vy) * dt, ty = (B * vx + A * vy) * dt;
  float qx = px - tx, qy = py - ty, c = cosf(phi), s = sinf(phi);
  px = c * qx + s * qy;
  py = -s * qx + c * qy;
}

static inline float smoothstep(float s) {
  s = gmClamp(s, 0, 1);
  return s * s * (3 - 2 * s);
}

static inline float minjerk(float s) {
  s = gmClamp(s, 0, 1);
  return s * s * s * (10 + s * (-15 + 6 * s));
}

static void swingXY(float s, float p0x, float p0y, float tdx, float tdy, float u0x, float u0y,
                    float u1x, float u1y, float Tsw, float &x, float &y) {
  s = gmClamp(s, 0, 1);
  float s3 = s * s * s, sg = minjerk(s);
  float h0 = s + s3 * (-6 + s * (8 - 3 * s));
  float h1 = s3 * (-4 + s * (7 - 3 * s));
  x = p0x + (tdx - p0x) * sg - (u0x * h0 + u1x * h1) * Tsw;
  y = p0y + (tdy - p0y) * sg - (u0y * h0 + u1y * h1) * Tsw;
}

// This bump has zero vertical velocity and acceleration at both contact transitions.
static inline float swingLift(float s) {
  if (s <= 0 || s >= 1)
    return 0;
  float a = s * (1 - s);
  return 64 * a * a * a;
}

static bool solve3(const double M[3][3], const double b[3], double x[3]) {
  double det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
               M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
               M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  if (fabs(det) < 1e-12)
    return false;
  for (int k = 0; k < 3; k++) {
    double A[3][3];
    memcpy(A, M, sizeof A);
    for (int r = 0; r < 3; r++)
      A[r][k] = b[r];
    x[k] = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
            A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
            A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) /
           det;
  }
  return true;
}

static inline float legStiffRel(float r, float q1Deg) {
  float lt = r - FEMUR * cosf(q1Deg * GM_D2R);
  return (FEMUR * FEMUR) / fmaxf(r * r + lt * lt, 400.0f);
}

static bool springSolve(const float fx[6], const float fy[6], const float kn[6], const float *e,
                        float cx, float cy, float F[6], float *d);
static bool loadSplitK(const float fx[6], const float fy[6], const float knIn[6], const float *e,
                       float cx, float cy, float F[6]);

// Predict centre sink from six feet to the sag-test tripod, including existing feed-forward and
// foot errors.
static float sagSinkDrop(float Sphys, float Scmd, const float *e, float width, float comX,
                         float comY, float h) {
  float fx[6], fy[6], kr[6], lev[6], sink[2];
  for (int i = 0; i < 6; i++) {
    neutralFoot(i, width, comX, comY, fx[i], fy[i]);
    float q[3], r;
    if (ikRaw(i, fx[i], fy[i], -h, q, &r)) {
      float lt = r - FEMUR * cosf(q[1] * GM_D2R);
      kr[i] = legStiffRel(r, q[1]);
      lev[i] = fmaxf(r * r + lt * lt, 400.0f) / (FEMUR * FEMUR);
    } else {
      kr[i] = 1;
      lev[i] = 1;
    }
  }
  for (int st = 0; st < 2; st++) {
    float kc[6], kp[6], F[6], d[6], ee[6], Sm = Scmd > 0 ? Scmd : 2.0f;
    for (int i = 0; i < 6; i++) {
      bool on = st == 0 || i == 1 || i == 3 || i == 5;
      kc[i] = on ? kr[i] / (3 * Sm) : 0;
      kp[i] = on ? kr[i] / (3 * Sphys) : 0;
    }
    loadSplitK(fx, fy, kc, e, comX, comY, F);
    for (int i = 0; i < 6; i++)
      ee[i] = (e ? e[i] : 0) + (Scmd > 0 ? 3 * Scmd * F[i] * lev[i] : 0);
    if (!loadSplitK(fx, fy, kp, ee, comX, comY, F))
      return NAN;
    for (int i = 0; i < 6; i++)
      if (F[i] <= 0)
        kp[i] = 0;
    if (!springSolve(fx, fy, kp, ee, comX, comY, F, d))
      return NAN;
    double M[3][3], b[3], c[3];
    int id[3] = {1, 3, 5};
    for (int r = 0; r < 3; r++) {
      M[r][0] = 1;
      M[r][1] = fx[id[r]];
      M[r][2] = fy[id[r]];
      b[r] = d[id[r]] - ee[id[r]];
    }
    if (!solve3(M, b, c))
      return NAN;
    sink[st] = (float)c[0];
  }
  return sink[1] - sink[0];
}

// Search valid brackets for physical compliance. Multiple roots cannot identify one reliable
// setting.
static float sagFromDrop(float dMeas, float Scmd, const float *e, float width, float comX,
                         float comY, float h) {
  if (!isfinite(dMeas) || !isfinite(Scmd) || !isfinite(width) || !isfinite(comX) ||
      !isfinite(comY) || !isfinite(h))
    return NAN;
  if (e)
    for (int i = 0; i < 6; i++)
      if (!isfinite(e[i]))
        return NAN;

  float lo = 0, hi = 0, flo = 0, prevS = 0, prevF = 0;
  bool prevValid = false, found = false;
  const int N = 96;
  for (int k = 0; k <= N; k++) {
    float s = k == 0 ? .05f : k == N ? 25.0f : .05f * expf(logf(500.0f) * k / N);
    float f = sagSinkDrop(s, Scmd, e, width, comX, comY, h) - dMeas;
    if (!isfinite(f)) {
      prevValid = false;
      continue;
    }
    bool exact = f == 0, crossing = prevValid && prevF != 0 && f != 0 && ((prevF < 0) != (f < 0));
    if (exact || crossing) {
      if (found)
        return NAN;
      found = true;
      lo = exact ? s : prevS;
      hi = s;
      flo = exact ? 0 : prevF;
    }
    prevS = s;
    prevF = f;
    prevValid = true;
  }
  if (!found)
    return NAN;
  if (lo == hi)
    return lo;
  for (int it = 0; it < 32 && hi - lo > 1e-5f; it++) {
    float m = .5f * (lo + hi), fm = sagSinkDrop(m, Scmd, e, width, comX, comY, h) - dMeas;
    if (!isfinite(fm))
      return NAN;
    if (fm == 0)
      return m;
    if ((fm < 0) == (flo < 0)) {
      lo = m;
      flo = fm;
    } else
      hi = m;
  }
  return .5f * (lo + hi);
}

static float sagDropCoef(float width, float comX, float comY, float h) {
  float c = sagSinkDrop(1.0f, 0, nullptr, width, comX, comY, h);
  return (c == c && c > 0.05f && c < 5.0f) ? c : 0.5f;
}

static bool springSolve(const float fx[6], const float fy[6], const float kn[6], const float *e,
                        float cx, float cy, float F[6], float *d = nullptr) {

  float ks = 0, xc = 0, yc = 0;
  int n = 0;
  for (int i = 0; i < 6; i++) {
    F[i] = 0;
    if (d)
      d[i] = -1e9f;
    if (kn[i] > 1e-7f) {
      ks += kn[i];
      xc += kn[i] * fx[i];
      yc += kn[i] * fy[i];
      n++;
    }
  }
  if (n == 0)
    return false;
  xc /= ks;
  yc /= ks;
  const float SC = 0.01f;
  float M[3][3] = {{0}}, b[3] = {1, (cx - xc) * SC, (cy - yc) * SC};
  for (int i = 0; i < 6; i++) {
    if (kn[i] <= 1e-7f)
      continue;
    float v[3] = {1, (fx[i] - xc) * SC, (fy[i] - yc) * SC}, ei = e ? e[i] : 0;
    for (int a = 0; a < 3; a++) {
      b[a] -= kn[i] * ei * v[a];
      for (int q = 0; q < 3; q++)
        M[a][q] += kn[i] * v[a] * v[q];
    }
  }
  float tr = M[1][1] + M[2][2], dt2 = M[1][1] * M[2][2] - M[1][2] * M[2][1];
  float disc = sqrtf(fmaxf(0.0f, tr * tr * 0.25f - dt2)), lmax = tr * 0.5f + disc,
        lmin = tr * 0.5f - disc;
  if (n < 3 || !(lmin > 1e-4f * lmax)) {
    for (int i = 0; i < 6; i++)
      F[i] = kn[i] / ks;
    return false;
  }
  float det = M[0][0] * (M[1][1] * M[2][2] - M[1][2] * M[2][1]) -
              M[0][1] * (M[1][0] * M[2][2] - M[1][2] * M[2][0]) +
              M[0][2] * (M[1][0] * M[2][1] - M[1][1] * M[2][0]);
  float c[3];
  for (int k = 0; k < 3; k++) {
    float A[3][3];
    memcpy(A, M, sizeof A);
    for (int r = 0; r < 3; r++)
      A[r][k] = b[r];
    c[k] = (A[0][0] * (A[1][1] * A[2][2] - A[1][2] * A[2][1]) -
            A[0][1] * (A[1][0] * A[2][2] - A[1][2] * A[2][0]) +
            A[0][2] * (A[1][0] * A[2][1] - A[1][1] * A[2][0])) /
           det;
  }
  for (int i = 0; i < 6; i++) {
    float comp = c[0] + c[1] * (fx[i] - xc) * SC + c[2] * (fy[i] - yc) * SC + (e ? e[i] : 0);
    if (d)
      d[i] = comp;
    if (kn[i] > 1e-7f)
      F[i] = kn[i] * comp;
  }
  return true;
}

// Solve unilateral spring contact: included feet may push, never pull.
// False means no contact equilibrium was found; fallback shares do not satisfy tipping moments.
static bool loadSplitK(const float fx[6], const float fy[6], const float knIn[6], const float *e,
                       float cx, float cy, float F[6]) {
  const float EF = 1e-6f, ED = 1e-4f;
  bool act[6];
  float kn[6], d[6];
  for (int i = 0; i < 6; i++)
    act[i] = knIn[i] > 1e-7f;
  auto solveSet = [&](const bool *a) {
    for (int i = 0; i < 6; i++)
      kn[i] = a[i] ? knIn[i] : 0;
    return springSolve(fx, fy, kn, e, cx, cy, F, d);
  };
  auto kkt = [&](const bool *a) {
    for (int i = 0; i < 6; i++) {
      if (a[i] && F[i] < -EF)
        return false;
      if (!a[i] && knIn[i] > 1e-7f && d[i] > ED)
        return false;
    }
    return true;
  };
  for (int it = 0; it < 12; it++) {
    if (!solveSet(act))
      break;
    int worst = -1;
    float wv = -EF;
    for (int i = 0; i < 6; i++)
      if (act[i] && F[i] < wv) {
        wv = F[i];
        worst = i;
      }
    if (worst >= 0) {
      act[worst] = false;
      continue;
    }
    int back = -1;
    float bv = ED;
    for (int i = 0; i < 6; i++)
      if (!act[i] && knIn[i] > 1e-7f && d[i] > bv) {
        bv = d[i];
        back = i;
      }
    if (back >= 0) {
      act[back] = true;
      continue;
    }
    return true;
  }
  for (int m = 63; m > 0; m--) {
    bool a[6];
    int cnt = 0, bad = 0;
    for (int i = 0; i < 6; i++) {
      a[i] = (m >> i) & 1;
      cnt += a[i];
      if (a[i] && knIn[i] <= 1e-7f)
        bad = 1;
    }
    if (bad || cnt < 3)
      continue;
    if (solveSet(a) && kkt(a))
      return true;
  }
  for (int i = 0; i < 6; i++)
    act[i] = knIn[i] > 1e-7f;
  solveSet(act);
  float sum = 0;
  for (int i = 0; i < 6; i++) {
    if (F[i] < 0)
      F[i] = 0;
    sum += F[i];
  }
  if (sum > 1e-6f)
    for (int i = 0; i < 6; i++)
      F[i] /= sum;
  return false;
}

static bool loadSplit(const float fx[6], const float fy[6], const float wgt[6], float cx, float cy,
                      float F[6]) {
  return loadSplitK(fx, fy, wgt, nullptr, cx, cy, F);
}

static bool peakShares(const float off[6], float duty, float width, float comX, float comY,
                       float share[6], float h = 63.7f, const float *e = nullptr,
                       float sag = 2.0f) {
  float fx[6], fy[6], kb[6], kn[6], F[6], ev[13];
  int ne = 0;
  bool feasible = true;
  for (int i = 0; i < 6; i++) {
    neutralFoot(i, width, comX, comY, fx[i], fy[i]);
    share[i] = 0;
    float q[3], r;
    kb[i] = ikRaw(i, fx[i], fy[i], -h, q, &r) ? legStiffRel(r, q[1]) / (3 * sag) : 1.0f / (3 * sag);
    ev[ne++] = fmodf(off[i] + 1.0f, 1.0f);
    ev[ne++] = fmodf(off[i] + 1 - duty + 1.0f, 1.0f);
  }
  for (int a = 1; a < ne; a++) {
    float t = ev[a];
    int b = a - 1;
    while (b >= 0 && ev[b] > t) {
      ev[b + 1] = ev[b];
      b--;
    }
    ev[b + 1] = t;
  }
  ev[ne] = ev[0] + 1;
  for (int k = 0; k < ne; k++) {
    if (ev[k + 1] - ev[k] < 1e-6f)
      continue;
    float ph = fmodf(0.5f * (ev[k] + ev[k + 1]), 1.0f);
    for (int i = 0; i < 6; i++) {
      float psi = fmodf(ph - off[i] + 1.0f, 1.0f);
      kn[i] = psi < 1 - duty ? 0.0f : kb[i];
    }
    if (!loadSplitK(fx, fy, kn, e, comX, comY, F))
      feasible = false;
    for (int i = 0; i < 6; i++)
      share[i] = fmaxf(share[i], F[i]);
  }
  return feasible;
}

static bool lsqQR(int m, int n, double A[6][6], const double b[6], double x[6]) {
  double Q[6][6], R[6][6] = {{0}}, y[6];
  for (int j = 0; j < n; j++)
    for (int i = 0; i < m; i++)
      Q[i][j] = A[i][j];
  double scale = 0;
  for (int j = 0; j < n; j++)
    for (int i = 0; i < m; i++)
      scale = fmax(scale, fabs(A[i][j]));
  for (int j = 0; j < n; j++) {
    for (int k = 0; k < j; k++) {
      double d = 0;
      for (int i = 0; i < m; i++)
        d += Q[i][k] * Q[i][j];
      R[k][j] = d;
      for (int i = 0; i < m; i++)
        Q[i][j] -= d * Q[i][k];
    }
    double L = 0;
    for (int i = 0; i < m; i++)
      L += Q[i][j] * Q[i][j];
    L = sqrt(L);
    if (L < 1e-9 * fmax(scale, 1e-12))
      return false;
    R[j][j] = L;
    for (int i = 0; i < m; i++)
      Q[i][j] /= L;
  }
  for (int j = 0; j < n; j++) {
    double d = 0;
    for (int i = 0; i < m; i++)
      d += Q[i][j] * b[i];
    y[j] = d;
  }
  for (int j = n - 1; j >= 0; j--) {
    double v = y[j];
    for (int k = j + 1; k < n; k++)
      v -= R[j][k] * x[k];
    x[j] = v / R[j][j];
  }
  return true;
}

template <class LoadFn>
// Paper-release measurements reveal only uneven foot errors; a common height/tilt plane is
// unobservable.
static float fitFootErrors(const float fx[6], const float fy[6], const float u[6],
                           LoadFn standLoads, float e[6], float uFit[6]) {
  float zero[6] = {0}, F0[6], Fa[6], S[6], G[6][6];
  standLoads(zero, -1, 0.0f, F0);
  for (int j = 0; j < 6; j++) {
    standLoads(zero, j, 0.05f, Fa);
    S[j] = (F0[j] - Fa[j]) / 0.05f;
  }
  for (int k = 0; k < 6; k++) {
    float ek[6] = {0};
    ek[k] = 0.05f;
    standLoads(ek, -1, 0.0f, Fa);
    for (int j = 0; j < 6; j++)
      G[j][k] = (Fa[j] - F0[j]) / 0.05f;
  }

  double Q[6][6];
  int nq = 0;
  auto addVec = [&](const double *v) -> bool {
    double w[6];
    memcpy(w, v, sizeof w);
    for (int a = 0; a < nq; a++) {
      double d = 0;
      for (int i = 0; i < 6; i++)
        d += w[i] * Q[a][i];
      for (int i = 0; i < 6; i++)
        w[i] -= d * Q[a][i];
    }
    double L = 0;
    for (int i = 0; i < 6; i++)
      L += w[i] * w[i];
    L = sqrt(L);
    if (L < 1e-6)
      return false;
    for (int i = 0; i < 6; i++)
      Q[nq][i] = w[i] / L;
    nq++;
    return true;
  };
  {
    double v0[6], v1[6], v2[6];
    for (int i = 0; i < 6; i++) {
      v0[i] = 1;
      v1[i] = fx[i];
      v2[i] = fy[i];
    }
    addVec(v0);
    addVec(v1);
    addVec(v2);
  }
  int planeN = nq;
  for (int k = 0; k < 6 && nq < 6; k++) {
    double v[6] = {0};
    v[k] = 1;
    addVec(v);
  }

  double B[6][6] = {{0}}, r[6], p[6];
  for (int j = 0; j < 6; j++) {
    r[j] = S[j] * u[j] - F0[j];
    for (int a = 0; a < 3; a++) {
      B[j][a] = 0;
      for (int k = 0; k < 6; k++)
        B[j][a] += G[j][k] * Q[planeN + a][k];
    }
  }
  if (!lsqQR(6, 3, B, r, p)) {
    for (int i = 0; i < 6; i++) {
      e[i] = 0;
      uFit[i] = F0[i] / S[i];
    }
    return -1;
  }
  for (int i = 0; i < 6; i++) {
    e[i] = 0;
    for (int a = 0; a < 3; a++)
      e[i] += (float)(p[a] * Q[planeN + a][i]);
  }
  double ss = 0;
  for (int j = 0; j < 6; j++) {
    float Fj = F0[j];
    for (int k = 0; k < 6; k++)
      Fj += G[j][k] * e[k];
    uFit[j] = S[j] > 1e-6f ? Fj / S[j] : 0;
    ss += (uFit[j] - u[j]) * (uFit[j] - u[j]);
  }

  auto release = [&](const float *ev, int j) -> float {
    float Fz[6];
    standLoads(ev, j, 0.0f, Fz);
    if (Fz[j] <= 1e-6f)
      return 0;
    float lo = 0, hi = 30;
    for (int it = 0; it < 24; it++) {
      float m = 0.5f * (lo + hi);
      standLoads(ev, j, m, Fz);
      if (Fz[j] > 1e-6f)
        lo = m;
      else
        hi = m;
    }
    return 0.5f * (lo + hi);
  };
  auto eOf = [&](const double *pp, float *ev) {
    for (int i = 0; i < 6; i++) {
      ev[i] = 0;
      for (int a = 0; a < 3; a++)
        ev[i] += (float)(pp[a] * Q[planeN + a][i]);
    }
  };
  auto rms = [&](const double *pp, float *uo) {
    float ev[6];
    eOf(pp, ev);
    double s2 = 0;
    for (int j = 0; j < 6; j++) {
      uo[j] = release(ev, j);
      s2 += (uo[j] - u[j]) * (uo[j] - u[j]);
    }
    return sqrt(s2 / 6);
  };
  double pc[3] = {p[0], p[1], p[2]};
  float ub[6];
  double best = rms(pc, ub);
  for (int it = 0; it < 6; it++) {
    double J[6][6] = {{0}}, rr[6], dp[6];
    float uh[6];
    for (int j = 0; j < 6; j++)
      rr[j] = u[j] - ub[j];
    for (int a = 0; a < 3; a++) {
      double ph[3] = {pc[0], pc[1], pc[2]};
      ph[a] += 0.02;
      rms(ph, uh);
      for (int j = 0; j < 6; j++)
        J[j][a] = (uh[j] - ub[j]) / 0.02;
    }
    if (!lsqQR(6, 3, J, rr, dp))
      break;
    {
      double m = fmax(fabs(dp[0]), fmax(fabs(dp[1]), fabs(dp[2])));
      if (m > 1.0)
        for (int a = 0; a < 3; a++)
          dp[a] /= m;
    }
    double pn[3] = {pc[0] + dp[0], pc[1] + dp[1], pc[2] + dp[2]};
    float un[6];
    double rn = rms(pn, un);
    if (!(rn < best))
      break;
    memcpy(pc, pn, sizeof pc);
    memcpy(ub, un, sizeof ub);
    double step = fabs(dp[0]) + fabs(dp[1]) + fabs(dp[2]);
    best = rn;
    if (step < 1e-4)
      break;
  }
  float eg[6];
  eOf(pc, eg);
  float dev = 0;
  for (int i = 0; i < 6; i++)
    dev = fmaxf(dev, fabsf(eg[i] - e[i]));
  if (best < sqrt(ss / 6) && dev <= 3.0f) {
    memcpy(e, eg, sizeof eg);
    memcpy(uFit, ub, sizeof ub);
    return (float)best;
  }
  return (float)sqrt(ss / 6);
}

static float supportMargin(const float off[6], float duty, float width, float comX, float comY) {
  float fx[6], fy[6], worst = 1e9f;
  float ev[13];
  int ne = 0;
  for (int i = 0; i < 6; i++) {
    neutralFoot(i, width, comX, comY, fx[i], fy[i]);
    ev[ne++] = fmodf(off[i] + 1.0f, 1.0f);
    ev[ne++] = fmodf(off[i] + 1 - duty + 1.0f, 1.0f);
  }
  for (int a = 1; a < ne; a++) {
    float t = ev[a];
    int b = a - 1;
    while (b >= 0 && ev[b] > t) {
      ev[b + 1] = ev[b];
      b--;
    }
    ev[b + 1] = t;
  }
  ev[ne] = ev[0] + 1;
  for (int k = 0; k < ne; k++) {
    if (ev[k + 1] - ev[k] < 1e-6f)
      continue;
    float ph = fmodf(0.5f * (ev[k] + ev[k + 1]), 1.0f);
    bool on[6];
    for (int i = 0; i < 6; i++) {
      float psi = fmodf(ph - off[i] + 1.0f, 1.0f);
      on[i] = psi >= 1 - duty;
    }
    float m = 1e9f;
    for (int a = 0; a < 6; a++)
      for (int b = 0; b < 6; b++) {
        if (a == b || !on[a] || !on[b])
          continue;
        float ex = fx[b] - fx[a], ey = fy[b] - fy[a], L = sqrtf(ex * ex + ey * ey);
        bool edge = true;
        for (int c = 0; c < 6 && edge; c++)
          if (on[c] && c != a && c != b && ex * (fy[c] - fy[a]) - ey * (fx[c] - fx[a]) < -1e-3f)
            edge = false;
        if (edge && L > 1)
          m = fminf(m, (ex * (comY - fy[a]) - ey * (comX - fx[a])) / L);
      }
    worst = fminf(worst, m);
  }
  return worst;
}

static inline float stanceWeight(float p, float ramp = 0.15f) {
  float a = gmClamp(p / ramp, 0, 1), b = gmClamp((1 - p) / ramp, 0, 1);
  return a * a * (3 - 2 * a) * b * b * (3 - 2 * b);
}

// Average one gait period to remove repeating step rocking from the leveling signal.
struct CombAvg {
  static const int NMAX = 170;
  float buf[NMAX];
  int head = 0, n = 25;
  double sum = 0;

  void reset(float v) {
    for (int k = 0; k < NMAX; k++)
      buf[k] = v;
    sum = v * n;
  }

  void setLength(int len) {
    len = len < 5 ? 5 : len > NMAX ? NMAX : len;
    if (len == n)
      return;
    n = len;
    sum = 0;
    for (int k = 0; k < n; k++)
      sum += buf[(head - 1 - k + NMAX * 2) % NMAX];
  }

  float push(float v) {
    float old = buf[(head - n + NMAX * 2) % NMAX];
    buf[head] = v;
    head = (head + 1) % NMAX;
    sum += v - old;
    return (float)(sum / n);
  }
};

static inline float zmpShift(float hc, float a) {
  return hc * a / 9810.0f;
}

// Filter squared torque fraction as a load/heat proxy; this is not a temperature measurement.
static inline void heatStep(float &ms, float tauFrac, float dt, float tauS = 90.0f) {
  ms += (tauFrac * tauFrac - ms) * (1 - expf(-dt / tauS));
}

static bool solveN(int n, double A[6][6], double b[6], double x[6]) {
  for (int c = 0; c < n; c++) {
    int pv = c;
    for (int r = c + 1; r < n; r++)
      if (fabs(A[r][c]) > fabs(A[pv][c]))
        pv = r;
    if (fabs(A[pv][c]) < 1e-12)
      return false;
    if (pv != c) {
      for (int k = 0; k < n; k++) {
        double t = A[c][k];
        A[c][k] = A[pv][k];
        A[pv][k] = t;
      }
      double t = b[c];
      b[c] = b[pv];
      b[pv] = t;
    }
    for (int r = c + 1; r < n; r++) {
      double f = A[r][c] / A[c][c];
      for (int k = c; k < n; k++)
        A[r][k] -= f * A[c][k];
      b[r] -= f * b[c];
    }
  }
  for (int r = n - 1; r >= 0; r--) {
    double v = b[r];
    for (int k = r + 1; k < n; k++)
      v -= A[r][k] * x[k];
    x[r] = v / A[r][r];
  }
  return true;
}

template <class LoadFn>
// Fit uneven foot errors and one common scale-platform offset from the six measured weight shares.
static float fitFootLoads(const float fx[6], const float fy[6], const float meas[6],
                          LoadFn standLoads, float e[6], float &delta, float fit[6]) {
  float zero[6] = {0}, F0[6], Fa[6], G[6][6];
  standLoads(zero, -1, 0.0f, F0);
  for (int k = 0; k < 6; k++) {
    float ek[6] = {0};
    ek[k] = 0.05f;
    standLoads(ek, -1, 0.0f, Fa);
    for (int j = 0; j < 6; j++)
      G[j][k] = (Fa[j] - F0[j]) / 0.05f;
  }
  double Q[6][6];
  int nq = 0;
  auto addVec = [&](const double *v) {
    double w[6];
    memcpy(w, v, sizeof w);
    for (int a = 0; a < nq; a++) {
      double d = 0;
      for (int i = 0; i < 6; i++)
        d += w[i] * Q[a][i];
      for (int i = 0; i < 6; i++)
        w[i] -= d * Q[a][i];
    }
    double L = 0;
    for (int i = 0; i < 6; i++)
      L += w[i] * w[i];
    L = sqrt(L);
    if (L < 1e-6)
      return;
    for (int i = 0; i < 6; i++)
      Q[nq][i] = w[i] / L;
    nq++;
  };
  {
    double v0[6], v1[6], v2[6];
    for (int i = 0; i < 6; i++) {
      v0[i] = 1;
      v1[i] = fx[i];
      v2[i] = fy[i];
    }
    addVec(v0);
    addVec(v1);
    addVec(v2);
  }
  int pn = nq;
  for (int k = 0; k < 6 && nq < 6; k++) {
    double v[6] = {0};
    v[k] = 1;
    addVec(v);
  }
  const int NU = 4;
  double B[6][6] = {{0}}, r[6], x[6];
  for (int j = 0; j < 6; j++) {
    r[j] = meas[j] - F0[j];
    for (int a = 0; a < 3; a++) {
      B[j][a] = 0;
      for (int k = 0; k < 6; k++)
        B[j][a] += G[j][k] * Q[pn + a][k];
    }
    B[j][3] = G[j][j];
  }
  if (!lsqQR(6, NU, B, r, x))
    return -1;
  for (int i = 0; i < 6; i++) {
    e[i] = 0;
    for (int a = 0; a < 3; a++)
      e[i] += (float)(x[a] * Q[pn + a][i]);
  }
  delta = (float)x[3];
  double ss = 0;
  for (int j = 0; j < 6; j++) {
    double f = F0[j] + G[j][j] * delta;
    for (int k = 0; k < 6; k++)
      f += G[j][k] * e[k];
    fit[j] = (float)f;
    ss += (f - meas[j]) * (f - meas[j]);
  }

  auto model = [&](const double *xx, float *fo) {
    float ev[6];
    for (int i = 0; i < 6; i++) {
      ev[i] = 0;
      for (int a = 0; a < 3; a++)
        ev[i] += (float)(xx[a] * Q[pn + a][i]);
    }
    double s2 = 0;
    for (int j = 0; j < 6; j++) {
      float ej[6], Fz[6];
      memcpy(ej, ev, sizeof ej);
      ej[j] += (float)xx[3];
      standLoads(ej, -1, 0.0f, Fz);
      fo[j] = Fz[j];
      s2 += (fo[j] - meas[j]) * (fo[j] - meas[j]);
    }
    return sqrt(s2 / 6);
  };
  bool ident = true;
  for (int j = 0; j < 6; j++)
    if (meas[j] < 0.02f)
      ident = false;
  double xc[4] = {x[0], x[1], x[2], x[3]};
  float fb[6];
  double best = model(xc, fb);
  for (int it = 0; it < 6 && ident; it++) {
    double J[6][6] = {{0}}, rr[6], dx[6];
    float fh[6];
    for (int j = 0; j < 6; j++)
      rr[j] = meas[j] - fb[j];
    for (int a = 0; a < 4; a++) {
      double xh[4] = {xc[0], xc[1], xc[2], xc[3]};
      xh[a] += 0.02;
      model(xh, fh);
      for (int j = 0; j < 6; j++)
        J[j][a] = (fh[j] - fb[j]) / 0.02;
    }
    if (!lsqQR(6, 4, J, rr, dx))
      break;
    {
      double m = 0;
      for (int a = 0; a < 4; a++)
        m = fmax(m, fabs(dx[a]));
      if (m > 1.0)
        for (int a = 0; a < 4; a++)
          dx[a] /= m;
    }
    double xn[4] = {xc[0] + dx[0], xc[1] + dx[1], xc[2] + dx[2], xc[3] + dx[3]};
    float fn[6];
    double rn = model(xn, fn);
    if (!(rn < best))
      break;
    memcpy(xc, xn, sizeof xc);
    memcpy(fb, fn, sizeof fb);
    best = rn;
    if (fabs(dx[0]) + fabs(dx[1]) + fabs(dx[2]) + fabs(dx[3]) < 1e-4)
      break;
  }
  float eg[6], dev = fabsf((float)xc[3] - delta);
  for (int i = 0; i < 6; i++) {
    eg[i] = 0;
    for (int a = 0; a < 3; a++)
      eg[i] += (float)(xc[a] * Q[pn + a][i]);
    dev = fmaxf(dev, fabsf(eg[i] - e[i]));
  }
  if (best < sqrt(ss / 6) && dev <= 3.0f) {
    memcpy(e, eg, sizeof eg);
    delta = (float)xc[3];
    memcpy(fit, fb, sizeof fb);
    return (float)best;
  }
  return (float)sqrt(ss / 6);
}

// Fit V = rest_voltage - resistance_proxy * load_proxy; this is a voltage estimate, not a current
// sensor.
struct RestVoltKF {
  float vr = 0, b = 0.03f, P00 = 0, P01 = 0, P11 = 0;
  bool init = false;
  int gated = 0;
  static constexpr float qV = 5e-5f;
  static constexpr float qB = 2.5e-7f;
  static constexpr float r = 9e-4f;

  void reset() {
    init = false;
    gated = 0;
  }

  void update(float V, float x, float dt) {
    if (!init) {
      vr = V + b * x;
      P00 = 0.01f;
      P01 = 0;
      P11 = 0.0025f;
      init = true;
      return;
    }
    P00 += qV * dt;
    P11 += qB * dt;
    float Ph0 = P00 - P01 * x, Ph1 = P01 - P11 * x;
    float S = Ph0 - x * Ph1 + r;
    float innov = V - (vr - b * x);
    if (innov * innov > 25 * S) {
      if (++gated >= 5)
        reset();
      return;
    }
    gated = 0;
    float K0 = Ph0 / S, K1 = Ph1 / S;
    vr += K0 * innov;
    b += K1 * innov;

    float a00 = 1 - K0, a01 = K0 * x, a10 = -K1, a11 = 1 + K1 * x;
    float t00 = a00 * P00 + a01 * P01, t01 = a00 * P01 + a01 * P11, t10 = a10 * P00 + a11 * P01,
          t11 = a10 * P01 + a11 * P11;
    P00 = t00 * a00 + t01 * a01 + r * K0 * K0;
    P01 = t00 * a10 + t01 * a11 + r * K0 * K1;
    P11 = t10 * a10 + t11 * a11 + r * K1 * K1;
    b = gmClamp(b, 0.0f, 0.5f);
  }
};

// Track a unit up vector in body coordinates. Gyro input is rad/s; acceleration supplies the
// gravity direction.
struct UpFilter {
  float u[3] = {0, 0, 1}, bI[3] = {0, 0, 0};
  bool init = false;

  void reset(const float a[3], float trust = 1) {

    init = false;
    bI[0] = bI[1] = bI[2] = 0;
    float n2 = a[0] * a[0] + a[1] * a[1] + a[2] * a[2];
    if (!isfinite(trust) || trust < .5f || !isfinite(n2) || n2 <= .09f)
      return;
    float n = sqrtf(n2);
    for (int k = 0; k < 3; k++)
      u[k] = a[k] / n;
    init = true;
  }

  void update(const float a[3], const float w[3], float dt, float trust, float kp = 1.0f,
              float ki = 0.02f) {
    if (!isfinite(dt) || dt <= 0 || !isfinite(trust)) {
      init = false;
      return;
    }
    for (int k = 0; k < 3; k++)
      if (!isfinite(a[k]) || !isfinite(w[k])) {
        init = false;
        return;
      }
    trust = gmClamp(trust, 0, 1);
    if (!init) {
      reset(a, trust);
      return;
    }
    float n = sqrtf(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), e[3] = {0, 0, 0};
    if (n > 0.3f) {
      float ah[3] = {a[0] / n, a[1] / n, a[2] / n};
      e[0] = ah[1] * u[2] - ah[2] * u[1];
      e[1] = ah[2] * u[0] - ah[0] * u[2];
      e[2] = ah[0] * u[1] - ah[1] * u[0];
    }
    float theta[3];
    for (int k = 0; k < 3; k++) {
      bI[k] += ki * trust * e[k] * dt;
      theta[k] = (w[k] + kp * trust * e[k] + bI[k]) * dt;
    }
    float t2 = theta[0] * theta[0] + theta[1] * theta[1] + theta[2] * theta[2], A, B;
    if (!isfinite(t2)) {
      init = false;
      return;
    }

    if (t2 < 1e-4f) {
      A = 1 - t2 / 6 + t2 * t2 / 120;
      B = .5f - t2 / 24 + t2 * t2 / 720;
    } else {
      float t = sqrtf(t2);
      A = sinf(t) / t;
      B = (1 - cosf(t)) / t2;
    }
    float du[3] = {u[1] * theta[2] - u[2] * theta[1], u[2] * theta[0] - u[0] * theta[2],
                   u[0] * theta[1] - u[1] * theta[0]};
    float ddu[3] = {du[1] * theta[2] - du[2] * theta[1], du[2] * theta[0] - du[0] * theta[2],
                    du[0] * theta[1] - du[1] * theta[0]};
    for (int k = 0; k < 3; k++)
      u[k] += A * du[k] + B * ddu[k];
    float m = sqrtf(u[0] * u[0] + u[1] * u[1] + u[2] * u[2]);
    if (!isfinite(m) || m < .3f) {
      init = false;
      return;
    }
    for (int k = 0; k < 3; k++)
      u[k] /= m;
  }

  float rollDeg() const {
    return atan2f(u[1], u[2]) * GM_R2D;
  }

  float pitchDeg() const {
    return atan2f(u[0], sqrtf(u[1] * u[1] + u[2] * u[2])) * GM_R2D;
  }
};

static float energyMargin(const float fx[6], const float fy[6], const float fz[6], const bool on[6],
                          float cx, float cy, float cz, const float up[3]) {
  float best = 1e9f;
  int nOn = 0;
  float n2 = up[0] * up[0] + up[1] * up[1] + up[2] * up[2];
  if (!isfinite(cx) || !isfinite(cy) || !isfinite(cz) || !isfinite(n2) || fabsf(n2 - 1) > .01f)
    return 0;
  for (int i = 0; i < 6; i++)
    if (on[i]) {
      if (!isfinite(fx[i]) || !isfinite(fy[i]) || !isfinite(fz[i]))
        return 0;
      nOn++;
    }
  if (nOn < 3)
    return 0;
  float area = 0;
  for (int a = 0; a < 6; a++)
    for (int b = a + 1; b < 6; b++)
      for (int c = b + 1; c < 6; c++)
        if (on[a] && on[b] && on[c])
          area = fmaxf(
              area, fabsf((fx[b] - fx[a]) * (fy[c] - fy[a]) - (fy[b] - fy[a]) * (fx[c] - fx[a])));
  if (area < 1e-3f)
    return 0;
  for (int a = 0; a < 6; a++)
    for (int b = 0; b < 6; b++) {
      if (a == b || !on[a] || !on[b])
        continue;
      float ex = fx[b] - fx[a], ey = fy[b] - fy[a];
      bool edge = true;
      for (int c = 0; c < 6 && edge; c++)
        if (on[c] && c != a && c != b && ex * (fy[c] - fy[a]) - ey * (fx[c] - fx[a]) < -1e-3f)
          edge = false;
      if (!edge)
        continue;
      float e3[3] = {fx[b] - fx[a], fy[b] - fy[a], fz[b] - fz[a]},
            L = sqrtf(e3[0] * e3[0] + e3[1] * e3[1] + e3[2] * e3[2]);
      if (L < 1)
        continue;
      for (int k = 0; k < 3; k++)
        e3[k] /= L;
      float v[3] = {cx - fx[a], cy - fy[a], cz - fz[a]},
            t = v[0] * e3[0] + v[1] * e3[1] + v[2] * e3[2];
      float R[3] = {v[0] - t * e3[0], v[1] - t * e3[1], v[2] - t * e3[2]};
      float r = sqrtf(R[0] * R[0] + R[1] * R[1] + R[2] * R[2]),
            eu = e3[0] * up[0] + e3[1] * up[1] + e3[2] * up[2];
      float Ru = R[0] * up[0] + R[1] * up[1] + R[2] * up[2];

      float side = (e3[1] * v[2] - e3[2] * v[1]) * up[0] + (e3[2] * v[0] - e3[0] * v[2]) * up[1] +
                   (e3[0] * v[1] - e3[1] * v[0]) * up[2];
      float h = r * sqrtf(fmaxf(0.0f, 1 - eu * eu)) - Ru;
      best = fminf(best, side >= 0 ? h : -h);
    }
  return best == 1e9f || !isfinite(best) ? 0 : best;
}

static inline float accelTrust(float aMagG, float sigma = 0.12f) {
  float e = (aMagG - 1) / sigma;
  return expf(-e * e);
}

static inline float deadzone(float v, float d) {
  return v > d ? v - d : v < -d ? v + d : 0;
}
