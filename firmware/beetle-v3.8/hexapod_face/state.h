// Parse ESP32 telemetry and smooth values for display. Optional trailing fields support older
// senders.

#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

static const float HIP_OUT = 38.0f, HIP_UP = 25.9f, FEMUR = 65.9f, TIBIA = 108.64f,
                   SIDE = 16.7f + 17.6f, COXA_Z = 19.0f;
static const float PARK_H = 37.0f, BAT_PRESENT = 5.0f, STAND_CAD_H = 63.7f;

struct LegDef {
  const char *name;
  float x, y, out;
};

static const LegDef LEG[6] = {{"FL", 51.677f, 22.323f, 45},   {"ML", 5.2f, 33.0f, 90},
                              {"RL", -51.677f, 22.323f, 135}, {"FR", 51.677f, -22.323f, -45},
                              {"MR", 5.2f, -33.0f, -90},      {"RR", -51.677f, -22.323f, -135}};
static const char *const GAIT_NAME[3] = {"TRIPOD", "RIPPLE", "WAVE"};

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

static const char *const TRICK_TITLE[TR_COUNT] = {
    "",       "HI!",  "BOW",     "TWIST", "DANCE",  "PUSH-UPS", "LOOK",   "STOMP", "SHAKE",
    "YES",    "NOPE", "STRETCH", "BEG",   "CIRCLE", "HEADBANG", "RIPPLE", "POINT", "TAP TAP",
    "SLEEPY", "BRRR", "POUNCE",  "SWAY",  "FLEX",   "HUH?",     "EEK!"};
static const float TRICK_DUR[TR_COUNT] = {0,    2.0f, 3.2f, 3.6f, 7.2f, 4.8f, 4.6f, 3.4f, 1.8f,
                                          2.4f, 2.4f, 5.6f, 5.6f, 4.8f, 4.2f, 4.2f, 3.6f, 2.6f,
                                          5.4f, 2.0f, 4.4f, 4.8f, 3.8f, 3.8f, 3.6f};

enum Page {
  PG_FACE,
  PG_DASH,
  PG_LEGS,
  PG_LEVEL,
  PG_SYS,
  PG_OFF,
  PG_TRAIL,
  PG_STAB,
  PG_HEAT,
  PG_DRIVE,
  PG_COUNT
};

static const int PAGE_CYCLE[9] = {PG_FACE, PG_DASH,  PG_LEGS,  PG_TRAIL, PG_STAB,
                                  PG_HEAT, PG_DRIVE, PG_LEVEL, PG_SYS};

static inline int cyclePos(int p) {
  for (int k = 0; k < 9; k++)
    if (PAGE_CYCLE[k] == p)
      return k;
  return 0;
}

enum Park { P_NONE, P_LOWERING, P_PARKED };

enum Evt { EV_NONE, EV_TAP, EV_DOUBLE, EV_TRIPLE, EV_HOLD, EV_BT_ON, EV_BT_OFF, EV_CMD };

struct Robot {
  int ver = 0;
  float batV = 0;
  int batPct = -1;
  uint32_t flags = 0;
  int gait = 0;
  float speed = 0, targetH = 63.7f, bodyH = 63.7f, pitch = 0, roll = 0;
  int trick = 0;
  uint32_t trickMs = 0;
  float yaw = 0;
  uint8_t swing = 0;
  int park = 0;
  char move = 0;
  int page = 0, rot = 1, inv = 1, mhz = 75, test = 0;
  float width = 0;
  uint32_t upS = 0;
  float feet[6][3] = {};
  float vx = 0, vy = 0, wz = 0;
  uint32_t evt = 0;
  float lift = 25, period = 1;
  int party = 0;
  float amp = 1;
  float pTx = 0, pTy = 0, pTz = 0, pRoll = 0, pPitch = 0;
  int heat[6][2] = {{-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}, {-1, -1}};

  float gov = 1, comX = 0, comY = 0;
  int idleLeft = -1;
  uint32_t flags2 = 0, trailReset = 0;

  bool tipping() const {
    return flags2 & 1;
  }

  bool stumble() const {
    return flags2 & 2;
  }

  bool boardDown() const {
    return flags2 & 4;
  }

  bool joyHold() const {
    return flags2 & 8;
  }

  bool imuLost() const {
    return flags2 & 16;
  }

  bool balanceHold() const {
    return flags2 & 32;
  }

  bool gaitMode() const {
    return flags & 1;
  }

  bool servosOn() const {
    return flags & 2;
  }

  bool batLocked() const {
    return flags & 4;
  }

  bool stepping() const {
    return flags & 8;
  }

  bool sitting() const {
    return flags & 16;
  }

  bool imuOK() const {
    return flags & 32;
  }

  bool hasFront() const {
    return flags & 64;
  }

  bool levelOn() const {
    return flags & 128;
  }

  bool swapped() const {
    return flags & 256;
  }

  bool btClient() const {
    return flags & 512;
  }

  bool sleeping() const {
    return flags & 1024;
  }

  bool perfBox() const {
    return flags & 2048;
  }

  bool fidget() const {
    return flags & 4096;
  }

  bool calibrated() const {
    return flags & 8192;
  }

  bool onBatt() const {
    return batV >= BAT_PRESENT;
  }

  bool walking() const {
    return move != 0 || stepping();
  }

  int pct() const;
};

static int batPercentFromV(float v) {
  const float V[] = {6.6f, 7.0f, 7.2f, 7.4f, 7.6f, 7.8f, 8.0f, 8.4f};
  const float P[] = {0, 5, 15, 30, 50, 70, 85, 100};
  if (v <= V[0])
    return 0;
  for (int k = 1; k < 8; k++)
    if (v <= V[k])
      return (int)(P[k - 1] + (P[k] - P[k - 1]) * (v - V[k - 1]) / (V[k] - V[k - 1]));
  return 100;
}

inline int Robot::pct() const {
  return batPct >= 0 ? batPct : batPercentFromV(batV);
}

static float batPercentFromVf(float v) {
  const float V[] = {6.6f, 7.0f, 7.2f, 7.4f, 7.6f, 7.8f, 8.0f, 8.4f};
  const float P[] = {0, 5, 15, 30, 50, 70, 85, 100};
  if (v <= V[0])
    return 0;
  for (int k = 1; k < 8; k++)
    if (v <= V[k])
      return P[k - 1] + (P[k] - P[k - 1]) * (v - V[k - 1]) / (V[k] - V[k - 1]);
  return 100;
}

// Keep field indices in sync with ESP32 screen.ino; optional appended fields leave older senders
// usable.
static bool parseStatus(char *s, Robot &r) {
  if (s[0] != 'H' || s[1] != ',')
    return false;
  long v[96];
  int n = 0;
  char *p = s + 2;
  while (*p && n < 96) {
    char *e;
    v[n++] = strtol(p, &e, 10);
    if (e == p)
      return false;
    p = e;
    if (*p == ',')
      p++;
    else
      break;
  }
  if (n < 41 || (v[0] != 1 && v[0] != 2))
    return false;
  r.ver = v[0];
  r.batV = v[1] / 100.0f;
  r.batPct = v[2];
  r.flags = v[3];
  r.gait = v[4] < 0 ? 0 : v[4] > 2 ? 2 : (int)v[4];
  r.speed = v[5];
  r.targetH = v[6];
  r.bodyH = v[7] / 10.0f;
  r.pitch = v[8] / 10.0f;
  r.roll = v[9] / 10.0f;
  r.trick = (v[10] >= 0 && v[10] < TR_COUNT) ? v[10] : 0;
  r.trickMs = v[11];
  r.yaw = v[12] / 10.0f;
  r.swing = v[13];
  r.park = v[14];
  r.move = (char)v[15];
  r.page = v[16];
  r.rot = v[17];
  r.inv = v[18];
  r.mhz = v[19];
  r.test = v[20];
  r.width = v[21];
  r.upS = v[22];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      r.feet[i][j] = v[23 + i * 3 + j];
  if (n >= 49) {
    r.vx = v[41];
    r.vy = v[42];
    r.wz = v[43] / 1000.0f;
    r.evt = v[44];
    r.lift = v[45];
    r.period = v[46] / 100.0f;
    r.party = v[47];
    r.amp = v[48] / 100.0f;
  }
  if (n >= 54) {
    r.pTx = v[49];
    r.pTy = v[50];
    r.pTz = v[51];
    r.pRoll = v[52] / 10.0f;
    r.pPitch = v[53] / 10.0f;
  }
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 2; j++)
      r.heat[i][j] = n >= 66 ? (int)v[54 + i * 2 + j] : -1;
  if (n >= 72) {
    r.gov = v[66] / 100.0f;
    r.comX = v[67];
    r.comY = v[68];
    r.idleLeft = (int)v[69];
    r.flags2 = v[70];
    r.trailReset = v[71];
  } else {
    r.gov = 1;
    r.comX = r.comY = 0;
    r.idleLeft = -1;
    r.flags2 = 0;
  }
  return true;
}

static const int NS = 27;

struct Smoother {
  float from[NS], to[NS], now[NS];
  uint32_t t0 = 0;
  float ivl = 33;
  bool init = false;

  static void pack(const Robot &r, float *a) {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        a[i * 3 + j] = r.feet[i][j];
    a[18] = r.pitch;
    a[19] = r.roll;
    a[20] = r.yaw;
    a[21] = r.bodyH;
    a[22] = r.pTx;
    a[23] = r.pTy;
    a[24] = r.pTz;
    a[25] = r.pRoll;
    a[26] = r.pPitch;
  }

  void push(const Robot &r, uint32_t ms) {
    float t[NS];
    pack(r, t);
    if (init) {
      float d = (float)(ms - t0);
      if (d > 5 && d < 200)
        ivl += (d - ivl) * 0.2f;
    }
    for (int k = 0; k < NS; k++) {
      from[k] = init ? now[k] : t[k];
      to[k] = t[k];
    }
    t0 = ms;
    init = true;
  }

  void apply(Robot &r, uint32_t ms) {
    if (!init)
      return;
    float u = (ms - t0) / (ivl > 10 ? ivl : 10);
    u = u < 0 ? 0 : u > 1 ? 1 : u;
    for (int k = 0; k < NS; k++)
      now[k] = from[k] + (to[k] - from[k]) * u;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        r.feet[i][j] = now[i * 3 + j];
    r.pitch = now[18];
    r.roll = now[19];
    r.yaw = now[20];
    r.bodyH = now[21];
    r.pTx = now[22];
    r.pTy = now[23];
    r.pTz = now[24];
    r.pRoll = now[25];
    r.pPitch = now[26];
  }
};

static void neutralFoot(int i, float width, float &nx, float &ny) {
  float a = LEG[i].out * 0.0174533f;
  nx = LEG[i].x + (HIP_OUT + FEMUR + width) * cosf(a) + SIDE * sinf(a);
  ny = LEG[i].y + (HIP_OUT + FEMUR + width) * sinf(a) - SIDE * cosf(a);
}
