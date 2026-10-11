// Idle routines and event reactions. One animation owner chooses targets; springs smooth the eyes.

#pragma once

enum IdleAct {
  ID_SCAN,
  ID_DOUBLETAKE,
  ID_GRIN,
  ID_CURIOUS,
  ID_DOZE,
  ID_STRETCH,
  ID_SMUG,
  ID_HEART,
  ID_SPARKLE,
  ID_BOP,
  ID_PEEK,
  ID_TRACK,
  ID_COUNT
};

static const float IDLE_DUR[ID_COUNT] = {4.5f, 3.2f, 2.5f, 4.0f, 5.2f, 4.2f,
                                         3.3f, 3.2f, 3.4f, 4.2f, 4.4f, 4.6f};

enum FaceAct {
  FA_NONE,
  FA_START,
  FA_STOP,
  FA_WAKE,
  FA_SIT,
  FA_GREET,
  FA_AFFECTION,
  FA_DOUBLE,
  FA_TRIPLE,
  FA_HOLD,
  FA_BYE,
  FA_NOTICE,
  FA_FINISH
};

struct EyeVelocity {
  float dx = 0, dy = 0, w = 0, h = 0;
  float red = R8(C_EYE), green = G8(C_EYE), blue = B8(C_EYE);
  bool colorReady = false;
};

struct Personality {
  int style = 1;
  bool demo = false;
  int demoKind = 0;
  float demoT0 = 0;
  float vx = 0, vy = 0, vr = 0;
  EyeVelocity eyeV[2];
  float stepY = 0, stepV = 0, stepLean = 0, stepLeanV = 0, lastLanding = -10;
  float seenTouch[6] = {-10, -10, -10, -10, -10, -10};
  float wink[2] = {}, smile = 0, mouthO = 0, blush = 0;
  float smileT = 0, mouthOT = 0, blushT = 0, speedLines = 0;
  int idle = -1, idleBag[ID_COUNT] = {}, bagPos = ID_COUNT, lastIdle = -1;
  float idleT0 = -10, nextIdle = 7, idleSide = 1;
  uint32_t idleCues = 0;
  int act = FA_NONE, finishedTrick = TR_NONE;
  float actT0 = -10, actDur = 0;
  float lastReactT0 = -10, linkT0 = -10;
  bool hadLink = false, observed = false, wasWalking = false, wasBT = false;
  bool wasOn = false, wasSitting = false;
  int wasPark = P_NONE, wasTrick = TR_NONE;
  float prevTrickTime = -1;
  uint32_t trickCues = 0;
};

static Personality A;

static float animGain() {
  return A.style == 0 ? 0.55f : A.style == 2 ? 1.1f : 0.8f;
}

// Solve the damped spring over dt instead of Euler-stepping it, keeping the motion stable across
// frame rates.
static float springStep(float x, float target, float &velocity, float dt, float omega,
                        float damping, float lo, float hi) {
  dt = clampf(dt, 0, 0.1f);
  if (dt <= 0)
    return x;
  float y = x - target, v = velocity;
  if (fabsf(damping - 1) < 0.001f) {
    float decay = expf(-omega * dt), b = v + omega * y;
    x = target + (y + b * dt) * decay;
    velocity = (v - omega * b * dt) * decay;
  } else if (damping < 1) {
    float a = damping * omega, w = omega * sqrtf(1 - damping * damping);
    float decay = expf(-a * dt), cs = cosf(w * dt), sn = sinf(w * dt);
    x = target + decay * (y * cs + (v + a * y) * sn / w);
    velocity = decay * (v * cs - (a * v + omega * omega * y) * sn / w);
  } else {
    float root = sqrtf(damping * damping - 1);
    float r1 = -omega * (damping - root), r2 = -omega * (damping + root);
    float c1 = (v - r2 * y) / (r1 - r2), c2 = y - c1;
    float e1 = expf(r1 * dt), e2 = expf(r2 * dt);
    x = target + c1 * e1 + c2 * e2;
    velocity = r1 * c1 * e1 + r2 * c2 * e2;
  }
  if (x < lo) {
    x = lo;
    if (velocity < 0)
      velocity = 0;
  }
  if (x > hi) {
    x = hi;
    if (velocity > 0)
      velocity = 0;
  }
  return x;
}

static uint16_t eyeColor(EyeVelocity &v, uint16_t current, uint16_t target, float dt, float tau) {

  if (!v.colorReady) {
    v.red = R8(current);
    v.green = G8(current);
    v.blue = B8(current);
    v.colorReady = true;
  }
  v.red = approach(v.red, R8(target), dt, tau);
  v.green = approach(v.green, G8(target), dt, tau);
  v.blue = approach(v.blue, B8(target), dt, tau);
  return RGB((uint8_t)clampf(v.red + 0.5f, 0, 255), (uint8_t)clampf(v.green + 0.5f, 0, 255),
             (uint8_t)clampf(v.blue + 0.5f, 0, 255));
}

static float glide(float t, float start, float duration, float from, float to) {
  return from + (to - from) * easeIO((t - start) / duration);
}

static float pulse(float t, float a, float b) {
  if (t <= a || t >= b)
    return 0;
  float s = (t - a) / (b - a);
  float wave = sinf(s * 3.14159265f);
  return wave * wave;
}

static float winkPulse(float t, float start) {
  float s = t - start;
  if (s < 0 || s >= 0.48f)
    return 0;
  if (s < 0.10f)
    return easeIO(s / 0.10f);
  if (s < 0.28f)
    return 1;
  return 1 - easeIO((s - 0.28f) / 0.20f);
}

static void faceBurst(PType type, uint16_t col, int count = 5) {
  if (A.style == 0)
    count = (count + 1) / 2;
  for (int i = 0; i < count; i++) {
    float angle = (-155 + i * 130.0f / mx(count - 1, 1)) * 0.0174533f;
    spawn(type, i & 1 ? 246 : 74, 92, cosf(angle) * 42, sinf(angle) * 52, frand(0.9f, 1.5f),
          frand(0.6f, 0.9f), col);
  }
}

static void startFaceAct(int act, float dur) {
  A.act = act;
  A.actT0 = U.t;
  A.actDur = dur;
  A.idle = -1;
  A.nextIdle = U.t + dur + frand(6, 12);
}

static int nextIdleAct() {
  if (A.bagPos >= ID_COUNT) {
    for (int i = 0; i < ID_COUNT; i++)
      A.idleBag[i] = i;
    for (int i = ID_COUNT - 1; i > 0; i--) {
      int j = rand() % (i + 1), tmp = A.idleBag[i];
      A.idleBag[i] = A.idleBag[j];
      A.idleBag[j] = tmp;
    }
    if (A.idleBag[0] == A.lastIdle) {
      int tmp = A.idleBag[0];
      A.idleBag[0] = A.idleBag[1];
      A.idleBag[1] = tmp;
    }
    A.bagPos = 0;
  }
  A.lastIdle = A.idleBag[A.bagPos++];
  return A.lastIdle;
}

static bool faceUrgent(const Robot &R) {
  return R.batLocked() || R.sleeping() || R.tipping() || R.stumble() || R.boardDown() ||
         R.balanceHold() || (R.imuOK() && mx(fabsf(R.pitch), fabsf(R.roll)) > 16);
}

static bool idleAllowed(const Robot &R) {
  return U.linkOK && U.page == PG_FACE && R.servosOn() && !R.walking() && R.trick == TR_NONE &&
         !R.party && R.park == P_NONE && !faceUrgent(R) && !R.imuLost() &&
         (R.idleLeft < 0 || R.idleLeft > 60) && (!R.onBatt() || R.pct() > 20) &&
         U.t - U.tipT0 > 4.2f && U.t - U.stumT0 > 2.1f;
}

// Latch each cue at its threshold crossing so a low frame rate does not repeat or miss the event.
static bool trickCue(unsigned bit, float threshold, float tt) {
  uint32_t mask = 1u << bit;
  if (tt >= threshold && A.prevTrickTime < threshold && !(A.trickCues & mask)) {
    A.trickCues |= mask;
    return true;
  }
  return false;
}

static float finishDuration(int trick) {
  switch (trick) {
  case TR_PUSHUP:
  case TR_HEADBANG:
  case TR_SLEEPY:
    return 2.0f;
  case TR_STRETCH:
  case TR_SHIVER:
  case TR_SCARED:
    return 1.8f;
  case TR_DANCE:
    return 1.6f;
  case TR_BOW:
  case TR_BEG:
    return 1.5f;
  case TR_WAVE:
  case TR_POUNCE:
  case TR_TWIST:
  case TR_CIRCLE:
  case TR_SWAY:
    return 1.4f;
  default:
    return 1.2f;
  }
}

// Choose one social reaction at a time; urgent robot state takes priority over idle acting.
static void personalityUpdate(const Robot &R, float dt) {
  A.wink[0] = A.wink[1] = 0;
  A.smileT = A.mouthOT = A.blushT = 0;
  if (!U.linkOK) {
    A.hadLink = A.observed = false;
    A.act = FA_NONE;
    A.idle = -1;
    A.nextIdle = U.t + 6;
    A.stepY = A.stepV = A.stepLean = A.stepLeanV = 0;
    return;
  }
  if (!A.hadLink) {
    A.hadLink = true;
    A.linkT0 = mx(U.t, 2.4f);
    A.nextIdle = U.t + frand(6, 10);
    A.wasTrick = R.trick;
    A.prevTrickTime = R.trickMs / 1000.0f;
    A.trickCues = 0;
    for (int i = 0; i < 6; i++)
      A.seenTouch[i] = U.touchT[i];
  }
  if (A.act != FA_NONE && U.t - A.actT0 >= A.actDur)
    A.act = FA_NONE;
  bool urgent = faceUrgent(R);
  if (urgent || R.trick != TR_NONE || R.walking() || R.party)
    A.idle = -1;
  if (urgent) {
    A.act = FA_NONE;
    A.nextIdle = U.t + 8;
  }

  if (A.observed && !urgent) {
    if (R.walking() != A.wasWalking && R.trick == TR_NONE) {
      startFaceAct(R.walking() ? FA_START : FA_STOP, R.walking() ? 0.65f : 1.0f);
      A.stepV = (R.walking() ? -24 : 16) * animGain();
    } else if ((!A.wasOn && R.servosOn()) || (A.wasPark != P_NONE && R.park == P_NONE)) {
      startFaceAct(FA_WAKE, 1.6f);
      faceBurst(PT_SPARK, C_EYE, 4);
    } else if (R.sitting() != A.wasSitting && R.servosOn() && R.trick == TR_NONE) {
      startFaceAct(R.sitting() ? FA_SIT : FA_WAKE, 1.2f);
    }
    if (A.wasTrick != TR_NONE && R.trick == TR_NONE) {
      A.finishedTrick = A.wasTrick;
      startFaceAct(FA_FINISH, finishDuration(A.wasTrick));
      if (A.wasTrick == TR_DANCE || A.wasTrick == TR_POUNCE || A.wasTrick == TR_FLEX)
        faceBurst(PT_SPARK, A.wasTrick == TR_FLEX ? C_GOLD : C_EYE, 4);
    }
    if (R.btClient() != A.wasBT && U.t - U.reactT0 > 0.15f) {
      startFaceAct(R.btClient() ? FA_GREET : FA_BYE, 1.8f);
    }
  }
  if (R.trick != A.wasTrick) {
    A.prevTrickTime = R.trick == TR_NONE ? -1 : R.trickMs / 1000.0f - 0.001f;
    A.trickCues = 0;
  }
  if (U.reactT0 != A.lastReactT0) {
    A.lastReactT0 = U.reactT0;
    if (!urgent && R.trick == TR_NONE)
      switch (U.react) {
      case EV_TAP:
        startFaceAct(FA_AFFECTION, 2.1f);
        faceBurst(PT_HEART, C_PINK, 4);
        break;
      case EV_DOUBLE:
        startFaceAct(FA_DOUBLE, 1.2f);
        break;
      case EV_TRIPLE:
        startFaceAct(FA_TRIPLE, 1.5f);
        faceBurst(PT_SPARK, C_GOLD, 4);
        break;
      case EV_HOLD:
        startFaceAct(FA_HOLD, 1.6f);
        break;
      case EV_BT_ON:
        startFaceAct(FA_GREET, 1.8f);
        faceBurst(PT_SPARK, C_EYE, 4);
        break;
      case EV_BT_OFF:
        startFaceAct(FA_BYE, 1.8f);
        break;
      case EV_CMD:
        if (A.act == FA_NONE && !R.walking())
          startFaceAct(FA_NOTICE, 0.32f);
        break;
      default:
        break;
      }
  }

  int landings = 0;
  float lean = 0;
  for (int i = 0; i < 6; i++) {
    if (U.touchT[i] != A.seenTouch[i]) {
      A.seenTouch[i] = U.touchT[i];
      if (U.t - U.touchT[i] < 0.15f) {
        landings++;
        lean += i < 3 ? -1 : 1;
      }
    }
  }
  if (landings && R.walking() && R.trick == TR_NONE && !urgent) {
    float speed = clampf(sqrtf(R.vx * R.vx + R.vy * R.vy) * R.gov / 70, 0.2f, 1);
    A.stepV = clampf(A.stepV + mn(landings, 3) * (8 + speed * 7) * animGain(), -35, 45);
    A.stepLeanV += clampf(lean, -2, 2) * 0.22f * animGain();
    A.lastLanding = U.t;
  }
  A.stepY = springStep(A.stepY, 0, A.stepV, dt, 20, 1.0f, -2.5f, 3);
  A.stepLean = springStep(A.stepLean, 0, A.stepLeanV, dt, 20, 1.0f, -0.035f, 0.035f);
  float speed = R.walking() && !urgent
                    ? clampf((sqrtf(R.vx * R.vx + R.vy * R.vy) * R.gov - 40) / 55, 0, 1)
                    : 0;
  A.speedLines = approach(A.speedLines, speed * animGain(), dt, 0.18f);

  if (A.idle >= 0 && (U.t - A.idleT0 >= IDLE_DUR[A.idle] || !idleAllowed(R))) {
    A.idle = -1;
    A.nextIdle = U.t + (A.style == 0 ? frand(15, 28) : A.style == 2 ? frand(5, 10) : frand(8, 18));
  }
  if (A.idle < 0 && A.act == FA_NONE && idleAllowed(R) && U.t >= A.nextIdle) {
    A.idle = nextIdleAct();
    A.idleT0 = U.t;
    A.idleSide = rand() & 1 ? 1 : -1;
    A.idleCues = 0;

    U.nextBlink = U.t + IDLE_DUR[A.idle] + frand(0.5f, 1.4f);
  }
  if (A.idle >= 0 && !(A.idleCues & 1)) {
    float it = U.t - A.idleT0;
    if (it > 0.65f && (A.idle == ID_HEART || A.idle == ID_SPARKLE || A.idle == ID_CURIOUS ||
                       A.idle == ID_DOUBLETAKE)) {
      A.idleCues |= 1;
      if (A.idle == ID_HEART)
        faceBurst(PT_HEART, C_PINK, 3);
      if (A.idle == ID_SPARKLE)
        faceBurst(PT_SPARK, C_GOLD, 4);
      if (A.idle == ID_CURIOUS || A.idle == ID_DOUBLETAKE)
        spawn(PT_TEXT, A.idleSide > 0 ? 282 : 38, 72, 0, -12, 1.1f, 1, C_ACC, '?');
    }
  }
  A.wasWalking = R.walking();
  A.wasBT = R.btClient();
  A.wasOn = R.servosOn();
  A.wasSitting = R.sitting();
  A.wasPark = R.park;
  A.wasTrick = R.trick;
  A.observed = true;
}

static void idleTarget(FaceP &f) {
  if (A.idle < 0)
    return;
  float t = U.t - A.idleT0, d = IDLE_DUR[A.idle];
  float env = mn(easeIO(t / 0.30f), easeIO((d - t) / 0.45f)), side = A.idleSide;
  int eye = side > 0 ? 1 : 0;
  f.gx *= 1 - env;
  f.gy *= 1 - env;
  switch (A.idle) {
  case ID_SCAN: {
    float gaze = glide(t, 1.1f, 0.5f, -0.65f, 0.7f);
    gaze = glide(t, 2.65f, 0.5f, gaze, 0);
    f.gx = gaze * side * env;
    f.gy = -0.12f * env;
    f.e[eye].lidT += 0.16f * env;
    f.rot = -gaze * 0.07f * env;
    break;
  }
  case ID_DOUBLETAKE: {
    float gaze = glide(t, 0.6f, 0.22f, 0.55f, 0);
    gaze = glide(t, 1.05f, 0.24f, gaze, 0.8f);
    gaze = glide(t, 2.35f, 0.45f, gaze, 0);
    f.gx = gaze * side * env;
    f.rot = side * 0.09f * pulse(t, 1.0f, 2.8f);
    float pop = pulse(t, 1.05f, 2.1f);
    for (auto &e : f.e) {
      e.w += 10 * pop;
      e.h += 16 * pop;
      e.sparkle = pop * 0.4f;
    }
    A.mouthOT = 0.3f * pop;
    break;
  }
  case ID_GRIN:
    exHappy(f, 0.65f * env);
    f.e[eye].sparkle = 0.35f * env;
    f.rot = -side * 0.08f * env;
    A.smileT = 0.7f * env;
    break;
  case ID_CURIOUS:
    f.gx = side * 0.4f * env;
    f.gy = -0.3f * env;
    f.rot = side * 0.16f * env;
    f.e[eye].h += 14 * env;
    f.e[eye].w += 8 * env;
    f.e[1 - eye].lidT = 0.35f * env;
    f.e[eye].sparkle = 0.6f * env;
    break;
  case ID_DOZE: {
    float nod = pulse(t, 0.4f, 3.0f), wake = pulse(t, 3.0f, 4.4f);
    for (auto &e : f.e) {
      e.lidT = 0.88f * nod;
      e.dy += 8 * nod - 4 * wake;
      e.h += 14 * wake;
      e.w += 7 * wake;
    }
    f.gy = 0.3f * nod;
    f.rot = side * 0.10f * nod;
    A.mouthOT = 0.6f * pulse(t, 1.0f, 2.7f);
    break;
  }
  case ID_STRETCH: {
    float stretch = pulse(t, 0.3f, 3.4f);
    for (int k = 0; k < 2; k++) {
      f.e[k].dx = (k ? 1 : -1) * 5 * stretch;
      f.e[k].w += 7 * stretch;
      f.e[k].h += 12 * stretch;
      f.e[k].happy = 0.75f * stretch;
      f.e[k].dy -= 5 * stretch;
    }
    A.smileT = 0.5f * stretch;
    break;
  }
  case ID_SMUG:
    exHappy(f, 0.5f * env);
    f.e[eye].lidT = 0.25f * env;
    f.e[1 - eye].lidT = 0.1f * env;
    f.gx = side * 0.25f * env;
    f.rot = -side * 0.06f * env;
    A.smileT = 0.8f * env;
    break;
  case ID_HEART:
    if (t > 0.45f && t < d - 0.6f)
      f.shape = SH_HEART;
    for (auto &e : f.e) {
      e.col = mix(C_EYE, C_PINK, env);
      e.glow = 0.40f;
    }
    f.rot = 0.05f * sinf(t * 3.5f) * env;
    A.blushT = env * 0.7f;
    A.smileT = 0.7f * env;
    break;
  case ID_SPARKLE:
    for (auto &e : f.e) {
      e.sparkle = env;
      e.h += 5 * env;
    }
    if (t > 0.65f && t < 1.6f)
      f.shape = SH_STAR;
    f.e[eye].col = mix(C_EYE, C_GOLD, env);
    A.smileT = 0.4f * env;
    break;
  case ID_BOP:
    exHappy(f, 0.6f * env);
    f.gx = 0.35f * sinf(t * 4.5f) * env;
    f.rot = 0.06f * sinf(t * 4.5f) * env;
    for (int k = 0; k < 2; k++)
      f.e[k].dy += sinf(t * 4.5f) * 2 * env;
    break;
  case ID_PEEK: {
    float peek = pulse(t, 0.4f, 3.7f);
    f.gy = 0.6f * peek;
    f.gx = side * 0.45f * peek;
    for (auto &e : f.e) {
      e.h -= 30 * peek;
      e.dy += 14 * peek;
      e.lidB = 0.12f * peek;
    }
    f.e[eye].h += 12 * peek;
    break;
  }
  case ID_TRACK:
    f.gx = 0.75f * cosf(t * 1.65f) * env * side;
    f.gy = -0.35f * sinf(t * 1.65f) * env;
    f.rot = -f.gx * 0.06f;
    for (auto &e : f.e)
      e.sparkle = 0.4f * env;
    break;
  }
}

static void trickPolish(const Robot &R, FaceP &f) {
  float t = U.t - U.trickT0, dur = TRICK_DUR[R.trick];
  float env = mn(easeIO(t / 0.2f), easeIO((dur - t) / 0.3f));
  switch (R.trick) {
  case TR_WAVE:
    A.wink[1] = winkPulse(t, 0.65f);
    f.rot = -0.08f * sinf(t * 5) * env;
    A.smileT = 0.8f * env;
    break;
  case TR_BOW:
    A.blushT = 0.55f * env;
    A.smileT = 0.55f * env;
    break;
  case TR_TWIST:
  case TR_LOOK:
    f.rot = -f.gx * 0.07f;
    f.e[f.gx > 0 ? 1 : 0].h += 5 * fabsf(f.gx);
    break;
  case TR_DANCE: {
    float beat = sinf(t * 3.14159265f / 0.6f);
    f.rot = 0.075f * sinf(t * 3.14159265f / 1.2f);
    f.e[0].dx = -4 * beat;
    f.e[1].dx = 4 * beat;
    f.e[0].dy += 1.5f * beat;
    f.e[1].dy -= 1.5f * beat;
    A.smileT = 0.85f * env;
    break;
  }
  case TR_PUSHUP:
    for (auto &e : f.e) {
      e.w += 4 * sinf(t * 3.92699f);
      e.h -= 6 * sinf(t * 3.92699f);
    }
    break;
  case TR_STOMP:
    for (auto &e : f.e) {
      e.glow = 0.55f;
      e.w += 4 * fabsf(sinf(t * 4.49f));
    }
    break;
  case TR_SHAKE:
    f.rot = 0.055f * sinf(t * 11);
    A.mouthOT = 0.35f * env;
    break;
  case TR_NOD:
    A.smileT = 0.6f * env;
    for (auto &e : f.e)
      e.dy += f.gy * 3;
    break;
  case TR_NO:
    f.e[0].lidT += 0.1f * env;
    f.rot = -f.gx * 0.045f;
    break;
  case TR_STRETCH:
    for (int k = 0; k < 2; k++)
      f.e[k].dx = (k ? 1 : -1) * 4 * env;
    A.smileT = 0.5f * env;
    break;
  case TR_BEG: {
    float love = pulse(t, 2.4f, 4.9f);
    if (love > 0.5f)
      f.shape = SH_HEART;
    for (auto &e : f.e)
      e.col = mix(C_EYE, C_PINK, love * 0.85f);
    f.rot = sinf(t * 2.8f) * 0.065f;
    A.blushT = 0.65f * env;
    A.smileT = 0.6f * love;
    break;
  }
  case TR_CIRCLE:
    f.rot = 0.08f * sinf(t * 2.618f);
    A.mouthOT = 0.15f * env;
    break;
  case TR_HEADBANG:
    for (auto &e : f.e) {
      e.dy += 5 * sinf(t * 8.97598f);
      e.w += 5 * fabsf(sinf(t * 8.97598f));
    }
    break;
  case TR_RIPPLE:
    f.e[f.gx > 0 ? 1 : 0].sparkle = 0.85f;
    f.rot = -f.gx * 0.055f;
    break;
  case TR_POINT:
    f.e[1].h += 7 * env;
    f.e[0].lidT = 0.15f * env;
    f.rot = -0.065f * env;
    break;
  case TR_TAP:
    f.e[0].lidT += 0.12f * env;
    f.e[1].dy -= 3 * fabsf(sinf(t * 7));
    break;
  case TR_SLEEPY:
    A.mouthOT = 0.7f * pulse(t, 0.9f, 3.1f);
    if (t > 4.05f)
      A.mouthOT = 0.65f * pulse(t, 4.05f, 5.2f);
    break;
  case TR_SHIVER:
    f.rot = 0.022f * sinf(t * 25);
    A.mouthOT = 0.22f * env;
    break;
  case TR_POUNCE:
    if (t < 2.6f) {
      f.e[0].w -= 4 * env;
      f.e[1].w += 3 * env;
    } else {
      f.rot = 0.07f * sinf((t - 2.6f) * 12);
      A.mouthOT = 0.5f * pulse(t, 2.6f, 3.5f);
    }
    break;
  case TR_SWAY:
    f.rot = -0.10f * sinf(t * 3.92699f);
    A.smileT = 0.8f * env;
    break;
  case TR_FLEX:
    f.rot = 0.045f * sinf(t * 4);
    A.smileT = 0.8f * env;
    break;
  case TR_CURIOUS:
    f.gy = -0.12f * env;
    A.mouthOT = 0.2f * env;
    break;
  case TR_SCARED:
    A.mouthOT = 0.65f * env;
    f.rot = 0.025f * sinf(t * 32) * env;
    break;
  default:
    break;
  }
}

static void actionTarget(FaceP &f) {
  if (A.act == FA_NONE)
    return;
  float t = U.t - A.actT0, env = mn(easeIO(t / 0.12f), easeIO((A.actDur - t) / 0.3f));
  switch (A.act) {
  case FA_START:
    for (auto &e : f.e) {
      e.w += 4 * env;
      e.h += 5 * env;
      e.sparkle = 0.3f * env;
    }
    break;
  case FA_STOP:
    for (auto &e : f.e) {
      e.happy = 0.4f * env;
      e.lidT = 0.13f * env;
    }
    f.rot += 0.04f * sinf(t * 10) * env;
    A.smileT = 0.5f * env;
    break;
  case FA_SIT:
    exHappy(f, 0.5f * env);
    for (auto &e : f.e)
      e.dy += 3 * env;
    A.smileT = 0.6f * env;
    break;
  case FA_WAKE:
    for (auto &e : f.e) {
      e.w += 8 * env;
      e.h += 13 * env;
      e.sparkle = 0.8f * env;
      e.lidT *= 1 - env;
    }
    A.mouthOT = 0.35f * pulse(t, 0.1f, 0.8f);
    A.smileT = pulse(t, 0.7f, 1.6f);
    break;
  case FA_GREET:
    exHappy(f, 0.8f * env);
    f.gx = 0;
    f.rot = 0.06f * sinf(t * 6) * env;
    A.wink[1] = winkPulse(t, 0.65f);
    A.smileT = 0.75f * env;
    break;
  case FA_AFFECTION:
    if (t > 0.25f && t < 1.25f)
      f.shape = SH_HEART;
    for (auto &e : f.e) {
      e.col = mix(e.col, C_PINK, env * 0.85f);
      e.happy = 0.75f * env;
      e.sparkle = 0.6f * env;
    }
    f.gx = 0;
    f.rot = 0.06f * sinf(t * 5) * env;
    A.blushT = 0.8f * env;
    A.smileT = 0.8f * env;
    break;
  case FA_DOUBLE:
    for (auto &e : f.e) {
      e.h += 8 * env;
      e.sparkle = 0.5f * env;
    }
    A.wink[0] = A.wink[1] = winkPulse(t, 0.45f);
    A.smileT = 0.4f * env;
    break;
  case FA_TRIPLE:
    f.shape = t > 0.2f && t < 0.95f ? SH_STAR : SH_NORMAL;
    for (auto &e : f.e)
      e.col = mix(e.col, C_GOLD, env);
    A.smileT = 0.8f * env;
    break;
  case FA_HOLD:
    for (auto &e : f.e) {
      e.lidT = 0.3f * env;
      e.slant = -6 * env;
    }
    f.gy = 0.25f * env;
    A.smileT = 0.3f * env;
    break;
  case FA_BYE:
    f.gx = -0.6f * env;
    for (auto &e : f.e) {
      e.slant = -12 * env;
      e.lidT = 0.1f * env;
    }
    break;
  case FA_NOTICE:
    for (auto &e : f.e) {
      e.w += 2 * env;
      e.h += 2 * env;
    }
    break;
  case FA_FINISH: {
    float q = clampf(t / A.actDur, 0, 1);
    float settle = sinf(q * 3.14159265f) * expf(-3 * q);
    switch (A.finishedTrick) {
    case TR_WAVE:
      exHappy(f, 0.55f * env);
      f.gx = 0;
      A.smileT = 0.7f * env;
      break;
    case TR_POUNCE:
      exHappy(f, 0.7f * env);
      A.wink[1] = winkPulse(t, 0.42f);
      f.rot = -0.06f * env;
      A.smileT = 0.8f * env;
      break;
    case TR_DANCE:
      exHappy(f, 0.65f * env);
      f.rot = 0.07f * sinf(t * 7) * settle;
      for (auto &e : f.e)
        e.sparkle = 0.65f * env;
      A.smileT = 0.8f * env;
      break;
    case TR_BOW:
    case TR_BEG:
      exHappy(f, 0.4f * env);
      f.gy = 0.15f * env;
      A.blushT = (A.finishedTrick == TR_BEG ? 0.6f : 0.35f) * env;
      A.smileT = 0.55f * env;
      break;
    case TR_PUSHUP:
    case TR_HEADBANG:
      for (auto &e : f.e) {
        e.lidT = 0.32f * env;
        e.happy = 0.25f * env;
      }
      A.mouthOT = 0.3f * pulse(t, 0.15f, 1.3f);
      A.smileT = 0.25f * env;
      break;
    case TR_SLEEPY:
    case TR_STRETCH:
      for (auto &e : f.e)
        e.lidT = 0.3f * env;
      f.gy = 0.2f * env;
      A.mouthOT = 0.3f * pulse(t, 0.2f, 1.35f);
      A.smileT = 0.25f * env;
      break;
    case TR_SCARED:
    case TR_SHIVER:
      exHappy(f, 0.3f * env);
      f.gx = 0;
      f.rot = 0;
      A.mouthOT = 0.25f * pulse(t, 0.1f, 0.85f);
      A.smileT = 0.45f * env;
      break;
    case TR_LOOK:
    case TR_CURIOUS:
      f.gx = 0.3f * sinf(q * 3.14159265f) * env;
      f.rot = 0.08f * env;
      f.e[1].h += 7 * env;
      A.mouthOT = 0.15f * env;
      break;
    case TR_POINT:
      f.gx = 0.45f * env;
      f.rot = -0.05f * env;
      f.e[1].sparkle = 0.4f * env;
      A.smileT = 0.35f * env;
      break;
    case TR_NOD:
      exHappy(f, 0.4f * env);
      f.gy = 0.2f * settle;
      A.smileT = 0.5f * env;
      break;
    case TR_NO:
      f.gx = -0.25f * env;
      f.rot = -0.035f * env;
      for (auto &e : f.e)
        e.lidT = 0.12f * env;
      A.smileT = 0.15f * env;
      break;
    case TR_FLEX:
      exHappy(f, 0.45f * env);
      f.rot = 0.045f * env;
      for (auto &e : f.e) {
        e.lidT = 0.18f * env;
        e.sparkle = 0.4f * env;
      }
      A.smileT = 0.7f * env;
      break;
    case TR_TWIST:
    case TR_CIRCLE:
    case TR_SWAY:
    case TR_RIPPLE:
      exHappy(f, 0.3f * env);
      f.rot = 0.05f * sinf(t * 6) * settle;
      A.smileT = 0.4f * env;
      break;
    case TR_STOMP:
    case TR_TAP:
      for (auto &e : f.e)
        e.dy += 3 * settle;
      A.smileT = 0.35f * env;
      break;
    case TR_SHAKE:
      for (auto &e : f.e)
        e.h += 5 * env;
      f.rot = 0.025f * sinf(t * 10) * settle;
      A.mouthOT = 0.2f * env;
      break;
    default:
      A.smileT = 0.2f * env;
      break;
    }
    break;
  }
  default:
    break;
  }
}

static void personalityTarget(const Robot &R, FaceP &f) {
  float tip = U.t - U.tipT0, stumble = U.t - U.stumT0;
  bool warning = faceUrgent(R);
  if (warning || tip < 4.2f || stumble < 2.1f) {

    exNeutral(f);
    f.gx = clampf(R.roll / 30, -0.8f, 0.8f);
    if (R.boardDown()) {
      for (auto &e : f.e) {
        e.col = mix(C_EYE, C_BAD, 0.5f);
        e.slant = -16;
        e.w = 64;
        e.lidT = 0.12f;
      }
      f.gx = 0.45f * sinf(U.t * 2.2f);
    } else if (tip < 0.4f || (stumble < 0.35f && tip >= 4.2f)) {
      for (auto &e : f.e) {
        e.w = 82;
        e.h = 110;
        e.sparkle = 0.3f;
      }
      A.mouthOT = 0.7f;
    } else if (tip < 2.1f) {
      f.shape = SH_SPIRAL;
      for (auto &e : f.e)
        e.col = mix(C_EYE, C_WARN, 0.45f);
      A.mouthOT = 0.4f;
    } else if (warning) {
      for (auto &e : f.e) {
        e.slant = -17;
        e.w = 64;
        e.h = 86;
        e.lidT = 0.08f;
        e.col = C_WARN;
      }
      A.mouthOT = 0.3f;
    } else {
      float t = tip < 4.2f ? tip - 2.1f : stumble - 0.35f;
      float d = tip < 4.2f ? 2.1f : 1.75f;
      float relief = pulse(t, 0, d);
      exHappy(f, 0.55f * relief);
      f.gx = 0;
      A.smileT = relief * 0.65f;
      A.wink[0] = A.wink[1] = winkPulse(t, 0.3f);
    }
    return;
  }
  if (!U.linkOK || R.sleeping() || R.batLocked() || R.park != P_NONE)
    return;
  if (R.trick != TR_NONE) {
    trickPolish(R, f);
    return;
  }
  if (R.imuOK() && !R.imuLost()) {
    float sw = R.swapped() ? -1 : 1;
    f.rot += clampf(-R.roll * sw * 0.0025f, -0.04f, 0.04f);
    f.gy += clampf(-R.pitch * sw * 0.008f, -0.12f, 0.12f);
  }
  if (R.walking()) {
    float sw = R.swapped() ? -1 : 1;
    float turn = clampf(R.wz * 0.13f * sw, -0.1f, 0.1f);
    f.rot += turn + A.stepLean;
    for (int k = 0; k < 2; k++) {
      f.e[k].dy += A.stepY;
      f.e[k].w += mx(A.stepY, 0) * 0.9f;
      f.e[k].h -= mx(A.stepY, 0) * 1.4f;
      if (R.gait != 0)
        f.e[k].dy += sinf(U.t * 3 + k * 1.8f) * 0.7f * animGain();
    }
    if (R.vx * sw < -1) {
      f.e[0].lidT += 0.07f;
      A.mouthOT = 0.12f;
    }
    if (A.act == FA_START || A.act == FA_STOP || A.act == FA_NOTICE)
      actionTarget(f);
    return;
  }
  if (A.act != FA_NONE && R.servosOn())
    actionTarget(f);
  else if (A.idle >= 0)
    idleTarget(f);
  else if (R.servosOn() && U.t >= A.linkT0 && U.t - A.linkT0 < 1.6f) {
    float e = pulse(U.t - A.linkT0, 0, 1.6f);
    exHappy(f, 0.6f * e);
    A.smileT = 0.6f * e;
  }

  if (A.idle < 0 && A.act == FA_NONE && R.servosOn() && !R.party) {
    f.e[0].dy += 0.35f * sinf(U.t * 1.7f + 0.4f);
    f.e[1].dy += 0.35f * sinf(U.t * 1.7f - 0.4f);
  }
  float yt = U.t - U.yawnT0;
  if (R.idleLeft >= 0 && R.idleLeft <= 60 && yt < 1.8f)
    A.mouthOT = pulse(yt, 0, 1.8f) * 0.6f;
}

static void personalityFinish(float dt) {
  A.smile = approach(A.smile, A.smileT, dt, 0.10f);
  A.mouthO = approach(A.mouthO, A.mouthOT, dt, 0.08f);
  A.blush = approach(A.blush, A.blushT, dt, 0.15f);
}

static void faceDetailsDraw() {
  float gain = animGain();
  if (A.speedLines > 0.03f) {
    for (int i = 0; i < 3; i++) {
      float phase = fmodf(U.t * (1.9f + 0.25f * i) + i * 0.31f, 1.0f);
      float y = 82 + i * 37 + phase * 18;
      float a = A.speedLines * 0.25f * sinf(phase * 3.14159265f);
      line(8, y, 26 + phase * 9, y, 1.6f, C_EYE, a);
      line(312, y, 294 - phase * 9, y, 1.6f, C_EYE, a);
    }
  }
  if (A.blush > 0.03f) {
    for (int side = 0; side < 2; side++) {
      float x = side ? 268 : 52;
      for (int i = 0; i < 3; i++)
        line(x - 5 + i * 5, 162, x - 8 + i * 5, 168, 2, C_PINK, A.blush * 0.6f);
    }
  }

  float opening = easeIO(A.mouthO / 0.22f);
  if (A.smile > 0.01f && opening < 1)
    arc(160, 179, 14 + A.smile * 3, 2, 135, 225, C_EYE, A.smile * 0.65f * (1 - opening));
  if (A.mouthO > 0.01f) {
    float s = A.mouthO;
    rrectLine(160 - 3 - s * 4, 185, 6 + s * 8, 5 + s * 12, 3 + s * 3, 1.8f, C_EYE,
              mn(0.7f, s * gain) * opening);
  }
}

static void demoRobot(Robot &r) {
  float elapsed = U.t - A.demoT0;
  float total = 24;
  for (int i = 1; i < TR_COUNT; i++)
    total += TRICK_DUR[i] + 2.1f;
  float t = fmodf(mx(elapsed, 0), total);
  r = Robot();
  r.ver = 2;
  r.flags = 1 | 2 | 32 | 64 | 512;
  r.batV = 7.8f;
  r.batPct = 70;
  r.page = PG_FACE;
  r.speed = 55;
  r.period = 1.2f;
  r.upS = (uint32_t)elapsed;
  if (A.demoKind == 1) {
    float span = 0;
    for (int i = 0; i < ID_COUNT; i++)
      span += IDLE_DUR[i] + 0.7f;
    float it = fmodf(mx(elapsed, 0), span);
    for (int i = 0; i < ID_COUNT; i++) {
      if (it < IDLE_DUR[i] + 0.7f) {
        int idle = it < IDLE_DUR[i] ? i : -1;
        if (A.idle != idle) {
          A.idle = idle;
          A.idleT0 = U.t - it;
          A.idleSide = i & 1 ? -1 : 1;
          A.idleCues = 0;
          U.nextBlink = U.t + IDLE_DUR[i] + 1;
        }
        A.nextIdle = U.t + 10;
        break;
      }
      it -= IDLE_DUR[i] + 0.7f;
    }
  } else if (t >= 8 && t < 24) {
    r.move = 'f';
    r.flags |= 8;
    r.vx = t < 16 ? 40 : 80;
    r.gait = t < 16 ? 0 : 1;
    r.vy = 12 * sinf(t);
    r.wz = 0.2f * sinf(t * 0.6f);
    r.swing = (int)(t * 3) & 1 ? 0x15 : 0x2a;
  } else if (t >= 24) {
    t -= 24;
    for (int i = 1; i < TR_COUNT; i++) {
      float span = TRICK_DUR[i] + 2.1f;
      if (t < span) {
        if (t < TRICK_DUR[i]) {
          r.trick = i;
          r.trickMs = (uint32_t)(t * 1000);
        }
        r.yaw = 12 * sinf(t * 2.5f);
        r.pPitch = 7 * sinf(t * 4);
        break;
      }
      t -= span;
    }
  }
  for (int i = 0; i < 6; i++) {
    neutralFoot(i, 0, r.feet[i][0], r.feet[i][1]);
    r.feet[i][2] = -r.bodyH + ((r.swing >> i) & 1) * 20;
  }
}
