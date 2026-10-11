// Walking assists driven by fresh IMU samples. Heading is relative gyro heading, not a compass.

#pragma once
#include <math.h>
#include <stdint.h>

namespace wc {
inline float clamp(float x, float a, float b) {
  return x < a ? a : x > b ? b : x;
}

inline float approach(float x, float y, float rate, float dt) {
  return x + clamp(y - x, -rate * dt, rate * dt);
}

inline float wrap(float a) {
  return a - 360.0f * floorf((a + 180.0f) / 360.0f);
}

struct Config {
  bool heading = true, terrain = true, landing = true, balance = true;
  float headingK = 1.8f, headingD = .35f, yawMax = 20, roughGain = 1, guardDeg = 27;
};

enum Turn { IDLE, RUNNING, SETTLING, DONE, ABORTED };

enum Reason { NO_REASON, USER_STOP, SENSOR_LOST, FAULT, BALANCE, TIMEOUT, OVERRIDE };

struct Input {
  bool fresh = false, eligible = false, walking = false, translating = false, manualYaw = false;
  bool airborne = false, feetDown = true;
  float vx = 0, vy = 0, yawCmd = 0, height = 63.7f;
};

struct Controller {
  Config cfg;
  float heading = 0, yawRate = 0, pitch = 0, roll = 0, pitchRate = 0, rollRate = 0;
  float linearG = 0, impactG = 0, rough = 0, accelMS = 0, tiltMS = 0;
  float headingTarget = 0, headingError = 0, yawOut = 0;
  float speedScale = 1, strokeScale = 1, periodScale = 1, heightDrop = 0;
  float landingK = .30f, lastImpact = 0, landingPeak = 0, landingWindow = 0;
  float walkAge = 0, roughUp = 0, roughQuiet = 0, guardT = 0, stableT = 0;
  float turnStart = 0, turnTarget = 0, turnDegrees = 0, turnMeasured = 0, turnAge = 0,
        turnQuiet = 0, turnLimit = 20;
  uint32_t landings = 0;
  int roughLevel = 0;
  Turn turn = IDLE;
  Reason reason = NO_REASON;
  bool headingActive = false, recovery = false, sensorReady = false;

  bool turnActive() const {
    return turn == RUNNING || turn == SETTLING;
  }

  float progress() const {
    return turnDegrees ? turnMeasured / turnDegrees : 0;
  }

  void cancel(Reason r) {
    if (turnActive()) {
      turnMeasured = heading - turnStart;
      turn = ABORTED;
      reason = r;
    }
    headingActive = false;
    yawOut = 0;
  }

  void resetSensor() {
    cancel(SENSOR_LOST);
    sensorReady = false;
    yawRate = 0;
  }

  void sample(float dt, float yaw, float p, float r, float pr, float rr, float lin, float impact) {
    if (!isfinite(dt) || dt <= 0 || dt > .1f || !isfinite(yaw) || !isfinite(p) || !isfinite(r) ||
        !isfinite(pr) || !isfinite(rr) || !isfinite(lin) || !isfinite(impact)) {
      resetSensor();
      return;
    }

    if (sensorReady)
      heading += yaw * dt;
    if (fabsf(heading) > 100000) {
      cancel(SENSOR_LOST);
      heading = 0;
    }
    sensorReady = true;
    yawRate = yaw;
    pitch = p;
    roll = r;
    pitchRate = pr;
    rollRate = rr;
    linearG = clamp(lin, 0, 3);
    impactG = clamp(impact, 0, 3);
    float a = 1 - expf(-dt / .55f);
    accelMS += (linearG * linearG - accelMS) * a;
    tiltMS += ((pr * pr + rr * rr) - tiltMS) * a;
    rough = clamp(cfg.roughGain * (.7f * sqrtf(accelMS) / .22f + .3f * sqrtf(tiltMS) / 35), 0, 9);
  }

  bool beginTurn(float degrees, float limit, bool allowed) {
    if (!allowed || recovery || !sensorReady || !isfinite(degrees) || fabsf(degrees) < 5 ||
        fabsf(degrees) > 180)
      return false;
    headingActive = false;
    turn = RUNNING;
    reason = NO_REASON;
    turnStart = heading;
    turnDegrees = degrees;
    turnTarget = heading + degrees;
    turnMeasured = 0;
    turnAge = turnQuiet = 0;
    turnLimit = clamp(limit, 5, cfg.yawMax);
    return true;
  }

  bool resume(bool fresh) {
    if (!recovery)
      return true;
    if (!fresh || stableT < 1.0f)
      return false;
    recovery = false;
    guardT = stableT = 0;
    headingActive = false;
    return true;
  }

  void touchdown() {

    if (cfg.landing && !recovery && walkAge > .8f && landingWindow <= 0) {
      landingWindow = .14f;
      landingPeak = impactG;
    }
  }

  void update(float dt, const Input &in) {
    if (!isfinite(dt) || dt <= 0 || dt > .1f) {
      cancel(FAULT);
      return;
    }
    bool usable = in.fresh && sensorReady;
    bool enabled = usable && in.eligible;
    if (!enabled)
      cancel(usable ? FAULT : SENSOR_LOST);
    walkAge = enabled && in.walking ? walkAge + dt : 0;

    float tilt = fmaxf(fabsf(pitch), fabsf(roll));
    float predicted = fmaxf(fabsf(pitch + .20f * pitchRate), fabsf(roll + .20f * rollRate));
    bool danger = enabled && cfg.balance && in.walking && tilt > 12 && predicted > cfg.guardDeg;
    guardT = danger ? guardT + dt : 0;
    if (!recovery && guardT >= .08f) {
      recovery = true;
      stableT = 0;
      cancel(BALANCE);
    }
    if (recovery) {
      stableT = usable && in.feetDown && tilt < 12 && fabsf(pitchRate) < 8 && fabsf(rollRate) < 8
                    ? stableT + dt
                    : 0;

      heightDrop = 0;
    } else
      heightDrop = approach(heightDrop, 0, 3, dt);
    if (!cfg.balance) {
      recovery = false;
      stableT = guardT = 0;
    }

    if (enabled && cfg.terrain && in.walking && !recovery && walkAge > .8f) {
      float threshold = roughLevel == 0 ? .95f : 1.8f;
      roughUp = rough > threshold ? roughUp + dt : 0;
      roughQuiet = rough < .60f ? roughQuiet + dt : 0;
      if (roughLevel < 2 && roughUp > .65f) {
        roughLevel++;
        roughUp = roughQuiet = 0;
      }
      if (roughLevel && roughQuiet > 5) {
        roughLevel--;
        roughQuiet = roughUp = 0;
      }
    } else {
      roughUp = 0;
      roughQuiet += dt;
      if (roughQuiet > 5) {
        roughLevel = 0;
        roughQuiet = 0;
      }
    }
    if (!cfg.terrain || !enabled)
      roughLevel = 0;
    float ss = roughLevel == 2 ? .48f : roughLevel == 1 ? .72f : 1;
    speedScale = approach(speedScale, ss, .4f, dt);
    strokeScale = approach(strokeScale,
                           roughLevel == 2   ? .65f
                           : roughLevel == 1 ? .82f
                                             : 1,
                           .2f, dt);
    periodScale = approach(periodScale,
                           roughLevel == 2   ? 1.25f
                           : roughLevel == 1 ? 1.12f
                                             : 1,
                           .2f, dt);
    yawOut = 0;
    if (turnActive() && !recovery) {
      turnAge += dt;
      headingError = turnTarget - heading;
      turnMeasured = heading - turnStart;
      if (turnAge > 8 + fabsf(turnDegrees) / 3)
        cancel(TIMEOUT);
      else {

        yawOut = clamp(1.3f * headingError - .45f * yawRate, -turnLimit, turnLimit);
        if (fabsf(headingError) < 2.5f && fabsf(yawRate) < 3) {
          turn = SETTLING;
          yawOut = 0;
        } else
          turn = RUNNING;
        turnQuiet =
            fabsf(headingError) < 2.5f && fabsf(yawRate) < 3 && in.feetDown ? turnQuiet + dt : 0;
        if (turnQuiet > .35f) {
          turn = DONE;
          yawOut = 0;
        }
      }
    } else if (enabled && cfg.heading && in.translating && !in.manualYaw && !recovery) {
      if (!headingActive) {
        headingTarget = heading;
        headingActive = true;
      }
      headingError = clamp(headingTarget - heading, -25, 25);
      yawOut = clamp(cfg.headingK * headingError - cfg.headingD * yawRate, -cfg.yawMax, cfg.yawMax);
    } else {
      headingActive = false;
      if (!turnActive())
        headingError = 0;
    }
    if (recovery || !enabled)
      yawOut = 0;
    if (landingWindow > 0) {
      if (!enabled || !cfg.landing || recovery || !in.walking)
        landingWindow = 0;
      else {
        landingPeak = fmaxf(landingPeak, impactG);
        landingWindow -= dt;
        if (landingWindow <= 0) {
          lastImpact = landingPeak;
          landings++;
          landingK = clamp(landingK + (landingPeak > .18f   ? .015f
                                       : landingPeak < .07f ? -.004f
                                                            : 0),
                           .20f, .80f);
        }
      }
    }
  }
};

inline float softLift(float s, float k) {
  s = clamp(s, 0, 1);
  k = clamp(k, 0, .8f);
  if (s > .5f) {
    float a = s - .5f, b = 1 - s;
    s += k * 16 * a * a * b * b;
  }
  float a = s * (1 - s);
  return 64 * a * a * a;
}
}
