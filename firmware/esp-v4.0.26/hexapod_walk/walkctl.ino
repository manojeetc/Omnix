// Connect the walking-assist state machine to the firmware's sensors, settings and gait clock.

void walkCancel() {
  walkCtl.cancel(wc::USER_STOP);
}

// Assist decisions need calibrated, recent IMU data; a readable sensor alone is not enough.
bool walkFresh() {
  return imuOK && hasLevel && hasFront && walkCtl.sensorReady && ms64() - walkSampleAt <= 120;
}

bool walkEligible() {
  return !servoHold && !servoSettling && calibrated && mode == GAIT && servosOn &&
         strideMax >= 10 && park == P_NONE && !sitting && !batLocked && !boardDown[0] &&
         !boardDown[1] && supportOK && trick == TR_NONE && zcLeg < 0 && zsLeg < 0 &&
         ms64() >= sagTestUntil;
}

float walkHeight() {

  return park == P_NONE && !sitting && mode == GAIT ? fmaxf(40, targetH - walkCtl.heightDrop)
                                                    : targetH;
}

// Recovery clears only after the robot is upright, stable and ready for a new command.
bool walkResume() {
  if (walkCtl.resume(walkFresh()))
    return true;
  say("Balance hold: set the robot upright; wait for all feet down and 1 s of stable MPU data, then command again.");
  return false;
}

void walkSample(float dt, float ax, float ay, float az, float gx, float gy, float gz) {
  float ux = upF.u[0], uy = upF.u[1], uz = upF.u[2];
  float gyro[3] = {gx, gy, gz};
  imath::Rates rate;
  if (!upF.init || !imath::rates(upF.u, gyro, rate)) {
    turnSensorFault("orientation/rates invalid", dt * 1000);
    walkCtl.resetSensor();
    walkSampleAt = 0;
    return;
  }
  float ex = ax - ux - accBX / 9810.0f, ey = ay - uy - accBY / 9810.0f, ez = az - uz;
  float lin = sqrtf(ex * ex + ey * ey + ez * ez);
  float impact = fmaxf(0, ax * ux + ay * uy + az * uz - 1);
  if (!isfinite(dt) || dt <= 0 || dt > .1f || !isfinite(pitch) || !isfinite(roll) ||
      !isfinite(lin) || !isfinite(impact))
    turnSensorFault("body sample invalid", dt * 1000);
  walkCtl.sample(dt, rate.heading, pitch, roll, rate.pitch, rate.roll, lin, impact);
  if (walkCtl.sensorReady)
    walkSampleAt = ms64();
}

void walkUpdate(float dt) {
  if (servoHold || servoSettling)
    return;
  bool fresh = walkFresh();
  bool air = false;
  for (int i = 0; i < 6; i++)
    air |= swinging[i];
  wc::Input in;
  in.fresh = fresh;
  in.eligible = walkEligible();
  in.walking = lastMove || stepping;
  in.translating = sqrtf(cmdVx * cmdVx + cmdVy * cmdVy) > .5f;
  in.manualYaw = fabsf(cmdWz) > .001f;
  in.airborne = air;
  in.feetDown = !air;
  in.height = targetH;
  if (walkCtl.turnActive() && lastMove != 't')
    walkCtl.cancel(wc::OVERRIDE);
  if (walkCtl.turnActive() && !fresh)
    turnSensorFault("body MPU sample unavailable/stale at control update",
                    (float)(ms64() - walkSampleAt));

  if (lastMove == 't' && walkCtl.turn == wc::ABORTED) {
    stopMoving();
    say("Turn stopped: %s; measured %+.1f deg.", walkReason(), walkCtl.heading - walkCtl.turnStart);
    turnSensorReport();
  }
  bool wasRecovery = walkCtl.recovery;
  wc::Turn before = walkCtl.turn;
  walkCtl.update(dt, in);
  if (!wasRecovery && walkCtl.recovery) {
    stopMoving();
    say("Balance recovery: finishing raised feet, holding current height. New command required after upright/stable.");
  }
  if ((before == wc::RUNNING || before == wc::SETTLING) && !walkCtl.turnActive()) {
    stopMoving();
    if (walkCtl.turn == wc::DONE)
      say("Turn complete: measured %+.1f deg (relative gyro).",
          walkCtl.heading - walkCtl.turnStart);
    else
      say("Turn stopped: %s; measured %+.1f deg.", walkReason(),
          walkCtl.heading - walkCtl.turnStart);
    turnSensorReport();
  }
  if (walkCtl.turnActive())
    lastActivity = millis();
}

void walkVelocity(float &x, float &y, float &w) {
  x *= walkCtl.speedScale;
  y *= walkCtl.speedScale;
  w *= walkCtl.speedScale;
  if (walkCtl.turnActive()) {
    x = y = 0;
    w = constrain(walkCtl.yawOut * DEG_TO_RAD, -speed / 130, speed / 130) * walkCtl.speedScale;
  } else if (walkCtl.headingActive) {
    float cap = fminf(walkCtl.cfg.yawMax * DEG_TO_RAD, .35f * speed / 130);
    w += constrain(walkCtl.yawOut * DEG_TO_RAD, -cap, cap);
  }
  if (walkCtl.recovery)
    x = y = w = 0;
}

void walkGait(float dt) {
  if (servoHold || servoSettling)
    return;
  if (walkCtl.recovery)
    return;
  int want = preferredGait;
  if (autoGait) {
    autoGaitHold -= dt;
    float vT = fmaxf(strideMax / (GAIT_DUTY[0] * PERIOD), 1);
    float cd = fmaxf(sqrtf(cmdVx * cmdVx + cmdVy * cmdVy) / vT, fabsf(cmdWz) * 130 / vT);
    if (walkCtl.turnActive())
      cd = fabsf(walkCtl.yawOut) * DEG_TO_RAD * 130 / vT;
    if (cd > .01f) {
      if (preferredGait == 2) {
        if (cd > .62f)
          want = 0;
        else if (cd > .42f)
          want = 1;
      } else if (preferredGait == 1) {
        if (cd > .62f)
          want = 0;
        else if (cd < .30f)
          want = 2;
      } else {
        if (cd < .30f)
          want = 2;
        else if (cd < .50f)
          want = 1;
      }
    }
    if (autoGaitHold <= 0 && want != preferredGait) {
      preferredGait = want;
      autoGaitHold = 2;
    }
  }
  want = max(preferredGait, walkCtl.roughLevel);

  if (want != gait) {
    if (stepping)
      pendingGait = want;
    else
      applyGait(want);
  } else if (pendingGait >= 0 && pendingGait != want)
    pendingGait = -1;
}

void walkLoad() {
  preferredGait = gait;
  unsigned f = prefs.getUInt("assist", 15);
  walkCtl.cfg.heading = f & 1;
  walkCtl.cfg.terrain = f & 2;
  walkCtl.cfg.landing = f & 4;
  walkCtl.cfg.balance = f & 8;
  walkCtl.cfg.headingK = finiteSetting(prefs.getFloat("headkp", 1.8f), 1.8f, .3f, 4.0f);
  walkCtl.cfg.yawMax = finiteSetting(prefs.getFloat("turnrate", 20), 20, 5.0f, 30.0f);
  walkCtl.cfg.roughGain = finiteSetting(prefs.getFloat("roughgain", 1), 1, .5f, 2.0f);
  walkCtl.cfg.guardDeg = finiteSetting(prefs.getFloat("guarddeg", 27), 27, 20.0f, 30.0f);
  walkCtl.landingK = finiteSetting(prefs.getFloat("landsoft", .30f), .30f, .2f, .8f);
}

unsigned walkFlags() {
  return walkCtl.cfg.heading | walkCtl.cfg.terrain << 1 | walkCtl.cfg.landing << 2 |
         walkCtl.cfg.balance << 3;
}

void walkSave() {
  prefs.putUInt("assist", walkFlags());
  prefs.putFloat("headkp", walkCtl.cfg.headingK);
  prefs.putFloat("turnrate", walkCtl.cfg.yawMax);
  prefs.putFloat("roughgain", walkCtl.cfg.roughGain);
  prefs.putFloat("guarddeg", walkCtl.cfg.guardDeg);
  prefs.putFloat("landsoft", walkCtl.landingK);
}

const char *walkReason() {
  static const char *const names[] = {"none",
                                      "user stop",
                                      "MPU stale/lost",
                                      "motion/safety unavailable",
                                      "balance recovery",
                                      "turn timeout",
                                      "manual override"};
  return names[(int)walkCtl.reason];
}

void walkStatus() {
  turnSensorReport();
  say("Assists: heading %s, terrain %s, landing %s, balance %s | MPU %s (needs levelcal + mpufront)",
      walkCtl.cfg.heading ? "on" : "off", walkCtl.cfg.terrain ? "on" : "off",
      walkCtl.cfg.landing ? "on" : "off", walkCtl.cfg.balance ? "on" : "off",
      walkFresh() ? "ready" : "unavailable");
  say("Heading %+.1f deg, error %+.1f, steering %+.1f deg/s | turn state %d: %s",
      wc::wrap(walkCtl.heading), walkCtl.headingError, walkCtl.yawOut, (int)walkCtl.turn,
      walkReason());
  say("Ground %d, rough %.2f, speed %.0f%%, stroke %.0f%% | landing %.2f, last body jolt %.2f g, samples %lu",
      walkCtl.roughLevel, walkCtl.rough, walkCtl.speedScale * 100, walkCtl.strokeScale * 100,
      walkCtl.landingK, walkCtl.lastImpact, (unsigned long)walkCtl.landings);
  say("Balance %s, height offset %.1f mm | headkp %.2f, turnrate %.0f, roughgain %.2f, guarddeg %.0f",
      walkCtl.recovery ? (walkCtl.stableT >= 1 ? "ready for new command" : "holding") : "ready",
      walkCtl.heightDrop, walkCtl.cfg.headingK, walkCtl.cfg.yawMax, walkCtl.cfg.roughGain,
      walkCtl.cfg.guardDeg);
}

bool walkNumber(const String &s, float &v) {
  char *end;
  v = strtof(s.c_str(), &end);
  return end != s.c_str() && !*end && isfinite(v);
}

bool walkCommand(const String &c, const String &a0, const String &b0) {
  String a = a0, b = b0;
  a.toLowerCase();
  b.toLowerCase();
  if (c == "assist") {
    if (a == "" || a == "status") {
      walkStatus();
      return true;
    }
    if (a == "on" || a == "off") {
      bool on = a == "on";
      walkCtl.cfg.heading = walkCtl.cfg.terrain = walkCtl.cfg.landing = walkCtl.cfg.balance = on;
      if (!on) {
        if (walkCtl.turnActive())
          stopMoving();
        walkCtl.recovery = false;
      }
    } else if ((a == "heading" || a == "terrain" || a == "landing" || a == "balance") &&
               (b == "on" || b == "off")) {
      bool on = b == "on";
      if (a == "heading")
        walkCtl.cfg.heading = on;
      if (a == "terrain")
        walkCtl.cfg.terrain = on;
      if (a == "landing")
        walkCtl.cfg.landing = on;
      if (a == "balance") {
        walkCtl.cfg.balance = on;
        if (!on)
          walkCtl.recovery = false;
      }
    } else {
      say("assist [status/on/off] | assist heading/terrain/landing/balance on/off | save to keep");
      return true;
    }
    walkCtl.headingActive = false;
    walkStatus();
    return true;
  }
  if (c == "headkp" || c == "turnrate" || c == "roughgain" || c == "guarddeg" || c == "landsoft") {
    float v;
    if (!walkNumber(a, v)) {
      say("Use a finite number: headkp .3..4, turnrate 5..30 deg/s, roughgain .5..2, guarddeg 20..30, landsoft .2...8");
      return true;
    }
    if (c == "headkp")
      walkCtl.cfg.headingK = constrain(v, .3f, 4.0f);
    if (c == "turnrate")
      walkCtl.cfg.yawMax = constrain(v, 5.0f, 30.0f);
    if (c == "roughgain")
      walkCtl.cfg.roughGain = constrain(v, .5f, 2.0f);
    if (c == "guarddeg")
      walkCtl.cfg.guardDeg = constrain(v, 20.0f, 30.0f);
    if (c == "landsoft")
      walkCtl.landingK = constrain(v, .2f, .8f);
    walkStatus();
    return true;
  }
  if (c != "turn")
    return false;
  if (a == "stop") {
    stopMoving();
    say("Turn stopped.");
    return true;
  }
  float deg;
  if (!walkNumber(a, deg) || fabsf(deg) < 5 || fabsf(deg) > 180) {
    say("turn <degrees>: +left / -right, 5..180 deg; e.g. turn 90 | turn -45 | turn stop");
    return true;
  }
  if (!walkFresh() || !walkEligible() || !supportOK || stepping || lastMove ||
      fabsf(bodyH - targetH) > .5f || !walkResume()) {
    say("Turn needs a calibrated MPU and robot, standing still with all feet down, battery/boards healthy; use stand first.");
    return true;
  }
  float cap = fminf(walkCtl.cfg.yawMax, speed / 130 * RAD_TO_DEG);
  if (strideMax < 10 || cap < 5) {
    say("Turn unavailable: increase safe walking speed above 12 mm/s or fix the stride budget.");
    return true;
  }
  if (walkCtl.beginTurn(deg, cap, true)) {
    turnSensorReset();
    lastMove = 't';
    applyMove();
    say("Turning %+.0f deg, up to %.0f deg/s (relative gyro heading).", deg, cap);
  }
  return true;
}

void walkLinkSend() {
  char b[160];
  unsigned fl = walkFlags() | (walkFresh() ? 16 : 0) | (walkCtl.headingActive ? 32 : 0) |
                (walkCtl.recovery ? 64 : 0) | (walkCtl.stableT >= 1 ? 128 : 0);
  int n = snprintf(b, sizeof b, "D,1,%lu,%u,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
                   millis() / 1000UL, fl, (int)lroundf(wc::wrap(walkCtl.heading) * 10),
                   (int)lroundf(constrain(walkCtl.headingError, -360.0f, 360.0f) * 10),
                   (int)walkCtl.turn, (int)lroundf(walkCtl.turnDegrees * 10),
                   (int)lroundf(constrain(walkCtl.turnMeasured, -360.0f, 360.0f) * 10),
                   walkCtl.roughLevel, (int)lroundf(walkCtl.rough * 100),
                   (int)lroundf(walkCtl.speedScale * 100), (int)lroundf(walkCtl.strokeScale * 100),
                   (int)lroundf(walkCtl.landingK * 100), (int)lroundf(walkCtl.lastImpact * 100),
                   (int)lroundf(walkCtl.heightDrop * 10), (int)walkCtl.reason);
  if (n > 0 && n < (int)sizeof b)
    Serial2.write((const uint8_t *)b, n);
}
