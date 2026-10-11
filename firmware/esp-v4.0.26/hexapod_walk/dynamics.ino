// Record inverse-dynamics demand after servo output. Opt-in shaping adjusts the shared gait clock.

bool equationDynamicsEligible() {
  return stepping && lastMove && walkEligible() && walkFresh() && !walkCtl.recovery &&
         walkCtl.turn != wc::SETTLING;
}

void equationDynamicsInvalidate() {
  dynDemand.history.reset();
  dynSampleAt = 0;
  dynEligible = false;
  dynDemand.requested = 1;
  dynDemand.ease(1, tickDt);
}

void equationDynamicsUpdate(bool hadSample, const bool validLeg[6]) {
  uint32_t start = micros(), now = millis();
  dynEligible = equationDynamicsEligible();
  if (dynDemand.mode == heq::DynamicDemand::OFF) {
    equationDynamicsInvalidate();
    return;
  }

  if (massG != dynDemand.lastMassG || stallKgcm != dynDemand.lastStallKgcm ||
      dynDemand.limit != dynDemand.lastLimit) {
    dynDemand.reset();
    dynSampleAt = 0;
    dynDemand.lastMassG = massG;
    dynDemand.lastStallKgcm = stallKgcm;
    dynDemand.lastLimit = dynDemand.limit;
  }
  float measuredDt = dynSampleAt ? (now - dynSampleAt) * .001f : tickDt;
  dynSampleAt = now;
  bool valid = hadSample && servosOn && !servoHold && !servoSettling && supportOK && walkFresh() &&
               isfinite(massG) && massG > 0 && isfinite(stallKgcm) && stallKgcm > 0;
  for (int i = 0; i < 6; i++)
    valid &= validLeg[i] && isfinite(loadF[i]) && loadF[i] >= 0;
  float share = 0;
  for (int i = 0; i < 6; i++)
    share += loadF[i];
  valid &= fabsf(share - 1) < .02f;
  valid &= isfinite(tickDt) && tickDt >= .005f && tickDt <= .060f && measuredDt >= .005f &&
           measuredDt <= .060f;
  if (!valid) {
    dynDemand.invalidate(tickDt);
    return;
  }
  float q[18];
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      q[3 * i + j] = qCmd[i][j] * GM_D2R;
  if (!dynDemand.history.push(q, measuredDt)) {
    dynDemand.skipped++;
    dynDemand.requested = 1;
    dynDemand.ease(1, tickDt);
    return;
  }
  float up[3], gravity[3], base[9] = {}, zero[3] = {}, stationary[18], moving[18];
  equationUp(up);
  if (!heq::finiteArray(up, 3) || fabsf(up[0] * up[0] + up[1] * up[1] + up[2] * up[2] - 1) > .02f) {
    dynDemand.invalidate(tickDt);
    return;
  }
  for (int j = 0; j < 3; j++)
    gravity[j] = -9.81f * up[j];
  for (int i = 0; i < 6; i++) {
    float model[42], force[3], a[9], b[9];
    equationModel(i, model);
    for (int j = 0; j < 3; j++)
      force[j] = massG * .00981f * loadF[i] * up[j];
    heq::inverse_dynamics(q + 3 * i, zero, zero, heq::cadAz[i], model, base, gravity, force, a);
    heq::inverse_dynamics(q + 3 * i, dynDemand.history.rate + 3 * i,
                          dynDemand.history.accel + 3 * i, heq::cadAz[i], model, base, gravity,
                          force, b);
    for (int j = 0; j < 3; j++) {
      stationary[3 * i + j] = a[j];
      moving[3 * i + j] = b[j];
    }
  }
  dynDemand.observe(stationary, moving, stallKgcm * .0980665f, govScale, measuredDt, stepping,
                    dynEligible);
  uint32_t elapsed = micros() - start;
  dynDemand.worstUs = max(dynDemand.worstUs, elapsed);
}

void equationDynamicsStatus() {
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the robot settle before dynamics reports; the recorder stays quiet while walking.");
    return;
  }
  const char *name = dynDemand.mode == heq::DynamicDemand::ON        ? "ON"
                     : dynDemand.mode == heq::DynamicDemand::MONITOR ? "monitor"
                                                                     : "off";
  say("Dynamics %s: estimated-demand target %.0f%% of configured stall; clock ceiling %.0f%% (floor 60%%). Not saved.",
      name, dynDemand.limit * 100, dynDemand.ceiling * 100);
  say("Model: commanded motion, CAD masses/inertias, estimated gravity/contact shares; base motion assumed zero. No measured torque.");
  say("Window: %lu valid frames, %lu walking, %lu skipped/warm-up; %lu shaped; recorder worst %lu us.",
      (unsigned long)dynDemand.frames, (unsigned long)dynDemand.walkFrames,
      (unsigned long)dynDemand.skipped, (unsigned long)dynDemand.shaped,
      (unsigned long)dynDemand.worstUs);
  if (!dynDemand.walkFrames) {
    say("No walking samples yet. dynamics reset, short walk, x, then dynamics.");
    return;
  }
  say("Walking peaks: modeled total %.1f%%, static %.1f%%, motion increment %.1f%% of configured stall (%.2f kg*cm, mass %.0f g).",
      dynDemand.peakFraction * 100, dynDemand.peakStaticFraction * 100,
      dynDemand.peakIncrementFraction * 100, dynDemand.lastStallKgcm, dynDemand.lastMassG);
  say("Lowest applied demand ceiling %.0f%%; static above target %lu frames; requested below 60%% floor %lu frames.",
      dynDemand.minCeiling * 100, (unsigned long)dynDemand.staticHigh,
      (unsigned long)dynDemand.floorLimited);
  const char *jointName[3] = {"C", "F", "T"};
  for (int i = 0; i < 6; i++)
    say("%s peak modeled C/F/T N*m: %.4f %.4f %.4f", legName(i), dynDemand.peakMoving[3 * i],
        dynDemand.peakMoving[3 * i + 1], dynDemand.peakMoving[3 * i + 2]);
  if (dynDemand.peakJoint >= 0)
    say("Highest modeled load: %s %s.", legName(dynDemand.peakJoint / 3),
        jointName[dynDemand.peakJoint % 3]);
  if (dynDemand.staticHigh)
    say("Static load exceeded the target in this model; slowing cannot remove that weight demand.");
  if (dynDemand.floorLimited)
    say("Demand shaping reached its 60%% floor; this is not enforcement of a torque limit.");
}

void equationDynamicsCommand(const char *action, const char *value) {
  if (!action || !*action || !strcmp(action, "status") || !strcmp(action, "check")) {
    equationDynamicsStatus();
    return;
  }

  if (!strcmp(action, "off")) {
    dynDemand.mode = heq::DynamicDemand::OFF;
    equationDynamicsInvalidate();
    say("Dynamics off. Existing motion guards remain active.");
    return;
  }
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the robot settle before changing dynamics settings.");
    return;
  }
  if (!strcmp(action, "on")) {
    if (!isfinite(massG) || massG <= 0 || !isfinite(stallKgcm) || stallKgcm <= 0) {
      say("Dynamics needs a positive mass and configured stall estimate first.");
      return;
    }
    dynDemand.mode = heq::DynamicDemand::ON;
    dynDemand.reset();
    dynSampleAt = 0;
    say("Dynamics ON: experimental coordinated demand shaping, 60-100%% clock ceiling; model target %.0f%% of configured stall. Not saved.",
        100 * dynDemand.limit);
    return;
  }
  if (!strcmp(action, "monitor")) {
    dynDemand.mode = heq::DynamicDemand::MONITOR;
    dynDemand.reset();
    dynSampleAt = 0;
    say("Dynamics monitor: records modeled walking loads; no demand shaping. Not saved.");
    return;
  }
  if (!strcmp(action, "reset")) {
    dynDemand.reset();
    dynSampleAt = 0;
    say("Dynamics comparison window cleared.");
    return;
  }
  if (!strcmp(action, "limit")) {
    char *end = nullptr;
    float v = value ? strtof(value, &end) : 0;
    if (!value || end == value || *end || !isfinite(v) || v < .20f || v > .90f) {
      say("Use dynamics limit 0.20..0.90 (fraction of configured stall). This is a model target.");
      return;
    }
    dynDemand.limit = v;
    dynDemand.reset();
    dynSampleAt = 0;
    say("Dynamics model target %.0f%% of configured stall. Not saved.", 100 * v);
    return;
  }
  say("dynamics [status/check/monitor/on/off/reset] | dynamics limit 0.70");
}
