// Equation diagnostics and timing checks. These estimates use commanded motion, not servo feedback.

void equationModel(int i, float out[42]) {
  float scale = massG > 0 ? massG / heq::referenceMassG : 1;
  for (int k = 0; k < 42; k++)
    out[k] = heq::cadLeg[i][k] * ((k >= 12 && k < 15) || k >= 24 ? scale : 1);
}

void equationUp(float out[3]) {
  bool calibratedUp = imuOK && hasLevel && hasFront && upF.init;
  for (int k = 0; k < 3; k++)
    out[k] = calibratedUp ? upF.u[k] : (k == 2 ? 1 : 0);
}

void equationStatus() {
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the robot settle before equations reports.");
    return;
  }
  say("Equations: general float functions, offline; endpoint rate %.1f deg/s, guard raises %lu, skipped intervals %lu.",
      eqEndpointPeakDps, (unsigned long)eqRateRaises, (unsigned long)eqRateSkipped);
  say("Rigid-link load estimates use commanded joints and CAD mass/inertia estimates; base motion assumed zero. No torque commands.");
  if (!qValid) {
    say("No current joint target yet.");
    return;
  }
  float u[3], gravity[3], z[3] = {}, base[9] = {};
  equationUp(u);
  for (int k = 0; k < 3; k++)
    gravity[k] = -9.81f * u[k];
  for (int i = 0; i < 6; i++) {
    float model[42], q[3], qd[3], qdd[3], force[3], result[9];
    equationModel(i, model);
    for (int k = 0; k < 3; k++) {
      q[k] = qCmd[i][k] * GM_D2R;
      qd[k] = eqCommandRate[i][k] * GM_D2R;
      qdd[k] = eqCommandAccel[i][k] * GM_D2R;
      force[k] = (massG > 0 ? massG * .00981f : 0) * loadF[i] * u[k];
    }
    if (!servosOn || servoHold || millis() - eqSampleAt > 100) {
      for (int k = 0; k < 3; k++)
        qd[k] = qdd[k] = 0;
    }
    heq::inverse_dynamics(q, qd, qdd, heq::cadAz[i], model, base, gravity, force, result);
    say("%s modeled actuator C/F/T: %+.4f %+.4f %+.4f N*m; modeled coxa RMS reaction %.0f%% stall.",
        legName(i), result[0], result[1], result[2], 100 * sqrtf(eqCoxaHeat[i]));
  }
}

void equationBenchmark() {
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the robot settle before equations bench.");
    return;
  }
  float z[3] = {}, base[9] = {}, g[3] = {0, 0, -9.81f}, force[3] = {}, q[3],
        qd[3] = {.2f, -.3f, .4f}, qdd[3] = {1, -2, 3};
  float model[6][42];
  for (int i = 0; i < 6; i++)
    equationModel(i, model[i]);
  uint32_t total = 0, worst = 0;
  float check = 0;
  for (int round = 0; round < 16; round++) {
    uint32_t start = micros();
    for (int i = 0; i < 6; i++) {
      for (int k = 0; k < 3; k++)
        q[k] = (qValid ? qCmd[i][k] : 0) * GM_D2R;
      q[0] += round * .0001f;
      float out[9];
      heq::inverse_dynamics(q, qd, qdd, heq::cadAz[i], model[i], base, g, force, out);
      check += out[0] + out[2];
    }
    uint32_t elapsed = micros() - start;
    total += elapsed;
    worst = max(worst, elapsed);
    delay(0);
  }
  say("Equations bench: 6-leg dynamics average %lu us, worst %lu us; checksum %.5f. Measured on this ESP32.",
      (unsigned long)(total / 16), (unsigned long)worst, check);
  say("Inverse dynamics: dynamics monitor/on/off; shaping is opt-in. mem reset clears command timing.");
}

void equationState() {
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the robot settle before equations state.");
    return;
  }
  if (!qValid) {
    say("No current joint target yet.");
    return;
  }
  bool fresh = servosOn && !servoHold && millis() - eqSampleAt <= 100;
  say("{");
  say("  \"schema\":1, \"state_source\":\"commanded_targets_not_joint_feedback\",");
  const char *names[3] = {"q_deg", "qd_deg_s", "qdd_deg_s2"};
  for (int type = 0; type < 3; type++) {
    say("  \"%s\":[", names[type]);
    for (int i = 0; i < 6; i++) {
      float v[3];
      for (int k = 0; k < 3; k++)
        v[k] = type == 0   ? qCmd[i][k]
               : !fresh    ? 0
               : type == 1 ? eqCommandRate[i][k]
                           : eqCommandAccel[i][k];
      say("    [%.6f,%.6f,%.6f]%s", v[0], v[1], v[2], i < 5 ? "," : "");
    }
    say("  ],");
  }
  float u[3];
  equationUp(u);
  say("  \"base\":[0,0,0,0,0,0,0,0,0], \"base_motion_assumed_zero\":true,");
  say("  \"gravity_m_s2\":[%.6f,%.6f,%.6f],", -9.81f * u[0], -9.81f * u[1], -9.81f * u[2]);
  say("  \"ground_normal_body\":[%.6f,%.6f,%.6f],", u[0], u[1], u[2]);
  say("  \"contact_mask\":[%s,%s,%s,%s,%s,%s]", loadF[0] > 0 ? "true" : "false",
      loadF[1] > 0 ? "true" : "false", loadF[2] > 0 ? "true" : "false",
      loadF[3] > 0 ? "true" : "false", loadF[4] > 0 ? "true" : "false",
      loadF[5] > 0 ? "true" : "false");
  say("}");
}
