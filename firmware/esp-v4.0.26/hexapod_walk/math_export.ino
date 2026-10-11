// Export the current geometry and calibration for offline analysis. Leg arrays use physical order.

void mathDump() {
  if (!sensorMaintenanceIdle()) {
    say("Stop and let the feet/body settle before mathdump.");
    return;
  }
  say("{");
  say("  \"schema\":1, \"firmware\":\"4.0.26\", \"calibrated\":%s,", calibrated ? "true" : "false");
  say("  \"height_mm\":%.6f, \"width_mm\":%.6f, \"lift_mm\":%.6f,", targetH, width, LIFT);
  say("  \"period_s\":%.6f, \"speed_mm_s\":%.6f, \"gait\":%d,", PERIOD, speed, gait);
  say("  \"mass_g\":%.6f, \"stall_kgcm\":%.6f, \"servo_dps\":%.6f,", massG, stallKgcm, servoDps);
  say("  \"com_x_mm\":%.6f, \"com_y_mm\":%.6f, \"com_z_mm\":20.0,", comX, comY);
  say("  \"sag_mm\":%.6f, \"backlash_mm\":%.6f, \"zfix\":%s,", sagFull, blPlay,
      zFix ? "true" : "false");
  say("  \"autogait\":%s, \"dutyadapt\":%s, \"ends_swapped\":%s,", autoGait ? "true" : "false",
      dutyAdapt ? "true" : "false", endsSwapped ? "true" : "false");
  say("  \"zerr_mm\":[");
  for (int i = 0; i < 6; i++)
    say("    %.6f%s", zErr[i], i < 5 ? "," : "");
  say("  ],");
  for (int side = 0; side < 2; side++) {
    say(side == 0 ? "  \"q_min_deg\":[" : "  \"q_max_deg\":[");
    for (int i = 0; i < 6; i++) {
      float v[3];
      for (int j = 0; j < 3; j++) {
        float a, b;
        servoRange(i, j, a, b);
        v[j] = side == 0 ? a : b;
      }
      say("    [%.6f,%.6f,%.6f]%s", v[0], v[1], v[2], i < 5 ? "," : "");
    }
    say(side == 0 ? "  ]," : "  ]");
  }
  say("}");
}
