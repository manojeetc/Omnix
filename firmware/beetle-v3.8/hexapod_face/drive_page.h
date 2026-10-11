// Walking-assist dashboard: heading, terrain response, landing and balance state.

#pragma once
static DriveState DS;

static const char *driveAction() {
  if (!DS.valid)
    return "WAITING FOR DRIVE DATA";
  if (!(DS.flags & 16))
    return "MPU NEEDS DATA / CALIBRATION";
  if (DS.flags & 64)
    return DS.flags & 128 ? "UPRIGHT - COMMAND TO RESUME" : "BALANCE HOLD - FEET DOWN";
  if (DS.turn == 1)
    return "TURNING TO ANGLE";
  if (DS.turn == 2)
    return "TURN - SETTLING FEET";
  if (DS.ground == 2)
    return "ROUGH - WAVE / SHORT STEPS";
  if (DS.ground == 1)
    return "ROUGH - RIPPLE / SLOWING";
  if (DS.flags & 32)
    return "KEEPING HEADING";
  return "READY";
}

static void pageDrive(const Robot &R) {
  bgPage();
  header("WALK CONTROL", R);
  if (!DS.valid) {
    text(F_SMALL, "WAITING FOR WALK DATA", 160, 100, C_DIM, AL_C);
    text(F_TINY, "ESP32 firmware v4.0.14 required", 160, 124, C_DIM, AL_C);
    pageDots(PG_DRIVE);
    return;
  }
  bool hold = DS.flags & 64, imu = DS.flags & 16;
  uint16_t c = hold ? C_BAD : DS.ground ? C_WARN : C_EYE;
  card(10, 31, 144, 91, "RELATIVE HEADING");
  textf(F_BIG, 22, 82, imu ? C_TXT : C_DIM, AL_L, "%+.1f", DS.heading);
  textf(F_TINY, 22, 106, C_DIM, AL_L, "error %+.1f deg", DS.error);
  card(162, 31, 148, 91, "MEASURED TURN");
  if (DS.turn) {
    textf(F_SMALL, 174, 70, DS.turn == 4 ? C_WARN : C_TXT, AL_L, "%+.0f / %+.0f", DS.turned,
          DS.degrees);
    float f = DS.degrees ? DS.turned / DS.degrees : 0;
    bar(174, 79, 124, 5, f, C_ACC);
    static const char *const names[] = {"IDLE", "TURNING", "SETTLING", "DONE", "STOPPED"};
    text(F_TINY, names[DS.turn], 174, 107, C_DIM);
  } else {
    text(F_SMALL, "turn 90", 174, 72, C_TXT);
    text(F_TINY, "+left / -right", 174, 107, C_DIM);
  }
  card(10, 130, 144, 65, "GROUND RESPONSE");
  textf(F_TINY, 22, 164, c, AL_L, "%s %.2f",
        DS.ground == 2   ? "WAVE"
        : DS.ground == 1 ? "RIPPLE"
                         : "NORMAL",
        DS.rough);
  textf(F_TINY, 22, 183, C_DIM, AL_L, "v %.0f%%  step %.0f%%", DS.speed * 100, DS.stroke * 100);
  card(162, 130, 148, 65, "SOFT DESCENT");
  textf(F_TINY, 174, 164, C_TXT, AL_L, "soft %.2f  %.2fg", DS.landing, DS.impact);
  textf(F_TINY, 174, 183, C_DIM, AL_L, "lowered %.1f mm", DS.drop);
  text(F_TINY, driveAction(), 160, 214, c, AL_C);
  text(F_TINY, "GYRO RELATIVE / BODY JOLTS", 160, 233, C_MUTE, AL_C);
  pageDots(PG_DRIVE);
}
