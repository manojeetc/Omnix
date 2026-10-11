// Parse the walking-assist telemetry sent alongside the main robot status line.

#pragma once
#include <stdint.h>
#include <stdlib.h>
#include <errno.h>

struct DriveState {
  bool valid = false;
  uint32_t upS = 0, receivedAt = 0;
  unsigned flags = 0;
  float heading = 0, error = 0, degrees = 0, turned = 0, rough = 0, speed = 1, stroke = 1,
        landing = .3f, impact = 0, drop = 0;
  int turn = 0, ground = 0, reason = 0;

  bool fresh(uint32_t now, uint32_t robotUp) const {
    return valid && (uint32_t)(now - receivedAt) < 750 &&
           (upS == robotUp || (robotUp > 0 && upS == robotUp - 1));
  }
};

static bool parseDrive(const char *s, DriveState &out, uint32_t now) {
  if (s[0] != 'D' || s[1] != ',')
    return false;
  long v[16];
  const char *p = s + 2;
  for (int i = 0; i < 16; i++) {
    if (!*p || *p == ' ' || *p == '\t')
      return false;
    errno = 0;
    char *e;
    v[i] = strtol(p, &e, 10);
    if (e == p || errno == ERANGE)
      return false;
    if (i < 15) {
      if (*e != ',')
        return false;
      p = e + 1;
    } else if (*e)
      return false;
  }
  if (v[0] != 1 || v[1] < 0 || v[2] < 0 || v[2] > 255 || v[3] < -1800 || v[3] > 1800 ||
      v[4] < -3600 || v[4] > 3600 || v[5] < 0 || v[5] > 4 || v[6] < -1800 || v[6] > 1800 ||
      v[7] < -3600 || v[7] > 3600 || v[8] < 0 || v[8] > 2 || v[9] < 0 || v[9] > 900 || v[10] < 0 ||
      v[10] > 100 || v[11] < 0 || v[11] > 100 || v[12] < 20 || v[12] > 80 || v[13] < 0 ||
      v[13] > 300 || v[14] < 0 || v[14] > 80 || v[15] < 0 || v[15] > 6)
    return false;
  DriveState d;
  d.valid = true;
  d.upS = (uint32_t)v[1];
  d.receivedAt = now;
  d.flags = (unsigned)v[2];
  d.heading = v[3] / 10.0f;
  d.error = v[4] / 10.0f;
  d.turn = (int)v[5];
  d.degrees = v[6] / 10.0f;
  d.turned = v[7] / 10.0f;
  d.ground = (int)v[8];
  d.rough = v[9] / 100.0f;
  d.speed = v[10] / 100.0f;
  d.stroke = v[11] / 100.0f;
  d.landing = v[12] / 100.0f;
  d.impact = v[13] / 100.0f;
  d.drop = v[14] / 10.0f;
  d.reason = (int)v[15];
  out = d;
  return true;
}
