// Send robot status to the Beetle over UART. Field order must match state.h and drive_state.h.

const int LINK_TX = 17;
const uint32_t LINK_BAUD = 230400;

enum ScrPage {
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

const char *const PAGE_NAME[PG_COUNT] = {"face", "dash",  "legs", "level", "sys",
                                         "off",  "trail", "stab", "heat",  "drive"};
const int PAGE_CYCLE[] = {PG_FACE, PG_DASH,  PG_LEGS,  PG_TRAIL, PG_STAB,
                          PG_HEAT, PG_DRIVE, PG_LEVEL, PG_SYS};
const int NCYCLE = sizeof PAGE_CYCLE / sizeof PAGE_CYCLE[0];

int cycleIndex(int p) {
  for (int k = 0; k < NCYCLE; k++)
    if (PAGE_CYCLE[k] == p)
      return k;
  return -1;
}

volatile int scrPage = PG_FACE;
volatile bool scrSleeping = false;
uint8_t scrRot = 1, scrMHz = 75, scrTest = 0, scrTrailReset = 0;
bool scrInv = true, scrOK = false, scrFps = false;
extern int partyLeft;

uint32_t linkDropped = 0;

void linkSend() {
  char b[440];
  uint32_t fl = (mode == GAIT) | servosOn << 1 | batLocked << 2 | stepping << 3 | sitting << 4 |
                imuOK << 5 | hasFront << 6 | levelOn << 7 | endsSwapped << 8 | BT.hasClient() << 9 |
                scrSleeping << 10 | scrFps << 11 | fidgetOn << 12 | calibrated << 13;
  uint8_t sw = 0;
  for (int i = 0; i < 6; i++)
    if (swinging[i])
      sw |= 1 << i;
  int n = snprintf(
      b, sizeof b, "H,2,%d,%d,%u,%d,%d,%d,%d,%d,%d,%d,%lu,%d,%u,%d,%d,%d,%d,%d,%d,%d,%d,%lu",
      (int)(batRest() * 100), batPct, (unsigned)fl, gait, (int)speed, (int)targetH,
      (int)(bodyH * 10), (int)(pitch * 10), (int)(roll * 10), (int)trick,
      trick ? (unsigned long)(trickPlayer.time * 1000) : 0UL, (int)(pYaw * 10), sw, (int)park,
      (int)lastMove, (int)scrPage, scrRot, scrInv, scrMHz, scrTest, (int)width, millis() / 1000UL);
  for (int i = 0; i < 6 && n < (int)sizeof b - 80; i++) {
    float fx, fy, fz;
    if (mode == GAIT) {
      fx = feet[i][0] + pFoot[i][0];
      fy = feet[i][1] + pFoot[i][1];
      fz = feet[i][2] + pFoot[i][2];
    } else {
      neutral(i, fx, fy);
      fz = -PARK_H;
    }
    n += snprintf(b + n, sizeof b - n, ",%d,%d,%d", (int)fx, (int)fy, (int)fz);
  }
  n += snprintf(b + n, sizeof b - n, ",%d,%d,%d,%lu,%d,%d,%d,%d,%d,%d,%d,%d,%d", (int)vx, (int)vy,
                (int)(wz * 1000), (unsigned long)scrEvt, (int)LIFT, (int)(PERIOD * 100), partyLeft,
                (int)(trickAmp * 100), (int)pTx, (int)pTy, (int)pTz, (int)(pRoll * 10),
                (int)(pPitch * 10));
  for (int i = 0; i < 6 && n < (int)sizeof b - 20; i++)
    n += snprintf(b + n, sizeof b - n, ",%d,%d", heatPct(i, 0), heatPct(i, 1));
  long idleLeft = -1;
  if (batV >= BAT_PRESENT && !batLocked && park == P_NONE && idleMin > 0 && !(lastMove || stepping))
    idleLeft = max(0L, (long)idleMin * 60L - (long)((millis() - lastActivity) / 1000UL));
  unsigned f2 = (tipping ? 1 : 0) | (stabSlowT > 0 ? 2 : 0) |
                (servoHold || servoSettling || boardDown[0] || boardDown[1] ? 4 : 0) |
                (imuLost ? 16 : 0) | (walkCtl.recovery ? 32 : 0);
  n += snprintf(b + n, sizeof b - n, ",%d,%d,%d,%ld,%u,%u",
                (int)lroundf(constrain(govScale, 0.0f, 1.0f) * 100), (int)lroundf(comX),
                (int)lroundf(comY), idleLeft, f2, (unsigned)scrTrailReset);
  if (n >= 418) {
    linkDropped++;
    return;
  }
  b[n++] = '\n';
  b[n] = 0;
  Serial2.write((const uint8_t *)b, n);
  walkLinkSend();
}

void linkService() {
  static uint32_t t0 = 0;
  uint32_t now = millis();
  if (scrOK && now - t0 >= 33) {
    t0 = now;
    linkSend();
  }
}

void screenBegin() {
  scrRot = prefs.getUChar("scrrot", 1);
  scrInv = prefs.getBool("scrinv", true);
  scrMHz = prefs.getUChar("scrmhz3", 75);
  scrPage = constrain(prefs.getUChar("scrpage", PG_FACE), 0, PG_COUNT - 1);
  Serial2.setTxBufferSize(1024);
  Serial2.begin(LINK_BAUD, SERIAL_8N1, -1, LINK_TX);
  scrOK = true;
  say("Screen link on TX2 (D17) -> Beetle.");
}

void screenSleep() {
  if (!scrOK)
    return;
  scrSleeping = true;
  for (int k = 0; k < 4; k++) {
    linkSend();
    delay(80);
  }
}

void setPage(int p) {
  scrPage = (p + PG_COUNT) % PG_COUNT;
  prefs.putUChar("scrpage", scrPage);
  say("Screen: %s (saved)", PAGE_NAME[scrPage]);
}

bool screenCommand(const String &c, const String &arg) {
  if (c == "screen") {
    String a = arg;
    a.toLowerCase();
    if (a == "" || a == "next") {
      setPage(PAGE_CYCLE[(cycleIndex(scrPage) + 1) % NCYCLE]);
      return true;
    }
    if (a == "prev") {
      int k = cycleIndex(scrPage);
      setPage(PAGE_CYCLE[k < 0 ? 0 : (k + NCYCLE - 1) % NCYCLE]);
      return true;
    }
    if (a == "motion") {
      setPage(PG_DRIVE);
      return true;
    }
    if (a == "map") {
      setPage(PG_TRAIL);
      return true;
    }
    if (a == "stability" || a == "support") {
      setPage(PG_STAB);
      return true;
    }
    if (a == "trail reset") {
      scrTrailReset++;
      say("Screen: trail cleared.");
      return true;
    }
    if (a == "on" || a == "main") {
      setPage(PG_FACE);
      return true;
    }
    if (a == "bal" || a == "balance") {
      setPage(PG_LEVEL);
      return true;
    }
    for (int k = 0; k < PG_COUNT; k++)
      if (a == PAGE_NAME[k]) {
        setPage(k);
        return true;
      }
    say("Pages: face dash legs trail stab heat drive level sys off (screen = next) | screen trail reset");
    return true;
  }
  if (c == "fps") {
    scrFps = !scrFps;
    say(scrFps ? "Screen performance box on." : "Screen performance box off.");
    return true;
  }
  if (c == "scrtest") {
    scrTest++;
    say("Screen test (red, green, blue, white, black).");
    return true;
  }
  if (c == "scrspeed") {
    int m = arg.toInt();
    if (m < 1 || m > 80) {
      say("Screen SPI now %d MHz. Usage: scrspeed 75 (or 37, 25)", scrMHz);
      return true;
    }
    scrMHz = m;
    prefs.putUChar("scrmhz3", scrMHz);
    say("Screen SPI %d MHz (saved). The Beetle can do 75, 37.5, 25... and rounds down. Check with fps.",
        scrMHz);
    return true;
  }
  if (c == "scrrot") {
    scrRot = scrRot == 1 ? 3 : 1;
    prefs.putUChar("scrrot", scrRot);
    say("Screen turned 180 (saved).");
    return true;
  }
  if (c == "scrinv") {
    scrInv = !scrInv;
    prefs.putBool("scrinv", scrInv);
    say("Screen colours inverted (saved).");
    return true;
  }
  return false;
}
