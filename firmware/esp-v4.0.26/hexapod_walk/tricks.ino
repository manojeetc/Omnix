// Trick playback and returns to the walking stance. Fault stops bypass the normal return sequence.

int partyLeft = 0;
uint64_t nextPartyAt = 0;
int lastPartyTrick = TR_NONE;

bool canTrick() {
  return !servoHold && !servoSettling && !walkCtl.recovery && walkCtl.heightDrop < .5f &&
         mode == GAIT && !stepping && !lastMove && park == P_NONE && !sitting && servosOn &&
         !batLocked && !centerActive && zcLeg < 0 && zsLeg < 0 && ms64() >= sagTestUntil &&
         fabsf(targetH - bodyH) < 0.5f;
}

void trickClearPose() {
  pTx = 0;
  pTy = 0;
  pTz = 0;
  pRoll = 0;
  pPitch = 0;
  pYaw = 0;
  memset(pFoot, 0, sizeof pFoot);
}

void trickStop() {
  trickPlayer.clear();
  trick = TR_NONE;
  partyLeft = 0;
  trickClearPose();
}

// Land raised feet before easing the body back to its normal stance.
void trickReturn() {
  partyLeft = 0;
  trickPlayer.cancel();
}

bool trickStart(int id, float amp) {
  if (id <= TR_NONE || id >= TR_COUNT || !canTrick())
    return false;
  trickPlayer.request(id, amp);
  trick = trickPlayer.id;
  trickAmp = trickPlayer.amp;
  if (!trickPlayer.returning) {
    trickClearPose();
    trickT0 = millis();
  }
  return true;
}

// Playback follows the governed clock; return motion keeps feet and body transitions separate.
void trickUpdate(uint32_t now) {
  if (trickPlayer.id == TR_NONE)
    return;
  if (park != P_NONE || mode != GAIT || stepping) {
    trickStop();
    return;
  }
  if (lastMove)
    trickReturn();
  bool ended = trickPlayer.tick(constrain(tickDt, 0.005f, 0.1f) * constrain(govScale, 0.02f, 1.0f));
  trick = trickPlayer.id;
  trickAmp = trickPlayer.amp;
  if (ended) {
    lastTrickEnd = now;
    if (partyLeft > 0 && trick == TR_NONE)
      nextPartyAt = ms64() + 250;
  }
  if (trick == TR_NONE) {
    trickClearPose();
    return;
  }
  trickT0 = now - (uint32_t)(trickPlayer.time * 1000);
  const TrickPose &p = trickPlayer.pose;
  float s = endsSwapped ? -1 : 1;
  pTx = s * p.tx;
  pTy = s * p.ty;
  pTz = p.tz;
  pRoll = s * p.roll;
  pPitch = s * p.pitch;
  pYaw = p.yaw;
  for (int n = 0; n < 6; n++) {
    int i = physLeg(n);
    pFoot[i][0] = s * p.foot[n][0];
    pFoot[i][1] = s * p.foot[n][1];
    pFoot[i][2] = p.foot[n][2];
  }
}

void partyCheck() {
  if (partyLeft <= 0 || trick != TR_NONE || !nextPartyAt || ms64() < nextPartyAt)
    return;
  nextPartyAt = 0;
  const int N = sizeof PARTY_SET / sizeof PARTY_SET[0];
  int id;
  do
    id = PARTY_SET[esp_random() % N];
  while (id == lastPartyTrick);
  int left = partyLeft;
  if (trickStart(id, 1.0f)) {
    partyLeft = left - 1;
    lastPartyTrick = id;
    say("Party: %s (%d more)", TRICK_NAME[id], partyLeft);
  } else
    partyLeft = 0;
}

void trickRandom(bool small) {
  if (small) {
    const int N = sizeof FIDGET_SET / sizeof FIDGET_SET[0];
    trickStart(FIDGET_SET[esp_random() % N], 0.55f);
    return;
  }
  int id = 1 + esp_random() % (TR_COUNT - 1);
  if (trickStart(id, 1.0f))
    say("Trick: %s", TRICK_NAME[id]);
}

void fidgetCheck() {
  uint32_t now = millis();
  if (!fidgetOn || trick != TR_NONE || partyLeft > 0 || !canTrick())
    return;
  if (ms64() < nextFidgetAt || now - lastActivity < 15000 || now - lastTrickEnd < 15000)
    return;
  trickRandom(true);
  nextFidgetAt = ms64() + 25000 + esp_random() % 35000;
}

bool trickCommand(const String &c, const String &arg) {
  if (c == "fidget") {
    String v = arg;
    v.toLowerCase();
    fidgetOn = (v != "off");
    prefs.putBool("fidget", fidgetOn);
    say(fidgetOn ? "Fidgeting on (saved): small random moves when it stands still."
                 : "Fidgeting off (saved).");
    return true;
  }
  if (c == "tricks") {
    String s = "Tricks:";
    for (int k = 1; k < TR_COUNT; k++) {
      s += ' ';
      s += TRICK_NAME[k];
    }
    say("%s | trick | party", s.c_str());
    return true;
  }
  int id = -1;
  if (c == "trick")
    id = 0;
  if (c == "party")
    id = 100;
  if (c == "hi")
    id = TR_WAVE;
  if (c == "yes")
    id = TR_NOD;
  if (c == "pushups")
    id = TR_PUSHUP;
  for (int k = 1; k < TR_COUNT; k++)
    if (c == TRICK_NAME[k])
      id = k;
  if (id < 0)
    return false;
  if (mode != GAIT || stepping || lastMove) {
    say("Stand still first (x, then stand).");
    return true;
  }
  if (sitting) {
    say("Stand up first (stand).");
    return true;
  }
  if (id == 0) {
    trickRandom(false);
    return true;
  }
  if (id == 100) {
    String v = arg;
    v.toLowerCase();
    if (v == "off") {
      trickReturn();
      say("Party over: landing feet, then returning to neutral.");
      return true;
    }
    trickReturn();
    partyLeft = 8;
    lastPartyTrick = TR_NONE;
    nextPartyAt = ms64();
    say("PARTY! 8 moves (x or party off to stop).");
    return true;
  }
  if (trickStart(id, 1.0f))
    say("Trick: %s", TRICK_NAME[id]);
  else
    say("Can't do a trick right now (stand still, servos on, not parked).");
  return true;
}
