// Optional PCA clock diagnostics. Channel 15 is a timing marker, not a servo channel.
// Only its signal pin connects to the ESP32; see the wiring notes in the firmware README.

#include "pca_monitor.h"
#include "hal/gpio_ll.h"
#include "soc/gpio_struct.h"

// The ISR publishes edge data with release ordering; the loop consumes it with acquire ordering.
struct PmRing {
  static const uint16_t SZ = 128;
  volatile uint32_t t[SZ];
  volatile uint8_t lv[SZ];
  std::atomic<uint16_t> head{0}, tail{0};
  volatile uint32_t overflow = 0, winStart = 0, winCount = 0;
  volatile bool killed = false;
  uint8_t pin = 0;
};

static PmRing pmRing[2];
static PcaMon pmon[2];
static bool pmOn = false, pmKillSaid[2] = {false, false}, pmAutoSaid = false;
static uint32_t pmOnAt = 0;
static const uint8_t PM_PIN[2] = {35, 4};

// Disable a noisy input after 400 edges in 100 ms so diagnostics cannot starve the control loop.
static void IRAM_ATTR pmIsr(void *arg) {
  PmRing &r = *(PmRing *)arg;
  uint32_t t = micros();
  int lv = gpio_ll_get_level(&GPIO, r.pin);
  if (t - r.winStart > 100000) {
    r.winStart = t;
    r.winCount = 0;
  }
  if (++r.winCount > 400) {
    gpio_ll_intr_disable(&GPIO, r.pin);
    r.killed = true;
    return;
  }
  uint16_t h = r.head.load(std::memory_order_relaxed), tl = r.tail.load(std::memory_order_acquire);
  if ((uint16_t)(h - tl) >= PmRing::SZ) {
    r.overflow++;
    return;
  }
  r.t[h % PmRing::SZ] = t;
  r.lv[h % PmRing::SZ] = (uint8_t)lv;
  r.head.store((uint16_t)(h + 1), std::memory_order_release);
}

bool pcamonActive() {
  return pmOn;
}

static void pmMarker(bool on) {
  for (int b = 0; b < 2; b++) {
    Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
    if (on)
      p.setPWM(SYNC_CH, MARKER_ON, MARKER_ON + MARKER_TICKS);
    else
      p.setPWM(SYNC_CH, 0, 4096);
  }
}

static uint32_t outputChecksum() {
  uint32_t h = 2166136261u;
  auto mix = [&](const void *p, size_t n) {
    const uint8_t *c = (const uint8_t *)p;
    for (size_t k = 0; k < n; k++) {
      h ^= c[k];
      h *= 16777619u;
    }
  };
  mix(pulseUs, sizeof pulseUs);
  mix(pcaTickUs, sizeof pcaTickUs);
  mix(trim, sizeof trim);
  mix(gainP, sizeof gainP);
  mix(gainN, sizeof gainN);
  mix(sgn, sizeof sgn);
  uint8_t ps[2] = {pcaL.readPrescale(), pcaR.readPrescale()};
  mix(ps, 2);
  return h;
}

static void pmStatus() {
  uint32_t now = micros();
  for (int b = 0; b < 2; b++) {
    Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
    PcaMonResult r = pmon[b].result(now);
    int ps = p.readPrescale();
    const char *nm = b == 0 ? "left " : "right";
    if (pmRing[b].killed) {
      say("PCA %s: interrupt storm on D%d (noise on the wire): this board's monitor stopped. Fix wiring, then pcamon on.",
          nm, PM_PIN[b]);
      continue;
    }
    if (r.frames == 0 && !r.tracking) {
      say("PCA %s: no marker seen on D%d yet (%lu bad-width pulses, %lu malformed). Wire connected? 3 s after pcamon on it should lock.",
          nm, PM_PIN[b], (unsigned long)r.badWidth, (unsigned long)r.malformed);
      continue;
    }
    double T = r.periodUs, osc = T > 0 ? 4096.0 * (ps + 1) / (T * 1e-6) : 0;
    float hiExp = (float)(T * MARKER_TICKS / 4096.0);
    say("PCA %s: %s %.4f Hz (period %.2f us, median %.1f, spread %.0f) -> clock %.3f MHz (prescale %d) | marker %.1f us (expect %.1f)",
        nm, r.valid ? "VALID" : (r.stale ? "STALE" : "acquiring"), T > 0 ? 1e6 / T : 0.0, T,
        r.medianUs, r.spreadUs, osc / 1e6, ps, r.hiUs, hiExp);
    say("        frames %lu, missing %lu, unexpected pulses %lu, bad width %lu, malformed %lu, lost lock %lu, buffer overflow %lu%s",
        (unsigned long)r.frames, (unsigned long)r.gaps, (unsigned long)r.unexpected,
        (unsigned long)r.badWidth, (unsigned long)r.malformed, (unsigned long)r.lostCount,
        (unsigned long)pmRing[b].overflow, r.stale ? " | last frame long ago: reading is OLD" : "");
  }
  say("Servo output checksum %08lX (diagnostics only: it must read the same with pcamon on and off).",
      (unsigned long)outputChecksum());
}

void pcamonCommand(String a) {
  a.toLowerCase();
  if (a == "on") {
    if (pmOn) {
      say("pcamon already on. pcamon = status, pcamon off = stop.");
      return;
    }
    say("pcamon: channel 15 of both boards now carries a timing marker. Nothing may be plugged into channel 15 (bench servo!).");
    uint32_t before = outputChecksum();
    for (int b = 0; b < 2; b++) {
      pmon[b].reset();
      pmRing[b].head = 0;
      pmRing[b].tail = 0;
      pmRing[b].overflow = 0;
      pmRing[b].winCount = 0;
      pmRing[b].killed = false;
      pmRing[b].pin = PM_PIN[b];
      pmKillSaid[b] = false;
    }
    pinMode(PM_PIN[0], INPUT);
    pinMode(PM_PIN[1], INPUT_PULLDOWN);
    pmMarker(true);
    for (int b = 0; b < 2; b++)
      attachInterruptArg(digitalPinToInterrupt(PM_PIN[b]), pmIsr, &pmRing[b], CHANGE);
    pmOn = true;
    pmOnAt = millis();
    pmAutoSaid = false;
    say("pcamon ON (diagnostics only; servo output unchanged, checksum %08lX). Report in ~3 s, or type pcamon.",
        (unsigned long)before);
    return;
  }
  if (a == "off") {
    if (pmOn)
      for (int b = 0; b < 2; b++)
        detachInterrupt(digitalPinToInterrupt(PM_PIN[b]));
    pmMarker(false);
    pmOn = false;
    say("pcamon OFF: channel 15 off, interrupts detached. Servo output checksum %08lX.",
        (unsigned long)outputChecksum());
    return;
  }
  if (a == "reset") {
    for (int b = 0; b < 2; b++)
      pmon[b].reset();
    say("pcamon counters cleared.");
    return;
  }
  if (!pmOn) {
    say("pcamon is off. pcamon on = start the clock monitor (diagnostics only). Servo output checksum %08lX.",
        (unsigned long)outputChecksum());
    return;
  }
  pmStatus();
}

void pcamonService() {
  if (!pmOn)
    return;
  for (int b = 0; b < 2; b++) {
    PmRing &r = pmRing[b];
    uint16_t tl = r.tail.load(std::memory_order_relaxed),
             h = r.head.load(std::memory_order_acquire);
    while (tl != h) {
      pmon[b].edge(r.t[tl % PmRing::SZ], r.lv[tl % PmRing::SZ]);
      tl++;
    }
    r.tail.store(tl, std::memory_order_release);
    pmon[b].tick(micros());
    if (r.killed && !pmKillSaid[b]) {
      pmKillSaid[b] = true;
      say("pcamon: interrupt storm on D%d (%s board): monitor for that board stopped (noise on the wire).",
          PM_PIN[b], b == 0 ? "left" : "right");
    }
  }
  if (!pmAutoSaid && millis() - pmOnAt > 3000) {
    pmAutoSaid = true;
    pmStatus();
  }
}
