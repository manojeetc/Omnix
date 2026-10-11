// Omnix Beetle display. Core 0 updates state and renders the top band; core 1 renders the bottom.
// DMA sends the previous frame while both cores draw the next one. See ../../README.md for setup.

#pragma GCC optimize("O3", "fast-math")
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ST7789.h>
#include <hardware/dma.h>
#include <hardware/spi.h>
#include <pico/platform.h>

#define GFX_CORE() get_core_num()
#include "pages.h"

const int TFT_CS = 26, TFT_DC = 27, TFT_SCK = 18, TFT_MOSI = 19, LINK_RX = 1;
const uint32_t LINK_BAUD = 230400;
Adafruit_ST7789 tft(&SPI, TFT_CS, TFT_DC, -1);

// Two complete RGB565 buffers: one can be on the SPI bus while the other is being drawn.
alignas(4) static uint16_t fbuf[2][gfx::W * gfx::H];
static int back = 0;
static bool dualCore = true, useDMA = true, haveFrame = false;

enum PictureMode { LIVE, HOLD, REPEAT };

static PictureMode pictureMode = LIVE;
static const uint16_t *frozenFrame = nullptr;
static float frameLight = 0, lightMin = 100, lightMax = 0;
static uint32_t frameSamples = 0;

// Core 0 owns incoming telemetry and prepares a stable snapshot for rendering.
static DriveState driveLive;
static Robot link;
static Smoother smoother;
static char rxb[420];
static int rxLen = 0;
static uint32_t lines = 0, linesAtSec = 0, secT0 = 0;

static void pollLink() {
  while (Serial1.available()) {
    char ch = Serial1.read();
    if (ch == '\n') {
      rxb[rxLen] = 0;
      if (parseStatus(rxb, link)) {
        smoother.push(link, millis());
        U.lastRx = millis();
        lines++;
      } else if (parseDrive(rxb, driveLive, millis())) {
      } else if (rxLen > 2)
        U.linkErr++;
      rxLen = 0;
    } else if (rxLen < (int)sizeof rxb - 1)
      rxb[rxLen++] = ch;
    else
      rxLen = 0;
  }
}

static int dmaCh = -1;
static bool pushing = false;

static void setFormat(int bits) {
  uint32_t cr0 = spi_get_hw(spi0)->cr0;
  spi_set_format(spi0, bits, (cr0 & SPI_SSPCR0_SPO_BITS) ? SPI_CPOL_1 : SPI_CPOL_0,
                 (cr0 & SPI_SSPCR0_SPH_BITS) ? SPI_CPHA_1 : SPI_CPHA_0, SPI_MSB_FIRST);
}

// Send RGB565 as 16-bit SPI words, with chip select held for the entire DMA transfer.
static void pushStart(const uint16_t *frame) {
  tft.startWrite();
  tft.setAddrWindow(0, 0, gfx::W, gfx::H);
  setFormat(16);
  dma_channel_config cfg = dma_channel_get_default_config(dmaCh);
  channel_config_set_transfer_data_size(&cfg, DMA_SIZE_16);
  channel_config_set_read_increment(&cfg, true);
  channel_config_set_write_increment(&cfg, false);
  channel_config_set_dreq(&cfg, spi_get_dreq(spi0, true));
  dma_channel_configure(dmaCh, &cfg, &spi_get_hw(spi0)->dr, frame, gfx::W * gfx::H, true);
  pushing = true;
}

// DMA completion is not SPI completion: wait for the final word before releasing CS or reusing the
// buffer.
static void pushFinish() {
  if (!pushing)
    return;
  dma_channel_wait_for_finish_blocking(dmaCh);
  spi_hw_t *hw = spi_get_hw(spi0);
  while (hw->sr & SPI_SSPSR_BSY_BITS) {
  }
  while (spi_is_readable(spi0))
    (void)hw->dr;
  hw->icr = SPI_SSPICR_RORIC_BITS;
  setFormat(8);
  tft.endWrite();
  pushing = false;
}

static void pushBlocking(const uint16_t *frame) {
  tft.startWrite();
  tft.setAddrWindow(0, 0, gfx::W, gfx::H);
  tft.writePixels(const_cast<uint16_t *>(frame), gfx::W * gfx::H, true, false);
  tft.endWrite();
}

static void sendFrame(const uint16_t *frame) {
  if (useDMA)
    pushStart(frame);
  else
    pushBlocking(frame);
}

static void measureFrame(const uint16_t *frame) {
  uint32_t sum = 0;
  for (int i = 0; i < gfx::W * gfx::H; ++i) {
    uint16_t c = frame[i];
    sum += ((c >> 11) & 31) * 2 + ((c >> 5) & 63) * 3 + (c & 31);
  }
  frameLight = sum * (100.0f / (282.0f * gfx::W * gfx::H));
  lightMin = min(lightMin, frameLight);
  lightMax = max(lightMax, frameLight);
  frameSamples++;
}

static void resetLightStats() {
  lightMin = 100;
  lightMax = 0;
  frameSamples = 0;
}

static void diagnosticInfo() {
  if (haveFrame)
    measureFrame(fbuf[back ^ 1]);
  uint32_t age = U.lastRx ? millis() - U.lastRx : 0;
  Serial.printf(
      "DIAG mode=%s render=%s transfer=%s up_ms=%lu view=%d SPI=%.2fMHz rx_age=%lums bad=%lu light=%.2f%% range=%.2f..%.2f%% samples=%lu\n",
      pictureMode == LIVE   ? "live"
      : pictureMode == HOLD ? "hold"
                            : "repeat",
      dualCore ? "dual" : "single", useDMA ? "dma" : "blocking", (unsigned long)millis(), U.view,
      spi_get_baudrate(spi0) / 1e6, (unsigned long)age, (unsigned long)U.linkErr, frameLight,
      frameSamples ? lightMin : frameLight, frameSamples ? lightMax : frameLight,
      (unsigned long)frameSamples);
  Serial.printf("ANIM style=%s demo=%s idle=%d action=%d\n",
                A.style == 0   ? "calm"
                : A.style == 2 ? "lively"
                               : "normal",
                A.demo ? "on" : "off", A.idle, A.act);
}

// Diagnostics run between frames while core 1 is idle. HOLD leaves the displayed picture untouched.
static void diagnosticCommand(const char *cmd) {
  if (!strcmp(cmd, "help") || !strcmp(cmd, "?")) {
    Serial.println(
        "Beetle: anim calm/anim normal/anim lively, idle next, demo/demo idle/live, info.");
    Serial.println(
        "Diagnostics: single/dual, blocking/dma, hold/repeat. Robot commands still go to ESP32.");
  } else if (!strcmp(cmd, "info")) {
    diagnosticInfo();
    return;
  } else if (!strcmp(cmd, "anim calm") || !strcmp(cmd, "anim normal") ||
             !strcmp(cmd, "anim lively")) {
    A.style = !strcmp(cmd, "anim calm") ? 0 : !strcmp(cmd, "anim lively") ? 2 : 1;
  } else if (!strcmp(cmd, "idle next")) {
    if (!idleAllowed(RS)) {
      Serial.println(
          "Use the face page while standing or sitting, with a healthy live link (or demo).");
      return;
    }
    A.idle = -1;
    A.act = FA_NONE;
    A.nextIdle = U.t;
  } else if (!strcmp(cmd, "demo") || !strcmp(cmd, "demo idle")) {
    int style = A.style;
    A = Personality();
    A.style = style;
    A.demo = true;
    A.demoT0 = U.t;
    A.demoKind = !strcmp(cmd, "demo idle") ? 1 : 0;
    U.prevTrick = TR_NONE;
    U.lastEvt = 0;
    U.prevF2 = 0;
    U.prevState = -1;
    U.tipT0 = U.stumT0 = U.yawnT0 = U.bannerT0 = U.capT0 = -10;
    U.react = EV_NONE;
    U.reactT0 = -10;
    U.bannerTrick = 0;
    for (auto &p : U.p)
      p.type = PT_NONE;
    pictureMode = LIVE;
    Serial.println(
        "Local display showcase: no commands are sent to the ESP32. Send live to return.");
  } else if (!strcmp(cmd, "single") || !strcmp(cmd, "dual")) {
    dualCore = !strcmp(cmd, "dual");
  } else if (!strcmp(cmd, "blocking") || !strcmp(cmd, "dma")) {
    pushFinish();
    useDMA = !strcmp(cmd, "dma");
  } else if (!strcmp(cmd, "hold") || !strcmp(cmd, "repeat")) {
    if (!haveFrame) {
      Serial.println("Wait until the first picture is displayed.");
      return;
    }
    pushFinish();
    frozenFrame = fbuf[back ^ 1];
    pictureMode = !strcmp(cmd, "hold") ? HOLD : REPEAT;
  } else if (!strcmp(cmd, "live")) {
    if (A.demo) {
      int style = A.style;
      A = Personality();
      A.style = style;
      U.prevTrick = TR_NONE;
      U.lastEvt = link.evt;
      U.prevF2 = link.flags2;
      U.tipT0 = U.stumT0 = U.yawnT0 = U.bannerT0 = U.capT0 = -10;
      U.bannerTrick = 0;
      U.prevState = -1;
      U.react = EV_NONE;
      U.reactT0 = -10;
      for (auto &p : U.p)
        p.type = PT_NONE;
    }
    pictureMode = LIVE;
  } else {
    Serial.println("Unknown command. Send help with a newline.");
    return;
  }
  resetLightStats();
  diagnosticInfo();
}

static void pollDiagnostic() {
  static char command[32];
  static unsigned len = 0;
  static bool overflow = false;
  while (Serial.available()) {
    char ch = Serial.read();
    if (ch == '\r')
      continue;
    if (ch == '\n') {
      if (!overflow && len) {
        command[len] = 0;
        diagnosticCommand(command);
      }
      len = 0;
      overflow = false;
    } else if (len < sizeof command - 1)
      command[len++] = ch;
    else
      overflow = true;
  }
}

static int curRot = 1, curMhz = 75, testCount = -1;
static bool curInv = true;

static void applySettings() {
  if (!U.linkOK || A.demo)
    return;
  if (link.rot != curRot && (link.rot == 1 || link.rot == 3)) {
    Serial.printf("SETTING up_ms=%lu rotation=%d\n", (unsigned long)millis(), link.rot);
    curRot = link.rot;
    tft.setRotation(curRot);
  }
  if ((link.inv != 0) != curInv) {
    Serial.printf("SETTING up_ms=%lu inversion=%d\n", (unsigned long)millis(), link.inv);
    curInv = link.inv != 0;
    tft.invertDisplay(curInv);
  }
  if (link.mhz != curMhz && link.mhz > 0 && link.mhz <= 80) {
    Serial.printf("SETTING up_ms=%lu requested_SPI=%dMHz\n", (unsigned long)millis(), link.mhz);
    curMhz = link.mhz;
    tft.setSPISpeed(curMhz * 1000000UL);
  }
  if (testCount < 0)
    testCount = link.test;
  else if (link.test != testCount) {
    Serial.printf("SETTING up_ms=%lu colour_test=%d\n", (unsigned long)millis(), link.test);
    testCount = link.test;
    const uint16_t cols[5] = {ST77XX_RED, ST77XX_GREEN, ST77XX_BLUE, ST77XX_WHITE, ST77XX_BLACK};
    for (int k = 0; k < 5; k++) {
      tft.fillScreen(cols[k]);
      delay(700);
    }
  }
}

static volatile uint32_t c1us = 0;

void setup1() {}

// Draw only the assigned bottom rows; all animation and telemetry updates belong to core 0.
void loop1() {
  uint32_t cmd = rp2040.fifo.pop();
  if (cmd == 1) {
    uint32_t t0 = micros();
    renderFrame();
    c1us = micros() - t0;
    rp2040.fifo.push(2);
  }
}

void setup() {
  Serial.begin(115200);
  Serial1.setRX(LINK_RX);
  Serial1.setFIFOSize(4096);
  Serial1.begin(LINK_BAUD);
  SPI.setSCK(TFT_SCK);
  SPI.setTX(TFT_MOSI);
  tft.init(240, 320);
  tft.setSPISpeed(curMhz * 1000000UL);
  tft.setRotation(curRot);
  tft.invertDisplay(curInv);
  tft.fillScreen(0);
  dmaCh = dma_claim_unused_channel(true);
  uiInit();
  srand(micros());
  bootMs = millis();
  Serial.println("hexapod_face v3.8 ready. Send help on Beetle USB Serial.");
}

// Publish one frame snapshot, render disjoint bands, then wait before swapping buffer ownership.
void loop() {
  static uint32_t lastFrame = 0, frames = 0, fpsT0 = 0, lastTemp = 0;
  static float c0f = 6, c1f = 6;
  pollLink();
  pollDiagnostic();
  uint32_t now = millis();
  if (pictureMode != LIVE) {
    if (pictureMode == REPEAT && now - lastFrame >= 15) {
      pushFinish();
      sendFrame(frozenFrame);
      lastFrame = now;
    }
    if (now - fpsT0 >= 1000) {
      if (Serial)
        diagnosticInfo();
      fpsT0 = now;
    }
    return;
  }
  if (now - lastFrame < 15)
    return;
  float dt = (now - lastFrame) / 1000.0f;
  if (dt > 0.1f)
    dt = 0.1f;
  uint32_t fStart = micros();
  lastFrame = now;

  U.ms = now;
  U.t = now / 1000.0f;
  U.dt = dt;
  U.linkOK = U.lastRx && now - U.lastRx < 1500;
  RS = link;
  smoother.apply(RS, now);
  DS = driveLive;
  DS.valid = DS.fresh(now, RS.upS) && U.linkOK;
  if (A.demo) {
    demoRobot(RS);
    U.linkOK = true;
  }
  beetleUpS = now / 1000.0f;
  if (now - secT0 >= 1000) {
    U.linkHz = (lines - linesAtSec) * 1000.0f / (now - secT0);
    linesAtSec = lines;
    secT0 = now;
  }
  if (now - lastTemp > 2000) {
    lastTemp = now;
    U.tempC = analogReadTemp();
  }
  U.spiMHz = spi_get_baudrate(spi0) / 1e6f;
  uiUpdate(dt);

  uint16_t *fb = fbuf[back];
  gfx::T[0] = {fb, 0, dualCore ? U.split : gfx::H, 0};
  gfx::T[1] = {fb, U.split, gfx::H, 0};
  if (dualCore)
    rp2040.fifo.push(1);
  uint32_t t0 = micros();
  renderFrame();
  uint32_t c0us = micros() - t0;
  pollLink();
  if (dualCore)
    rp2040.fifo.pop();

  pushFinish();
  applySettings();
  sendFrame(fb);
  back ^= 1;
  haveFrame = true;

  c0f += (c0us / 1000.0f - c0f) * 0.2f;
  if (dualCore)
    c1f += (c1us / 1000.0f - c1f) * 0.2f;
  else
    c1f = 0;
  if (dualCore) {
    if (c0us > c1us * 1.06f + 150)
      U.split -= 4;
    else if (c1us > c0us * 1.06f + 150)
      U.split += 4;
  }
  U.split = gfx::clampi(U.split, 24, gfx::H - 24);
  U.c0ms = c0f;
  U.c1ms = c1f;
  frames++;
  U.frameMs += ((micros() - fStart) / 1000.0f - U.frameMs) * 0.1f;
  if (now - fpsT0 >= 1000) {
    U.fps = frames * 1000.0f / (now - fpsT0);
    frames = 0;
    fpsT0 = now;
    if (Serial)
      Serial.printf("fps %.1f  core0 %.1f ms  core1 %.1f ms  split %d  link %.0f/s  bad %lu\n",
                    U.fps, U.c0ms, U.c1ms, U.split, U.linkHz, (unsigned long)U.linkErr);
  }
}
