// Omnix ESP32 motion controller. See ../../README.md for wiring and build settings.
// Body coordinates are x forward, y left, z up; distances are mm and servo angles are degrees.

#include <Wire.h>
#include "esp32-hal-i2c.h"
#include <esp_log.h>
#include "esp_err.h"
#include <Adafruit_PWMServoDriver.h>
#include <BluetoothSerial.h>
#include <Preferences.h>
#include <esp_sleep.h>
#include <esp_timer.h>
#include <math.h>
#include <stdarg.h>
#include <atomic>
#include "trick_poses.h"
#include "walk_control.h"
#include "mpu_math.h"
#include "loop_timing.h"
#include "equation_adapter.h"
#include "model_generated.h"
#include "dynamics_control.h"
#include "gait_math.h"

// Use 64-bit uptime for deadlines that outlive the millis() wrap.
static inline uint64_t ms64() {
  return (uint64_t)esp_timer_get_time() / 1000;
}

// Board connections and battery thresholds. The PCA boards and MPU use separate I2C buses.
const int SDA_PIN = 27, SCL_PIN = 14;
const uint8_t ADDR_LEFT = 0x40, ADDR_RIGHT = 0x60;
const int MPU_SDA = 21, MPU_SCL = 22;
int mpuSda = MPU_SDA, mpuScl = MPU_SCL;
uint8_t MPU_ADDR = 0x68;
const int BAT_PIN = 34;
const int LED_PIN = 2;
const int TOUCH_PIN = 13;
const bool USE_BUTTON = false;

const float BAT_START_MIN = 6.9f;
const float BAT_WARN = 7.0f;
const float BAT_EMPTY = 6.6f;
const float BAT_PRESENT = 5.0f;

const float PARK_H = 37.0f;
const uint32_t SLEEP_AFTER_PARK_MIN = 30;
const uint32_t LOCKED_SLEEP_S = 60;

// Geometry and pulse conversion. PCA channels 0-8 drive three legs; channel 15 is reserved.
const float US_PER_DEG = 1000.0 / 90.0;
const float STAND_CAD_H = 63.7;
const char *JOINT[3] = {"coxa", "femur", "tibia"};

const char *GAIT_NAME[3] = {"tripod", "ripple", "wave"};
// Swing-start phases in physical leg order: FL, ML, RL, FR, MR, RR.
const float GAIT_OFF[3][6] = {{0.0, 0.5, 0.0, 0.5, 0.0, 0.5},
                              {2.0 / 3, 1.0 / 3, 0.0, 1.0 / 6, 5.0 / 6, 0.5},
                              {5.0 / 6, 4.0 / 6, 3.0 / 6, 2.0 / 6, 1.0 / 6, 0.0}};
const float GAIT_DUTY[3] = {0.55, 2.0 / 3, 5.0 / 6};

// Persistent calibration and live motion state.
Adafruit_PWMServoDriver pcaL(ADDR_LEFT, Wire), pcaR(ADDR_RIGHT, Wire);
BluetoothSerial BT;
Preferences prefs;

int8_t sgn[6][3];
int16_t trim[6][3];
bool calibrated = false, sidesSwapped = false;
bool endsSwapped = false;
const char *resetWhy = "?";

enum Mode { MANUAL, GAIT };

Mode mode = MANUAL;
bool servosOn = false;
float feet[6][3], lo[6][2], loZ[6], loVz[6], qCmd[6][3];
bool swinging[6];
uint8_t settled = 0;
float phase = 0;
bool stepping = false;
// Actual body velocity, ramped toward cmdVx/cmdVy/cmdWz: mm/s, mm/s and rad/s.
float vx = 0, vy = 0, wz = 0;
float cmdVx = 0, cmdVy = 0, cmdWz = 0;
const float ACC_LIN = 320, ACC_ROT = 320 / 130.0f;
float periodEff = 1.0;
int pendingGait = -1;
float swStart[6];
float switchSlow = 1, switchHold = 0;
float gainP[6][3], gainN[6][3];

// Saved gait tuning. sagFull is leg deflection at one third of the robot's weight (mm).
float comX = 0, comY = 0;
float sagFull = 0;
float lagMs = 0;
float servoDps = 600;

float strideMax = 70, strideH = -1, strideW = -99, strideCX = -99, strideCY = -99;
float stanceW[6] = {1, 1, 1, 1, 1, 1};
float loadF[6], reach[6] = {66, 66, 66, 66, 66, 66};
float qPrev[6][3];
bool qValid = false;
// Elapsed wall time in seconds; the gait clock may advance more slowly through govScale.
float tickDt = 0.02f;
float govScale = 1, maxRateDps = 0;

float eqEndpointPeakDps = 0, eqCoxaHeat[6] = {};
uint32_t eqRateRaises = 0, eqRateSkipped = 0, eqSampleAt = 0;
float eqCommandRate[6][3] = {}, eqCommandAccel[6][3] = {};
heq::DynamicDemand dynDemand;
uint32_t dynSampleAt = 0;
bool dynEligible = false;
float accBX = 0, accBY = 0;
uint64_t sagTestUntil = 0;
float tdX[6], tdY[6];

// Mass in grams and assumed stall torque in kgf cm. Zero mass disables workspace torque limits.
float massG = 0, stallKgcm = 2.2f;
const float TAU_SOFT = 0.35f, TAU_HARD = 0.55f;
float strideR[6][NDIR];
float legShare[6], tauStand[6][2];
float budLevP = 0, budLevR = 0, budSag = -1, budMass = -1, budStall = -1, budCool = 0;
int budGait = -1;
float govLowT = 0;

const float DUTY_SLOW[3] = {0.65f, 0.72f, 0.86f};
const float TSW_MIN = 0.30f;
bool dutyAdapt = false;
float dutyEff = 0.55f;
float swEnd[6] = {0.45f, 0.45f, 0.45f, 0.45f, 0.45f, 0.45f};
bool autoGait = false;
float autoGaitHold = 0;
float blPlay = 0;
float stabSlow = 1, stabSlowT = 0;

// The loop owns pending requests. Core 0 computes one immutable workspace snapshot at a time.
struct StrideJob {
  float h, w, cx, cy;
  LegLimits L;
  uint32_t seq;
};

StrideJob sJob, sPending;
float sOutMax = 70, sOutR[6][NDIR];
std::atomic<uint32_t> sOutSeq{0};
uint32_t strideSeq = 0;

// Release/acquire publication keeps the loop from reading a half-written workspace result.
std::atomic<bool> strideBusy{false}, strideReady{false};
bool strideAgain = false;
#if defined(ESP32)
TaskHandle_t strideTaskH = nullptr;
#endif
float heatMS[6][2];

// Positive foot-height error means the foot sits too low and takes extra load (mm).
float zErr[6] = {0, 0, 0, 0, 0, 0};
bool supportOK = true, planFeasible = true;
bool packSeen = false;
uint32_t ikFailTicks = 0, overruns = 0, slewHits = 0;
float worstGapMs = 0;
LoopTiming loopTiming;
bool timingResetRequested = false;
float supportBadT = 0, ikBadT = 0;
bool zFix = false;
// Foot-height calibration state: paper release test, followed by the optional scale test.
int zcLeg = -1;
float zcLift = 0;
bool zcUp = false;
float zcExpect = 3, zcMax = 8;
float zcSum[6], zcN[6];

float zsH = 18;
int zsLeg = -1, zsNext = -1, zsPhase = 0;
float zsZ = 0, zsTarget = 0;
float zsGrams[6];
bool zsHave[6];

float sagModel() {
  return sagFull > 0 ? sagFull : 2.0f;
}

const float *zResid() {
  static float r[6];
  for (int i = 0; i < 6; i++)
    r[i] = zFix ? 0 : zErr[i];
  return r;
}

uint64_t heatWarnAt = 0;
char lastMove = 0;

int gait = 0;
float speed = 40, LIFT = 25, PERIOD = 1.0, width = 0;
float targetH = STAND_CAD_H;
float bodyH = STAND_CAD_H;
float DUTY = 0.5;
uint32_t idleMin = 5;

// Parking and battery-lock state. Battery protection remains active independently of walking
// assists.
enum Park { P_NONE, P_LOWERING, P_PARKED };

Park park = P_NONE;
bool batLocked = false;
uint32_t lastActivity = 0, parkedAt = 0, lockedAt = 0, lowerDoneAt = 0;
float heldH = STAND_CAD_H;

// Battery divider: battery+ through 10k to D34, then 4.7k to ground. batcal adjusts the scale.
float batScale = 14.7f / 4.7f, batOffset = 0, batV = 0;
float batVslow = 0;
RestVoltKF batKF;
float loadX = 0;
double batSumV = 0, batSumX = 0;
int batN = 0;

float batRest() {
  return batKF.init ? batKF.vr : batV;
}

int batPct = -1;

// IMU state and mounting calibration. axX/axY/axZ map sensor coordinates into body coordinates.
bool imuOK = false, hasLevel = false, hasFront = false, levelOn = false, tipping = false;
imath::SampleClock imuClock;
imath::GyroQuietWindow imuBiasWindow;
float imuLinearBody[3] = {}, imuLinearUp = 0, imuAccelTrust = 0;
UpFilter upF;
float nesmLast = 0;
uint64_t nesmWarnAt = 0, stumbleWarnAt = 0;
float gyroBias[3] = {0, 0, 0};
float axZ[3] = {0, 0, 1}, axX[3] = {1, 0, 0}, axY[3] = {0, 1, 0};
float roll = 0, pitch = 0, levP = 0, levR = 0;
const float LEVEL_MAX_DZ = 35, FALL_DEG = 35;
wc::Controller walkCtl;
uint64_t walkSampleAt = 0;
int preferredGait = 0;
float walkLiftK[6] = {};

uint64_t ledFlickerUntil = 0;
float touchBase = 0;
bool touchOn = false, holdFired = false;
uint32_t touchDownAt = 0, lastTapEnd = 0, waveStart = 0, waveUntil = 0;
int tapCount = 0;
bool sitting = false;
float sitSavedH = STAND_CAD_H;

// Trick body offsets are layered onto the gait's foot targets.
volatile int trick = TR_NONE;
uint32_t trickT0 = 0, lastTrickEnd = 0;
uint64_t nextFidgetAt = 30000;
TrickPlayer trickPlayer;
float trickAmp = 1;
volatile float pTx = 0, pTy = 0, pTz = 0, pRoll = 0, pPitch = 0, pYaw = 0;
float pFoot[6][3];

enum ScrEvt { EV_NONE, EV_TAP, EV_DOUBLE, EV_TRIPLE, EV_HOLD, EV_BT_ON, EV_BT_OFF, EV_CMD };

extern uint32_t linkDropped;
volatile uint32_t scrEvt = 0;

void screenEvent(int type) {
  scrEvt = (((scrEvt >> 4) + 1) << 4) | (type & 15);
}

bool fidgetOn = true;

void say(const char *fmt, ...) {
  char b[200];
  va_list a;
  va_start(a, fmt);
  vsnprintf(b, sizeof b, fmt, a);
  va_end(a);
  Serial.println(b);
  if (BT.hasClient())
    BT.println(b);
}

const float PCA_OSC = 27000000.0f;
float pcaTickUs[2] = {4.889f, 4.889f};
float pcaOsc[2] = {PCA_OSC, PCA_OSC};

// Optional channel-15 signal inputs: left PCA to D35, right PCA to D4.
const int SYNC_PIN[2] = {35, 4};
const int SYNC_CH = 15;
volatile uint32_t syncEdges[2] = {0, 0}, syncLastUs[2] = {0, 0};
volatile bool frameFlag = false;
bool syncOK = false;
float framePeriodS = 0.02f;

volatile uint32_t syncMinI[2] = {0xFFFFFFFF, 0xFFFFFFFF}, syncMaxI[2] = {0, 0},
                  syncNoise[2] = {0, 0};

static inline void IRAM_ATTR syncEdge(int b) {
  uint32_t t = micros(), iv = t - syncLastUs[b];
  if (syncEdges[b] && iv < 12000) {
    syncNoise[b]++;
    return;
  }
  if (syncEdges[b]) {
    if (iv < syncMinI[b])
      syncMinI[b] = iv;
    if (iv > syncMaxI[b])
      syncMaxI[b] = iv;
  }
  syncLastUs[b] = t;
  syncEdges[b]++;
  if (b == 0)
    frameFlag = true;
}

void IRAM_ATTR syncIsrL() {
  syncEdge(0);
}

void IRAM_ATTR syncIsrR() {
  syncEdge(1);
}

bool calTrue[2] = {false, false};
float pcaOscSaved[2] = {0, 0};
float pcaOscMeas[2] = {0, 0};
uint32_t syncRate[2] = {0, 0}, syncSpread[2] = {0, 0}, syncNoiseLast[2] = {0, 0};

int boardOfLeg(int leg) {
  return ((leg < 3) != sidesSwapped) ? 0 : 1;
}

uint16_t pulseUs[6][3];
bool batching = false;

bool servoHold = false, servoSettling = false, servoLinksReady = false;
bool centerActive = false;
int centerJoint = 0;
uint32_t centerNextAt = 0;
uint64_t servoRetryAt = 0, servoGoodAt = 0, servoLastPairAt = 0, servoFaultAt = 0;
float servoResumeT = 0;
int servoFaultBoard = -1;
uint8_t servoFaultCode = 0;
char servoFaultMove = 0;

static const uint8_t CH_ONH[9] = {0, 1, 3, 5, 6, 8, 10, 12, 13};
uint8_t chOnL[2][9] = {{128, 128, 128, 128, 128, 128, 128, 128, 128},
                       {128, 128, 128, 128, 128, 128, 128, 128, 128}};
uint8_t chOffL[2][9] = {{128, 128, 128, 128, 128, 128, 128, 128, 128},
                        {128, 128, 128, 128, 128, 128, 128, 128, 128}};

void chanBytes(int b, int ch, uint16_t us, uint8_t out[4]) {
  int onl = chOnL[b][ch], offl = chOffL[b][ch];
  if (us) {
    int t = constrain((int)lroundf(us / pcaTickUs[b]), 1, 511);
    int d = t - (256 + offl - onl);
    if (d > 0) {
      int a = min(d, 255 - offl);
      offl += a;
      onl -= d - a;
    } else if (d < 0) {
      int a = min(-d, offl);
      offl -= a;
      onl += -d - a;
    }
    chOnL[b][ch] = (uint8_t)onl;
    chOffL[b][ch] = (uint8_t)offl;
  }
  out[0] = (uint8_t)onl;
  out[1] = CH_ONH[ch];
  out[2] = (uint8_t)offl;
  out[3] = (uint8_t)((CH_ONH[ch] + 1) | (us ? 0 : 0x10));
}

uint8_t pcaMode1[2] = {}, pcaMode2[2] = {}, pcaPrescale[2] = {};

uint8_t pcaReadByte(int b, uint8_t reg, uint8_t &value) {
  uint8_t ad = b == 0 ? ADDR_LEFT : ADDR_RIGHT;
  Wire.beginTransmission(ad);
  Wire.write(reg);
  uint8_t err = Wire.endTransmission();
  if (err)
    return err;
  if (Wire.requestFrom(ad, (uint8_t)1) != 1)
    return 6;
  value = Wire.read();
  return 0;
}

uint8_t pcaCheckConfig(int b) {
  uint8_t err = pcaReadByte(b, 0x00, pcaMode1[b]);
  if (err)
    return err;
  err = pcaReadByte(b, 0x01, pcaMode2[b]);
  if (err)
    return err;
  err = pcaReadByte(b, 0xFE, pcaPrescale[b]);
  if (err)
    return err;
  return (pcaMode1[b] & 0x70) == 0x20 && (pcaMode2[b] & 0x1F) == 0x04 && pcaPrescale[b] == 131 ? 0
                                                                                               : 7;
}

uint8_t pcaWriteBoard(int board, int leg0) {
  bool allOff = true;
  for (int i = 0; i < 3; i++)
    for (int j = 0; j < 3; j++)
      allOff &= pulseUs[leg0 + i][j] == 0;
  if (allOff) {
    for (int ch = 0; ch < 9; ch++) {
      Wire.beginTransmission(board == 0 ? ADDR_LEFT : ADDR_RIGHT);
      Wire.write(0x09 + 4 * ch);
      Wire.write((uint8_t)((CH_ONH[ch] + 1) | 0x10));
      uint8_t r = Wire.endTransmission();
      if (r)
        return r;
    }
    return 0;
  }
  Wire.beginTransmission(board == 0 ? ADDR_LEFT : ADDR_RIGHT);
  Wire.write(0x06);
  for (int ch = 0; ch < 9; ch++) {
    uint8_t c4[4];
    chanBytes(board, ch, pulseUs[leg0 + ch / 3][ch % 3], c4);
    Wire.write(c4, 4);
  }
  return Wire.endTransmission();
}

uint32_t boardFails[2] = {0, 0}, boardFailRun[2] = {0, 0};
bool boardDown[2] = {false, false};

// Latch a hold on the first failed PCA write so further commands cannot separate the two sides.
void boardResult(int b, uint8_t r) {
  const char *nm = b == 0 ? "left (0x40)" : "right (0x60)";
  if (!r) {
    boardFailRun[b] = 0;
    boardDown[b] = false;
    return;
  }
  boardFails[b]++;
  boardFailRun[b]++;
  boardDown[b] = true;
  servoGoodAt = 0;
  servoLinksReady = false;
  if (servoHold)
    return;
  servoHold = true;
  servoSettling = false;
  servoResumeT = 0;
  servoRetryAt = 0;
  centerActive = false;
  servoFaultAt = ms64();
  servoFaultBoard = b;
  servoFaultCode = r;
  servoFaultMove = lastMove;
  stopNow();
  walkCtl.reason = wc::FAULT;
  accBX = accBY = 0;

  trickStop();
  say("Servo board %s: error %u (Wire 2/4/5; 6=short read, 7=config changed). Output HOLD latched; gait and leveling frozen.",
      nm, r);
  say("Fault snapshot: cmd %c, height %.1f/%.1f mm, stride %.1f mm, tilt P%+.1f R%+.1f, level P%+.1f R%+.1f deg, tick %.0f ms",
      servoFaultMove ? servoFaultMove : '-', bodyH, targetH, strideMax, pitch, roll,
      levP * RAD_TO_DEG, levR * RAD_TO_DEG, tickDt * 1000);
  say("Use servostatus. Reconnection never resumes movement; servorecover releases the hold, off relaxes.");
}

void servoFlush() {
  int b0 = sidesSwapped ? 1 : 0, b1 = 1 - b0;
  uint8_t r0 = servoHold ? pcaCheckConfig(b0) : 0;
  if (!r0 || !servosOn) {
    uint8_t w = pcaWriteBoard(b0, 0);
    if (w || !r0)
      r0 = w;
  }
  boardResult(b0, r0);
  uint8_t r1 = servoHold ? pcaCheckConfig(b1) : 0;
  if (!r1 || !servosOn) {
    uint8_t w = pcaWriteBoard(b1, 3);
    if (w || !r1)
      r1 = w;
  }
  boardResult(b1, r1);
  if (servoHold) {
    servoLastPairAt = ms64();
    if (r0 || r1) {
      servoGoodAt = 0;
      servoLinksReady = false;
    } else {
      if (!servoGoodAt)
        servoGoodAt = servoLastPairAt;
      if (!servoLinksReady && servoLastPairAt - servoGoodAt >= 1000) {
        servoLinksReady = true;
        say("Both servo boards accepted frozen PWM targets for 1 s. Hold remains; use servorecover when upright.");
      }
    }
  }
}

void servoHoldService() {
  if (!servoHold || ms64() < servoRetryAt)
    return;
  servoRetryAt = ms64() + 100;
  servoFlush();
}

void servoStatus() {
  say("Servo output: %s | link L %s R %s | failed writes L %lu R %lu",
      servoHold       ? "HOLD"
      : servoSettling ? "settling after hold"
                      : "running",
      boardDown[0] ? "failed" : "last write OK", boardDown[1] ? "failed" : "last write OK",
      (unsigned long)boardFails[0], (unsigned long)boardFails[1]);
  say("PCA bus: SDA D%d=%d SCL D%d=%d, 100000 Hz; hold release ready %s (1 s of complete acknowledged writes).",
      SDA_PIN, digitalRead(SDA_PIN), SCL_PIN, digitalRead(SCL_PIN), servoLinksReady ? "yes" : "no");
  if (servoFaultBoard >= 0)
    say("Last hold: %s board, error %u, cmd %c, %lu ms ago. 4=other driver error, 6=short read, 7=config changed; the cause is unconfirmed.",
        servoFaultBoard == 0 ? "left" : "right", servoFaultCode,
        servoFaultMove ? servoFaultMove : '-', (unsigned long)(ms64() - servoFaultAt));
  if (servoHold && servosOn)
    for (int b = 0; b < 2; b++)
      say("PCA %s config: MODE1 0x%02X MODE2 0x%02X prescale %u (want AI/awake/internal clock, output 0x04, prescale 131).",
          b == 0 ? "left" : "right", pcaMode1[b], pcaMode2[b], pcaPrescale[b]);
}

// Recovery requires an explicit command; a returning board does not resume old motion.
void servoRecover() {
  if (!servoHold) {
    servoStatus();
    return;
  }
  if (batLocked) {
    say("Battery locked: charge / swap it first.");
    return;
  }
  if (!servoLinksReady || ms64() - servoLastPairAt > 150) {
    say("Servo hold: need 1 s of complete writes to BOTH boards. Check servostatus.");
    return;
  }
  if (hasLevel && hasFront &&
      (!walkFresh() || fabsf(pitch) >= 12 || fabsf(roll) >= 12 || fabsf(walkCtl.pitchRate) >= 8 ||
       fabsf(walkCtl.rollRate) >= 8)) {
    say("Servo hold: set the robot upright/still and wait for fresh MPU data, then servorecover.");
    return;
  }
  stopNow();
  servoHold = false;
  servoLinksReady = false;
  servoGoodAt = 0;
  servoSettling = mode == GAIT && servosOn;
  servoResumeT = servoSettling ? 2.0f : 0;

  pendingGait = -1;
  govLowT = ikBadT = supportBadT = 0;
  accBX = accBY = 0;
  say(servoSettling
          ? "Servo hold released: finishing raised feet at limited joint speed. Send a new movement command after settling."
          : "Servo hold released. Servos stay off until a new command.");
}

void writeUs(int leg, int j, int us) {
  if (servoHold)
    return;
  pulseUs[leg][j] = constrain(us, 600, 2400);
  if (!batching) {
    int b = boardOfLeg(leg), ch = (leg % 3) * 3 + j;
    uint8_t c4[4];
    chanBytes(b, ch, pulseUs[leg][j], c4);
    Wire.beginTransmission(b == 0 ? ADDR_LEFT : ADDR_RIGHT);
    Wire.write(0x06 + 4 * ch);
    Wire.write(c4, 4);
    boardResult(b, Wire.endTransmission());
  }
}

void writeAngle(int leg, int j, float deg) {
  if (servoHold)
    return;
  if (!isfinite(deg))
    return;
  if (batLocked && park != P_LOWERING)
    return;
  float g = (deg >= 0 ? gainP : gainN)[leg][j];
  if (!isfinite(g) || g < 0.5f || g > 2.0f)
    g = 1.0f;
  writeUs(leg, j, 1500 + trim[leg][j] + (int)lroundf(sgn[leg][j] * deg * US_PER_DEG * g));
  servosOn = true;
}

void relaxAll() {
  memset(pulseUs, 0, sizeof pulseUs);
  qValid = false;
  for (int b = 0; b < 2; b++) {
    for (int k = 0; k < 3; k++) {
      uint8_t r = pcaWriteBoard(b, 0);
      boardResult(b, r);
      if (!r)
        break;
    }
  }
  servosOn = false;
}

void neutral(int i, float &nx, float &ny) {
  neutralFoot(i, width, comX, comY, nx, ny);
}

// Translate calibrated pulse limits back to usable joint angles.
void servoRange(int i, int j, float &lo, float &hi) {
  float room = sgn[i][j] > 0 ? 2400 - 1500 - trim[i][j] : 1500 + trim[i][j] - 600;
  float roomN = sgn[i][j] > 0 ? 1500 + trim[i][j] - 600 : 2400 - 1500 - trim[i][j];
  hi = fminf(JOINT_MAX[j], fmaxf(0.0f, room) / (US_PER_DEG * gainP[i][j]));
  lo = fmaxf(JOINT_MIN[j], -fmaxf(0.0f, roomN) / (US_PER_DEG * gainN[i][j]));
}

// Reject unreachable targets before applying calibrated final joint limits.
bool legIK(int i, float fx, float fy, float fz, float q[3], float *r = nullptr) {
  if (!ikRaw(i, fx, fy, fz, q, r))
    return false;
  for (int j = 0; j < 3; j++) {
    float lo, hi;
    servoRange(i, j, lo, hi);
    q[j] = constrain(q[j], lo, hi);
  }
  return true;
}

void standLoads(const float e[6], int j, float z, float F[6]) {
  float fx[6], fy[6], kn[6], ee[6], sg = sagModel();
  for (int i = 0; i < 6; i++) {
    fx[i] = feet[i][0];
    fy[i] = feet[i][1];
    ee[i] = e[i] - (i == j ? z : 0);
    kn[i] = legStiffRel(reach[i], qValid ? qPrev[i][1] : 0) / (3 * sg);
  }
  loadSplitK(fx, fy, kn, ee, comX, comY, F);
}

void legLimits(LegLimits &L, float h) {
  planFeasible =
      peakShares(GAIT_OFF[gait], DUTY, width, comX, comY, legShare, h, zResid(), sagModel());
  float stall = stallKgcm * 98.0665f, W = massG * 0.00981f, worst = 0;
  for (int i = 0; i < 6; i++) {
    float nx, ny;
    neutral(i, nx, ny);
    float lz = constrain(nx * tanf(levP) + ny * tanf(levR), -LEVEL_MAX_DZ, LEVEL_MAX_DZ);
    float q[3], r, lever2 = 1;
    if (ikRaw(i, nx, ny, -h + lz, q, &r)) {
      float lt = r - FEMUR * cosf(q[1] * DEG_TO_RAD);
      lever2 = fmaxf(r * r + lt * lt, 400.0f) / (FEMUR * FEMUR);
    }
    L.zOff[i] = -(sagFull > 0 ? sagFull * 3 * legShare[i] * lever2 : 0) + (zFix ? zErr[i] : 0);
    L.zAir[i] = (zFix ? zErr[i] : 0) + blPlay;
    L.Fn[i] = W * legShare[i];
    tauStand[i][0] = tauStand[i][1] = 0;
    if (massG > 0 && ikRaw(i, nx, ny, -h + lz + L.zOff[i], q, &r))
      torqueFromIK(L.Fn[i], r, q[1], tauStand[i][0], tauStand[i][1]);
    worst = fmaxf(worst, fmaxf(tauStand[i][0], tauStand[i][1]));
  }

  for (int i = 0; i < 6; i++)
    L.tauLim[i] = fmaxf(TAU_HARD * stall, worst + (TAU_HARD - TAU_SOFT) * stall);
  L.tanP = tanf(levP);
  L.tanR = tanf(levR);
  L.lift = LIFT;
  L.hasQ = true;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++)
      servoRange(i, j, L.qLo[i][j], L.qHi[i][j]);
}

void strideCompute(float h, float w, float cx, float cy, const LegLimits &L, float &sm,
                   float R[6][NDIR]);
void strideApply(float sm, const float R[6][NDIR]);

bool workspaceCalibrationChanged() {

  static bool seen = false, fix = false;
  static float lift = 0, play = 0, ze[6], gp[6][3], gn[6][3];
  static int8_t dir[6][3];
  static int16_t tr[6][3];
  bool changed = !seen || fix != zFix || lift != LIFT || play != blPlay ||
                 memcmp(ze, zErr, sizeof ze) || memcmp(dir, sgn, sizeof dir) ||
                 memcmp(tr, trim, sizeof tr) || memcmp(gp, gainP, sizeof gp) ||
                 memcmp(gn, gainN, sizeof gn);
  if (changed) {
    seen = true;
    fix = zFix;
    lift = LIFT;
    play = blPlay;
    memcpy(ze, zErr, sizeof ze);
    memcpy(dir, sgn, sizeof dir);
    memcpy(tr, trim, sizeof tr);
    memcpy(gp, gainP, sizeof gp);
    memcpy(gn, gainN, sizeof gn);
  }
  return changed;
}

// Recompute workspace only when its geometry or calibration changes.
void updateStride(bool force = false) {
  if (workspaceCalibrationChanged())
    force = true;
  bool tilt = budCool <= 0 && (fabsf(levP - budLevP) > 0.0175f || fabsf(levR - budLevR) > 0.0175f);
  if (!force && !tilt && fabsf(walkHeight() - strideH) < 2 && width == strideW &&
      comX == strideCX && comY == strideCY && gait == budGait && sagFull == budSag &&
      massG == budMass && stallKgcm == budStall)
    return;
  strideH = walkHeight();
  strideW = width;
  strideCX = comX;
  strideCY = comY;
  budGait = gait;
  budSag = sagFull;
  budMass = massG;
  budStall = stallKgcm;
  budLevP = levP;
  budLevR = levR;
  budCool = 0.5f;
  float h = constrain(walkHeight(), 40.0f, 85.0f);
  LegLimits L;
  legLimits(L, h);
  strideSeq++;
#if defined(ESP32)
  if (!force && strideTaskH) {
    if (!strideBusy.load(std::memory_order_acquire) &&
        !strideReady.load(std::memory_order_acquire)) {
      sJob.h = h;
      sJob.w = width;
      sJob.cx = comX;
      sJob.cy = comY;
      sJob.L = L;
      sJob.seq = strideSeq;
      strideAgain = false;
      strideBusy.store(true, std::memory_order_release);
      xTaskNotifyGive(strideTaskH);
    } else {
      sPending.h = h;
      sPending.w = width;
      sPending.cx = comX;
      sPending.cy = comY;
      sPending.L = L;
      sPending.seq = strideSeq;
      strideAgain = true;
    }
    return;
  }
#endif
  strideAgain = false;
  float sm, R[6][NDIR];
  strideCompute(h, width, comX, comY, L, sm, R);
  strideApply(sm, R);
}

void strideCompute(float h, float w, float cx, float cy, const LegLimits &L, float &sm,
                   float R[6][NDIR]) {
  sm = constrain(strideBudget(h, w, cx, cy, 7, 6, &L) - 32, 0.0f, 70.0f);
  strideDirs(h, w, cx, cy, &L, sm / 2, R);
}

void strideApply(float sm, const float R[6][NDIR]) {
  strideMax = sm;
  memcpy(strideR, R, sizeof strideR);
  static bool warned = false;
  if (strideMax < 10 && !warned)
    say("This height / width leaves almost no safe stride (%.0f mm): walking disabled. Change height or width.",
        strideMax);
  warned = strideMax < 10;
}
#if defined(ESP32)
// Compute off the control core; publish the result only after all workspace bins are ready.
void strideTaskFn(void *) {
  for (;;) {
    ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    strideCompute(sJob.h, sJob.w, sJob.cx, sJob.cy, sJob.L, sOutMax, sOutR);
    sOutSeq.store(sJob.seq, std::memory_order_relaxed);
    strideReady.store(true, std::memory_order_release);
    strideBusy.store(false, std::memory_order_release);
  }
}

void strideCollect() {
  if (strideReady.load(std::memory_order_acquire)) {
    if (sOutSeq.load(std::memory_order_relaxed) == strideSeq)
      strideApply(sOutMax, sOutR);
    strideReady.store(false, std::memory_order_release);
  }

  if (strideAgain && !strideBusy.load(std::memory_order_acquire) &&
      !strideReady.load(std::memory_order_acquire)) {
    strideAgain = false;
    if (sPending.seq == strideSeq) {
      sJob = sPending;
      strideBusy.store(true, std::memory_order_release);
      xTaskNotifyGive(strideTaskH);
    }
  }
}
#else
void strideCollect() {}
#endif

float sagTestLift() {
  uint64_t now = ms64();
  if (stepping || now >= sagTestUntil)
    return 0;
  float left = constrain((sagTestUntil - now) * .001f, 0.0f, 4.0f);
  return 15 * minjerk(left / .5f) * minjerk((4 - left) / .5f);
}

bool poseFootRaised(int i, float sagLift) {
  return pFoot[i][2] > 1 || (sagLift > 1 && (i == 0 || i == 2 || i == 4)) ||
         (i == zsLeg && zsZ > zsH + 2);
}

// Solve feet, share contact loads, apply joint guards, then flush both PCA boards.
// Dynamics observes the final commanded pose after output; it does not command servo torque.
void applyFeet() {
  if (servoHold) {
    equationDynamicsInvalidate();
    return;
  }
  bool hadJointSample = qValid;
  uint32_t now = millis();
  trickUpdate(now);
  PoseFrame poseFrame(pRoll, pPitch, pYaw);

  float sagLift = 0;
  float calDt = constrain(tickDt, 0.005f, 0.1f);
  if (zcLeg >= 0) {
    if (zcUp && (stepping || trick != TR_NONE)) {
      zcUp = false;
      say("zcal: cancelled (moving).");
    }
    if (zcUp) {
      zcLift += (zcLift < 0.5f * zcExpect ? 2.0f : 0.5f) * calDt;
      if (zcLift > zcMax) {
        zcUp = false;
        say("zcal: no release by %.0f mm (expected ~%.1f). Paper right under the foot tip? Is the foot visibly rising? Lowering.",
            zcMax, zcExpect);
      }
    } else {
      zcLift -= 5.0f * calDt;
      if (zcLift <= 0) {
        zcLift = 0;
        zcLeg = -1;
      }
    }
  }
  if (zsLeg >= 0) {
    zsZ += constrain(zsTarget - zsZ, -10.0f * calDt, 10.0f * calDt);
    if (zsTarget == 0 && fabsf(zsZ) < 0.01f) {
      zsZ = 0;
      zsLeg = -1;
      zsPhase = 0;
      if (zsNext >= 0) {
        zsLeg = zsNext;
        zsNext = -1;
        zsPhase = 1;
        zsTarget = zsH + 12;
      }
    }
    if (stepping || trick != TR_NONE) {
      zsTarget = 0;
      zsNext = -1;
    }
  }
  sagLift = sagTestLift();

  float lx[6], ly[6], w[6];
  for (int i = 0; i < 6; i++) {
    lx[i] = feet[i][0] + pFoot[i][0];
    ly[i] = feet[i][1] + pFoot[i][1];
    bool lifted = poseFootRaised(i, sagLift);
    w[i] = lifted ? 0 : stanceW[i];
  }
  {
    float kn[6], sg = sagModel();
    for (int i = 0; i < 6; i++)
      kn[i] = w[i] * legStiffRel(reach[i], qValid ? qPrev[i][1] : 0) / (3 * sg);

    float levelBodyUp[3];
    poseFrame.toBody(0, 0, 1, levelBodyUp[0], levelBodyUp[1], levelBodyUp[2]);
    const float *bodyUp = imuOK && hasLevel && hasFront && upF.init ? upF.u : levelBodyUp;
    SupportFrame frame(poseFrame, pTx, pTy, pTz, comX, comY, 20.0f, bodyUp);
    float px = frame.cx, py = frame.cy;
    bool tiltOK = frame.projection(-bodyH, px, py);
    float gx = constrain(px - frame.cx, -25.0f, 25.0f),
          gy = constrain(py - frame.cy, -25.0f, 25.0f);
    bool clampedOK = loadSplitK(lx, ly, kn, zResid(), frame.cx + gx, frame.cy + gy, loadF);
    if (frame.cx + gx == px && frame.cy + gy == py)
      supportOK = tiltOK && clampedOK;
    else {
      float Ft[6];
      supportOK = tiltOK && loadSplitK(lx, ly, kn, zResid(), px, py, Ft);
    }
  }
  float q[6][3], footZ[6];
  bool ok[6];
  for (int i = 0; i < 6; i++) {

    float wx = feet[i][0] + pFoot[i][0] - pTx, wy = feet[i][1] + pFoot[i][1] - pTy,
          wz = feet[i][2] + pFoot[i][2] - pTz;
    if (sagLift > 0 && (i == 0 || i == 2 || i == 4))
      wz += sagLift;
    if (i == zcLeg)
      wz += zcLift;
    if (i == zsLeg)
      wz += zsZ;
    if (zFix)
      wz += zErr[i];
    if (blPlay > 0)
      wz += blPlay * (1 - w[i]);
    float fx, fy, fz;
    poseFrame.toBody(wx, wy, wz, fx, fy, fz);

    fz += constrain(fx * tanf(levP) + fy * tanf(levR), -LEVEL_MAX_DZ, LEVEL_MAX_DZ);

    if (sagFull > 0) {
      float lt = reach[i] - FEMUR * cosf((qValid ? qPrev[i][1] : 0) * DEG_TO_RAD);
      fz -=
          sagFull * (loadF[i] * 3) * fmaxf(reach[i] * reach[i] + lt * lt, 400.0f) / (FEMUR * FEMUR);
    }
    ok[i] = legIK(i, fx, fy, fz, q[i], &reach[i]);
    footZ[i] = fz - COXA_Z - HIP_UP;
    if (!ok[i])
      ikFailTicks++;
  }
  {
    bool anyIK = false;
    for (int i = 0; i < 6; i++)
      anyIK |= !ok[i];
    float dtr = constrain(tickDt, 0.005f, 0.1f);
    ikBadT = (anyIK && stepping) ? ikBadT + dtr : 0;
    supportBadT = (!supportOK && stepping) ? supportBadT + dtr : 0;
    if (ikBadT > 0.2f) {
      ikBadT = 0;
      walkCtl.cancel(wc::FAULT);
      stopMoving();
      say("A foot target is out of reach (IK) while walking: stopping.");
    }
    if (supportBadT > 0.5f) {
      supportBadT = 0;
      walkCtl.cancel(wc::FAULT);
      stopMoving();
      say("No static support for 0.5 s (CoM outside the feet): stopping.");
    }
  }

  float xLoad = servosOn ? 0.5f : 0;
  float dt = constrain(tickDt, 0.005f, 0.1f), worst = 0;
  {
    float W = (massG > 0 ? massG : 800.0f) * 0.00981f, stall = stallKgcm * 98.0665f;
    for (int i = 0; i < 6; i++) {
      float tc = 0, tf = 0, tt = 0;
      if (ok[i]) {
        const float levelUp[3] = {0, 0, 1};
        const float *u = imuOK && hasLevel && hasFront && upF.init ? upF.u : levelUp;
        float nm[3];
        if (heq::firmwareReactionTorque(i, q[i], W * loadF[i], u, nm)) {
          tc = fabsf(nm[0]) * 1000;
          tf = fabsf(nm[1]) * 1000;
          tt = fabsf(nm[2]) * 1000;
        }
      }
      if (massG > 0) {
        heatStep(eqCoxaHeat[i], tc / stall, dt);
        heatStep(heatMS[i][0], tf / stall, dt);
        heatStep(heatMS[i][1], tt / stall, dt);
      }
      if (servosOn)
        xLoad += (tc + tf + tt) / stall;
    }
  }

  batching = true;
  eqEndpointPeakDps = 0;
  for (int i = 0; i < 6; i++) {
    if (!ok[i]) {
      for (int j = 0; j < 3; j++)
        eqCommandRate[i][j] = eqCommandAccel[i][j] = 0;
      continue;
    }
    if (qValid && !servoSettling) {
      float predicted;
      if (heq::firmwareEndpointRate(i, qPrev[i], q[i], tickDt, predicted)) {
        eqEndpointPeakDps = fmaxf(eqEndpointPeakDps, predicted);
        float measured = 0;
        for (int j = 0; j < 3; j++)
          measured = fmaxf(measured, fabsf(q[i][j] - qPrev[i][j]) / dt);
        if (predicted > measured + 1)
          eqRateRaises++;
        worst = fmaxf(worst, predicted);
      } else
        eqRateSkipped++;
    }
    for (int j = 0; j < 3; j++) {
      float rate = qValid ? (q[i][j] - qPrev[i][j]) / dt : 0;
      worst = fmaxf(worst, fabsf(rate));

      if (servosOn)
        xLoad += 0.3f * fabsf(rate) / servoDps;
      float lead = constrain(rate * lagMs / 1000.0f, -6.0f, 6.0f);
      qPrev[i][j] = q[i][j];
      float cmdLo, cmdHi;
      servoRange(i, j, cmdLo, cmdHi);
      float cmd = constrain(q[i][j] + lead, cmdLo, cmdHi);
      if (qValid) {
        float mx = (servoSettling ? fminf(60.0f, 1.5f * servoDps) : 1.5f * servoDps) * dt;
        float dc = constrain(cmd - qCmd[i][j], -mx, mx);
        if (dc != cmd - qCmd[i][j])
          slewHits++;
        cmd = qCmd[i][j] + dc;
      }

      float actualRate =
          qValid && tickDt >= .005f && tickDt <= .060f ? (cmd - qCmd[i][j]) / tickDt : 0;
      float alpha = 1 - expf(-dt / .12f);
      eqCommandAccel[i][j] +=
          alpha * ((actualRate - eqCommandRate[i][j]) / dt - eqCommandAccel[i][j]);
      eqCommandRate[i][j] = actualRate;
      qCmd[i][j] = cmd;
      writeAngle(i, j, cmd);
    }
  }
  batching = false;
  qValid = true;
  eqSampleAt = now;
  servoFlush();
  loadX = xLoad;
  if (servoHold) {
    equationDynamicsInvalidate();
    return;
  }
  equationDynamicsUpdate(hadJointSample, ok);
  maxRateDps = worst;
  if (servoSettling) {
    servoResumeT = fmaxf(0, servoResumeT - dt);
    float err = 0;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        if (ok[i])
          err = fmaxf(err, fabsf(qCmd[i][j] - q[i][j]));
    if (!stepping && servoResumeT <= 0 && err < 1) {
      servoSettling = false;
      say("Servo recovery settled. Waiting for a new command.");
    }
  }

  float want = worst > 1 ? fminf(1.0f, govScale * 0.7f * servoDps / worst) : 1.0f;
  want = fminf(want, dynDemand.clockCeiling(dynEligible));
  govScale += (want - govScale) * (want < govScale ? 0.5f : 0.05f);
  govScale = fmaxf(govScale, 0.02f);
  govLowT = (stepping && govScale < 0.15f) ? govLowT + dt : 0;
  if (govLowT > 1.0f) {
    govLowT = 0;
    walkCtl.cancel(wc::FAULT);
    stopMoving();
    say("Servos can't keep up (governor under 15%%): stopped. Slower speed, or check servospeed.");
  }
}

float maxSpeed() {
  return strideMax / (DUTY * fmaxf(PERIOD, TSW_MIN / (1 - DUTY)));
}

void startGait() {
  if (mode == GAIT)
    return;
  for (int i = 0; i < 6; i++) {
    neutral(i, feet[i][0], feet[i][1]);
    feet[i][2] = -bodyH;
    swinging[i] = false;
    stanceW[i] = 1;
  }
  stepping = false;
  settled = 0;
  phase = 0;
  qValid = false;
  dutyEff = DUTY;
  for (int i = 0; i < 6; i++)
    swEnd[i] = 1 - DUTY;
  mode = GAIT;
}

float bestPhase(int g, float duty) {
  float sw = 1 - duty, best = 0, bestCost = 1e9f;
  for (int k = 0; k < 50; k++) {
    float ph = k / 50.0f, cost = -1e9f;
    for (int i = 0; i < 6; i++) {
      float psi = fmodf(ph - GAIT_OFF[g][i] + 1.0f, 1.0f);
      if (psi < 0.45f * sw)
        continue;
      float nx, ny;
      neutral(i, nx, ny);
      float ux = cmdVx - cmdWz * ny, uy = cmdVy + cmdWz * nx, us = sqrtf(ux * ux + uy * uy);
      if (us < 1)
        continue;
      float used = ((nx - feet[i][0]) * ux + (ny - feet[i][1]) * uy) / us;
      cost = fmaxf(cost, used + (1 - psi) * periodEff * us - strideMax / 2);
    }
    if (cost < bestCost - 0.01f) {
      bestCost = cost;
      best = ph;
    }
  }
  return best;
}

void applyGait(int k) {
  gait = k;
  DUTY = GAIT_DUTY[k];
  pendingGait = -1;
  dutyEff = DUTY;
  for (int i = 0; i < 6; i++)
    swEnd[i] = 1 - DUTY;
  phase = bestPhase(k, DUTY);
  for (int i = 0; i < 6; i++)
    swStart[i] = 0;
  switchHold = periodEff;
  applyMove();
  say(autoGait ? "Gait: %s (auto)" : "Gait: %s", GAIT_NAME[k]);
}

// Use slowed gait time for trajectories and real elapsed time for joint-rate limits.
void gaitUpdate(float dtReal) {
  if (servoHold)
    return;
  float dt = dtReal * govScale;
  budCool -= dtReal;
  strideCollect();
  float dBH = (servoSettling || trick != TR_NONE)
                  ? 0
                  : constrain(walkHeight() - bodyH, -20.0f * dtReal, 20.0f * dtReal);
  bodyH += dBH;
  float hDot = dBH / fmaxf(dtReal, 1e-3f);
  updateStride();

  if (switchHold > 0)
    switchHold -= dt;
  switchSlow += constrain((pendingGait >= 0 || switchHold > 0 ? 0.45f : 1.0f) - switchSlow,
                          -1.5f * dt, 0.6f * dt);

  if (stabSlowT > 0)
    stabSlowT -= dtReal;
  else
    stabSlow += (1 - stabSlow) * fminf(1.0f, dtReal / 1.0f);
  walkGait(dtReal);
  float tvx = cmdVx, tvy = cmdVy, twz = cmdWz;
  walkVelocity(tvx, tvy, twz);
  if (trick != TR_NONE)
    tvx = tvy = twz = 0;
  tvx *= switchSlow * stabSlow;
  tvy *= switchSlow * stabSlow;
  twz *= switchSlow * stabSlow;
  float activeStride = strideMax * walkCtl.strokeScale;

  {
    float kt = footSpeedScale(tvx, tvy, twz, dutyEff * periodEff, activeStride, width, comX, comY,
                              strideR);
    tvx *= kt;
    tvy *= kt;
    twz *= kt;
  }
  static float pvx = 0, pvy = 0;
  vx += constrain(tvx - vx, -ACC_LIN * dt, ACC_LIN * dt);
  vy += constrain(tvy - vy, -ACC_LIN * dt, ACC_LIN * dt);
  wz += constrain(twz - wz, -ACC_ROT * dt, ACC_ROT * dt);

  float vmax = fmaxf(maxSpeed(), 1.0f);
  float demand =
      constrain(fmaxf(sqrtf(vx * vx + vy * vy) / vmax, fabsf(wz) * 130.0f / vmax), 0.0f, 1.0f);
  float pT = fmaxf(PERIOD * walkCtl.periodScale * (1.6f - 0.6f * demand), TSW_MIN / (1 - dutyEff));
  periodEff += (pT - periodEff) * fminf(1.0f, dt / 0.4f);

  {
    float dCmd = constrain(fmaxf(sqrtf(tvx * tvx + tvy * tvy) / vmax, fabsf(twz) * 130.0f / vmax),
                           0.0f, 1.0f);
    float dT = dutyAdapt ? DUTY + (DUTY_SLOW[gait] - DUTY) * (1 - fmaxf(demand, dCmd)) : DUTY;
    dutyEff += constrain(dT - dutyEff, -0.8f * dtReal, 0.15f * dtReal);
  }

  float Tst = dutyEff * periodEff;
  float k = footSpeedScale(vx, vy, wz, Tst, activeStride, width, comX, comY, strideR);
  vx *= k;
  vy *= k;
  wz *= k;

  {
    float rSoft = strideMax / 2 + 4, rHard = strideMax / 2 + 16, kr = 1;
    for (int i = 0; i < 6; i++) {
      if (swinging[i])
        continue;
      float nx, ny;
      neutral(i, nx, ny);
      float ex = feet[i][0] - nx, ey = feet[i][1] - ny, d = sqrtf(ex * ex + ey * ey);
      float ux = vx - wz * feet[i][1], uy = vy + wz * feet[i][0];
      if (d > rSoft && (ex * -ux + ey * -uy) > 0)
        kr = fminf(kr, gmClamp((rHard - d) / (rHard - rSoft), 0, 1));
    }
    vx *= kr;
    vy *= kr;
    wz *= kr;
  }
  {
    float ax = vx * govScale, ay = vy * govScale, aw = wz * govScale;
    accBX = (ax - pvx) / fmaxf(dtReal, 1e-3f) - aw * ay;
    accBY = (ay - pvy) / fmaxf(dtReal, 1e-3f) + aw * ax;
    pvx = ax;
    pvy = ay;
  }
  bool moving = fabsf(vx) > 0.5f || fabsf(vy) > 0.5f || fabsf(wz) > 0.003f;
  if (moving) {
    if (!stepping) {
      periodEff = pT;
      phase = bestPhase(gait, dutyEff);
    }
    stepping = true;
    settled = 0;
  }
  if (pendingGait >= 0 && !stepping)
    applyGait(pendingGait);
  if (stepping)
    phase = fmodf(phase + dt / periodEff, 1.0f);
  float sw = 1 - dutyEff;
  int airborne = 0;
  for (int i = 0; i < 6; i++) {
    float psi = fmodf(phase - GAIT_OFF[gait][i] + 1.0f, 1.0f);
    bool inSwing = stepping && (swinging[i] ? psi < swEnd[i] : psi < sw);
    if ((pendingGait >= 0 || servoSettling || walkCtl.recovery || walkCtl.turn == wc::SETTLING) &&
        !swinging[i])
      inSwing = false;
    if (inSwing && !swinging[i]) {
      if (psi > 0.6f * sw || (sw - psi) * periodEff < 0.2f)
        inSwing = false;
      else {
        swStart[i] = psi - dt / periodEff;
        swEnd[i] = sw;
      }
    }
    airborne += inSwing;
    if (inSwing && !swinging[i]) {
      lo[i][0] = feet[i][0];
      lo[i][1] = feet[i][1];
      loZ[i] = feet[i][2];
      walkLiftK[i] = walkCtl.cfg.landing && walkFresh() ? walkCtl.landingK : 0;
      loVz[i] = constrain(-hDot + (-bodyH + dBH - feet[i][2]) / 0.06f, -30.0f, 30.0f);
    }
    if (inSwing) {
      float s = constrain((psi - swStart[i]) / (swEnd[i] - swStart[i]), 0.0f, 1.0f);
      float Tsw = (swEnd[i] - swStart[i]) * periodEff, tRem = (1 - s) * Tsw;

      float hx = rampAvg(vx, tvx * k, ACC_LIN, tRem, tRem + Tst / 2);
      float hy = rampAvg(vy, tvy * k, ACC_LIN, tRem, tRem + Tst / 2);
      float hw = rampAvg(wz, twz * k, ACC_ROT, tRem, tRem + Tst / 2);
      float nx, ny, tx, ty;
      neutral(i, nx, ny);
      touchdownTarget(nx, ny, hx, hy, hw, Tst, tx, ty);

      if (!swinging[i]) {
        tdX[i] = tx;
        tdY[i] = ty;
      } else if (s < 0.85f) {
        float mv = 250 * dt * fminf(1.0f, (0.85f - s) / 0.25f), ex = tx - tdX[i], ey = ty - tdY[i],
              el = sqrtf(ex * ex + ey * ey);
        if (el > mv) {
          ex *= mv / el;
          ey *= mv / el;
        }
        tdX[i] += ex;
        tdY[i] += ey;
      }
      tx = tdX[i];
      ty = tdY[i];

      float ux = vx - wz * ty, uy = vy + wz * tx;
      float u0x = vx - wz * lo[i][1], u0y = vy + wz * lo[i][0];
      swingXY(s, lo[i][0], lo[i][1], tx, ty, u0x, u0y, ux, uy, Tsw, feet[i][0], feet[i][1]);
      {
        float sg = s * s * s * (10 - 15 * s + 6 * s * s);

        float x = fminf(s * 4.0f, 1.0f),
              h0 = 0.25f * (x - 6 * x * x * x + 8 * x * x * x * x - 3 * x * x * x * x * x);
        float vs = constrain(loVz[i] * Tsw / fmaxf(govScale, 0.2f), -30.0f, 30.0f);
        feet[i][2] =
            loZ[i] + (-bodyH - loZ[i]) * sg + vs * h0 + LIFT * wc::softLift(s, walkLiftK[i]);
      }
      stanceW[i] = 0;
    } else {
      if (swinging[i])
        walkCtl.touchdown();
      if (swinging[i] && !moving)
        settled |= 1 << i;
      stanceStep(feet[i][0], feet[i][1], vx, vy, wz, dt);
      feet[i][2] += -dBH + (-bodyH + dBH - feet[i][2]) * fminf(1.0f, dtReal / 0.06f);

      stanceW[i] =
          stepping ? stanceWeight(psi < swEnd[i] ? 0 : (psi - swEnd[i]) / (1 - swEnd[i])) : 1;
    }
    swinging[i] = inSwing;
  }
  if (pendingGait >= 0 && airborne == 0 && switchSlow < 0.55f)
    applyGait(pendingGait);
  if (stepping && !moving &&
      (settled == 0x3F ||
       ((servoSettling || walkCtl.recovery || walkCtl.turn == wc::SETTLING) && airborne == 0)))
    stepping = false;
}

void restep() {
  startGait();
  if (!stepping) {
    stepping = true;
    settled = 0;
    phase = 0;
  }
}

void applyMove() {
  if (servoHold || servoSettling)
    return;
  float fx = 0, fy = 0, fw = 0;
  float driveSpeed = fminf(speed, autoGait ? strideMax / (GAIT_DUTY[0] * PERIOD) : maxSpeed());
  switch (lastMove) {
  case 'w':
    fx = 1;
    break;
  case 's':
    fx = -1;
    break;
  case 'a':
    fy = 1;
    break;
  case 'd':
    fy = -1;
    break;
  case 'q':
    fw = 1;
    break;
  case 'e':
    fw = -1;
    break;
  }
  float demand = sqrtf(fx * fx + fy * fy) + fabsf(fw);
  if (demand > 1) {
    fx /= demand;
    fy /= demand;
    fw /= demand;
  }
  cmdVx = fx * driveSpeed;
  cmdVy = fy * driveSpeed;
  cmdWz = fw * driveSpeed / 130.0f;
  if (strideMax < 10) {
    cmdVx = cmdVy = cmdWz = 0;
  }
  if (endsSwapped) {
    cmdVx = -cmdVx;
    cmdVy = -cmdVy;
  }
  if (lastMove && sitting) {
    targetH = savedHeight();
    sitting = false;
  }
  if (lastMove) {
    trickReturn();
    startGait();
  }
}

void stopMoving() {
  walkCancel();
  lastMove = 0;
  cmdVx = cmdVy = cmdWz = 0;
}

void stopNow() {
  stopMoving();
  vx = vy = wz = 0;
}

bool servoOK() {
  if (centerActive) {
    say("Centering: wait for completion, or x / off to cancel.");
    return false;
  }
  if (servoHold || servoSettling) {
    say(servoHold ? "Servo HOLD: use servostatus / servorecover, or off."
                  : "Servo recovery: wait for raised feet to settle.");
    return false;
  }
  if (batLocked) {
    say("Battery empty: locked. Charge / swap the battery, then press EN.");
    return false;
  }
  return true;
}

void unpark() {
  if (park == P_NONE)
    return;
  park = P_NONE;
  if (mode != GAIT) {
    bodyH = PARK_H;
    startGait();
  }
  targetH = savedHeight();
  say("Standing up (%d mm).", (int)targetH);
}

void startPark(const char *why) {
  if (servoHold || servoSettling) {
    if (batLocked) {
      mode = MANUAL;
      stepping = false;
      relaxAll();
      park = P_PARKED;
      parkedAt = millis();
    }
    return;
  }
  if (park != P_NONE)
    return;
  stopNow();
  trickStop();
  heldH = sitting ? sitSavedH : targetH;
  sitting = false;
  if (!servosOn) {
    park = P_PARKED;
    parkedAt = millis();
    say("Parked (%s).", why);
    return;
  }
  if (mode != GAIT) {
    bodyH = STAND_CAD_H;
    startGait();
  }
  targetH = PARK_H;
  park = P_LOWERING;
  lowerDoneAt = 0;
  say("Lowering to rest (%s).", why);
}

void parkUpdate() {
  if (park == P_LOWERING && mode == GAIT && !stepping && fabsf(bodyH - PARK_H) < 0.5f) {
    if (!lowerDoneAt)
      lowerDoneAt = millis();
    if (millis() - lowerDoneAt > 800) {
      mode = MANUAL;
      relaxAll();
      park = P_PARKED;
      parkedAt = millis();
      say(batLocked ? "Resting, servos off."
                    : "Parked: servos off to save battery. Any command or a tap wakes it.");
    }
  }
}

void deepSleep(const char *why) {
  say("Going to deep sleep (%s). Press EN or replug to wake. Unplug the battery!", why);
  relaxAll();
  pcaL.sleep();
  pcaR.sleep();
  digitalWrite(LED_PIN, LOW);
  screenSleep();
  delay(200);
  esp_deep_sleep_start();
}

void powerCheck() {
  uint32_t now = millis();
  bool walking = lastMove || stepping;
  bool onBattery = batV >= BAT_PRESENT;
  if (walking)
    lastActivity = now;
  if (onBattery && !batLocked && park == P_NONE && idleMin > 0 &&
      now - lastActivity > idleMin * 60000UL)
    startPark("idle");
  if (onBattery && park == P_PARKED && !batLocked &&
      now - parkedAt > SLEEP_AFTER_PARK_MIN * 60000UL &&
      now - lastActivity > SLEEP_AFTER_PARK_MIN * 60000UL)
    deepSleep("parked a long time");
  if (onBattery && batLocked && park == P_PARKED && now - lockedAt > LOCKED_SLEEP_S * 1000UL)
    deepSleep("battery empty");
}

float readPinVolts() {
  uint32_t mv = 0;
  for (int k = 0; k < 16; k++)
    mv += analogReadMilliVolts(BAT_PIN);
  return mv / 16 / 1000.0f;
}

int batPercent(float v) {
  const float V[] = {6.6, 7.0, 7.2, 7.4, 7.6, 7.8, 8.0, 8.4};
  const float P[] = {0, 5, 15, 30, 50, 70, 85, 100};
  if (v <= V[0])
    return 0;
  for (int k = 1; k < 8; k++)
    if (v <= V[k])
      return (int)(P[k - 1] + (P[k] - P[k - 1]) * (v - V[k - 1]) / (V[k] - V[k - 1]));
  return 100;
}

void heatCheck() {
  if (massG <= 0)
    return;
  if (mode != GAIT || !servosOn)
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 2; j++)
        heatStep(heatMS[i][j], 0, 1.0f);
  float worst = 0;
  int wi = 0, wj = 0;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 2; j++)
      if (heatMS[i][j] > worst) {
        worst = heatMS[i][j];
        wi = i;
        wj = j;
      }
  float rms = sqrtf(worst);
  if (rms > TAU_HARD && ms64() > heatWarnAt) {
    heatWarnAt = ms64() + 300000UL;
    say("%s %s has been working hard (est. RMS load %.0f%% of stall): servos run hot. Rest it, or see torque / width / com.",
        legName(wi), wj ? "tibia" : "femur", rms * 100);
  }
}

int heatPct(int i, int j) {
  return massG > 0 ? (int)lroundf(sqrtf(heatMS[i][j]) * 100) : -1;
}

void lockBattery(const char *msg) {
  if (batLocked)
    return;
  batLocked = true;
  lockedAt = millis();
  say("%s (%.2f V). Servos off, locked. Unplug the battery and charge it.", msg, batV);
  startPark("battery empty");
}

void updateBatPct() {
  static int settle = 0, moveT = 0;
  if (batV < BAT_PRESENT) {
    batVslow = 0;
    batPct = -1;
    settle = 0;
    return;
  }
  batVslow = batRest();
  int p = batPercent(batVslow);
  if (settle < 20) {
    settle++;
    batPct = p;
    moveT = 0;
    return;
  }

  if (abs(p - batPct) >= 2) {
    if (++moveT >= 5) {
      batPct += p > batPct ? 1 : -1;
      moveT = 0;
    }
  } else
    moveT = 0;
}

void batteryCheck() {
  static int lowCount = 0, warnTimer = 0;
  static uint32_t lastT = 0;
  uint32_t now = millis();
  float dtb = lastT ? (now - lastT) / 1000.0f : 1.0f;
  lastT = now;
  float pin = batN ? (float)(batSumV / batN / 1000.0) : readPinVolts(),
        x = batN ? (float)(batSumX / batN) : loadX;
  batSumV = batSumX = 0;
  batN = 0;
  float v = pin * batScale + batOffset;
  batV = (batV == 0) ? v : batV * 0.8f + v * 0.2f;
  if (v >= BAT_PRESENT)
    batKF.update(v, x, dtb);
  else
    batKF.reset();
  {
    static float savedB = -1;
    static uint32_t savedAt = 0;
    if (savedB < 0)
      savedB = batKF.b;
    if (batKF.init && sqrtf(batKF.P11) < 0.008f && fabsf(batKF.b - savedB) > 0.004f &&
        now - savedAt > 600000UL) {
      prefs.putFloat("batb", batKF.b);
      savedB = batKF.b;
      savedAt = now;
    }
  }
  updateBatPct();
  if (batV >= BAT_PRESENT + 1.0f)
    packSeen = true;
  if (batV < BAT_PRESENT) {
    if (packSeen && servosOn && !batLocked)
      lockBattery("Battery reading lost while powered (unplugged, sense wire, or collapsed pack)");
    lowCount = 0;
    return;
  }
  if (v < BAT_EMPTY) {
    if (++lowCount >= 5)
      lockBattery("BATTERY EMPTY");
  }

  else
    lowCount = 0;
  if (batRest() < BAT_WARN && --warnTimer <= 0) {
    say(batLocked ? "Battery still connected at %.2f V: unplug it now."
                  : "Battery low: %.2f V, charge soon.",
        batRest());
    warnTimer = 30;
  }
}

uint8_t mpuWho = 0;

static inline bool mpu6500Family(uint8_t w) {
  return w == 0x70 || w == 0x71 || w == 0x73;
}

const char *mpuChipName(uint8_t w) {
  return w == 0x68   ? "MPU6050"
         : w == 0x70 ? "MPU6500"
         : w == 0x71 ? "MPU9250"
         : w == 0x73 ? "MPU9255"
                     : "unknown chip";
}

const char *mpuFailStage = "none";
esp_err_t mpuFailNative = ESP_OK;
uint32_t mpuRxErrors = 0;
int mpuFailGot = 0, mpuFailWant = 0;
uint32_t mpuRetryAttempts = 0, mpuRetryRecovered = 0, mpuRetrySkipped = 0;
uint32_t mpuWorstReadUs = 0;
const uint32_t MPU_READ_TIMEOUT_MS = 5, MPU_SAMPLE_BUDGET_US = 12000;

const char *turnSensorCause = "none";
uint64_t turnSensorAt = 0;
float turnSensorGapMs = 0;
esp_err_t turnSensorNative = ESP_OK;
uint32_t turnSensorErrors = 0;
bool turnSensorReadFailure = false;
int turnSensorGot = 0, turnSensorWant = 0;

void turnSensorReset() {
  turnSensorCause = "none";
  turnSensorAt = 0;
  turnSensorGapMs = 0;
  turnSensorNative = ESP_OK;
  turnSensorErrors = 0;
  turnSensorReadFailure = false;
  turnSensorGot = turnSensorWant = 0;
}

void turnSensorFault(const char *cause, float elapsedMs, bool readFailure = false) {
  if (!walkCtl.turnActive())
    return;
  turnSensorCause = cause;
  turnSensorAt = ms64();
  turnSensorGapMs = elapsedMs;
  turnSensorNative = readFailure ? mpuFailNative : ESP_OK;
  turnSensorErrors = mpuRxErrors;
  turnSensorReadFailure = readFailure;
  turnSensorGot = readFailure ? mpuFailGot : 0;
  turnSensorWant = readFailure ? mpuFailWant : 0;
}

void turnSensorReport() {
  if (walkCtl.reason != wc::SENSOR_LOST)
    return;
  if (!strcmp(turnSensorCause, "none")) {
    say("Turn sensor cause: no sample fault snapshot captured.");
    return;
  }
  say("Turn sensor cause: %s; interval %.1f ms; failed read transfers %lu; captured %lu ms ago.",
      turnSensorCause, turnSensorGapMs, (unsigned long)turnSensorErrors,
      (unsigned long)(ms64() - turnSensorAt));
  if (turnSensorReadFailure)
    say("Transfer at cancellation: %d/%d bytes, %s (%d). Missing sensor data was not integrated.",
        turnSensorGot, turnSensorWant, esp_err_to_name(turnSensorNative), (int)turnSensorNative);
}

uint8_t mpuFailCode = 0;

uint32_t mpuFailAt = 0, mpuTxErrors = 0, mpuBeginErrors = 0;

void mpuFailure(const char *stage, uint8_t code, int got, int want) {
  mpuFailStage = stage;
  mpuFailCode = code;
  mpuFailNative = ESP_OK;
  mpuFailGot = got;
  mpuFailWant = want;
  mpuFailAt = millis();
}

bool mpuWrite(uint8_t reg, uint8_t val) {
  Wire1.beginTransmission(MPU_ADDR);
  Wire1.write(reg);
  Wire1.write(val);
  uint8_t err = Wire1.endTransmission();
  if (err) {
    mpuTxErrors++;
    mpuFailure("config write", err, 0, 0);
    return false;
  }
  return true;
}

bool mpuReadBytesTimed(uint8_t reg, uint8_t *data, uint8_t count, uint32_t timeoutMs) {

  size_t got = 0;
  esp_err_t err = i2cWriteReadNonStop(1, MPU_ADDR, &reg, 1, data, count, timeoutMs, &got);
  if (err != ESP_OK || got != count) {
    mpuRxErrors++;
    mpuFailure("combined register read", 0xFE, (int)got, count);
    mpuFailNative = err;
    return false;
  }
  return true;
}

bool mpuReadBytes(uint8_t reg, uint8_t *data, uint8_t count) {
  return mpuReadBytesTimed(reg, data, count, 20);
}

bool mpuRead(float a[3], float g[3]) {
  uint8_t data[14];
  uint32_t started = micros();
  bool ok = mpuReadBytesTimed(0x3B, data, sizeof data, MPU_READ_TIMEOUT_MS);
  if (!ok) {

    uint32_t elapsed = micros() - started;
    uint32_t remainingMs =
        elapsed < MPU_SAMPLE_BUDGET_US ? (MPU_SAMPLE_BUDGET_US - elapsed) / 1000 : 0;
    if (remainingMs >= 3) {
      uint32_t retryMs = remainingMs < MPU_READ_TIMEOUT_MS ? remainingMs : MPU_READ_TIMEOUT_MS;
      mpuRetryAttempts++;
      ok = mpuReadBytesTimed(0x3B, data, sizeof data, retryMs);
      if (ok)
        mpuRetryRecovered++;
    } else
      mpuRetrySkipped++;
  }
  mpuWorstReadUs = max(mpuWorstReadUs, (uint32_t)(micros() - started));
  if (!ok)
    return false;
  int16_t r[7];
  for (int k = 0; k < 7; k++)
    r[k] = (int16_t)(((uint16_t)data[2 * k] << 8) | data[2 * k + 1]);
  for (int k = 0; k < 3; k++) {
    a[k] = r[k] / 16384.0f;
    g[k] = r[k + 4] / 65.5f - gyroBias[k];
  }
  return true;
}

float viewPitch() {
  return endsSwapped ? -pitch : pitch;
}

float viewRoll() {
  return endsSwapped ? -roll : roll;
}

float dot3(const float a[3], const float b[3]) {
  return a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
}

void norm3(float v[3]) {
  float n = sqrtf(dot3(v, v));
  if (n > 1e-6f)
    for (int k = 0; k < 3; k++)
      v[k] /= n;
}

void cross3(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}

// Collect a stillness window from valid complete samples; recovered reads may count, missing ones
// do not.
bool mpuAverage(float a[3], int n) {
  if (n <= 0)
    return false;
  imath::CalibrationMean mean;
  float ac[3], gy[3];
  uint32_t errorsBefore = mpuRxErrors;
  say("MPU calibration: hold still; collecting up to 3 quiet sample windows.");
  for (int window = 0; window < 3; window++) {
    mean = imath::CalibrationMean();
    for (int i = 0; i < n; i++) {
      if (mpuRead(ac, gy))
        mean.add(ac, gy);
      delay(5);
    }
    if (mean.finish(a, n)) {
      say("MPU calibration sample accepted: %d/%d usable reads, window %d/3.", mean.count, n,
          window + 1);
      return true;
    }
  }
  imath::CalibrationQuality q = mean.quality(n);
  say("MPU calibration rejected: %s. Usable %d/%d; acceleration %.3f g; noise %.3f g; gyro peak %.2f deg/s.",
      imath::calibrationReason(q.reason), mean.count, n, q.accelNorm, q.accelNoise, mean.gyroPeak);
  say("Calibration read transfers failed %lu across 3 windows (successful full retries are usable). Previous mounting kept.",
      (unsigned long)(mpuRxErrors - errorsBefore));
  if (q.reason == imath::CAL_GYRO_MOVING)
    say("If completely still: run mpu to re-zero the gyro while settled, then retry levelcal / mpufront.");
  return false;
}

bool imuLost = false, levelWasOn = false, imuRecovering = false;
bool mpuLog = false;
int imuFailRun = 0, imuGoodRun = 0;
uint64_t imuRetryAt = 0, imuCheckAt = 0;
uint32_t imuRetryTries = 0, imuErrors = 0, imuDrops = 0, gyroRezeros = 0;

uint8_t mpuReg(uint8_t reg) {
  uint8_t val;
  return mpuReadBytes(reg, &val, 1) ? val : 0xFF;
}

void mpuStatus() {
  say("MPU: %s; %s (WHO_AM_I 0x%02X); SDA D%d / SCL D%d, address 0x%02X, %lu Hz",
      imuOK           ? "reading"
      : imuRecovering ? "verifying reads"
      : imuLost       ? "lost, retrying"
                      : "not found",
      mpuChipName(mpuWho), mpuWho, mpuSda, mpuScl, MPU_ADDR, (unsigned long)Wire1.getClock());
  say("MPU: failed sample frames %lu, dropouts %lu, failed TX %lu, failed read transfers %lu, bus init errors %lu",
      (unsigned long)imuErrors, (unsigned long)imuDrops, (unsigned long)mpuTxErrors,
      (unsigned long)mpuRxErrors, (unsigned long)mpuBeginErrors);
  say("MPU retry: recovered %lu/%lu; skipped %lu (budget exhausted); worst complete-sample call %.1f ms.",
      (unsigned long)mpuRetryRecovered, (unsigned long)mpuRetryAttempts,
      (unsigned long)mpuRetrySkipped, mpuWorstReadUs / 1000.0f);
  say("MPU sample waits: %lu ms per transfer, %lu ms retry budget; actual driver/scheduler time can exceed these requests.",
      (unsigned long)MPU_READ_TIMEOUT_MS, (unsigned long)(MPU_SAMPLE_BUDGET_US / 1000));
  if (!strcmp(mpuFailStage, "none"))
    say("No MPU transfer failures recorded.");
  else if (mpuFailCode == 0xFE)
    say("Last MPU failure: %s, %d/%d bytes, %s (%d), %lu ms ago.", mpuFailStage, mpuFailGot,
        mpuFailWant, esp_err_to_name(mpuFailNative), (int)mpuFailNative,
        (unsigned long)(millis() - mpuFailAt));
  else if (mpuFailCode == 0xFF)
    say("Last MPU failure: %s, %d/%d bytes, %lu ms ago; requestFrom does not expose the driver error.",
        mpuFailStage, mpuFailGot, mpuFailWant, (unsigned long)(millis() - mpuFailAt));
  else
    say("Last MPU failure: %s, Wire code %u, %lu ms ago (2=NACK, 4=other, 5=timeout).",
        mpuFailStage, mpuFailCode, (unsigned long)(millis() - mpuFailAt));
  say("Gyro quiet window: %u/%u usable samples; %s; re-zeroes %lu.", imuBiasWindow.count,
      imuBiasWindow.attempts, imuBiasWindow.state, (unsigned long)gyroRezeros);
  say("MPU lines now: SDA=%d SCL=%d (idle should be HIGH; HIGH alone does not prove a good connection).",
      digitalRead(mpuSda), digitalRead(mpuScl));
}

static inline bool mpuSayTry(uint32_t n) {
  return n == 1 || n == 5 || n % 20 == 0;
}

bool mpuConfigure() {
  return mpuWrite(0x6B, 0x00) && mpuWrite(0x1A, 0x04) && mpuWrite(0x1B, 0x08) &&
         mpuWrite(0x1C, 0x00) &&

         (!mpu6500Family(mpuWho) || mpuWrite(0x1D, 0x04));
}

void i2cScan(TwoWire &bus, const char *name) {
  String found;
  int n = 0;
  for (uint8_t ad = 1; ad < 127; ad++) {
    bus.beginTransmission(ad);
    if (bus.endTransmission() == 0) {
      char b[8];
      snprintf(b, sizeof b, " 0x%02X", ad);
      found += b;
      n++;
    }
  }
  say("%s: %s", name,
      n ? found.c_str()
        : "nothing answers (check power, GND, and that SDA/SCL go to the right pins)");
}

bool mpuBusBegin() {
  Wire1.end();
  bool started = Wire1.begin(mpuSda, mpuScl, 100000);
  esp_log_level_set("i2c.master", mpuLog ? ESP_LOG_ERROR : ESP_LOG_NONE);
  if (!started) {
    mpuBeginErrors++;
    mpuFailure("bus init", 4, 0, 0);
    return false;
  }
  Wire1.setTimeOut(20);
  return true;
}

bool mpuProbe() {
  uint8_t lastErr = 0;
  for (uint8_t ad : {0x68, 0x69}) {
    Wire1.beginTransmission(ad);
    lastErr = Wire1.endTransmission();
    if (!lastErr) {
      MPU_ADDR = ad;
      return true;
    }
  }
  mpuTxErrors++;
  mpuFailure("address probe (0x68/0x69)", lastErr, 0, 0);
  return false;
}

void setupIMU() {
  imuClock.reset();
  imuBiasWindow.reset();
  walkCtl.resetSensor();
  walkSampleAt = 0;
  imuOK = false;
  imuLost = false;
  imuRecovering = false;
  imuFailRun = 0;
  imuGoodRun = 0;
  imuRetryTries = 0;
  upF.init = false;
  hasLevel = prefs.getBytesLength("axZ") == sizeof axZ;
  hasFront = hasLevel && prefs.getBytesLength("axX") == sizeof axX;
  if (hasLevel)
    prefs.getBytes("axZ", axZ, sizeof axZ);
  if (hasFront) {
    prefs.getBytes("axX", axX, sizeof axX);
    cross3(axZ, axX, axY);
  }
  levelWasOn = prefs.getBool("level", false) && hasFront;
  levelOn = false;
  bool found = false;
  for (int k = 0; k < 2 && !found; k++) {
    mpuSda = k ? MPU_SCL : MPU_SDA;
    mpuScl = k ? MPU_SDA : MPU_SCL;
    found = mpuBusBegin() && mpuProbe();
    if (found && k)
      say("Note: MPU SDA/SCL are swapped (SDA D%d, SCL D%d).", mpuSda, mpuScl);
  }
  if (!found) {
    mpuSda = MPU_SDA;
    mpuScl = MPU_SCL;
    mpuBusBegin();
    imuLost = true;
    imuRetryAt = ms64() + 3000;
    say("MPU not found on D21/D22 (tried both ways). Retrying every 3 s; type mpustatus.");
    return;
  }
  uint8_t who = mpuReg(0x75);
  mpuWho = who;
  say("MPU found at 0x%02X, WHO_AM_I = 0x%02X (%s)%s", MPU_ADDR, who, mpuChipName(who),
      who == 0x68          ? ""
      : mpu6500Family(who) ? ": works the same, its accelerometer filter is set too"
                           : ": trying anyway, check tilt readings");
  if (!mpuConfigure()) {
    imuLost = true;
    imuRetryAt = ms64() + 3000;
    say("MPU address answers, but configuration failed. Retrying every 3 s.");
    mpuStatus();
    return;
  }
  delay(50);
  float s[3] = {0, 0, 0}, a[3], g[3];
  int ok = 0, goodRun = 0;

  for (int i = 0; i < 200; i++) {
    if (mpuRead(a, g)) {
      for (int k = 0; k < 3; k++)
        s[k] += g[k] + gyroBias[k];
      ok++;
      goodRun++;
    } else {
      imuErrors++;
      goodRun = 0;
    }
    delay(5);
  }
  if (ok < 100 || goodRun < 5) {
    imuLost = true;
    imuRetryAt = ms64() + 3000;
    say("MPU address answers, but reads are unreliable (%d/200). Retrying every 3 s.", ok);
    mpuStatus();
    return;
  }
  for (int k = 0; k < 3; k++)
    gyroBias[k] = s[k] / ok;
  imuOK = true;
  levelOn = levelWasOn && hasFront;
  say("MPU6050 ready (sensor reads verified). %s",
      hasFront ? (levelOn ? "Leveling ON." : "Leveling off (level on).")
               : "Not set up yet: levelcal on a flat floor, then mpufront.");
}

// Rotate raw sensor vectors into calibrated body axes before updating attitude and walking assists.
void imuUpdate(float dt) {
  static float fallT = 0;
  float a[3], g[3];
  bool still = mode == GAIT && !stepping && !lastMove && trick == TR_NONE && park == P_NONE &&
               !sitting && !servoHold && !servoSettling && zcLeg < 0 && zsLeg < 0 &&
               ms64() >= sagTestUntil && fabsf(targetH - bodyH) < .5f;
  if (!still || !imuOK) {
    imuBiasWindow.reset();
    imuBiasWindow.state = "waiting for a settled robot";
  }
  if (!imuOK) {
    if (!imuLost)
      return;
    uint64_t now = ms64();
    if (imuRecovering) {
      if (now < imuCheckAt)
        return;
      if (!mpuRead(a, g)) {
        imuErrors++;
        imuRecovering = false;
        imuGoodRun = 0;
      } else if (++imuGoodRun >= 5) {
        imuOK = true;
        imuLost = false;
        imuRecovering = false;
        imuFailRun = 0;
        levelOn = levelWasOn && hasFront;
        upF.init = false;
        say("MPU6050 reading again (5 complete sensor reads verified; dropouts %lu).",
            (unsigned long)imuDrops);
      }
      if (!imuOK && !imuRecovering && mpuSayTry(imuRetryTries)) {
        say("MPU recovery failed: address/configuration can succeed while sensor reads fail.");
        mpuStatus();
      }
    } else if (now >= imuRetryAt) {
      imuRetryAt = now + 3000;
      imuRetryTries++;
      if (mpuBusBegin() && mpuProbe() && mpuConfigure()) {
        imuRecovering = true;
        imuGoodRun = 0;
        imuCheckAt = ms64() + 50;
      } else if (mpuSayTry(imuRetryTries)) {
        say("MPU still unavailable; retrying every 3 s.");
        mpuStatus();
      }
    }
    return;
  }
  if (!mpuRead(a, g)) {
    imuErrors++;
    imuClock.fail();
    imuBiasWindow.miss(micros());
    turnSensorFault("complete sample failed after retry/budget",
                    (uint32_t)(micros() - imuClock.last) * .001f, true);
    walkCtl.resetSensor();
    walkSampleAt = 0;

    if (++imuFailRun >= 50) {
      imuOK = false;
      imuLost = true;
      imuRecovering = false;
      imuDrops++;
      levelWasOn = levelOn;
      levelOn = false;
      imuRetryAt = ms64() + 3000;
      imuRetryTries = 0;
      say("MPU sensor reads failed for 50 control ticks: tilt + leveling paused; retrying every 3 s.");
      mpuStatus();
    }
    return;
  }
  imuFailRun = 0;
  imath::SampleTime sampleTime = imuClock.accept(micros());
  dt = sampleTime.dt;
  if (!sampleTime.continuous) {
    turnSensorFault(sampleTime.elapsed > .1f     ? "successful MPU reads >100 ms apart"
                    : sampleTime.elapsed < .001f ? "sample interval too short/unseeded"
                                                 : "previous sample frame failed",
                    sampleTime.elapsed * 1000);
    walkCtl.resetSensor();
    walkSampleAt = 0;
    fallT = 0;
  }

  if (!hasLevel || !hasFront) {
    walkCtl.resetSensor();
    walkSampleAt = 0;
    upF.init = false;
    return;
  }

  if (sampleTime.restart)
    imuBiasWindow.reset();
  float correction[3];
  if (still && imuBiasWindow.add(a, g, micros(), correction)) {
    for (int k = 0; k < 3; k++) {
      gyroBias[k] += correction[k];
      // This sample was read with the old bias; use the new bias immediately.
      g[k] -= correction[k];
    }
    upF.bI[0] = upF.bI[1] = upF.bI[2] = 0;
    gyroRezeros++;
  }
  float ab[3] = {dot3(a, axX), dot3(a, axY), dot3(a, axZ)};
  float rawBody[3] = {ab[0], ab[1], ab[2]};

  if (mode == GAIT) {
    ab[0] -= accBX / 9810.0f;
    ab[1] -= accBY / 9810.0f;
  }
  float gx = dot3(g, axX), gy = dot3(g, axY), gz = dot3(g, axZ);
  float amag = sqrtf(ab[0] * ab[0] + ab[1] * ab[1] + ab[2] * ab[2]);

  bool attitudeRestart = sampleTime.restart || !upF.init;
  {
    float w3[3] = {gx * DEG_TO_RAD, gy * DEG_TO_RAD, gz * DEG_TO_RAD};
    imuAccelTrust = accelTrust(amag);
    if (attitudeRestart)
      upF.reset(ab, imuAccelTrust);
    else
      upF.update(ab, w3, dt, imuAccelTrust);
    roll = upF.rollDeg();
    pitch = upF.pitchDeg();
  }
  if (!upF.init) {
    turnSensorFault("attitude reacquisition waiting for trusted acceleration",
                    sampleTime.elapsed * 1000);
    walkCtl.resetSensor();
    walkSampleAt = 0;
    fallT = 0;
    return;
  }
  for (int k = 0; k < 3; k++)
    imuLinearBody[k] = rawBody[k] - upF.u[k];
  imuLinearUp = dot3(imuLinearBody, upF.u);
  walkSample(dt, rawBody[0], rawBody[1], rawBody[2], gx, gy, gz);
  if (servoHold || servoSettling)
    return;

  {
    static float gms = 25;
    float wm = sqrtf(gx * gx + gy * gy);
    if (mode == GAIT && stepping && wm > fmaxf(60.0f, 4 * sqrtf(gms)) && fabsf(amag - 1) > 0.25f) {
      stabSlow = 0.5f;
      stabSlowT = 2;
      if (ms64() > stumbleWarnAt) {
        stumbleWarnAt = ms64() + 3000;
        say("Stumble (IMU %.0f deg/s): slowing down for 2 s.", wm);
      }
    }
    gms += (wm * wm - gms) * (1 - expf(-dt / 2.0f));
  }

  if (mode == GAIT && hasLevel && hasFront) {
    static float mn = 1e9f;
    static int cnt = 0;
    float fx[6], fy[6], fz[6];
    bool on[6];
    float sagLift = sagTestLift();
    for (int i = 0; i < 6; i++) {
      fx[i] = feet[i][0] + pFoot[i][0];
      fy[i] = feet[i][1] + pFoot[i][1];
      fz[i] = feet[i][2] + pFoot[i][2];
      on[i] = !swinging[i] && !poseFootRaised(i, sagLift);
    }
    PoseFrame poseFrame(pRoll, pPitch, pYaw);
    SupportFrame frame(poseFrame, pTx, pTy, pTz, comX, comY, 20.0f, upF.u);
    mn = fminf(mn, energyMargin(fx, fy, fz, on, frame.cx, frame.cy, frame.cz, frame.up));
    if (++cnt >= 50) {
      nesmLast = mn;
      mn = 1e9f;
      cnt = 0;
      if (stepping && nesmLast < 6) {
        stabSlow = 0.5f;
        stabSlowT = 2;
        if (ms64() > nesmWarnAt) {
          nesmWarnAt = ms64() + 10000;
          say("Low stability on this slope (tip margin %.1f mm): slowing down. Try ripple / wave or a wider stance.",
              nesmLast);
        }
      }
    }
  }

  tipping = hasFront && (fabsf(roll) > FALL_DEG || fabsf(pitch) > FALL_DEG);
  if (tipping) {
    float before = fallT;
    fallT += dt;
    if (before < 0.3f && fallT >= 0.3f && (lastMove || stepping)) {
      stopNow();
      say("Tipping (roll %.0f, pitch %.0f): stopped.", roll, pitch);
    }
  } else
    fallT = 0;

  static CombAvg cp, cr;
  static bool combInit = false;
  if (!combInit || attitudeRestart) {
    cp.reset(0);
    cr.reset(0);
    combInit = true;
  }
  cp.setLength(stepping ? (int)lroundf(periodEff / (fmaxf(govScale, 0.05f) * dt)) : 25);
  cr.setLength(cp.n);
  float ep = cp.push(pitch), er = cr.push(roll);
  if (trick != TR_NONE || walkCtl.recovery || stabSlowT > 0 || !sampleTime.continuous ||
      attitudeRestart || imuAccelTrust < .5f) {
  } else if (levelOn && mode == GAIT && park == P_NONE) {
    float ki = stepping ? 1.0f / (2.5f * periodEff) : 1.0f;
    float lim = constrain((bodyH - 35) * 0.012f, 0.12f, 0.30f);
    levP += (ki * deadzone(ep, 0.4f) * DEG_TO_RAD - 0.02f * levP) * dt;
    levR += (ki * deadzone(er, 0.4f) * DEG_TO_RAD - 0.02f * levR) * dt;
    levP = constrain(levP, -lim, lim);
    levR = constrain(levR, -lim, lim);
  } else {
    float d = expf(-dt / 0.39f);
    levP *= d;
    levR *= d;
  }
}

void touchTap() {
  if (park != P_NONE) {
    if (servoOK())
      unpark();
    return;
  }
  screenEvent(EV_TAP);
  if (trickStart(TR_WAVE, 1.0f))
    say("Hi!");
}

void touchTriple() {
  screenEvent(EV_TRIPLE);
  if (park == P_NONE)
    trickRandom(false);
}

void touchDoubleTap() {
  if (centerActive || servoHold || servoSettling)
    return;
  screenEvent(EV_DOUBLE);
  if (park != P_NONE || mode != GAIT || lastMove || stepping)
    return;
  trickReturn();
  if (!sitting) {
    sitSavedH = targetH;
    targetH = 40;
    sitting = true;
    say("Sitting.");
  } else {
    targetH = savedHeight();
    sitting = false;
    say("Standing (%d mm).", (int)targetH);
  }
}

void touchHold() {
  screenEvent(EV_HOLD);
  if (lastMove || stepping) {
    stopMoving();
    say("Touch: stopping.");
  }
}

void touchUpdate() {
  float v = 0;
  bool on;
  if (USE_BUTTON)
    on = digitalRead(TOUCH_PIN) == LOW;
  else {
    if (touchBase <= 0)
      return;
    v = touchRead(TOUCH_PIN);
    on = v < touchBase * 0.7f;
  }
  uint32_t t = millis();
  if (on && !touchOn) {
    touchOn = true;
    touchDownAt = t;
    holdFired = false;
    lastActivity = t;
  }
  if (on && touchOn && !holdFired && t - touchDownAt > 1000) {
    holdFired = true;
    tapCount = 0;
    touchHold();
  }
  if (!on && touchOn) {
    touchOn = false;
    if (!holdFired && t - touchDownAt < 500) {
      tapCount++;
      lastTapEnd = t;
    }
  }
  if (tapCount && !touchOn && t - lastTapEnd > 400) {
    if (tapCount == 1)
      touchTap();
    else if (tapCount == 2)
      touchDoubleTap();
    else
      touchTriple();
    tapCount = 0;
  }
  if (!on && !USE_BUTTON)
    touchBase = touchBase * 0.995f + v * 0.005f;
}

void statusLED() {
  uint32_t t = millis();
  bool on;
  bool batLow = batV > BAT_PRESENT && batV < BAT_WARN;
  if (batLocked || batLow || tipping)
    on = (t / 100) % 2;
  else if (park != P_NONE)
    on = (t % 3000) < 60;
  else if (!calibrated)
    on = (t % 2000) < 150 || ((t % 2000) > 300 && (t % 2000) < 450);
  else if (lastMove || stepping)
    on = (t / 500) % 2;
  else if (mode == GAIT)
    on = true;
  else
    on = false;
  if (ms64() < ledFlickerUntil)
    on = !on;
  digitalWrite(LED_PIN, on);
}

void pcaSyncPulse(int b) {
  Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
  pcaTickUs[b] = 1e6f * (p.readPrescale() + 1) / PCA_OSC;
  p.setPWM(SYNC_CH, 0, 410);
}

bool syncClean(int b, uint32_t edges, float &T) {
  uint32_t mn = syncMinI[b], mx = syncMaxI[b], nz = syncNoise[b];
  syncMinI[b] = 0xFFFFFFFF;
  syncMaxI[b] = 0;
  syncNoise[b] = 0;
  syncSpread[b] = mx >= mn ? mx - mn : 0;
  syncNoiseLast[b] = nz;
  T = (mx >= mn) ? (mn + mx) / 2.0f : 0;
  return edges >= 40 && edges <= 60 && mx >= mn && mx - mn < 600 && nz == 0 && T > 16000 &&
         T < 25000;
}

const bool PCA_SYNC = false;

void pcaClockSetup() {
  if (!PCA_SYNC) {
    for (int b = 0; b < 2; b++) {
      Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
      pcaTickUs[b] = 1e6f * (p.readPrescale() + 1) / PCA_OSC;
      p.setPWM(SYNC_CH, 0, 4096);
    }
    syncOK = false;
    return;
  }
  for (int b = 0; b < 2; b++) {
    pinMode(SYNC_PIN[b], INPUT);
    pcaSyncPulse(b);
  }
  attachInterrupt(digitalPinToInterrupt(SYNC_PIN[0]), syncIsrL, RISING);
  attachInterrupt(digitalPinToInterrupt(SYNC_PIN[1]), syncIsrR, RISING);
  delay(60);
  for (int b = 0; b < 2; b++) {
    float T;
    syncClean(b, 0, T);
  }
  uint32_t e0[2] = {syncEdges[0], syncEdges[1]}, t0[2] = {syncLastUs[0], syncLastUs[1]};
  delay(1000);
  for (int b = 0; b < 2; b++) {
    uint32_t de = syncEdges[b] - e0[b];
    float T;
    bool ok = syncClean(b, de, T);
    if (de >= 2)
      T = (float)(syncLastUs[b] - t0[b]) / de;
    syncRate[b] = de;
    if (ok) {
      Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
      pcaOscMeas[b] = 4096.0f * (p.readPrescale() + 1) / (T * 1e-6f);
      if (b == 0)
        framePeriodS = T * 1e-6f;
      say("PCA %s: sync clean, frame %.2f Hz (its clock ~%.2f MHz; pulse widths unchanged).",
          b == 0 ? "left" : "right", 1e6f / T, pcaOscMeas[b] / 1e6f);
    } else if (de == 0) {
      if (b == 0)
        say("PCA sync wire not connected (optional): normal timing.");
    } else
      say("PCA %s sync signal NOISY (%lu edges/s, spread %lu us, %lu noise edges): ignored. Shorter wire, away from servo cables.",
          b == 0 ? "left" : "right", (unsigned long)de, (unsigned long)syncSpread[b],
          (unsigned long)syncNoiseLast[b]);
    if (b == 0)
      syncOK = ok;
  }
  if (syncOK)
    say("Control loop synced to the PCA frames.");
}

void pcaClockApply() {
  bool changed = false;
  for (int b = 0; b < 2; b++) {
    if (!calTrue[b])
      continue;
    if (pcaOscSaved[b] > 20e6f && pcaOscSaved[b] < 32e6f) {
      float k = PCA_OSC / pcaOscSaved[b];
      for (int leg = 0; leg < 6; leg++)
        if (boardOfLeg(leg) == b)
          for (int j = 0; j < 3; j++) {
            trim[leg][j] =
                (int16_t)constrain(lroundf((trim[leg][j] - 1500.0f * (k - 1)) / k), -400L, 400L);
            gainP[leg][j] /= k;
            gainN[leg][j] /= k;
          }
      say("PCA %s: restored your original trims / scales (undid the v3.9.2 clock conversion). Saved.",
          b == 0 ? "left" : "right");
    }
    calTrue[b] = false;
    changed = true;
  }
  if (changed) {
    prefs.putBytes("trim", trim, sizeof trim);
    prefs.putBytes("gainP", gainP, sizeof gainP);
    prefs.putBytes("gainN", gainN, sizeof gainN);
    prefs.putBytes("caltrue", calTrue, sizeof calTrue);
  }
}

void pcaClockService() {
  if (!PCA_SYNC)
    return;
  static uint32_t pe[2] = {0, 0};
  for (int b = 0; b < 2; b++) {
    uint32_t e = syncEdges[b], de = e - pe[b];
    pe[b] = e;
    float T;
    bool ok = syncClean(b, de, T);
    syncRate[b] = de;
    if (ok) {
      Adafruit_PWMServoDriver &p = b == 0 ? pcaL : pcaR;
      pcaOscMeas[b] = 4096.0f * (p.readPrescale() + 1) / (T * 1e-6f);
      if (b == 0)
        framePeriodS = T * 1e-6f;
    }
    if (b == 0) {
      if (syncOK && !ok)
        say(de ? "PCA sync signal noisy (%lu edges/s, %lu noise): back to normal timing."
               : "PCA sync lost: back to normal timing.",
            (unsigned long)de, (unsigned long)syncNoiseLast[0]);
      if (!syncOK && ok)
        say("PCA sync found (clean signal): control loop synced.");
      syncOK = ok;
    }
  }
}

float finiteSetting(float value, float fallback, float lo, float hi) {
  return isfinite(value) ? constrain(value, lo, hi) : fallback;
}

void sanitizeSettings() {
  bool actuatorBad = false;
  for (int i = 0; i < 6; i++)
    for (int j = 0; j < 3; j++) {
      if (sgn[i][j] != 1 && sgn[i][j] != -1) {
        sgn[i][j] = 1;
        actuatorBad = true;
      }

      if (!isfinite(gainP[i][j]) || gainP[i][j] <= 0) {
        gainP[i][j] = 1;
        actuatorBad = true;
      }
      if (!isfinite(gainN[i][j]) || gainN[i][j] <= 0) {
        gainN[i][j] = 1;
        actuatorBad = true;
      }
    }
  if (actuatorBad) {
    calibrated = false;
    say("Invalid saved servo direction/gain: defaults used in RAM; check dir/cal before save.");
  }
  speed = finiteSetting(speed, 40, 5, 1000);
  targetH = finiteSetting(targetH, STAND_CAD_H, 40, 85);
  heldH = sitSavedH = targetH;
  LIFT = finiteSetting(LIFT, 25, 10, 40);
  PERIOD = finiteSetting(PERIOD, 1, .5f, 2);
  width = finiteSetting(width, 0, -40, 30);
  idleMin = min(idleMin, (uint32_t)120);
  comX = finiteSetting(comX, 0, -30, 30);
  comY = finiteSetting(comY, 0, -30, 30);
  massG = finiteSetting(massG, 0, 0, 3000);
  stallKgcm = finiteSetting(stallKgcm, 2.2f, .5f, 20);
  zsH = finiteSetting(zsH, 18, 3, 60);
  blPlay = finiteSetting(blPlay, 0, 0, 4);
  sagFull = finiteSetting(sagFull, 0, 0, 20);
  lagMs = finiteSetting(lagMs, 0, 0, 60);
  servoDps = finiteSetting(servoDps, 600, 200, 1000);
  for (int i = 0; i < 6; i++)
    if (!isfinite(zErr[i]))
      zErr[i] = 0;
  if (!isfinite(batScale) || batScale <= 0)
    batScale = 14.7f / 4.7f;
  if (!isfinite(batOffset))
    batOffset = 0;
  if (!isfinite(batKF.b) || batKF.b < 0)
    batKF.b = .03f;
}

// Load this board's NVS calibration, then validate settings before using them in motion math.
void loadSettings() {
  prefs.begin("hexapod", false);
  calibrated = prefs.getBool("cal", false);
  if (prefs.getBytesLength("sgn") == sizeof sgn)
    prefs.getBytes("sgn", sgn, sizeof sgn);
  else
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        sgn[i][j] = 1;
  if (prefs.getBytesLength("trim") == sizeof trim)
    prefs.getBytes("trim", trim, sizeof trim);
  else
    memset(trim, 0, sizeof trim);
  if (prefs.getBytesLength("caltrue") == sizeof calTrue)
    prefs.getBytes("caltrue", calTrue, sizeof calTrue);
  else
    calTrue[0] = calTrue[1] = false;
  if (prefs.getBytesLength("pcaosc") == sizeof pcaOscSaved)
    prefs.getBytes("pcaosc", pcaOscSaved, sizeof pcaOscSaved);
  else
    pcaOscSaved[0] = pcaOscSaved[1] = 0;
  if (prefs.getBytesLength("gainP") == sizeof gainP &&
      prefs.getBytesLength("gainN") == sizeof gainN) {
    prefs.getBytes("gainP", gainP, sizeof gainP);
    prefs.getBytes("gainN", gainN, sizeof gainN);
  } else if (prefs.getBytesLength("gain") == sizeof gainP) {
    prefs.getBytes("gain", gainP, sizeof gainP);
    memcpy(gainN, gainP, sizeof gainN);
  } else
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++)
        gainP[i][j] = gainN[i][j] = 1.0f;
  sidesSwapped = prefs.getBool("swap", false);
  endsSwapped = prefs.getBool("ends", false);
  gait = constrain(prefs.getInt("gait", 0), 0, 2);
  speed = prefs.getFloat("speed", 40);
  targetH = prefs.getFloat("height", STAND_CAD_H);
  heldH = sitSavedH = targetH;
  LIFT = prefs.getFloat("lift", 25);
  PERIOD = prefs.getFloat("period", 1.0);
  width = prefs.getFloat("width", 0);
  idleMin = prefs.getUInt("idle", 5);
  fidgetOn = prefs.getBool("fidget", true);
  batScale = prefs.getFloat("batscale", 14.7f / 4.7f);
  batOffset = prefs.getFloat("batoff", 0);
  batKF.b = prefs.getFloat("batb", 0.03f);
  comX = prefs.getFloat("comx", 0);
  comY = prefs.getFloat("comy", 0);
  massG = prefs.getFloat("massg", 0);
  stallKgcm = prefs.getFloat("stall", 2.2f);
  if (prefs.getBytesLength("zerr") == sizeof zErr)
    prefs.getBytes("zerr", zErr, sizeof zErr);
  zFix = prefs.getBool("zfix", false);
  zsH = prefs.getFloat("zscaleh", 18);
  autoGait = prefs.getBool("autogait", false);
  dutyAdapt = prefs.getBool("dutyadapt", false);
  blPlay = prefs.getFloat("blplay", 0);
  sagFull = prefs.getFloat("sag", 0);
  lagMs = prefs.getFloat("lag", 0);
  servoDps = prefs.getFloat("sdps", 600);
  sanitizeSettings();
  walkLoad();
  DUTY = GAIT_DUTY[gait];
  updateStride(true);
}

float savedHeight() {
  return finiteSetting(prefs.getFloat("height", STAND_CAD_H), STAND_CAD_H, 40.0f, 85.0f);
}

void saveHeight() {
  prefs.putFloat("height", targetH);
}

void saveSettings() {
  walkSave();
  prefs.putBytes("sgn", sgn, sizeof sgn);
  prefs.putBytes("trim", trim, sizeof trim);
  prefs.putBytes("caltrue", calTrue, sizeof calTrue);
  prefs.putBytes("pcaosc", pcaOscSaved, sizeof pcaOscSaved);
  prefs.putBool("cal", true);
  calibrated = true;
  prefs.putBytes("gainP", gainP, sizeof gainP);
  prefs.putBytes("gainN", gainN, sizeof gainN);
  prefs.putInt("gait", preferredGait);
  prefs.putFloat("speed", speed);
  prefs.putFloat("height", sitting ? sitSavedH : targetH);
  prefs.putFloat("lift", LIFT);
  prefs.putFloat("period", PERIOD);
  prefs.putFloat("width", width);
  prefs.putUInt("idle", idleMin);
  prefs.putFloat("comx", comX);
  prefs.putFloat("comy", comY);
  prefs.putFloat("massg", massG);
  prefs.putFloat("stall", stallKgcm);
  prefs.putBytes("zerr", zErr, sizeof zErr);
  prefs.putBool("zfix", zFix);
  prefs.putBool("autogait", autoGait);
  prefs.putBool("dutyadapt", dutyAdapt);
  prefs.putFloat("blplay", blPlay);
  prefs.putFloat("batb", batKF.b);
  prefs.putFloat("sag", sagFull);
  prefs.putFloat("lag", lagMs);
  prefs.putFloat("sdps", servoDps);
}

void showParams() {
  walkStatus();
  say("gait %s | speed %d mm/s (max %d) | height %d | lift %d | period %.2f s | width %+d | idle %u min",
      GAIT_NAME[gait], (int)speed, (int)maxSpeed(), (int)targetH, (int)LIFT, PERIOD, (int)width,
      (unsigned)idleMin);
  say("stride %d mm | com %+.0f %+.0f | sag %.1f mm | lag %.0f ms | servospeed %.0f deg/s | governor %.0f%% (peak joint %.0f deg/s)",
      (int)strideMax, comX, comY, sagFull, lagMs, servoDps, govScale * 100, maxRateDps);
  say("autogait %s | dutyadapt %s (duty now %.2f) | backlash %.1f mm", autoGait ? "on" : "off",
      dutyAdapt ? "on" : "off", dutyEff, blPlay);
  if (massG > 0)
    say("torque limit on: mass %.0f g, stall %.1f kg*cm (type torque)", massG, stallKgcm);
  else
    say("torque limit off (set it with: mass <grams>)");
}

int physLeg(int named) {
  return endsSwapped ? 5 - named : named;
}

const char *legName(int phys) {
  return LEG[endsSwapped ? 5 - phys : phys].name;
}

void showTable() {
  say("leg   coxa                  femur                 tibia            (direction, trim us, scale +/-)%s%s",
      sidesSwapped ? "  [sides swapped]" : "", endsSwapped ? "  [ends swapped]" : "");
  for (int n = 0; n < 6; n++) {
    int i = physLeg(n);
    say("%s    %+d %+5d %.2f/%.2f    %+d %+5d %.2f/%.2f    %+d %+5d %.2f/%.2f", LEG[n].name,
        sgn[i][0], trim[i][0], gainP[i][0], gainN[i][0], sgn[i][1], trim[i][1], gainP[i][1],
        gainN[i][1], sgn[i][2], trim[i][2], gainP[i][2], gainN[i][2]);
  }
  say(calibrated ? "Saved: walking unlocked." : "Not saved yet: walking locked (dir, flip, save).");
}

void centerAll() {
  mode = MANUAL;
  stopNow();
  stepping = false;
  park = P_NONE;
  trickStop();
  qValid = false;
  centerActive = true;
  centerJoint = 0;
  centerNextAt = millis();
  say("Centering joints (c): telemetry stays active. x / off cancels.");
}

void centerUpdate() {
  if (!centerActive)
    return;
  if (servoHold || batLocked) {
    centerActive = false;
    return;
  }
  uint32_t now = millis();
  if ((int32_t)(now - centerNextAt) < 0)
    return;
  writeAngle(centerJoint / 3, centerJoint % 3, 0);
  if (servoHold) {
    centerActive = false;
    return;
  }
  if (++centerJoint >= 18) {
    centerActive = false;
    bodyH = STAND_CAD_H;
    say("Centred: every servo at its middle (calibration pose, %d mm). Type stand for your saved height.",
        (int)STAND_CAD_H);
  } else
    centerNextAt = now + 100;
}

void dirTest(int i) {
  mode = MANUAL;
  stopNow();
  stepping = false;
  park = P_NONE;
  for (int j = 0; j < 3; j++)
    writeAngle(i, j, 0);
  delay(300);
  const char *expect[3] = {LEG[physLeg(i)].y > 0 ? "leg should swing toward the REAR"
                                                 : "leg should swing toward the FRONT",
                           "foot should go UP", "foot should swing OUTWARD, away from the body"};
  for (int j = 0; j < 3; j++) {
    say("%s %s: %s", legName(i), JOINT[j], expect[j]);
    writeAngle(i, j, 15);
    delay(900);
    writeAngle(i, j, 0);
    delay(600);
  }
  bodyH = STAND_CAD_H;
}

int legIndex(String s) {
  s.toUpperCase();
  for (int i = 0; i < 6; i++)
    if (s == LEG[i].name)
      return physLeg(i);
  return -1;
}

int jointIndex(const String &s) {
  char c = tolower(s.charAt(0));
  return c == 'c' ? 0 : c == 'f' ? 1 : c == 't' ? 2 : -1;
}

int split(const String &s, String tok[], int maxT) {
  int n = 0, i = 0, L = s.length();
  while (i < L && n < maxT) {
    while (i < L && s[i] == ' ')
      i++;
    int j = i;
    while (j < L && s[j] != ' ')
      j++;
    if (j > i)
      tok[n++] = s.substring(i, j);
    i = j;
  }
  return n;
}

void help() {
  say("Walk: w s a d q e, x stop, + - speed, [ ] height (saved), stand (saved height)");
  say("Assists: assist [status/on/off] | assist heading/terrain/landing/balance on/off | turn 90 / turn -45 / turn stop");
  say("Assist tuning: headkp 1.8 | turnrate 20 | roughgain 1 | landsoft .3 | guarddeg 27 | screen drive | save");
  say("Tune: gait tripod/ripple/wave | speed | height | lift | period | width | idle 5 | params | save");
  say("Gait math: com X Y | sagtest -> sag <mm> | lag <ms> | servospeed <deg/s>  (all saved with save)");
  say("Dynamics: dynamics monitor (default) | dynamics reset | dynamics on/off | dynamics limit 0.70 | dynamics (stopped report).");
  say("Cloud math: mathdump (settings JSON) | equations (status) | equations state (commanded state JSON) | equations bench (stopped).");
  say("Gait extras: autogait on/off | dutyadapt on/off | backlash <mm play at the foot>");
  say("Torque: mass <grams> | stall <kg*cm> (2.2 = MG90S datasheet, rated at 6.6 V) | torque (load per leg + best stance) | heat (servo load estimate)");
  say("Foot heights: zscale (kitchen scale, easiest) | zcal (paper test) | zfix on/off (correct them) | loads (standing load per leg)");
  say("Setup: dir | dir FL | flip FL f | trim FL f 20 | cal (servo scale) | show | swapsides | swapends | c | off | reset");
  say("Servo link: servostatus | servorecover (explicit release after a bus fault) | off");
  say("Battery: bat | batcal 7.84    MPU: tilt | motion | levelcal | mpufront | level on/off | i2c | i2cpins | i2cbb | mpustatus | mpu");
  say("Tricks: tricks (list all 24) | dance | wave | beg | headbang | ... | trick (random) | party | fidget on/off");
  say("Screen: screen (next) | screen face/dash/legs/trail/stab/heat/drive/level/sys/off | fps | scrtest | scrspeed 75 | scrrot | scrinv");
  say("Other: touch | mem | mem reset (timing counters) | bt | btforget | pcaclock | pcamon on/off | sleep");
}

bool nonfiniteLiteral(const char *text, bool numericContext = false) {
  char *end;
  float value = strtof(text, &end);
  return end != text && (!*end || numericContext) && !isfinite(value);
}

bool numericCommand(const String &c) {
  return c == "speed" || c == "height" || c == "lift" || c == "period" || c == "width" ||
         c == "idle" || c == "com" || c == "mass" || c == "stall" || c == "sag" || c == "lag" ||
         c == "servospeed" || c == "backlash" || c == "zscale" || c == "batcal";
}

// Sensor resets and calibration need a quiet robot; defer them while motion is still settling.
bool sensorMaintenanceIdle() {
  if (lastMove || stepping || trick != TR_NONE || centerActive || servoSettling || zcLeg >= 0 ||
      zsLeg >= 0 || park == P_LOWERING || ms64() < sagTestUntil)
    return false;
  if (mode == GAIT && servosOn && !servoHold && fabsf(walkHeight() - bodyH) > .5f)
    return false;
  return true;
}

bool trickGeometryCommand(const String &c) {
  return c == "width" || c == "com" || c == "swapends" || c == "swapsides" || c == "sagtest" ||
         c == "zfix" || c == "zcal" || c == "zscale";
}

bool sensorMaintenanceCommand(const String &c) {
  return c == "i2c" || c == "i2cpins" || c == "i2cbb" || c == "mpu" || c == "levelcal" ||
         c == "mpufront";
}

bool mountingCalibrationQuiet() {

  return !servosOn || (!levelOn && fabsf(levP) < .001f && fabsf(levR) < .001f);
}

// Typed USB and Bluetooth commands share this dispatcher.
void handle(String line) {
  ledFlickerUntil = ms64() + 80;
  lastActivity = millis();
  screenEvent(EV_CMD);
  String t[4];
  int n = split(line, t, 4);
  if (!n)
    return;
  String c = t[0];
  c.toLowerCase();
  if (trick != TR_NONE && trickGeometryCommand(c)) {
    say("Let the trick finish, or use x and wait for feet down before changing stance or foot calibration.");
    return;
  }
  if (sensorMaintenanceCommand(c) && !sensorMaintenanceIdle()) {
    say("Stop and let the feet/body settle before sensor maintenance or calibration.");
    return;
  }
  if ((c == "levelcal" || c == "mpufront") && !mountingCalibrationQuiet()) {
    say("Use level off and wait for the correction to fade before mounting calibration (or relax with off).");
    return;
  }
  for (int k = 1; k < n; k++)
    if (nonfiniteLiteral(t[k].c_str(), numericCommand(c))) {
      say("Command rejected: numbers must be finite (no NaN, infinity or overflow).");
      return;
    }
  if (c == "servostatus") {
    servoStatus();
    return;
  }
  if (c == "servorecover") {
    servoRecover();
    return;
  }
  if (walkCommand(c, n > 1 ? t[1] : String(""), n > 2 ? t[2] : String("")))
    return;

  if (c == "bat") {
    if (batV < BAT_PRESENT) {
      say("No battery detected (running on USB).");
      return;
    }
    say("Battery %.2f V at rest (%.2f V per cell), about %d%%%s", batRest(), batRest() / 2,
        batPct >= 0 ? batPct : batPercent(batRest()), batLocked ? "  [LOCKED: charge it]" : "");
    say("  now %.2f V under load %.1f | sag %.3f V per load unit (standing ~%.2f V, walking ~%.2f V)%s",
        batV, loadX, batKF.b, batKF.b * 3, batKF.b * 4.5f, batKF.init ? "" : " (learning)");
    return;
  }
  if (screenCommand(c, n > 2 ? t[1] + " " + t[2] : n > 1 ? t[1] : String("")))
    return;
  if (c == "show") {
    showTable();
    showParams();
    return;
  }
  if (c == "mem") {
    if (n > 1 && t[1] == "reset") {
      timingResetRequested = true;
      say("Timing counters will reset at the next control tick. Calibration and fault counters are kept.");
      return;
    }
    say("Free RAM %u bytes (largest block %u). Bluetooth client: %s. Last restart: %s, up %lu s",
        (unsigned)ESP.getFreeHeap(), (unsigned)ESP.getMaxAllocHeap(), BT.hasClient() ? "yes" : "no",
        resetWhy, millis() / 1000);
    say("Loop: worst gap %.0f ms, ticks over 30 ms %lu | IK misses %lu | slew guard hits %lu | support %s | failed board updates L %lu R %lu",
        worstGapMs, (unsigned long)overruns, (unsigned long)ikFailTicks, (unsigned long)slewHits,
        supportOK ? "OK" : "INVALID", (unsigned long)boardFails[0], (unsigned long)boardFails[1]);
    say("Stage peaks ms: input %.1f | centre %.1f | MPU %.1f | assist %.1f | IO %.1f",
        loopTiming.peakUs[LoopTiming::INPUT_SERVICE] * .001f,
        loopTiming.peakUs[LoopTiming::CENTER] * .001f, loopTiming.peakUs[LoopTiming::MPU] * .001f,
        loopTiming.peakUs[LoopTiming::ASSIST] * .001f, loopTiming.peakUs[LoopTiming::IO] * .001f);
    say("Stage peaks ms: feet/PCA %.1f | screen %.1f | housekeeping %.1f | tick work %.1f; work over 20 ms %lu",
        loopTiming.peakUs[LoopTiming::FEET] * .001f, loopTiming.peakUs[LoopTiming::SCREEN] * .001f,
        loopTiming.peakUs[LoopTiming::HOUSEKEEPING] * .001f, loopTiming.workPeakUs * .001f,
        (unsigned long)loopTiming.workOver20);
    if (linkDropped)
      say("Screen link: %lu status lines dropped (too long).", (unsigned long)linkDropped);
    return;
  }
  if (c == "bt") {
    btStatus();
    return;
  }
  if (c == "pcaclock") {
    for (int b = 0; b < 2; b++)
      say("PCA %s: %lu edges/s (want ~50), spread %lu us (want <600), noise edges %lu (want 0)%s | tick %.3f us (27 MHz basis, unchanged)",
          b == 0 ? "left" : "right", (unsigned long)syncRate[b], (unsigned long)syncSpread[b],
          (unsigned long)syncNoiseLast[b], pcaOscMeas[b] > 0 ? "" : "", pcaTickUs[b]);
    for (int b = 0; b < 2; b++)
      if (pcaOscMeas[b] > 0)
        say("  %s clock measured ~%.2f MHz (info only)", b == 0 ? "left" : "right",
            pcaOscMeas[b] / 1e6f);
    say(syncOK
            ? "Control loop synced to the left board's frames (%.2f Hz)."
            : "Not synced: normal 20 ms timing (needs a clean signal on D35). Last frame %.2f Hz",
        1.0f / framePeriodS);
    return;
  }
  if (c == "pcamon") {
    pcamonCommand(n > 1 ? t[1] : String(""));
    return;
  }
  if (c == "btforget") {
    BT.deleteAllBondedDevices();
    say("Bluetooth: forgot all paired PCs/phones. Now remove Hexapod in Windows too, then pair again.");
    return;
  }
  if (c == "mathdump") {
    mathDump();
    return;
  }
  if (c == "dynamics") {
    equationDynamicsCommand(n >= 2 ? t[1].c_str() : "", n >= 3 ? t[2].c_str() : nullptr);
    return;
  }
  if (c == "equations") {
    if (n >= 2 && t[1] == "bench")
      equationBenchmark();
    else if (n >= 2 && t[1] == "state")
      equationState();
    else
      equationStatus();
    return;
  }
  if (c == "params") {
    showParams();
    return;
  }
  if (c == "help" || c == "?") {
    help();
    return;
  }
  if (c == "mpustatus") {
    mpuStatus();
    return;
  }
  if (c == "mpulog") {
    String v = n > 1 ? t[1] : String("");
    v.toLowerCase();
    if (v == "on" || v == "off") {
      mpuLog = v == "on";
      esp_log_level_set("i2c.master", mpuLog ? ESP_LOG_ERROR : ESP_LOG_NONE);
    }
    say("I2C driver error logs %s (both buses; not saved). mpulog on / off", mpuLog ? "ON" : "off");
    return;
  }
  if (c == "i2c") {
    i2cScan(Wire, "Wire D27/D14 (servo boards, expect 0x40 0x60)");
    if (!imuOK) {
      int s0 = mpuSda, s1 = mpuScl;
      mpuSda = MPU_SDA;
      mpuScl = MPU_SCL;
      mpuBusBegin();
      i2cScan(Wire1, "Wire1 SDA D21 / SCL D22 (MPU, expect 0x68)");
      mpuSda = MPU_SCL;
      mpuScl = MPU_SDA;
      mpuBusBegin();
      i2cScan(Wire1, "Wire1 swapped: SDA D22 / SCL D21");
      mpuSda = s0;
      mpuScl = s1;
      mpuBusBegin();
    } else
      i2cScan(Wire1, "Wire1 (MPU)");
    say("MPU: %s, read errors %lu, dropouts %lu",
        imuOK     ? "OK"
        : imuLost ? "lost, retrying"
                  : "not found at boot",
        (unsigned long)imuErrors, (unsigned long)imuDrops);
    return;
  }
  if (c == "i2cpins") {

    Wire1.end();
    const int pins[2] = {MPU_SDA, MPU_SCL};
    const char *nm[2] = {"D21 (SDA)", "D22 (SCL)"};
    for (int k = 0; k < 2; k++) {
      pinMode(pins[k], INPUT_PULLDOWN);
      delay(5);
      int dn = digitalRead(pins[k]);
      pinMode(pins[k], INPUT_PULLUP);
      delay(5);
      int up = digitalRead(pins[k]);
      pinMode(pins[k], INPUT);
      if (dn && up)
        say("%s: connected to the MPU's pull-up. OK.", nm[k]);
      else if (!dn && up)
        say("%s: NOT connected (floating). Wire, breadboard row or the pin you're in is wrong.",
            nm[k]);
      else
        say("%s: stuck LOW even with pull-up: shorted to GND, or the MPU has no power.", nm[k]);
    }
    setupIMU();
    return;
  }
  if (c == "i2cbb") {
    Wire1.end();
    for (int k = 0; k < 2; k++) {
      int sda = k ? MPU_SCL : MPU_SDA, scl = k ? MPU_SDA : MPU_SCL;
      auto rel = [](int p) { pinMode(p, INPUT); };
      auto low = [](int p) {
        pinMode(p, OUTPUT);
        digitalWrite(p, LOW);
      };
      auto dly = []() { delayMicroseconds(8); };
      rel(sda);
      rel(scl);
      dly();

      for (int i = 0; i < 9 && !digitalRead(sda); i++) {
        low(scl);
        dly();
        rel(scl);
        dly();
      }
      bool idleOK = digitalRead(sda) && digitalRead(scl);
      String found;
      for (uint8_t ad = 8; ad < 120; ad++) {
        low(sda);
        dly();
        low(scl);
        dly();
        uint8_t byte = ad << 1;
        for (int bit = 7; bit >= 0; bit--) {
          if (byte & (1 << bit))
            rel(sda);
          else
            low(sda);
          dly();
          rel(scl);
          dly();
          low(scl);
          dly();
        }
        rel(sda);
        dly();
        rel(scl);
        dly();
        bool ack = !digitalRead(sda);
        low(scl);
        dly();
        low(sda);
        dly();
        rel(scl);
        dly();
        rel(sda);
        dly();
        if (ack) {
          char t[8];
          snprintf(t, sizeof t, " 0x%02X", ad);
          found += t;
        }
      }
      say("Hand-made scan, SDA D%d / SCL D%d: lines idle %s, answers:%s", sda, scl,
          idleOK ? "HIGH (good)" : "NOT HIGH (problem)", found.length() ? found.c_str() : " none");
    }
    setupIMU();
    return;
  }
  if (c == "mpu") {
    if (stepping || lastMove) {
      say("Stop and let all feet settle before restarting the MPU.");
      return;
    }
    say("Restarting the MPU6050...");
    imuLost = false;
    Wire1.end();
    setupIMU();
    return;
  }
  if (c == "motion") {
    if (!walkFresh()) {
      say("MPU motion unavailable: needs fresh reads, levelcal and mpufront.");
      return;
    }
    say("Measured linear accel: forward %+.3f, left %+.3f, body-up %+.3f m/s^2",
        imuLinearBody[0] * 9.81f, imuLinearBody[1] * 9.81f, imuLinearBody[2] * 9.81f);
    say("World vertical %+.3f m/s^2 | accel trust %.0f%% | relative heading %+.1f deg",
        imuLinearUp * 9.81f, imuAccelTrust * 100, wc::wrap(walkCtl.heading));
    say("Gravity removed using estimated tilt; offsets/vibration remain. This is acceleration, not position.");
    return;
  }
  if (c == "tilt") {
    if (!imuOK) {
      say(imuLost ? "MPU6050 lost, retrying every 3 s (type i2c)."
                  : "No MPU6050 (type i2c to see what answers).");
      return;
    }
    say("Tilt: roll %+.1f (left up +), pitch %+.1f (nose up +)%s", viewRoll(), viewPitch(),
        hasFront ? "" : "  [not set up yet]");
    if (hasLevel && hasFront && mode == GAIT)
      say("Tip margin (energy, worst of the last second): %.1f mm  (flat tripod ~22, standing ~69; under 6 it slows down)",
          nesmLast);
    say("Gyro: drift being cancelled %.2f %.2f deg/s (learned); re-zeroed while standing still %lu times",
        -upF.bI[0] * RAD_TO_DEG, -upF.bI[1] * RAD_TO_DEG, (unsigned long)gyroRezeros);
    return;
  }
  if (c == "touch") {
    if (USE_BUTTON) {
      say("Button mode: %s", digitalRead(TOUCH_PIN) == LOW ? "pressed" : "not pressed");
      return;
    }
    say("Touch pad: now %d, untouched %d (touched below %d)", (int)touchRead(TOUCH_PIN),
        (int)touchBase, (int)(touchBase * 0.7f));
    return;
  }
  if (c == "off") {
    centerActive = false;
    servoSettling = false;
    mode = MANUAL;
    stopNow();
    stepping = false;
    relaxAll();
    say("Relaxed.");
    return;
  }
  if (c == "x") {
    centerActive = false;
    stopMoving();
    trickReturn();
    say(trick != TR_NONE ? "Stopping: landing feet, then returning to neutral." : "Stopping.");
    return;
  }
  if (c == "sleep") {
    deepSleep("asked");
    return;
  }
  if (c == "batcal" && n >= 2) {
    String a = t[1];
    a.toLowerCase();
    if (a == "reset") {
      batScale = 14.7f / 4.7f;
      batOffset = 0;
      prefs.remove("bp1");
      prefs.putFloat("batscale", batScale);
      prefs.putFloat("batoff", 0);
      say("Battery calibration reset.");
      return;
    }
    float real = a.toFloat(), pin = readPinVolts();
    if (real < 5 || real > 9 || pin < 0.5f) {
      say("Usage: batcal 7.84 (multimeter reading of the battery), batcal reset");
      return;
    }
    float p1 = prefs.getFloat("bp1", 0), r1 = prefs.getFloat("br1", 0);
    if (p1 > 0.5f && fabsf(real - r1) >= 0.4f && fabsf(pin - p1) > 0.05f) {
      batScale = (real - r1) / (pin - p1);
      batOffset = real - batScale * pin;
      say("Battery calibrated from 2 points (%.2f V and %.2f V): scale %.3f, offset %+.3f V. Saved.",
          r1, real, batScale, batOffset);
    } else {
      batScale = real / pin;
      batOffset = 0;
      say("Battery calibrated at %.2f V (saved). For best accuracy, run batcal again later at a voltage ~0.5 V+ different.",
          real);
    }
    prefs.putFloat("bp1", pin);
    prefs.putFloat("br1", real);
    prefs.putFloat("batscale", batScale);
    prefs.putFloat("batoff", batOffset);
    batV = real;
    batKF.reset();
    return;
  }

  if (!servoOK())
    return;
  if (c != "levelcal" && c != "mpufront")
    unpark();
  if (trickCommand(c, n > 1 ? t[1] : String("")))
    return;

  if (c.length() == 1 && strchr("wsadqe", c[0])) {
    if (!calibrated) {
      say("Walking locked: run dir, fix joints with flip, then save.");
      return;
    }
    if (!walkResume())
      return;
    walkCtl.cancel(wc::OVERRIDE);
    lastMove = c[0];
    applyMove();
    say("Moving (%c) at %d mm/s.", lastMove, (int)speed);
    return;
  }
  if (c == "stand") {

    if (walkCtl.recovery && mode != GAIT) {
      sitting = false;
      targetH = savedHeight();
      startGait();
    }
    if (!walkResume())
      return;
    stopMoving();
    trickReturn();
    sitting = false;
    targetH = savedHeight();
    startGait();
    say("Standing at your saved height (%d mm).", (int)targetH);
    return;
  }
  if (c == "+" || c == "-") {
    speed = constrain(speed + (c == "+" ? 10 : -10), 5.0f, maxSpeed());
    applyMove();
    say("Speed %d mm/s.", (int)speed);
    return;
  }
  if (c == "[" || c == "]") {
    trickReturn();
    targetH = constrain(targetH + (c == "]" ? 5 : -5), 40.0f, 85.0f);
    sitting = false;
    startGait();
    saveHeight();
    say("Body height %d mm (saved).", (int)targetH);
    return;
  }
  if (c == "c") {
    centerAll();
    return;
  }
  if (c == "c90") {
    mode = MANUAL;
    stopNow();
    stepping = false;
    park = P_NONE;
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++) {
        writeUs(i, j, 1500);
        delay(60);
      }
    servosOn = true;
    bodyH = STAND_CAD_H;
    say("Every servo at exactly 1500 us (90 deg), trims ignored. Use this to check / re-seat horns. c = centre WITH your trims.");
    return;
  }
  if (c == "dir") {
    if (n > 1) {
      int i = legIndex(t[1]);
      if (i >= 0)
        dirTest(i);
      else
        say("Unknown leg (FL ML RL FR MR RR).");
    } else
      for (int n = 0; n < 6; n++)
        dirTest(physLeg(n));
    say("Wrong ones: flip <leg> <c/f/t>, then dir <leg>. All right: save. Then: stand.");
    return;
  }
  if (c == "flip" && n >= 3) {
    int i = legIndex(t[1]), j = jointIndex(t[2]);
    if (i < 0 || j < 0) {
      say("Usage: flip FL f");
      return;
    }
    sgn[i][j] = -sgn[i][j];
    calibrated = false;
    say("%s %s direction now %+d (not saved).", legName(i), JOINT[j], sgn[i][j]);
    return;
  }
  if (c == "trim" && n >= 2 && t[1] == "zero") {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++) {
        trim[i][j] = 0;
        gainP[i][j] = gainN[i][j] = 1.0f;
      }
    if (mode == MANUAL)
      for (int i = 0; i < 6; i++)
        for (int j = 0; j < 3; j++)
          writeAngle(i, j, 0);
    say("All trims 0 and all scales 1.00 (your original calibration). Type save to keep it.");
    return;
  }
  if (c == "trim" && n >= 4) {
    int i = legIndex(t[1]), j = jointIndex(t[2]);
    if (i < 0 || j < 0) {
      say("Usage: trim FL f 20");
      return;
    }
    trim[i][j] = constrain(trim[i][j] + t[3].toInt(), -300, 300);
    if (mode == MANUAL)
      writeAngle(i, j, 0);
    say("%s %s trim now %+d us (not saved).", legName(i), JOINT[j], trim[i][j]);
    return;
  }

  if (c == "cal" || c == "adj") {
    static int cLeg = -1, cJ = -1, cPt = 0, cAdj = 0;
    static float cA = 40;
    auto angOf = [&]() { return cPt == 0 ? 0.0f : cPt == 1 ? cA : -cA; };
    auto show = [&]() {
      float ang = angOf();
      float g = (ang >= 0 ? gainP : gainN)[cLeg][cJ];
      int us = 1500 + trim[cLeg][cJ] + (int)lroundf(sgn[cLeg][cJ] * ang * US_PER_DEG * g) + cAdj;
      writeUs(cLeg, cJ, us);
      servosOn = true;
      say("%s %s step %d/3: at %+.0f deg (nudge %+d us). Line it up with the %s mark: adj 10 / adj -10, then %s",
          legName(cLeg), JOINT[cJ], cPt + 1, ang, cAdj,
          cPt == 0   ? "0"
          : cPt == 1 ? "+"
                     : "-",
          cPt < 2 ? "cal next" : "cal done");
    };
    auto commit = [&]() {
      float k = US_PER_DEG, s0 = sgn[cLeg][cJ];
      if (cPt == 0)
        trim[cLeg][cJ] = constrain(trim[cLeg][cJ] + cAdj, -300, 300);
      else if (cPt == 1)
        gainP[cLeg][cJ] = constrain(gainP[cLeg][cJ] + s0 * cAdj / (cA * k), 0.7f, 1.3f);
      else
        gainN[cLeg][cJ] = constrain(gainN[cLeg][cJ] - s0 * cAdj / (cA * k), 0.7f, 1.3f);
      cAdj = 0;
    };
    String a1 = n > 1 ? t[1] : String("");
    a1.toLowerCase();
    if (c == "adj") {
      if (cLeg < 0) {
        say("Start first: cal FL f");
        return;
      }
      cAdj = constrain(cAdj + (int)a1.toInt(), -250, 250);
      show();
      return;
    }
    if (a1 == "") {
      say("Servo calibration (robot LIFTED, legs free): cal FL f -> centre: adj -> cal next -> + mark: adj -> cal next -> - mark: adj -> cal done -> save");
      say("Coxa uses +-30 deg (angle card under the leg), femur / tibia +-40 deg (card or a phone level app). cal cancel to quit.");
      String g = "Scales +/-:";
      for (int k2 = 0; k2 < 6; k2++) {
        int i = physLeg(k2);
        char b[64];
        snprintf(b, sizeof b, " %s %.2f/%.2f %.2f/%.2f %.2f/%.2f", LEG[k2].name, gainP[i][0],
                 gainN[i][0], gainP[i][1], gainN[i][1], gainP[i][2], gainN[i][2]);
        g += b;
      }
      say("%s", g.c_str());
      return;
    }
    if (a1 == "cancel") {
      if (cLeg >= 0)
        writeAngle(cLeg, cJ, 0);
      cLeg = -1;
      say("Calibration stopped (finished steps are kept, not saved).");
      return;
    }
    if (a1 == "next") {
      if (cLeg < 0) {
        say("Start first: cal FL f");
        return;
      }
      if (cPt >= 2) {
        say("Last step: type cal done");
        return;
      }
      commit();
      cPt++;
      show();
      return;
    }
    if (a1 == "done") {
      if (cLeg < 0) {
        say("Start first: cal FL f");
        return;
      }
      if (cPt < 2) {
        say("Finish all 3 marks first (cal next).");
        return;
      }
      commit();
      writeAngle(cLeg, cJ, 0);
      say("%s %s: trim %+d us, scale %.3f (+) / %.3f (-). Not saved yet: type save. Next: cal <leg> <c/f/t>",
          legName(cLeg), JOINT[cJ], trim[cLeg][cJ], gainP[cLeg][cJ], gainN[cLeg][cJ]);
      cLeg = -1;
      return;
    }
    if (n >= 3) {
      int i = legIndex(t[1]), j = jointIndex(t[2]);
      if (i < 0 || j < 0) {
        say("Usage: cal FL f");
        return;
      }
      mode = MANUAL;
      stopNow();
      stepping = false;
      park = P_NONE;
      for (int a = 0; a < 6; a++)
        for (int b = 0; b < 3; b++)
          writeAngle(a, b, 0);
      cLeg = i;
      cJ = j;
      cPt = 0;
      cAdj = 0;
      cA = j == 0 ? 30 : 40;
      delay(300);
      show();
      return;
    }
    say("Usage: cal FL f  (or just cal for help)");
    return;
  }
  if (c == "swapends") {
    stopMoving();
    endsSwapped = !endsSwapped;
    prefs.putBool("ends", endsSwapped);
    say(endsSwapped ? "Ends swapped (saved): the old REAR is now the front. w walks that way."
                    : "Ends back to normal (saved): original front.");
    say("Leg names in dir, flip, trim and show now follow the new front too.");
    return;
  }
  if (c == "swapsides") {
    sidesSwapped = !sidesSwapped;
    for (int i = 0; i < 3; i++)
      for (int j = 0; j < 3; j++) {
        int8_t sg = sgn[i][j];
        sgn[i][j] = sgn[i + 3][j];
        sgn[i + 3][j] = sg;
        int16_t tr = trim[i][j];
        trim[i][j] = trim[i + 3][j];
        trim[i + 3][j] = tr;
        float gp = gainP[i][j];
        gainP[i][j] = gainP[i + 3][j];
        gainP[i + 3][j] = gp;
        float gn = gainN[i][j];
        gainN[i][j] = gainN[i + 3][j];
        gainN[i + 3][j] = gn;
      }
    for (int i = 0; i < 6; i++)
      sgn[i][0] = -sgn[i][0];
    prefs.putBool("swap", sidesSwapped);
    prefs.putBytes("sgn", sgn, sizeof sgn);
    prefs.putBytes("trim", trim, sizeof trim);
    prefs.putBytes("gainP", gainP, sizeof gainP);
    prefs.putBytes("gainN", gainN, sizeof gainN);
    say("Sides swapped and saved. Check with: dir FL (robot's front-left leg).");
    return;
  }
  if (c == "bench" && pcamonActive()) {
    say("bench uses channel 15 and D35 like pcamon: type pcamon off first.");
    return;
  }
  if (c == "bench") {

    const int BP = 35, BCH = 15;
    auto put = [&](int us) { pcaL.setPWM(BCH, 0, (uint16_t)lroundf(us / pcaTickUs[0])); };
    auto rd = [&](int nAvg) {
      uint32_t m = 0;
      for (int k = 0; k < nAvg; k++)
        m += analogReadMilliVolts(BP);
      return (float)m / nAvg;
    };
    String a = n > 1 ? t[1] : String("");
    a.toLowerCase();
    if (a == "") {
      say("bench: measures a spare servo for real. Spare servo -> left board channel 15 (normal plug).");
      say("Open it, solder a thin wire to the MIDDLE pin of its potentiometer -> 47k -> GPIO35, plus 47k from GPIO35 to GND and 100 nF from GPIO35 to GND.");
      say("Then: bench test (speed, lag, deadband, ~10 s)  |  bench play (gear backlash: you wiggle the horn for 5 s)");
      return;
    }
    if (stepping) {
      say("Stand still first.");
      return;
    }
    put(1500);
    delay(900);
    float v15 = rd(32);
    put(1000);
    delay(900);
    float v10 = rd(32);
    put(2000);
    delay(900);
    float v20 = rd(32);
    put(1500);
    delay(700);
    float span = v20 - v10, k = span / 1000.0f;
    if (fabsf(span) < 200) {
      say("No pot signal (1000us %.0f mV, 2000us %.0f mV). Check the wiper wire, divider and GPIO35.",
          v10, v20);
      return;
    }
    say("Pot: 1000us %.0f mV, 1500us %.0f mV, 2000us %.0f mV (%.2f mV per us)%s", v10, v15, v20, k,
        fabsf((v15 - v10) - (v20 - v15)) > 0.15f * fabsf(span)
            ? "  [not very linear: servo or pot worn?]"
            : "");
    if (a == "test") {

      put(1000);
      delay(900);
      float v0 = rd(32);
      uint32_t t0 = micros();
      put(2000);
      float t10 = -1, t50 = -1, t90 = -1;
      while (micros() - t0 < 900000) {
        float v = rd(2), f = (v - v0) / span;
        float tm = (micros() - t0) / 1000.0f;
        if (t10 < 0 && f > 0.1f)
          t10 = tm;
        if (t50 < 0 && f > 0.5f)
          t50 = tm;
        if (t90 < 0 && f > 0.9f) {
          t90 = tm;
          break;
        }
      }
      if (t90 < 0) {
        say("Servo didn't reach the end in 0.9 s: stuck, or the supply is weak.");
        put(1500);
        return;
      }
      float dps = 0.8f * (1000.0f / US_PER_DEG) / ((t90 - t10) / 1000.0f);

      put(1500);
      delay(800);
      float base = rd(64), nz = 0;
      for (int q = 0; q < 32; q++)
        nz = fmaxf(nz, fabsf(rd(4) - base));
      int db = -1;
      for (int d = 1; d <= 25; d++) {
        put(1500 + d);
        delay(150);
        if (fabsf(rd(16) - base) > fmaxf(2 * nz, 3 * fabsf(k))) {
          db = d;
          break;
        }
      }
      put(1500);
      say("Servo (no load): 90 deg in ~%.0f ms -> top speed ~%.0f deg/s, starts moving after ~%.0f ms, deadband %s.",
          t90 - t10, dps, t10,
          db > 0 ? (String(db) + " us (" + String(db / US_PER_DEG, 1) + " deg)").c_str()
                 : "> 25 us");
      float dead = t10 - (t90 - t10) / 8;
      bool ramp = fabsf(t50 - 0.5f * (t10 + t90)) < 0.15f * (t90 - t10);
      if (!ramp)
        say("Note: t50 %.0f ms is not midway between t10 and t90 (%.0f): the move isn't a clean ramp, lag is rough.",
            t50, 0.5f * (t10 + t90));
      if (dead < 0)
        say("Note: dead time came out negative (%.0f ms): the ramp model doesn't fit this servo; using 0.",
            dead);
      say("Suggested: servospeed %.0f (80%% of no-load, legs carry weight)  |  lag %.0f (dead time)  -> then save",
          0.8f * dps, constrain(dead, 0.0f, 40.0f));
      return;
    }
    if (a == "play") {
      put(1500);
      delay(600);
      say("Wiggle the horn GENTLY back and forth (just until it stops moving freely) for 5 s... go!");
      float mn = 1e9f, mx = -1e9f;
      uint32_t t0 = millis();
      while (millis() - t0 < 5000) {
        float v = rd(4);
        mn = fminf(mn, v);
        mx = fmaxf(mx, v);
        delay(2);
      }
      float deg = (mx - mn) / fabsf(k) / US_PER_DEG, foot = deg * DEG_TO_RAD * FEMUR;
      say("Gear play: %.1f deg total -> %.1f mm at the foot (femur lever). Suggested: backlash %.1f  -> then save",
          deg, foot, foot);
      return;
    }
    say("Usage: bench | bench test | bench play");
    return;
  }
  if (c == "autogait" && n >= 2) {
    String v = t[1];
    v.toLowerCase();
    autoGait = (v == "on");
    say(autoGait
            ? "Autogait ON: wave when slow, ripple in between, tripod fast (switches smoothly). save to keep."
            : "Autogait off. save to keep.");
    return;
  }
  if (c == "dutyadapt" && n >= 2) {
    String v = t[1];
    v.toLowerCase();
    dutyAdapt = (v != "off");
    say(dutyAdapt ? "Speed-adaptive duty ON: more feet down when walking slowly. save to keep."
                  : "Speed-adaptive duty off (fixed duty per gait). save to keep.");
    return;
  }
  if (c == "backlash" && n >= 2) {
    blPlay = constrain(t[1].toFloat(), 0.0f, 4.0f);
    say(blPlay > 0 ? "Backlash: lifted feet raised %.1f mm to cancel the gear play. save to keep."
                   : "Backlash compensation off. save to keep.",
        blPlay);
    return;
  }
  if (c == "gait" && n >= 2) {
    String g = t[1];
    g.toLowerCase();
    if (autoGait) {
      autoGait = false;
      say("(autogait off: you picked a gait)");
    }
    for (int k = 0; k < 3; k++)
      if (g == GAIT_NAME[k] || g == String(k + 1)) {
        preferredGait = k;
        k = max(k, walkCtl.roughLevel);
        if (k == gait && pendingGait < 0) {
          say("Already %s.", GAIT_NAME[k]);
          return;
        }
        if (stepping) {
          pendingGait = k;
          say("Switching to %s at the next safe moment...", GAIT_NAME[k]);
          return;
        }
        applyGait(k);
        showParams();
        return;
      }
    say("Gaits: tripod, ripple, wave");
    return;
  }
  if (c == "com" && n >= 3) {
    comX = constrain(t[1].toFloat(), -30.0f, 30.0f);
    comY = constrain(t[2].toFloat(), -30.0f, 30.0f);
    updateStride(true);
    restep();
    say("Footholds now centred on CoM %+.0f / %+.0f mm (save to keep).", comX, comY);
    return;
  }
  if (c == "sagtest") {
    if (mode != GAIT || stepping) {
      say("Stand still first.");
      return;
    }
    sagTestUntil = ms64() + 4000;
    say("Lifting FL RL MR for 4 s: measure how many mm the body sinks at the MIDDLE of the LEFT side and of the RIGHT side; average them, then: sag <mm>");
    if (sagFull > 0)
      say("(Sag compensation stays on during the test; the math accounts for it. If the body RISES, enter a negative number.)");
    return;
  }
  if (c == "sag" && n >= 2) {
    if (!sensorMaintenanceIdle()) {
      say("Stop and let the feet/body settle before sag calibration.");
      return;
    }
    float d = constrain(t[1].toFloat(), -10.0f, 10.0f);
    if (d == 0 || (d < 0 && sagFull <= 0)) {
      sagFull = 0;
      say("Sag feed-forward off. save to keep.");
      return;
    }
    float h = constrain(targetH, 40.0f, 85.0f);
    float S = sagFromDrop(d, sagFull, zResid(), width, comX, comY, h);
    if (!isfinite(S)) {
      say("Sag measurement has no unique valid model fit; previous compensation kept. Check the reading/foot contacts, or use sag 0 to disable.");
      return;
    }
    sagFull = constrain(S, 0.0f, 20.0f);
    say("Sag feed-forward: %.1f mm per leg at a third of the weight (%s). save to keep.", sagFull,
        sagFull > 0 ? "on" : "off");
    return;
  }
  if (c == "mass" && n >= 2) {
    massG = constrain(t[1].toFloat(), 0.0f, 3000.0f);
    updateStride(true);
    if (massG > 0)
      say("Robot weight %.0f g: torque limit on. Stride now %d mm. save to keep, torque for details.",
          massG, (int)strideMax);
    else
      say("Torque limit off. save to keep.");
    return;
  }
  if (c == "stall" && n >= 2) {
    stallKgcm = constrain(t[1].toFloat(), 0.5f, 20.0f);
    updateStride(true);
    say("Servo stall torque %.2f kg*cm (limits: stand %.2f, stride %.2f). save to keep.", stallKgcm,
        stallKgcm * TAU_SOFT, stallKgcm * TAU_HARD);
    return;
  }
  if (c == "loads") {
    float F[6];
    standLoads(zResid(), -1, 0, F);
    String row = "";
    for (int n2 = 0; n2 < 6; n2++) {
      int i = physLeg(n2);
      char b[20];
      snprintf(b, sizeof b, "  %s %2.0f%%", LEG[n2].name, F[i] * 100);
      row += b;
    }
    say("Standing load per leg (model, ideal 16.7%% each when centred):%s", row.c_str());
    if (!supportOK)
      say("Warning: right now the CoM (projected along gravity) is outside the support: no static equilibrium.");
    if (!planFeasible)
      say("Warning: in this gait the CoM leaves the support of some foot set (no equilibrium): move the CoM or widen.");
    say("Height errors (mm, + = sits low, presses harder)%s:  FL %+.1f ML %+.1f RL %+.1f FR %+.1f MR %+.1f RR %+.1f | leg flex %.1f mm%s",
        zFix ? " [corrected: zfix on]" : "", zErr[physLeg(0)], zErr[physLeg(1)], zErr[physLeg(2)],
        zErr[physLeg(3)], zErr[physLeg(4)], zErr[physLeg(5)], sagModel(),
        sagFull > 0 ? "" : " (assumed: run sagtest)");
    return;
  }
  if (c == "zfix" && n >= 2) {
    String v = t[1];
    v.toLowerCase();
    zFix = (v == "on");
    updateStride(true);
    say(zFix ? "Foot height errors corrected (each foot moved by its zcal error). save to keep."
             : "Foot height correction off. save to keep.");
    return;
  }
  if (c == "zscale") {
    String a = n > 1 ? t[1] : String("");
    a.toLowerCase();
    if (a == "") {
      say("zscale: finds which feet sit low / high by weighing each foot on your kitchen scale. Robot standing on a flat table.");
      say("Once: measure the scale's platform height (table to top, mm): zscale h 18.5   Then for each foot (zeroed scale, nothing on it):");
      say("  zscale FL  (foot lifts)  -> slide the scale under it, only that foot on it  -> zscale down  -> read the grams -> zscale 152");
      say("Do all 6, pulling the scale out after each reading, then: zscale done   (zscale reset clears)");
      return;
    }
    if (a == "h" && n > 2) {
      zsH = constrain(t[2].toFloat(), 3.0f, 60.0f);
      prefs.putFloat("zscaleh", zsH);
      say("Scale platform height %.1f mm (saved). A few tenths off is fine: it works that out.",
          zsH);
      return;
    }
    if (a == "reset") {
      for (int i = 0; i < 6; i++)
        zsHave[i] = false;
      say("zscale readings cleared.");
      return;
    }
    if (a == "down") {
      if (zsLeg < 0 || zsPhase != 1) {
        say("Start with: zscale FL");
        return;
      }
      zsPhase = 2;
      zsTarget = zsH;
      say("%s lowering onto the scale. Wait 2-3 s for the number to settle, then type it: zscale <grams>",
          legName(zsLeg));
      return;
    }
    if (a == "done") {
      for (int i = 0; i < 6; i++)
        if (!zsHave[i]) {
          say("Still need %s (zscale %s).", legName(i), legName(i));
          return;
        }
      if (zsLeg >= 0) {
        zsTarget = 0;
        zsNext = -1;
      }
      if (massG <= 0) {
        say("Set the robot's weight first: mass <grams>");
        return;
      }
      float meas[6], fx[6], fy[6], e[6], fit[6], d, sum = 0;
      for (int i = 0; i < 6; i++) {
        meas[i] = zsGrams[i] / massG;
        sum += zsGrams[i];
        fx[i] = feet[i][0];
        fy[i] = feet[i][1];
      }
      float rms = fitFootLoads(fx, fy, meas, standLoads, e, d, fit);
      if (rms < 0) {
        say("Couldn't fit those readings. zscale reset and try again.");
        return;
      }
      float base[6];
      for (int i = 0; i < 6; i++)
        base[i] = zFix ? zErr[i] : 0;
      float F0[6], F1[6], z0[6] = {0};
      standLoads(e, -1, 0, F0);
      for (int i = 0; i < 6; i++) {
        zErr[i] = base[i] + e[i];
        z0[i] = 0;
      }
      zFix = true;
      updateStride(true);
      standLoads(z0, -1, 0, F1);
      String row = "";
      for (int n2 = 0; n2 < 6; n2++) {
        int i = physLeg(n2);
        char b[56];
        snprintf(b, sizeof b, "  %s %+.2f mm (%2.0f%% -> %2.0f%%)", LEG[n2].name, e[i], F0[i] * 100,
                 F1[i] * 100);
        row += b;
      }
      say("Foot height errors (+ = sat low) and load before -> after correction:%s", row.c_str());
      say("Readings vs model: %.1f g RMS%s | scale platform really ~%.1f mm | readings add up to %.0f g. zfix is ON now: save to keep.",
          rms * massG, rms * massG > 6 ? " (noisy: zscale reset and redo)" : " (good)", zsH - d,
          sum);
      return;
    }
    if (isDigit(a[0]) || a[0] == '.') {
      if (zsLeg < 0 || zsPhase != 2) {
        say("No foot on the scale (zscale FL, then zscale down).");
        return;
      }
      zsGrams[zsLeg] = a.toFloat();
      zsHave[zsLeg] = true;
      zsPhase = 1;
      zsTarget = zsH + 12;
      int left = 0;
      for (int i = 0; i < 6; i++)
        left += !zsHave[i];
      say("%s: %.0f g. Lifting: pull the scale out, then %s", legName(zsLeg), zsGrams[zsLeg],
          left ? "zscale <next foot>" : "zscale done");
      return;
    }
    int i = legIndex(a);
    if (i < 0) {
      say("Usage: zscale FL | zscale down | zscale <grams> | zscale done | zscale h 18.5");
      return;
    }
    if (mode != GAIT || stepping || trick != TR_NONE || park != P_NONE || sitting) {
      say("Stand still first (stand).");
      return;
    }
    if (zsLeg >= 0 && zsPhase == 2) {
      say("%s is still on the scale: type its grams first.", legName(zsLeg));
      return;
    }
    if (zsLeg >= 0) {
      zsTarget = 0;
      zsNext = i;
      say("Putting %s back down (scale out?), then lifting %s.", legName(zsLeg), legName(i));
    } else {
      zsLeg = i;
      zsPhase = 1;
      zsTarget = zsH + 12;
    }
    say("%s lifting %.0f mm: slide the zeroed scale under it (only that foot on the platform), then: zscale down",
        legName(i), zsH + 12);
    return;
  }
  if (c == "zcal") {
    String a = n > 1 ? t[1] : String("");
    a.toLowerCase();
    if (a == "") {
      say("zcal: finds which feet sit too low / high (a low foot carries more weight). Needs a flat table, robot standing.");
      say("For each foot: slide a strip of paper under it, type  zcal FL  (it rises 0.5 mm/s), tug the paper gently, type  zcal ok  the moment it slides free.");
      say("Do all 6 (twice each is better), then  zcal done . zcal reset clears. Run sagtest first for the best numbers.");
      return;
    }
    if (a == "reset") {
      for (int i = 0; i < 6; i++) {
        zcSum[i] = 0;
        zcN[i] = 0;
      }
      say("zcal readings cleared.");
      return;
    }
    if (a == "ok") {
      if (zcLeg < 0 || !zcUp) {
        say("Nothing rising. Start with: zcal FL");
        return;
      }
      zcSum[zcLeg] += zcLift;
      zcN[zcLeg]++;
      zcUp = false;
      say("%s released at %.2f mm (%d reading%s, average %.2f). Lowering.", legName(zcLeg), zcLift,
          (int)zcN[zcLeg], zcN[zcLeg] > 1 ? "s" : "", zcSum[zcLeg] / zcN[zcLeg]);
      return;
    }
    if (a == "done") {
      for (int i = 0; i < 6; i++)
        if (zcN[i] < 1) {
          say("Still need %s (zcal %s).", legName(i), legName(i));
          return;
        }
      if (zFix) {
        say("Turn zfix off first (measure the uncorrected feet), then zcal done again.");
        return;
      }
      float u[6], fx[6], fy[6], e[6], uf[6];
      for (int i = 0; i < 6; i++) {
        u[i] = zcSum[i] / zcN[i];
        fx[i] = feet[i][0];
        fy[i] = feet[i][1];
      }
      float rms = fitFootErrors(fx, fy, u, standLoads, e, uf);
      if (rms < 0) {
        say("Couldn't fit (odd readings). zcal reset and try again.");
        return;
      }
      float F0[6], F1[6], z0[6] = {0};
      standLoads(z0, -1, 0, F0);
      standLoads(e, -1, 0, F1);
      memcpy(zErr, e, sizeof zErr);
      updateStride(true);
      String row = "";
      for (int n2 = 0; n2 < 6; n2++) {
        int i = physLeg(n2);
        char b[48];
        snprintf(b, sizeof b, "  %s %+.2f mm (%2.0f%%)", LEG[n2].name, e[i], F1[i] * 100);
        row += b;
      }
      say("Foot height errors (+ = sits low) and the load each foot really carries standing:%s",
          row.c_str());
      say("Readings vs model: %.2f mm RMS%s. zfix on to correct them, then save.", rms,
          rms > 0.4f ? " (noisy: zcal reset, take 2 readings per foot)" : " (good)");
      return;
    }
    int i = legIndex(a);
    if (i < 0) {
      say("Usage: zcal FL | zcal ok | zcal done | zcal reset");
      return;
    }
    if (mode != GAIT || stepping || trick != TR_NONE || park != P_NONE || sitting) {
      say("Stand still first (stand).");
      return;
    }
    if (zcLeg >= 0) {
      say("Busy with %s (zcal ok, or wait for it to lower).", legName(zcLeg));
      return;
    }
    {
      float z0[6] = {0}, F0[6], Fa[6];
      standLoads(z0, -1, 0, F0);
      standLoads(z0, i, 0.05f, Fa);
      float S = (F0[i] - Fa[i]) / 0.05f;
      zcExpect = S > 1e-6f ? constrain(F0[i] / S, 1.0f, 15.0f) : 3.0f;
      zcMax = constrain(2 * zcExpect + 4, 8.0f, 25.0f);
    }
    zcLeg = i;
    zcLift = 0;
    zcUp = true;
    say("%s rising (fast to %.1f mm, then 0.5 mm/s). Expect release around %.1f mm if the foot is right; tug the paper, zcal ok when it slides free.",
        legName(i), 0.5f * zcExpect, zcExpect);
    return;
  }
  if (c == "heat") {
    if (massG <= 0) {
      say("Needs the robot's weight first: mass <grams>");
      return;
    }
    say("Estimated servo load (RMS over ~90 s, %% of stall; under 35 cool, 35-55 warm, over 55 hot):");
    String row = "";
    for (int n2 = 0; n2 < 6; n2++) {
      int i = physLeg(n2);
      char b[40];
      snprintf(b, sizeof b, "  %s femur %3d tibia %3d", LEG[n2].name, heatPct(i, 0), heatPct(i, 1));
      row += b;
    }
    say("%s", row.c_str());
    say("A model from the weight, the load split and the leg angles, not a sensor: compare legs, don't trust the absolute number too much.");
    return;
  }
  if (c == "torque") {
    if (massG <= 0) {
      say("Weigh the robot first (kitchen scale, battery in), then: mass <grams>");
      return;
    }
    if (stepping) {
      say("Stand still first.");
      return;
    }
    updateStride(true);
    float stallN = stallKgcm * 98.0665f, worstNow = 0;
    say("Torque standing in %s at height %d, width %+d (peak load share, femur / tibia as %% of stall):",
        GAIT_NAME[gait], (int)targetH, (int)width);
    String row = "";
    for (int n2 = 0; n2 < 6; n2++) {
      int i = physLeg(n2);
      char b[48];
      snprintf(b, sizeof b, "  %s %2.0f%%: %3.0f / %3.0f", LEG[n2].name, legShare[i] * 100,
               tauStand[i][0] / stallN * 100, tauStand[i][1] / stallN * 100);
      row += b;
      worstNow = fmaxf(worstNow, fmaxf(tauStand[i][0], tauStand[i][1]));
    }
    say("%s", row.c_str());
    float pct = worstNow / stallN * 100;
    say("Worst %.0f%% of stall: %s", pct,
        pct <= TAU_SOFT * 100   ? "good (under 35%)."
        : pct <= TAU_HARD * 100 ? "OK for walking, warm servos over time (35-55%)."
                                : "too high: servos near stall, expect heat / sag / shaking.");

    float h = constrain(targetH, 40.0f, 85.0f),
          m0 = supportMargin(GAIT_OFF[gait], DUTY, 0, comX, comY), pick = 99;
    say("width | worst torque | usable stride | tipping margin   (height %d, %s)", (int)h,
        GAIT_NAME[gait]);
    for (float ww = 10; ww >= -30.1f; ww -= 5) {
      float sh[6], wt = 0;
      peakShares(GAIT_OFF[gait], DUTY, ww, comX, comY, sh, h, zResid(), sagModel());
      for (int i = 0; i < 6; i++) {
        float nx, ny, q[3], r, tf, tt;
        neutralFoot(i, ww, comX, comY, nx, ny);
        if (!ikRaw(i, nx, ny, -h, q, &r)) {
          wt = 1e9f;
          break;
        }
        torqueFromIK(massG * 0.00981f * sh[i], r, q[1], tf, tt);
        wt = fmaxf(wt, fmaxf(tf, tt));
      }
      float st = constrain(strideBudget(h, ww, comX, comY) - 32, 0.0f, 70.0f),
            mg = supportMargin(GAIT_OFF[gait], DUTY, ww, comX, comY);
      bool ok = st >= 50 && mg >= 0.75f * m0 && wt < 1e8f;
      if (ok)
        pick = ww;
      say("%+4d  | %4.0f%% of stall | %3.0f mm | %3.0f mm%s", (int)ww, wt / stallN * 100, st, mg,
          (int)ww == (int)lroundf(width) ? "   <- now" : "");
      delay(0);
    }
    {
      float hc = targetH + 20, sh = zmpShift(hc, ACC_LIN),
            ms = supportMargin(GAIT_OFF[gait], DUTY, width, comX, comY);
      say("Tipping: static margin %.0f mm, at full acceleration the ZMP moves %.1f mm -> %.0f mm left (CoM taken ~20 mm above the chassis).",
          ms, sh, ms - sh);
    }
    if (pick < 98 && pick < width - 2)
      say("Suggestion: width %d (most torque saved while keeping a 50 mm stride). Then save.",
          (int)pick);
    if (pct > TAU_SOFT * 100)
      say("Also helps: com X Y (put the battery over the middle), ripple / wave spread the load differently (try them), or stronger femur servos.");
    return;
  }
  if (c == "lag" && n >= 2) {
    lagMs = constrain(t[1].toFloat(), 0.0f, 60.0f);
    say("Servo lag lead: %.0f ms (0 = off). save to keep.", lagMs);
    return;
  }
  if (c == "servospeed" && n >= 2) {
    servoDps = constrain(t[1].toFloat(), 200.0f, 1000.0f);
    say("Servo top speed: %.0f deg/s. save to keep.", servoDps);
    return;
  }
  if (n >= 2 && (c == "speed" || c == "height" || c == "lift" || c == "period" || c == "width" ||
                 c == "idle")) {
    float v = t[1].toFloat();
    if (c == "speed") {
      speed = constrain(v, 5.0f, maxSpeed());
      applyMove();
    }
    if (c == "height") {
      trickReturn();
      targetH = constrain(v, 40.0f, 85.0f);
      sitting = false;
      startGait();
      saveHeight();
      say("Height saved.");
    }
    if (c == "lift") {
      LIFT = constrain(v, 10.0f, 40.0f);
    }
    if (c == "period") {
      PERIOD = constrain(v, 0.5f, 2.0f);
      speed = fminf(speed, maxSpeed());
      applyMove();
    }
    if (c == "width") {
      width = constrain(v, -40.0f, 30.0f);
      updateStride(true);
      restep();
    }
    if (c == "idle") {
      idleMin = (uint32_t)constrain(v, 0.0f, 120.0f);
    }
    showParams();
    return;
  }
  if (c == "save") {
    saveSettings();
    say("Saved calibration + settings. Walking unlocked.");
    return;
  }
  if (c == "reset") {
    stopNow();
    walkCtl = wc::Controller();
    walkSampleAt = 0;
    prefs.clear();
    prefs.end();
    loadSettings();
    say("All saved calibration + settings cleared (redo dir, levelcal, mpufront, batcal).");
    return;
  }
  if (c == "levelcal") {
    if (stepping || lastMove || trick != TR_NONE) {
      say("Stand still before levelcal.");
      return;
    }
    if (!imuOK) {
      say("No MPU6050.");
      return;
    }
    float a[3];
    if (!mpuAverage(a, 100))
      return;
    norm3(a);
    memcpy(axZ, a, sizeof axZ);
    prefs.putBytes("axZ", axZ, sizeof axZ);
    walkCtl.resetSensor();
    walkSampleAt = 0;
    hasLevel = true;
    hasFront = false;
    levelOn = false;
    prefs.remove("axX");
    roll = pitch = 0;
    levP = levR = 0;
    upF.init = false;
    say("Level saved. Now lift the robot's FRONT (the end it walks towards with w) ~20 degrees, hold still, type: mpufront");
    return;
  }
  if (c == "mpufront") {
    if (stepping || lastMove || trick != TR_NONE) {
      say("Stand still before mpufront.");
      return;
    }
    if (!hasLevel) {
      say("Do levelcal first (flat floor).");
      return;
    }
    walkCtl.resetSensor();
    walkSampleAt = 0;
    float a[3];
    if (!mpuAverage(a, 100))
      return;
    norm3(a);
    float d = dot3(a, axZ), x[3] = {a[0] - d * axZ[0], a[1] - d * axZ[1], a[2] - d * axZ[2]};
    if (sqrtf(dot3(x, x)) < 0.15f) {
      say("Not tilted enough: lift the front more (~20 deg) and retry.");
      return;
    }
    norm3(x);
    if (endsSwapped)
      for (int k = 0; k < 3; k++)
        x[k] = -x[k];
    memcpy(axX, x, sizeof axX);
    cross3(axZ, axX, axY);
    upF.init = false;
    prefs.putBytes("axX", axX, sizeof axX);
    hasFront = true;
    roll = pitch = 0;
    say("Orientation saved. Check with tilt (front up = +pitch, left up = +roll), then: level on");
    return;
  }
  if (c == "level" && n >= 2) {
    String v = t[1];
    v.toLowerCase();
    if (v == "on" && !hasFront) {
      say("Set up the MPU first: levelcal, then mpufront.");
      return;
    }
    levelWasOn = (v == "on");
    levelOn = levelWasOn && imuOK;
    prefs.putBool("level", levelWasOn);
    say(levelWasOn ? (imuOK ? "Leveling ON (saved). Works with the feet on the floor."
                            : "Leveling ON saved; paused until MPU sensor reads recover.")
                   : "Leveling off (saved).");
    return;
  }
  help();
}

String bufS, bufB;
bool overflowS = false, overflowB = false;

// Bound input work per loop so long serial bursts cannot starve the control tick.
void pump(Stream &st, String &buf, bool &overflow) {
  uint32_t started = micros();
  unsigned bytes = 0;
  while (st.available() && bytes < 96 && (uint32_t)(micros() - started) < 1500) {
    char ch = st.read();
    bytes++;
    if (ch == '\n' || ch == '\r') {
      if (overflow)
        say("Command discarded: line exceeds 60 characters.");
      else {
        buf.trim();
        if (buf.length())
          handle(buf);
      }
      buf = "";
      overflow = false;
    } else if (buf.length() < 60 && !overflow)
      buf += ch;
    else
      overflow = true;
  }
}

const char *resetReasonText() {
  switch (esp_reset_reason()) {
  case ESP_RST_POWERON:
    return "power on";
  case ESP_RST_BROWNOUT:
    return "BROWNOUT (power dipped: battery, buck, wiring or J1)";
  case ESP_RST_PANIC:
    return "CRASH (software panic)";
  case ESP_RST_INT_WDT:
  case ESP_RST_TASK_WDT:
  case ESP_RST_WDT:
    return "WATCHDOG (code got stuck)";
  case ESP_RST_SW:
    return "software restart";
  case ESP_RST_DEEPSLEEP:
    return "woke from deep sleep";
  case ESP_RST_EXT:
    return "EN button / reset pin";
  default:
    return "other";
  }
}

void setup() {
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, HIGH);
  Serial.begin(115200);
  delay(300);
  resetWhy = resetReasonText();
  say("Restart reason: %s", resetWhy);
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setTimeOut(20);
  pcaL.begin();
  pcaL.setOscillatorFrequency(27000000);
  pcaL.setPWMFreq(50);
  pcaR.begin();
  pcaR.setOscillatorFrequency(27000000);
  pcaR.setPWMFreq(50);
  pcaClockSetup();
  relaxAll();
  loadSettings();
  pcaClockApply();
  xTaskCreatePinnedToCore(strideTaskFn, "stride", 8192, nullptr, 1, &strideTaskH, 0);
  BT.onAuthComplete(btAuthCb);
  BT.register_callback(btSppCb);
  if (BT.begin("Hexapod")) {
    say("Bluetooth ready: pair with \"Hexapod\" (free RAM %u).", (unsigned)ESP.getFreeHeap());
    btStatus();
  } else
    say("Bluetooth FAILED to start (free RAM %u). Send this line to Claude.",
        (unsigned)ESP.getFreeHeap());
  screenBegin();

  batV = readPinVolts() * batScale + batOffset;
  if (batV > BAT_PRESENT && batV < BAT_START_MIN) {
    say("Battery too low to use (%.2f V). Not standing up. Unplug it and charge.", batV);
    batLocked = true;
    lockedAt = millis();
    park = P_PARKED;
    parkedAt = millis();
  } else {
    for (int i = 0; i < 6; i++)
      for (int j = 0; j < 3; j++) {
        writeAngle(i, j, 0);
        delay(100);
      }
    bodyH = STAND_CAD_H;
    startGait();
    say("Hexapod v4.0.25 ready. Battery %.2f V (about %d%%).", batV, batPercent(batV));
  }
  setupIMU();
  if (USE_BUTTON)
    pinMode(TOUCH_PIN, INPUT_PULLUP);
  else {
    float sum = 0;
    for (int k = 0; k < 20; k++) {
      sum += touchRead(TOUCH_PIN);
      delay(5);
    }
    touchBase = sum / 20;
  }
  lastActivity = millis();
  showTable();
  help();
}

volatile int8_t btAuthRes = -1, btSrvRes = -1;
volatile uint8_t btConn = 0;

void btAuthCb(boolean ok) {
  btAuthRes = ok ? 1 : 0;
}

void btSppCb(esp_spp_cb_event_t e, esp_spp_cb_param_t *p) {
  if (e == ESP_SPP_START_EVT)
    btSrvRes = (p->start.status == ESP_SPP_SUCCESS) ? 1 : 0;
  else if (e == ESP_SPP_SRV_OPEN_EVT)
    btConn = 1;
  else if (e == ESP_SPP_CLOSE_EVT)
    btConn = 2;
}

void btStatus() {
  uint8_t m[6];
  BT.getBtAddress(m);
  say("Bluetooth: name Hexapod, address %02X:%02X:%02X:%02X:%02X:%02X, remembers %d paired device(s), client: %s",
      m[0], m[1], m[2], m[3], m[4], m[5], BT.getNumberOfBondedDevices(),
      BT.hasClient() ? "yes" : "no");
}

void btReport() {
  if (btSrvRes >= 0) {
    say(btSrvRes ? "Bluetooth: serial port service is up (Windows can make a COM port)."
                 : "Bluetooth: serial port service FAILED to start. Send this to Claude.");
    btSrvRes = -1;
  }
  if (btAuthRes >= 0) {
    say(btAuthRes
            ? "Bluetooth: pairing OK."
            : "Bluetooth: pairing FAILED. Type btforget, remove Hexapod in Windows, pair again.");
    btAuthRes = -1;
  }
  if (btConn) {
    screenEvent(btConn == 1 ? EV_BT_ON : EV_BT_OFF);
    say(btConn == 1 ? "Bluetooth: serial monitor connected."
                    : "Bluetooth: serial monitor disconnected.");
    btConn = 0;
  }
}

// Service inputs and sensors around the control tick; background work must not take over gait
// timing.
void loop() {
  uint32_t inputStart = micros();
  pump(Serial, bufS, overflowS);
  pump(BT, bufB, overflowB);
  btReport();
  loopTiming.record(LoopTiming::INPUT_SERVICE, inputStart, micros());
  static uint32_t last = millis(), lastSec = 0, lastUs = micros();
  uint32_t now = millis();

  bool due = syncOK ? frameFlag : (now - last >= 20);
  if (syncOK && now - last > 60)
    due = true;
  if (due) {
    frameFlag = false;
    uint32_t nowUs = micros();
    float dt = (nowUs - lastUs) * 1e-6f;
    lastUs = nowUs;
    bool resetTiming = timingResetRequested;
    if (resetTiming) {
      loopTiming.reset();
      worstGapMs = 0;
      overruns = 0;
      mpuWorstReadUs = 0;
      timingResetRequested = false;
    }
    float gapMs = dt * 1000;
    if (!resetTiming) {
      worstGapMs = fmaxf(worstGapMs, gapMs);
      if (gapMs > 30)
        overruns++;
    }
    if (gapMs > 250 && (lastMove || stepping)) {
      walkCtl.cancel(wc::FAULT);
      stopMoving();
      say("Control loop stalled %.0f ms (missed deadline): stopping.", gapMs);
    }
    if (dt > 0.1f)
      dt = 0.1f;
    if (dt < 0.005f)
      dt = 0.005f;
    tickDt = dt;
    last = now;
    uint32_t workStart = micros(), stageStart = workStart;
    centerUpdate();
    loopTiming.record(LoopTiming::CENTER, stageStart, micros());
    stageStart = micros();
    imuUpdate(dt);
    loopTiming.record(LoopTiming::MPU, stageStart, micros());
    stageStart = micros();
    walkUpdate(dt);
    loopTiming.record(LoopTiming::ASSIST, stageStart, micros());
    stageStart = micros();
    if (mode != GAIT)
      loadX = servosOn ? 0.5f : 0;
    batSumV += analogReadMilliVolts(BAT_PIN);
    batSumX += loadX;
    batN++;
    touchUpdate();
    servoHoldService();
    loopTiming.record(LoopTiming::IO, stageStart, micros());
    stageStart = micros();
    if (mode == GAIT && !servoHold) {
      gaitUpdate(dt);
      applyFeet();
      if (!servoHold)
        partyCheck();
    } else {
      equationDynamicsInvalidate();
      if (trick != TR_NONE)
        trickStop();
    }
    parkUpdate();
    statusLED();
    loopTiming.record(LoopTiming::FEET, stageStart, micros());
    stageStart = micros();
    linkService();
    loopTiming.record(LoopTiming::SCREEN, stageStart, micros());
    stageStart = micros();
    pcamonService();
    if (now - lastSec >= 1000) {
      lastSec = now;
      pcaClockService();
      batteryCheck();
      heatCheck();
      powerCheck();
      fidgetCheck();
    }
    loopTiming.record(LoopTiming::HOUSEKEEPING, stageStart, micros());
    loopTiming.work(workStart, micros());
  }
}
