// IMU timing, projected angular rates and stillness checks for calibration.

#pragma once
#include <math.h>
#include <stdint.h>

namespace imath {
struct Rates {
  float heading = 0, pitch = 0, roll = 0;
};

inline bool rates(const float u[3], const float w[3], Rates &out) {
  for (int k = 0; k < 3; k++)
    if (!isfinite(u[k]) || !isfinite(w[k]))
      return false;
  float yz2 = u[1] * u[1] + u[2] * u[2];
  if (yz2 < .10f)
    return false;
  float dx = u[1] * w[2] - u[2] * w[1];
  float dy = u[2] * w[0] - u[0] * w[2];
  float dz = u[0] * w[1] - u[1] * w[0];
  out.heading = (u[1] * w[1] + u[2] * w[2]) / yz2;
  out.pitch = dx / sqrtf(yz2);
  out.roll = (u[2] * dy - u[1] * dz) / yz2;
  return isfinite(out.heading) && isfinite(out.pitch) && isfinite(out.roll);
}

struct SampleTime {
  float dt;
  bool continuous, restart;
  float elapsed;
};

// Use complete-sample timestamps; a long gap restarts attitude instead of integrating missing
// motion.
struct SampleClock {
  uint32_t last = 0;
  bool seen = false, missed = false;

  void reset() {
    seen = false;
    missed = false;
  }

  void fail() {
    missed = true;
  }

  SampleTime accept(uint32_t now) {
    float dt = (uint32_t)(now - last) * 1e-6f;
    bool restart = !seen || dt > .1f || dt < .001f;
    SampleTime result = {restart ? .02f : dt, seen && !missed && !restart, restart, seen ? dt : 0};
    last = now;
    seen = true;
    missed = false;
    return result;
  }
};

// Learn residual gyro bias from valid samples while the commanded robot is settled.
// Short read gaps may occur inside the window; no failed sample contributes sensor data.
struct GyroQuietWindow {
  static constexpr uint32_t MAX_GAP_US = 80000, MAX_WINDOW_US = 4000000;
  static constexpr uint32_t MIN_WINDOW_US = 1500000;
  static constexpr unsigned MIN_SAMPLES = 75;
  double gyroSum[3] = {}, gyroSquare[3] = {}, accelSum[3] = {}, accelSquare[3] = {};
  double firstAccel[3] = {};
  uint32_t started = 0, lastGood = 0;
  unsigned count = 0, attempts = 0;
  const char *state = "waiting for a settled robot";

  void reset() {
    count = attempts = 0;
    for (int k = 0; k < 3; k++)
      gyroSum[k] = gyroSquare[k] = accelSum[k] = accelSquare[k] = firstAccel[k] = 0;
  }

  void miss(uint32_t now) {
    if (!count)
      return;
    attempts++;
    if ((uint32_t)(now - lastGood) > MAX_GAP_US || (uint32_t)(now - started) > MAX_WINDOW_US) {
      reset();
      state = "read gap too long";
    }
  }

  bool add(const float a[3], const float g[3], uint32_t now, float correction[3]) {
    if (count &&
        ((uint32_t)(now - lastGood) > MAX_GAP_US || (uint32_t)(now - started) > MAX_WINDOW_US))
      reset();
    float norm2 = 0, gyro2 = 0;
    for (int k = 0; k < 3; k++) {
      if (!isfinite(a[k]) || !isfinite(g[k])) {
        reset();
        state = "invalid sample";
        return false;
      }
      norm2 += a[k] * a[k];
      gyro2 += g[k] * g[k];
    }
    if (norm2 < .75f * .75f || norm2 > 1.25f * 1.25f || gyro2 > 3 * 3) {
      reset();
      state = "movement or acceleration";
      return false;
    }
    if (!count)
      started = now;
    lastGood = now;
    for (int k = 0; k < 3; k++) {
      gyroSum[k] += g[k];
      gyroSquare[k] += (double)g[k] * g[k];
      accelSum[k] += a[k];
      accelSquare[k] += (double)a[k] * a[k];
      if (count < MIN_SAMPLES / 2)
        firstAccel[k] += a[k];
    }
    count++;
    attempts++;
    state = "collecting quiet samples";
    if (count < MIN_SAMPLES || (uint32_t)(now - started) < MIN_WINDOW_US)
      return false;

    // Require 60% usable reads, low vibration and a stable gravity direction.
    bool usable = count * 5 >= attempts * 3;
    double accelNoise2 = 0;
    float first[3], second[3], first2 = 0, second2 = 0;
    for (int k = 0; k < 3; k++) {
      double mean = gyroSum[k] / count, amean = accelSum[k] / count;
      if (gyroSquare[k] / count - mean * mean >= .3 * .3)
        usable = false;
      correction[k] = (float)mean;
      accelNoise2 += fmax(0.0, accelSquare[k] / count - amean * amean);
      first[k] = (float)(firstAccel[k] / (MIN_SAMPLES / 2));
      second[k] = (float)((accelSum[k] - firstAccel[k]) / (count - MIN_SAMPLES / 2));
      first2 += first[k] * first[k];
      second2 += second[k] * second[k];
    }
    float dot = 0;
    for (int k = 0; k < 3; k++)
      dot += first[k] * second[k];
    float directionCos = dot / sqrtf(first2 * second2);
    usable = usable && accelNoise2 <= .025 * .025 && isfinite(directionCos) &&
             directionCos >= cosf(.15f * .0174532925f);
    reset();
    state = usable ? "quiet bias accepted" : "window too noisy, moving or missing reads";
    return usable;
  }
};

enum CalibrationReason {
  CAL_OK,
  CAL_TOO_FEW_READS,
  CAL_GYRO_MOVING,
  CAL_BAD_GRAVITY,
  CAL_ACCEL_UNSTABLE
};

struct CalibrationQuality {
  CalibrationReason reason = CAL_TOO_FEW_READS;
  float accelNorm = 0, accelNoise = 0;
};

inline const char *calibrationReason(CalibrationReason reason) {
  switch (reason) {
  case CAL_OK:
    return "quiet sample window";
  case CAL_TOO_FEW_READS:
    return "too few usable reads (need at least 80%)";
  case CAL_GYRO_MOVING:
    return "gyro movement or residual bias (limit 3 deg/s)";
  case CAL_BAD_GRAVITY:
    return "acceleration is not near gravity (need 0.75-1.25 g)";
  case CAL_ACCEL_UNSTABLE:
    return "acceleration changed too much (noise limit 0.025 g)";
  }
  return "invalid calibration status";
}

// Accept calibration only with enough valid reads, near-1 g acceleration and low gyro motion.
struct CalibrationMean {
  double sum[3] = {}, square[3] = {};
  float gyroPeak = 0;
  int count = 0;

  void add(const float a[3], const float g[3]) {
    for (int k = 0; k < 3; k++)
      if (!isfinite(a[k]) || !isfinite(g[k]))
        return;
    for (int k = 0; k < 3; k++) {
      sum[k] += a[k];
      square[k] += (double)a[k] * a[k];
    }
    gyroPeak = fmaxf(gyroPeak, sqrtf(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]));
    count++;
  }

  CalibrationQuality quality(int requested) const {
    CalibrationQuality q;
    double variance = 0, norm2 = 0;
    if (count > 0)
      for (int k = 0; k < 3; k++) {
        float mean = (float)(sum[k] / count);
        norm2 += (double)mean * mean;
        variance += fmax(0.0, square[k] / count - (sum[k] / count) * (sum[k] / count));
      }
    q.accelNorm = (float)sqrt(norm2);
    q.accelNoise = (float)sqrt(variance);
    if (requested <= 0 || count * 5 < requested * 4 || count < 2)
      q.reason = CAL_TOO_FEW_READS;
    else if (gyroPeak > 3)
      q.reason = CAL_GYRO_MOVING;
    else if (norm2 < .75 * .75 || norm2 > 1.25 * 1.25)
      q.reason = CAL_BAD_GRAVITY;
    else if (variance > .025 * .025)
      q.reason = CAL_ACCEL_UNSTABLE;
    else
      q.reason = CAL_OK;
    return q;
  }

  bool finish(float a[3], int requested) const {
    if (quality(requested).reason != CAL_OK)
      return false;
    for (int k = 0; k < 3; k++)
      a[k] = (float)(sum[k] / count);
    return true;
  }
};
}
