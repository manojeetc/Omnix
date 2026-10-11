// Checked wrappers for generated geometry and dynamics. Inputs and outputs use SI units.

#pragma once
#include "equations_generated.h"

namespace heq {
inline bool finiteArray(const float *v, int n) {
  for (int i = 0; i < n; i++)
    if (!isfinite(v[i]))
      return false;
  return true;
}

// Solve a row-major 3 x 3 system. Normalize the determinant guard so changing length units cannot
// bypass it.
inline bool solve3(const float j[9], const float v[3], float out[3]) {
  if (!finiteArray(j, 9) || !finiteArray(v, 3))
    return false;
  float a = j[0], b = j[1], c = j[2], d = j[3], e = j[4], f = j[5], g = j[6], h = j[7], i = j[8];
  float det = a * (e * i - f * h) - b * (d * i - f * g) + c * (d * h - e * g);
  float norms = sqrtf((a * a + d * d + g * g) * (b * b + e * e + h * h) * (c * c + f * f + i * i));
  if (!isfinite(norms) || norms <= 0 || fabsf(det) <= .005f * norms)
    return false;
  float candidate[3] = {
      ((e * i - f * h) * v[0] + (c * h - b * i) * v[1] + (b * f - c * e) * v[2]) / det,
      ((f * g - d * i) * v[0] + (a * i - c * g) * v[1] + (c * d - a * f) * v[2]) / det,
      ((d * h - e * g) * v[0] + (b * g - a * h) * v[1] + (a * e - b * d) * v[2]) / det};
  if (!finiteArray(candidate, 3))
    return false;
  for (int k = 0; k < 3; k++)
    out[k] = candidate[k];
  return true;
}

// J transpose times force gives signed external joint load in N m, not a hobby-servo torque
// command.
inline void externalTorque(const float kin[22], const float force[3], float tau[3]) {
  for (int k = 0; k < 3; k++)
    tau[k] = kin[3 + k] * force[0] + kin[6 + k] * force[1] + kin[9 + k] * force[2];
}

inline bool jointVelocity(const float q[3], float az, const float geom[6], const float v[3],
                          float out[3]) {
  if (!finiteArray(q, 3) || !finiteArray(geom, 6) || !isfinite(az))
    return false;
  const float z[3] = {};
  float k[22];
  kinematics(q, az, geom, z, k);
  return solve3(k + 3, v, out);
}

inline bool jointAcceleration(const float q[3], const float qd[3], float az, const float geom[6],
                              const float a[3], float out[3]) {
  if (!finiteArray(q, 3) || !finiteArray(qd, 3) || !finiteArray(geom, 6) || !finiteArray(a, 3) ||
      !isfinite(az))
    return false;
  float k[22], rhs[3];
  kinematics(q, az, geom, qd, k);
  for (int i = 0; i < 3; i++)
    rhs[i] = a[i] - k[12 + 3 * i] * qd[0] - k[13 + 3 * i] * qd[1] - k[14 + 3 * i] * qd[2];
  return solve3(k + 3, rhs, out);
}

// Estimate joint rates at both ends of a small command interval. This is not a bound between them;
// large jumps and pauses fall back to the firmware's angle-difference guard.
inline bool endpointRate(const float previous[3], const float current[3], float az,
                         const float geom[6], float dt, float &peak) {
  if (!isfinite(dt) || dt < .005f || dt > .060f || !finiteArray(previous, 3) ||
      !finiteArray(current, 3))
    return false;
  for (int i = 0; i < 3; i++)
    if (fabsf(current[i] - previous[i]) > .174533f)
      return false;
  const float z[3] = {};
  float a[22], b[22], v[3], qa[3], qb[3];
  kinematics(previous, az, geom, z, a);
  kinematics(current, az, geom, z, b);
  for (int i = 0; i < 3; i++)
    v[i] = (b[i] - a[i]) / dt;
  if (!solve3(a + 3, v, qa) || !solve3(b + 3, v, qb))
    return false;
  float p = 0;
  for (int i = 0; i < 3; i++)
    p = fmaxf(p, fmaxf(fabsf(qa[i]), fabsf(qb[i])));
  if (!isfinite(p))
    return false;
  peak = p;
  return true;
}

inline void jointMass(const float q[3], float az, const float model[42], float out[9]) {
  const float z[3] = {}, base[9] = {};
  float a[3] = {}, r[9];
  for (int col = 0; col < 3; col++) {
    a[col] = 1;
    inverse_dynamics(q, z, a, az, model, base, z, z, r);
    a[col] = 0;
    for (int row = 0; row < 3; row++)
      out[3 * row + col] = r[row];
  }
}

// Evaluate position, velocity, acceleration and jerk between the supplied endpoint conditions.
inline bool quintic(const float boundary[6], float duration, float elapsed, float out[4]) {
  if (!finiteArray(boundary, 6) || !isfinite(duration) || duration <= .0001f || !isfinite(elapsed))
    return false;
  float c[6], v[4];
  quintic_coefficients(boundary, duration, c);
  float u = fmaxf(0, fminf(1, elapsed / duration));
  quintic_evaluate(c, duration, u, v);
  if (!finiteArray(v, 4))
    return false;
  for (int i = 0; i < 4; i++)
    out[i] = v[i];
  return true;
}
}
