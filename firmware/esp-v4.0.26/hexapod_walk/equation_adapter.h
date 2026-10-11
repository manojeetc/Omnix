// Convert firmware mm/degrees to SI/radians at the generated-equation boundary.

#pragma once
#include "gait_math.h"
#include "equations.h"

namespace heq {
inline void firmwareGeometry(float out[6]) {
  out[0] = HIP_OUT * .001f;
  out[1] = HIP_UP * .001f;
  out[2] = FEMUR * .001f;
  out[3] = TIBIA * .001f;
  out[4] = SIDE * .001f;
  out[5] = COXA_Z * .001f;
}

inline void toRadians3(const float deg[3], float out[3]) {
  for (int j = 0; j < 3; j++)
    out[j] = deg[j] * GM_D2R;
}

inline bool firmwareEndpointRate(int leg, const float previousDeg[3], const float currentDeg[3],
                                 float dt, float &peakDeg) {
  if (leg < 0 || leg >= 6)
    return false;
  float g[6], a[3], b[3], rate;
  firmwareGeometry(g);
  toRadians3(previousDeg, a);
  toRadians3(currentDeg, b);
  if (!endpointRate(a, b, LEG[leg].out * GM_D2R, g, dt, rate))
    return false;
  peakDeg = rate * GM_R2D;
  return true;
}

// Resolve reaction force along bodyUp and return signed joint loads in N m.
inline bool firmwareReactionTorque(int leg, const float deg[3], float newtons,
                                   const float bodyUp[3], float nm[3]) {
  if (leg < 0 || leg >= 6 || !finiteArray(deg, 3) || !finiteArray(bodyUp, 3) ||
      !isfinite(newtons) || newtons < 0)
    return false;
  float q[3], g[6], k[22], z[3] = {}, f[3];
  toRadians3(deg, q);
  firmwareGeometry(g);
  kinematics(q, LEG[leg].out * GM_D2R, g, z, k);
  for (int j = 0; j < 3; j++)
    f[j] = newtons * bodyUp[j];
  float value[3];
  externalTorque(k, f, value);
  if (!finiteArray(value, 3))
    return false;
  for (int j = 0; j < 3; j++)
    nm[j] = value[j];
  return true;
}
}
