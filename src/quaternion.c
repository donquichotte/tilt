#include "quaternion.h"
#include <math.h>
#include <stdio.h>
#include <string.h>

#define EPS 1.0e-5f
#define DEBUG 0
#define DEBUGPRINT(fmt, ...)                                                   \
  do {                                                                         \
    if (DEBUG)                                                                 \
      fprintf(stdout, fmt, __VA_ARGS__);                                       \
  } while (0)

float vec3_abs(const float in[3]) {
  return sqrt(in[0] * in[0] + in[1] * in[1] + in[2] * in[2]);
}

bool vec3_normalize(const float in[3], float out[3]) {
  float abs = vec3_abs(in);
  if (abs < EPS) {
    DEBUGPRINT("Small absolute value in %s\n", __FUNCTION__);
    out[0] = 0.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    return false;
  }

  for (int i = 0; i < 3; i++) {
    out[i] = in[i] / abs;
  }

  return true;
}

void vec3_cross(const float u[3], const float v[3], float out[3]) {
  out[0] = u[1] * v[2] - u[2] * v[1];
  out[1] = u[2] * v[0] - u[0] * v[2];
  out[2] = u[0] * v[1] - u[1] * v[0];
}

float vec3_dot(const float u[3], const float v[3]) {
  return u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
}

void quaternion_fromAngleAndAxis(const float angle, const float axis[3],
                                 float out[4]) {
  float s = sin(angle / 2.0f);
  float c = cos(angle / 2.0f);
  out[0] = c;
  out[1] = axis[0] * s;
  out[2] = axis[1] * s;
  out[3] = axis[2] * s;
  float abs = quaternion_abs(out);
  if (abs < EPS) {
    DEBUGPRINT("Small absolute value in %s\n", __FUNCTION__);
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
  }
}

void quaternion_multiply(const float q[4], const float p[4], float out[4]) {
  float temp[4];
  temp[0] = q[0] * p[0] - q[1] * p[1] - q[2] * p[2] - q[3] * p[3];
  temp[1] = q[0] * p[1] + q[1] * p[0] + q[2] * p[3] - q[3] * p[2];
  temp[2] = q[0] * p[2] - q[1] * p[3] + q[2] * p[0] + q[3] * p[1];
  temp[3] = q[0] * p[3] + q[1] * p[2] - q[2] * p[1] + q[3] * p[0];
  memcpy(out, temp, sizeof(temp));
}

void quaternion_fromVector(const float vec[3], float out[4]) {
  out[0] = 0;
  out[1] = vec[0];
  out[2] = vec[1];
  out[3] = vec[2];
}

void quaternion_rotate(const float qr[4], const float qu[4], float out[4]) {
  float temp[4];
  float qrInverse[4];
  quaternion_inverse(qr, qrInverse);
  quaternion_multiply(qu, qrInverse, temp);
  quaternion_multiply(qr, temp, out);
}

float quaternion_abs(const float q[4]) {
  return sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
}

void quaternion_normalize(const float in[4], float out[4]) {
  float abs = quaternion_abs(in);
  if (abs < EPS) {
    DEBUGPRINT("Small absolute value in %s\n", __FUNCTION__);
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
  }
  for (int i = 0; i < 4; i++) {
    out[i] = in[i] / abs;
  }
}

void quaternion_inverse(const float in[4], float out[4]) {
  float qConj[4];
  quaternion_conjugate(in, qConj);

  float absSquared = quaternion_abs(in);
  absSquared = absSquared * absSquared;
  if (absSquared < EPS) {
    DEBUGPRINT("Small absolute value in %s\n", __FUNCTION__);
    out[0] = 1.0f;
    out[1] = 0.0f;
    out[2] = 0.0f;
    out[3] = 0.0f;
  }
  out[0] = qConj[0] / absSquared;
  out[1] = qConj[1] / absSquared;
  out[2] = qConj[2] / absSquared;
  out[3] = qConj[3] / absSquared;
}

void quaternion_conjugate(const float in[4], float out[4]) {
  out[0] = in[0];
  out[1] = -in[1];
  out[2] = -in[2];
  out[3] = -in[3];
}
